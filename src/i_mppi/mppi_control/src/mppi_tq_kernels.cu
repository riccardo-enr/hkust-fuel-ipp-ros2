#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <device_launch_parameters.h>

#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mppi_control/mppi_utils.cuh"

extern "C"
{

  struct MPPIParamsDeviceTq
  {
    int K;
    int H;
    float dt;
    float lambda;
    float sigma_thrust;
    float sigma_quat;
    float Q_pos_x;
    float Q_pos_y;
    float Q_pos_z;
    float Q_vel_x;
    float Q_vel_y;
    float Q_vel_z;
    float R_thrust;
    float R_quat;
    float R_rate_thrust;
    float R_rate_quat;
    float w_obs;
    float thrust_max;
    float thrust_min;
    float g;
  };

  // Control sample at each time step - structure
  struct ControlSample
  {
    float thrust;
    float4 quat; // (x, y, z, w)
  };

  struct ControlInput
  {
    float thrust;
    float4 quat;
  };

  __global__ void mppi_tq_kernel(
      const ControlInput *u_mean,
      const ControlInput u_prev,
      const float3 curr_p,
      const float3 curr_v,
      const float3 ref_p_base,
      const float3 ref_v_base,
      const float3 ref_a_base,
      const MPPIParamsDeviceTq params,
      unsigned int seed,
      ControlSample *samples_u, // Out: [K * H]
      float *costs              // Out: [K]
  )
  {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= params.K)
      return;

    curandState state;
    curand_init(seed, k, 0, &state);

    Eigen::Vector3f p(curr_p.x, curr_p.y, curr_p.z);
    Eigen::Vector3f v(curr_v.x, curr_v.y, curr_v.z);
    Eigen::Quaternionf prev_quat(u_prev.quat.w, u_prev.quat.x, u_prev.quat.y, u_prev.quat.z);
    float prev_thrust = u_prev.thrust; // For thrust rate penalty
    float total_cost = 0.0f;

    for (int h = 0; h < params.H; ++h)
    {
      // Constant reference (goal position, zero velocity)
      Eigen::Vector3f ref_p(ref_p_base.x, ref_p_base.y, ref_p_base.z);
      Eigen::Vector3f ref_v = Eigen::Vector3f::Zero();

      // Noise generation
      float thrust_noise = curand_normal(&state) * params.sigma_thrust;

      // Quaternion noise (3D tangent space -> exponential map)
      Eigen::Vector3f quat_noise;
      quat_noise.x() = curand_normal(&state) * params.sigma_quat;
      quat_noise.y() = curand_normal(&state) * params.sigma_quat;
      quat_noise.z() = curand_normal(&state) * params.sigma_quat;

      float omega_norm = quat_noise.norm();
      Eigen::Quaternionf quat_delta;
      if (omega_norm < 1e-6f)
      {
        quat_delta = Eigen::Quaternionf::Identity();
      }
      else
      {
        quat_delta = Eigen::Quaternionf(Eigen::AngleAxisf(omega_norm, quat_noise / omega_norm));
      }

      // Compose: q_sample = q_mean * q_delta (body frame perturbation)
      Eigen::Quaternionf q_mean(u_mean[h].quat.w, u_mean[h].quat.x, u_mean[h].quat.y, u_mean[h].quat.z);
      Eigen::Quaternionf q_sample = (q_mean * quat_delta).normalized();

      // Apply thrust constraints
      float thrust = u_mean[h].thrust + thrust_noise;
      if (thrust < params.thrust_min)
        thrust = params.thrust_min;
      if (thrust > params.thrust_max)
        thrust = params.thrust_max;

      // Store sample
      samples_u[k * params.H + h].thrust = thrust;
      samples_u[k * params.H + h].quat = make_float4(q_sample.x(), q_sample.y(), q_sample.z(), q_sample.w());

      // --- Dynamics ---
      // Rotate thrust vector by quaternion: F_world = R(q) * [0, 0, thrust]
      Eigen::Vector3f F_world = q_sample * Eigen::Vector3f(0.0f, 0.0f, thrust);

      // Acceleration (thrust is acceleration magnitude T/m)
      Eigen::Vector3f acc = F_world - Eigen::Vector3f(0.0f, 0.0f, params.g);

      // RK4 integration
      Eigen::Vector3f k1_p = v;
      Eigen::Vector3f k1_v = acc;

      Eigen::Vector3f k2_p = v + 0.5f * params.dt * k1_v;
      Eigen::Vector3f k2_v = acc;

      Eigen::Vector3f k3_p = v + 0.5f * params.dt * k2_v;
      Eigen::Vector3f k3_v = acc;

      Eigen::Vector3f k4_p = v + params.dt * k3_v;
      Eigen::Vector3f k4_v = acc;

      p += (params.dt / 6.0f) * (k1_p + 2.0f * k2_p + 2.0f * k3_p + k4_p);
      v += (params.dt / 6.0f) * (k1_v + 2.0f * k2_v + 2.0f * k3_v + k4_v);

      // --- Cost Calculation ---
      Eigen::Vector3f dp = p - ref_p;
      Eigen::Vector3f dv = v - ref_v;

      total_cost += params.Q_pos_x * dp.x() * dp.x();
      total_cost += params.Q_pos_y * dp.y() * dp.y();
      total_cost += params.Q_pos_z * dp.z() * dp.z();
      total_cost += params.Q_vel_x * dv.x() * dv.x();
      total_cost += params.Q_vel_y * dv.y() * dv.y();
      total_cost += params.Q_vel_z * dv.z() * dv.z();

      // Thrust control cost (penalize deviation from hover)
      float d_thrust = thrust - params.g;
      total_cost += params.R_thrust * d_thrust * d_thrust;

      // Quaternion control cost (penalize rotation from identity)
      total_cost += params.R_quat * q_sample.vec().squaredNorm();

      // Thrust rate penalty
      float d_thrust_rate = thrust - prev_thrust;
      total_cost += params.R_rate_thrust * d_thrust_rate * d_thrust_rate;

      // Quaternion rate penalty
      Eigen::Quaternionf q_rate_diff = prev_quat.conjugate() * q_sample;
      total_cost += params.R_rate_quat * q_rate_diff.vec().squaredNorm();

      prev_thrust = thrust;
      prev_quat = q_sample;
    }

    costs[k] = total_cost;
  }

  void launch_mppi_tq_kernel(
    const ControlInput *u_mean_host,
    ControlInput u_prev,
    float3 curr_p, float3 curr_v,
    float3 ref_p_base, float3 ref_v_base, float3 ref_a_base,
    int K, int H, float dt, float lambda,
    float sigma_thrust, float sigma_quat,
    float Q_pos_x, float Q_pos_y, float Q_pos_z,
    float Q_vel_x, float Q_vel_y, float Q_vel_z,
    float R_thrust, float R_quat,
    float R_rate_thrust, float R_rate_quat,
    float w_obs, float thrust_max, float thrust_min, float g,
    ControlSample *samples_u_host,
    float *costs_host,
    unsigned int seed)
{
  MPPIParamsDeviceTq params = {
      K, H, dt, lambda,
      sigma_thrust, sigma_quat,
      Q_pos_x, Q_pos_y, Q_pos_z,
      Q_vel_x, Q_vel_y, Q_vel_z,
      R_thrust, R_quat,
      R_rate_thrust, R_rate_quat,
      w_obs,
      thrust_max, thrust_min, g};

  ControlInput *d_u_mean;
  ControlSample *d_samples_u;
  float *d_costs;

  cudaMalloc(&d_u_mean, H * sizeof(ControlInput));
  cudaMalloc(&d_samples_u, K * H * sizeof(ControlSample));
  cudaMalloc(&d_costs, K * sizeof(float));

  cudaMemcpy(d_u_mean, u_mean_host, H * sizeof(ControlInput), cudaMemcpyHostToDevice);

  int threadsPerBlock = 256;
  int blocksPerGrid = (K + threadsPerBlock - 1) / threadsPerBlock;

  mppi_tq_kernel<<<blocksPerGrid, threadsPerBlock>>>(
      d_u_mean, u_prev, curr_p, curr_v,
      ref_p_base, ref_v_base, ref_a_base,
      params, seed, d_samples_u, d_costs);

  cudaMemcpy(samples_u_host, d_samples_u, K * H * sizeof(ControlSample), cudaMemcpyDeviceToHost);
  cudaMemcpy(costs_host, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

  cudaFree(d_u_mean);
  cudaFree(d_samples_u);
  cudaFree(d_costs);
}

} // extern "C"
