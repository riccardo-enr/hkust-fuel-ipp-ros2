#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <device_launch_parameters.h>
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
      const float4 ref_quat_base,
      const float ref_thrust_base,
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

    float3 p = curr_p;
    float3 v = curr_v;
    float4 prev_quat = u_prev.quat;    // For angular velocity calculation
    float prev_thrust = u_prev.thrust; // For thrust rate penalty
    float total_cost = 0.0f;

    for (int h = 0; h < params.H; ++h)
    {
      // Constant reference (goal position, zero velocity)
      float3 ref_p = ref_p_base;
      float3 ref_v = make_float3(0.0f, 0.0f, 0.0f);

      // Assume constant reference orientation/thrust (approximation)
      float4 ref_quat = ref_quat_base;
      float ref_thrust = ref_thrust_base;

      // Noise generation
      float thrust_noise = curand_normal(&state) * params.sigma_thrust;

      // Quaternion noise (3D tangent space -> exponential map)
      float3 quat_noise;
      quat_noise.x = curand_normal(&state) * params.sigma_quat;
      quat_noise.y = curand_normal(&state) * params.sigma_quat;
      quat_noise.z = curand_normal(&state) * params.sigma_quat;

      float omega_norm = sqrtf(quat_noise.x * quat_noise.x + quat_noise.y * quat_noise.y + quat_noise.z * quat_noise.z);
      float4 quat_delta;
      if (omega_norm < 1e-6f)
      {
        quat_delta = make_float4(0.0f, 0.0f, 0.0f, 1.0f); // Identity
      }
      else
      {
        float s = sinf(omega_norm / 2.0f) / omega_norm;
        quat_delta.x = quat_noise.x * s;
        quat_delta.y = quat_noise.y * s;
        quat_delta.z = quat_noise.z * s;
        quat_delta.w = cosf(omega_norm / 2.0f);
      }

      // Compose: q_sample = q_mean * q_delta (body frame perturbation)

      float4 q_mean = u_mean[h].quat;
      float4 q_sample;
      q_sample.x = q_mean.w * quat_delta.x + q_mean.x * quat_delta.w + q_mean.y * quat_delta.z - q_mean.z * quat_delta.y;
      q_sample.y = q_mean.w * quat_delta.y - q_mean.x * quat_delta.z + q_mean.y * quat_delta.w + q_mean.z * quat_delta.x;
      q_sample.z = q_mean.w * quat_delta.z + q_mean.x * quat_delta.y - q_mean.y * quat_delta.x + q_mean.z * quat_delta.w;
      q_sample.w = q_mean.w * quat_delta.w - q_mean.x * quat_delta.x - q_mean.y * quat_delta.y - q_mean.z * quat_delta.z;

      // Normalize
      float q_norm = sqrtf(q_sample.x * q_sample.x + q_sample.y * q_sample.y + q_sample.z * q_sample.z + q_sample.w * q_sample.w);
      q_sample.x /= q_norm;
      q_sample.y /= q_norm;
      q_sample.z /= q_norm;
      q_sample.w /= q_norm;

      // Apply thrust constraints
      float thrust = u_mean[h].thrust + thrust_noise;
      if (thrust < params.thrust_min)
        thrust = params.thrust_min;
      if (thrust > params.thrust_max)
        thrust = params.thrust_max;

      // Store sample
      samples_u[k * params.H + h].thrust = thrust;
      samples_u[k * params.H + h].quat = q_sample;

      // --- Dynamics ---
      // Rotate thrust vector by quaternion: F_world = R(q) * [0, 0, thrust]
      float3 F_world;
      F_world.x = (2.0f * q_sample.x * q_sample.z + 2.0f * q_sample.y * q_sample.w) * thrust;
      F_world.y = (2.0f * q_sample.y * q_sample.z - 2.0f * q_sample.x * q_sample.w) * thrust;
      F_world.z = (1.0f - 2.0f * q_sample.x * q_sample.x - 2.0f * q_sample.y * q_sample.y) * thrust;

      // Acceleration (thrust is acceleration magnitude T/m)
      float3 acc;
      acc.x = F_world.x;
      acc.y = F_world.y;
      acc.z = F_world.z - params.g;

      // RK4 integration
      float3 k1_p = v;
      float3 k1_v = acc;

      float3 k2_p = make_float3(v.x + 0.5f * params.dt * k1_v.x, v.y + 0.5f * params.dt * k1_v.y, v.z + 0.5f * params.dt * k1_v.z);
      float3 k2_v = acc;

      float3 k3_p = make_float3(v.x + 0.5f * params.dt * k2_v.x, v.y + 0.5f * params.dt * k2_v.y, v.z + 0.5f * params.dt * k2_v.z);
      float3 k3_v = acc;

      float3 k4_p = make_float3(v.x + params.dt * k3_v.x, v.y + params.dt * k3_v.y, v.z + params.dt * k3_v.z);
      float3 k4_v = acc;

      p.x += (params.dt / 6.0f) * (k1_p.x + 2.0f * k2_p.x + 2.0f * k3_p.x + k4_p.x);
      p.y += (params.dt / 6.0f) * (k1_p.y + 2.0f * k2_p.y + 2.0f * k3_p.y + k4_p.y);
      p.z += (params.dt / 6.0f) * (k1_p.z + 2.0f * k2_p.z + 2.0f * k3_p.z + k4_p.z);

      v.x += (params.dt / 6.0f) * (k1_v.x + 2.0f * k2_v.x + 2.0f * k3_v.x + k4_v.x);
      v.y += (params.dt / 6.0f) * (k1_v.y + 2.0f * k2_v.y + 2.0f * k3_v.y + k4_v.y);
      v.z += (params.dt / 6.0f) * (k1_v.z + 2.0f * k2_v.z + 2.0f * k3_v.z + k4_v.z);

      // --- Cost Calculation ---
      float dp_x = p.x - ref_p.x;
      float dp_y = p.y - ref_p.y;
      float dp_z = p.z - ref_p.z;
      float dv_x = v.x - ref_v.x;
      float dv_y = v.y - ref_v.y;
      float dv_z = v.z - ref_v.z;

      total_cost += params.Q_pos_x * dp_x * dp_x;
      total_cost += params.Q_pos_y * dp_y * dp_y;
      total_cost += params.Q_pos_z * dp_z * dp_z;
      total_cost += params.Q_vel_x * dv_x * dv_x;
      total_cost += params.Q_vel_y * dv_y * dv_y;
      total_cost += params.Q_vel_z * dv_z * dv_z;

      // Thrust control cost (penalize deviation from hover)
      float d_thrust = thrust - params.g;
      total_cost += params.R_thrust * d_thrust * d_thrust;

      // Quaternion control cost (penalize rotation from identity)
      total_cost += params.R_quat * quat_vec_norm_sq(q_sample);

      // Thrust rate penalty
      float d_thrust_rate = thrust - prev_thrust;
      total_cost += params.R_rate_thrust * d_thrust_rate * d_thrust_rate;

      // Quaternion rate penalty
      float4 q_rate_diff = quat_diff(q_sample, prev_quat);
      total_cost += params.R_rate_quat * quat_vec_norm_sq(q_rate_diff);

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
      float4 ref_quat_base, float ref_thrust_base,
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
        ref_p_base, ref_v_base, ref_a_base, ref_quat_base, ref_thrust_base,
        params, seed, d_samples_u, d_costs);

    cudaMemcpy(samples_u_host, d_samples_u, K * H * sizeof(ControlSample), cudaMemcpyDeviceToHost);
    cudaMemcpy(costs_host, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_u_mean);
    cudaFree(d_samples_u);
    cudaFree(d_costs);
  }

} // extern "C"
