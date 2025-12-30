#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <device_launch_parameters.h>

#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mppi_control/mppi_utils.cuh"

extern "C"
{

  struct MPPIParamsDevice
  {
    int K;
    int H;
    float dt;
    float sigma;
    float lambda;
    float Q_pos_x;
    float Q_pos_y;
    float Q_pos_z;
    float Q_vel_x;
    float Q_vel_y;
    float Q_vel_z;
    float R_x;
    float R_y;
    float R_z;
    float R_rate_x;
    float R_rate_y;
    float R_rate_z;
    float w_obs;
    float a_max;
    float tilt_max;
    float g;
  };

  __global__ void mppi_kernel(
      const float3 *u_mean,
      const float3 u_prev,
      const float3 curr_p,
      const float3 curr_v,
      const float3 ref_p_base,
      const float3 ref_v_base,
      const float3 ref_a_base,
      const MPPIParamsDevice params,
      unsigned int seed,
      float3 *samples_u, // Out: [K * H]
      float *costs       // Out: [K]
  )
  {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= params.K)
      return;

    curandState state;
    curand_init(seed, k, 0, &state);

    Eigen::Vector3f p(curr_p.x, curr_p.y, curr_p.z);
    Eigen::Vector3f v(curr_v.x, curr_v.y, curr_v.z);
    float total_cost = 0.0f;
    Eigen::Vector3f u_prev_timestep(u_prev.x, u_prev.y, u_prev.z); // Track previous control in this rollout

    Eigen::Vector3f ref_p_base_v(ref_p_base.x, ref_p_base.y, ref_p_base.z);
    Eigen::Vector3f ref_v_base_v(ref_v_base.x, ref_v_base.y, ref_v_base.z);
    Eigen::Vector3f ref_a_base_v(ref_a_base.x, ref_a_base.y, ref_a_base.z);

    for (int h = 0; h < params.H; ++h)
    {
      // Reference Trajectory Prediction (Constant Acceleration Model)
      float t = (h + 1) * params.dt;
      Eigen::Vector3f ref_p = ref_p_base_v + ref_v_base_v * t + 0.5f * ref_a_base_v * t * t;
      Eigen::Vector3f ref_v = ref_v_base_v + ref_a_base_v * t;

      // Generate noise
      Eigen::Vector3f noise;
      noise.x() = curand_normal(&state) * params.sigma;
      noise.y() = curand_normal(&state) * params.sigma;
      noise.z() = curand_normal(&state) * params.sigma;

      Eigen::Vector3f u(u_mean[h].x, u_mean[h].y, u_mean[h].z);
      u += noise;

      // Constraints (Simplified for GPU kernel)
      Eigen::Vector3f total_acc = u + Eigen::Vector3f(0.0f, 0.0f, params.g);
      float thrust = total_acc.norm();

      if (thrust > params.a_max + params.g)
      {
        total_acc *= (params.a_max + params.g) / thrust;
        thrust = params.a_max + params.g;
      }

      // Tilt constraint (Angle with Z-axis)
      float cos_tilt = total_acc.z() / (thrust + 1e-6f);
      if (cos_tilt < cosf(params.tilt_max))
      {
        // Project onto the cone
        float s = total_acc.head<2>().norm();
        float target_s = thrust * sinf(params.tilt_max);
        float target_z = thrust * cosf(params.tilt_max);
        if (s > 1e-6f)
        {
          total_acc.head<2>() *= target_s / s;
        }
        total_acc.z() = target_z;
      }

      // Constrained acceleration (total_u is acceleration in world frame minus gravity)
      Eigen::Vector3f total_u = total_acc - Eigen::Vector3f(0.0f, 0.0f, params.g);

      // Store sample (the total u)
      u = total_u;
      samples_u[k * params.H + h] = make_float3(u.x(), u.y(), u.z());

      // Dynamics Propagation using RK4 with total_u
      // State x = [p, v], dot(x) = [v, a]
      Eigen::Vector3f k1_p = v;
      Eigen::Vector3f k1_v = total_u;

      Eigen::Vector3f k2_p = v + 0.5f * params.dt * k1_v;
      Eigen::Vector3f k2_v = total_u; // Acceleration is constant over dt

      Eigen::Vector3f k3_p = v + 0.5f * params.dt * k2_v;
      Eigen::Vector3f k3_v = total_u;

      Eigen::Vector3f k4_p = v + params.dt * k3_v;
      Eigen::Vector3f k4_v = total_u;

      p += (params.dt / 6.0f) * (k1_p + 2.0f * k2_p + 2.0f * k3_p + k4_p);
      v += (params.dt / 6.0f) * (k1_v + 2.0f * k2_v + 2.0f * k3_v + k4_v);

      // Cost calculation
      Eigen::Vector3f dp = p - ref_p;
      Eigen::Vector3f dv = v - ref_v;
      Eigen::Vector3f da = u;

      total_cost += params.Q_pos_x * dp.x() * dp.x();
      total_cost += params.Q_pos_y * dp.y() * dp.y();
      total_cost += params.Q_pos_z * dp.z() * dp.z();
      total_cost += params.Q_vel_x * dv.x() * dv.x();
      total_cost += params.Q_vel_y * dv.y() * dv.y();
      total_cost += params.Q_vel_z * dv.z() * dv.z();
      total_cost += params.R_x * da.x() * da.x();
      total_cost += params.R_y * da.y() * da.y();
      total_cost += params.R_z * da.z() * da.z();

      // Control rate penalty: penalize change from previous control
      Eigen::Vector3f du_rate = u - u_prev_timestep;
      total_cost += params.R_rate_x * du_rate.x() * du_rate.x();
      total_cost += params.R_rate_y * du_rate.y() * du_rate.y();
      total_cost += params.R_rate_z * du_rate.z() * du_rate.z();

      // Update previous control for next timestep
      u_prev_timestep = u;
    }

    costs[k] = total_cost;
  }

  void launch_mppi_acc_kernel(
      const float3 *u_mean_host,
      float3 u_prev,
      float3 curr_p, float3 curr_v,
      float3 ref_p, float3 ref_v, float3 ref_a,
      int K, int H, float dt, float sigma, float lambda,
      float Q_pos_x, float Q_pos_y, float Q_pos_z,
      float Q_vel_x, float Q_vel_y, float Q_vel_z,
      float R_x, float R_y, float R_z,
      float R_rate_x, float R_rate_y, float R_rate_z,
      float w_obs,
      float a_max, float tilt_max, float g,
      float3 *samples_u_host,
      float *costs_host,
      unsigned int seed)
  {
    MPPIParamsDevice params = {
        K, H, dt, sigma, lambda,
        Q_pos_x, Q_pos_y, Q_pos_z,
        Q_vel_x, Q_vel_y, Q_vel_z,
        R_x, R_y, R_z,
        R_rate_x, R_rate_y, R_rate_z,
        w_obs, a_max, tilt_max, g};

    float3 *d_u_mean, *d_samples_u;
    float *d_costs;

    cudaMalloc(&d_u_mean, H * sizeof(float3));
    cudaMalloc(&d_samples_u, K * H * sizeof(float3));
    cudaMalloc(&d_costs, K * sizeof(float));

    cudaMemcpy(d_u_mean, u_mean_host, H * sizeof(float3), cudaMemcpyHostToDevice);

    int threadsPerBlock = 256;
    int blocksPerGrid = (K + threadsPerBlock - 1) / threadsPerBlock;

    mppi_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_u_mean, u_prev, curr_p, curr_v, ref_p, ref_v, ref_a, params, seed, d_samples_u, d_costs);

    cudaMemcpy(samples_u_host, d_samples_u, K * H * sizeof(float3), cudaMemcpyDeviceToHost);
    cudaMemcpy(costs_host, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_u_mean);
    cudaFree(d_samples_u);
    cudaFree(d_costs);
  }

} // extern "C"
