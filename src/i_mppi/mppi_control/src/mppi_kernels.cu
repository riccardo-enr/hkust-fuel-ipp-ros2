#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <device_launch_parameters.h>

extern "C" {

struct MPPIParamsDevice {
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
    const float3* u_mean,
    const float3 u_prev,
    const float3 curr_p,
    const float3 curr_v,
    const float3 ref_p_base,
    const float3 ref_v_base,
    const float3 ref_a_base,
    const MPPIParamsDevice params,
    unsigned int seed,
    float3* samples_u, // Out: [K * H]
    float* costs       // Out: [K]
) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= params.K) return;

  curandState state;
  curand_init(seed, k, 0, &state);

  float3 p = curr_p;
  float3 v = curr_v;
  float total_cost = 0.0f;
  float3 u_prev_timestep = u_prev; // Track previous control in this rollout

  for (int h = 0; h < params.H; ++h) {
    // Reference Trajectory Prediction (Constant Acceleration Model)
    float t = (h + 1) * params.dt;
    float3 ref_p, ref_v;
    ref_p.x = ref_p_base.x + ref_v_base.x * t + 0.5f * ref_a_base.x * t * t;
    ref_p.y = ref_p_base.y + ref_v_base.y * t + 0.5f * ref_a_base.y * t * t;
    ref_p.z = ref_p_base.z + ref_v_base.z * t + 0.5f * ref_a_base.z * t * t;

    ref_v.x = ref_v_base.x + ref_a_base.x * t;
    ref_v.y = ref_v_base.y + ref_a_base.y * t;
    ref_v.z = ref_v_base.z + ref_a_base.z * t;

    // Generate noise
    float3 noise;
    noise.x = curand_normal(&state) * params.sigma;
    noise.y = curand_normal(&state) * params.sigma;
    noise.z = curand_normal(&state) * params.sigma;

    float3 u = u_mean[h];
    u.x += noise.x;
    u.y += noise.y;
    u.z += noise.z;

    // Use ref_a_base as feed-forward
    float3 total_u;
    total_u.x = u.x + ref_a_base.x;
    total_u.y = u.y + ref_a_base.y;
    total_u.z = u.z + ref_a_base.z;

    // Constraints (Simplified for GPU kernel)
    float3 total_acc = make_float3(total_u.x, total_u.y, total_u.z + params.g);
    float thrust = sqrtf(total_acc.x * total_acc.x + total_acc.y * total_acc.y + total_acc.z * total_acc.z);
    
    if (thrust > params.a_max + params.g) {
      float scale = (params.a_max + params.g) / thrust;
      total_acc.x *= scale;
      total_acc.y *= scale;
      total_acc.z *= scale;
      thrust = params.a_max + params.g;
    }

    // Tilt constraint (Angle with Z-axis)
    float cos_tilt = total_acc.z / (thrust + 1e-6f);
    if (cos_tilt < cosf(params.tilt_max)) {
        // Project onto the cone
        float s = sqrtf(total_acc.x * total_acc.x + total_acc.y * total_acc.y);
        float target_s = thrust * sinf(params.tilt_max);
        float target_z = thrust * cosf(params.tilt_max);
        if (s > 1e-6f) {
            total_acc.x *= target_s / s;
            total_acc.y *= target_s / s;
        }
        total_acc.z = target_z;
    }

    total_u.x = total_acc.x;
    total_u.y = total_acc.y;
    total_u.z = total_acc.z - params.g;

    // Control to be stored and used in cost is the delta from reference
    u.x = total_u.x - ref_a_base.x;
    u.y = total_u.y - ref_a_base.y;
    u.z = total_u.z - ref_a_base.z;

    // Store sample (the delta u)
    samples_u[k * params.H + h] = u;

    // Dynamics Propagation using RK4 with total_u
    // State x = [p, v], dot(x) = [v, a]
    float3 k1_p = v;
    float3 k1_v = total_u;

    float3 k2_p = make_float3(v.x + 0.5f * params.dt * k1_v.x, v.y + 0.5f * params.dt * k1_v.y, v.z + 0.5f * params.dt * k1_v.z);
    float3 k2_v = total_u; // Acceleration is constant over dt

    float3 k3_p = make_float3(v.x + 0.5f * params.dt * k2_v.x, v.y + 0.5f * params.dt * k2_v.y, v.z + 0.5f * params.dt * k2_v.z);
    float3 k3_v = total_u;

    float3 k4_p = make_float3(v.x + params.dt * k3_v.x, v.y + params.dt * k3_v.y, v.z + params.dt * k3_v.z);
    float3 k4_v = total_u;

    p.x += (params.dt / 6.0f) * (k1_p.x + 2.0f * k2_p.x + 2.0f * k3_p.x + k4_p.x);
    p.y += (params.dt / 6.0f) * (k1_p.y + 2.0f * k2_p.y + 2.0f * k3_p.y + k4_p.y);
    p.z += (params.dt / 6.0f) * (k1_p.z + 2.0f * k2_p.z + 2.0f * k3_p.z + k4_p.z);

    v.x += (params.dt / 6.0f) * (k1_v.x + 2.0f * k2_v.x + 2.0f * k3_v.x + k4_v.x);
    v.y += (params.dt / 6.0f) * (k1_v.y + 2.0f * k2_v.y + 2.0f * k3_v.y + k4_v.y);
    v.z += (params.dt / 6.0f) * (k1_v.z + 2.0f * k2_v.z + 2.0f * k3_v.z + k4_v.z);

    // Cost calculation
    float dp_x = p.x - ref_p.x;
    float dp_y = p.y - ref_p.y;
    float dp_z = p.z - ref_p.z;
    float dv_x = v.x - ref_v.x;
    float dv_y = v.y - ref_v.y;
    float dv_z = v.z - ref_v.z;
    float da_x = u.x - ref_a_base.x;
    float da_y = u.y - ref_a_base.y;
    float da_z = u.z - ref_a_base.z;

    total_cost += params.Q_pos_x * dp_x * dp_x;
    total_cost += params.Q_pos_y * dp_y * dp_y;
    total_cost += params.Q_pos_z * dp_z * dp_z;
    total_cost += params.Q_vel_x * dv_x * dv_x;
    total_cost += params.Q_vel_y * dv_y * dv_y;
    total_cost += params.Q_vel_z * dv_z * dv_z;
    total_cost += params.R_x * da_x * da_x;
    total_cost += params.R_y * da_y * da_y;
    total_cost += params.R_z * da_z * da_z;

    // Control rate penalty: penalize change from previous control
    float3 du_rate;
    du_rate.x = u.x - u_prev_timestep.x;
    du_rate.y = u.y - u_prev_timestep.y;
    du_rate.z = u.z - u_prev_timestep.z;
    total_cost += params.R_rate_x * du_rate.x * du_rate.x;
    total_cost += params.R_rate_y * du_rate.y * du_rate.y;
    total_cost += params.R_rate_z * du_rate.z * du_rate.z;

    // Update previous control for next timestep
    u_prev_timestep = u;
  }

  costs[k] = total_cost;
}

void launch_mppi_kernel(
    const float3* u_mean_host,
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
    float3* samples_u_host,
    float* costs_host,
    unsigned int seed
) {
  MPPIParamsDevice params = {
    K, H, dt, sigma, lambda,
    Q_pos_x, Q_pos_y, Q_pos_z,
    Q_vel_x, Q_vel_y, Q_vel_z,
    R_x, R_y, R_z,
    R_rate_x, R_rate_y, R_rate_z,
    w_obs, a_max, tilt_max, g
  };
  
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
