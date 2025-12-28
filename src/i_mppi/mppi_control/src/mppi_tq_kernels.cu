#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <device_launch_parameters.h>

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
    float Q_thrust;
    float R_thrust;
    float R_rate_thrust;
    float Q_quat;
    float R_quat;
    float R_rate_quat;
    float w_obs;
    float thrust_max;
    float thrust_min;
    float g;
  };

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

  // Helper device function to compute quaternion difference: q_diff = q_a * q_b_inv
  __device__ float4 quat_diff(const float4 q_a, const float4 q_b)
  {
    // Compute inverse of q_b (for unit quaternion, inverse is conjugate)
    float4 q_b_inv = make_float4(-q_b.x, -q_b.y, -q_b.z, q_b.w);

    // Compute q_a * q_b_inv
    float4 result;
    result.x = q_a.w * q_b_inv.x + q_a.x * q_b_inv.w + q_a.y * q_b_inv.z - q_a.z * q_b_inv.y;
    result.y = q_a.w * q_b_inv.y - q_a.x * q_b_inv.z + q_a.y * q_b_inv.w + q_a.z * q_b_inv.x;
    result.z = q_a.w * q_b_inv.z + q_a.x * q_b_inv.y - q_a.y * q_b_inv.x + q_a.z * q_b_inv.w;
    result.w = q_a.w * q_b_inv.w - q_a.x * q_b_inv.x - q_a.y * q_b_inv.y - q_a.z * q_b_inv.z;

    return result;
  }

  // Helper device function to compute squared magnitude of quaternion vector part
  __device__ float quat_vec_norm_sq(const float4 q)
  {
    return q.x * q.x + q.y * q.y + q.z * q.z;
  }

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
      // Reference Trajectory Prediction (Constant Acceleration Model for position/velocity)
      // Note: For full tracking, we might want ref_p/v/a/q/thrust at each step,
      // but here we simplify using base + time evolution or just constant reference for some parts.
      // For simplicity, let's assume reference is moving as predicted by base state (like in Acc kernel)
      float t = (h + 1) * params.dt;
      float3 ref_p, ref_v;
      ref_p.x = ref_p_base.x + ref_v_base.x * t + 0.5f * ref_a_base.x * t * t;
      ref_p.y = ref_p_base.y + ref_v_base.y * t + 0.5f * ref_a_base.y * t * t;
      ref_p.z = ref_p_base.z + ref_v_base.z * t + 0.5f * ref_a_base.z * t * t;

      ref_v.x = ref_v_base.x + ref_a_base.x * t;
      ref_v.y = ref_v_base.y + ref_a_base.y * t;
      ref_v.z = ref_v_base.z + ref_a_base.z * t;

      // Assume reference orientation and thrust are constant or updated externally.
      // Ideally, they should be arrays passed in if they change over H.
      // Using base values for now (approximation).
      float4 ref_quat = ref_quat_base;
      float ref_thrust = ref_thrust_base;

      // --- Noise Generation ---

      // Thrust noise (1D)
      float thrust_noise = curand_normal(&state) * params.sigma_thrust;

      // Quaternion noise (3D tangent space)
      float3 quat_noise;
      quat_noise.x = curand_normal(&state) * params.sigma_quat;
      quat_noise.y = curand_normal(&state) * params.sigma_quat;
      quat_noise.z = curand_normal(&state) * params.sigma_quat;

      // Map tangent space noise to quaternion using exponential map
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

      // Compose with mean quaternion: q_sample = q_delta * q_mean
      // Note: Order of multiplication matters. Local perturbation: q_new = q_mean * q_delta (body frame) or q_delta * q_mean (world frame)?
      // Usually noise is applied in body frame or tangent space. Let's assume tangent space at identity, rotated by mean.
      // If q_mean maps body to world, and we want to perturb in body frame: q_sample = q_mean * q_delta.
      // If we want to perturb in world frame: q_sample = q_delta * q_mean.
      // Standard MPPI often adds noise directly to control. For quaternions, it's composition.
      // Let's use q_sample = q_mean * q_delta (Body frame perturbation is standard for aircraft)

      float4 q_mean = u_mean[h].quat;
      float4 q_sample;
      // q_mean * q_delta
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
      // F_world = R * [0, 0, thrust]
      // Rotate vector [0, 0, thrust] by q_sample
      // v' = q * v * q_inv
      // For v = [0, 0, T], this simplifies.
      // Fx = 2(xz - yw)T
      // Fy = 2(yz + xw)T
      // Fz = (1 - 2(x^2 + y^2))T

      // float3 F_body = make_float3(0, 0, thrust);
      // Standard rotation formula R(q) * v:
      // x = (1 - 2y^2 - 2z^2)vx + (2xy - 2zw)vy + (2xz + 2yw)vz
      // y = (2xy + 2zw)vx + (1 - 2x^2 - 2z^2)vy + (2yz - 2xw)vz
      // z = (2xz - 2yw)vx + (2yz + 2xw)vy + (1 - 2x^2 - 2y^2)vz

      // With vx=0, vy=0, vz=thrust:
      float3 F_world;
      F_world.x = (2.0f * q_sample.x * q_sample.z + 2.0f * q_sample.y * q_sample.w) * thrust;
      F_world.y = (2.0f * q_sample.y * q_sample.z - 2.0f * q_sample.x * q_sample.w) * thrust;
      F_world.z = (1.0f - 2.0f * q_sample.x * q_sample.x - 2.0f * q_sample.y * q_sample.y) * thrust;

      // Acceleration = F/m - g
      // Assuming mass is handled such that 'thrust' is force.
      // Wait, the plan says: F_world = R * [0, 0, thrust] - mass * g.
      // But `thrust` in control usually means force magnitude. Acceleration = Force / mass.
      // We can optimize 'thrust_acc' (acceleration magnitude) or 'thrust_force'.
      // The plan says: "F_world = R * [0, 0, thrust] - mass * g".
      // This implies 'thrust' is Force Magnitude.
      // So acc = F_world / mass. BUT, mass is a parameter.
      // The kernel doesn't have mass passed in.
      // However, acc version optimized `u` which was acceleration.
      // If we want to be consistent, `thrust` here should be "Acceleration Magnitude along Z-axis".
      // Or we assume `thrust` is force and we divide by mass.
      // Let's check `params`. No mass in params.
      // Let's assume `thrust` is "Thrust Acceleration" (F/m) for simplicity in kernel,
      // or passing mass is needed.
      // In `mppi_control_node.cpp`, `force = mass_ * (des_acc + g)`.
      // The kernel in `mppi_kernels.cu` uses `total_acc = ... + g`.
      // Let's assume `thrust` here represents the thrust ACCELERATION (T/m).
      // The user plan says "Thrust magnitude (1 DOF) ... F_world = R * [0, 0, thrust] - mass * g".
      // This is slightly ambiguous on whether `thrust` is force or acc.
      // But since we add `- mass * g` to get force, wait. Gravity is usually `mg`.
      // Net force = Thrust_vec + Gravity_vec.
      // F_net = F_thrust - m*g*z_hat.
      // a = F_net / m = (F_thrust / m) - g*z_hat.
      // So if `thrust` variable is `|F_thrust| / m`, then:
      // a = R * [0, 0, thrust] - [0, 0, g].

      float3 acc;
      acc.x = F_world.x; // Here F_world is actually Thrust_acc_vector
      acc.y = F_world.y;
      acc.z = F_world.z - params.g;

      // RK4 Integration
      // State x = [p, v]
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

      // Thrust regularization
      float dthrust = thrust - ref_thrust;
      total_cost += params.Q_thrust * dthrust * dthrust; // Deviation from reference
      // Optional: Minimize absolute thrust? usually we want to stay near hover or ref.

      // Quaternion tracking cost
      // Distance = 1 - (q . q_ref)^2 (standard mapping distance)
      // or 2*acos(|q . q_ref|)
      // Let's use 1 - |q . q_ref| which is approx theta^2/8 for small angles.
      // Or just 1 - (q.q_ref)^2 for antipodal symmetry.
      float dot = q_sample.x * ref_quat.x + q_sample.y * ref_quat.y + q_sample.z * ref_quat.z + q_sample.w * ref_quat.w;
      float dist = 1.0f - dot * dot;
      total_cost += params.Q_quat * dist;

      // Thrust rate penalty: penalize change from previous thrust
      float d_thrust_rate = thrust - prev_thrust;
      total_cost += params.R_rate_thrust * d_thrust_rate * d_thrust_rate;

      // Quaternion rate penalty: penalize change from previous quaternion
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
      float Q_thrust, float R_thrust, float R_rate_thrust,
      float Q_quat, float R_quat, float R_rate_quat,
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
        Q_thrust, R_thrust, R_rate_thrust,
        Q_quat, R_quat, R_rate_quat,
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
