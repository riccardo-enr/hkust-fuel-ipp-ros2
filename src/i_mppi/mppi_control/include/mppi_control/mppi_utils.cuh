#pragma once

#include <cuda_runtime.h>

// Quaternion utilities for MPPI control kernels

// Helper device function to compute quaternion difference: q_diff = q_a * q_b_inv
__device__ inline float4 quat_diff(const float4 q_a, const float4 q_b)
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
__device__ inline float quat_vec_norm_sq(const float4 q)
{
  return q.x * q.x + q.y * q.y + q.z * q.z;
}
