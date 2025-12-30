#ifndef MPPI_CONTROL__UTILS_HPP_
#define MPPI_CONTROL__UTILS_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>
#include <cuda_runtime.h>

namespace mppi_control
{

/**
 * @brief Publishes the elapsed time since start_time to the provided publisher.
 * 
 * @param pub Publisher for Float32 messages.
 * @param start_time The start time of the operation.
 */
inline void publish_execution_time(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &pub,
    const std::chrono::high_resolution_clock::time_point &start_time)
{
  if (!pub)
  {
    return;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> duration = end_time - start_time;
  
  std_msgs::msg::Float32 time_msg;
  time_msg.data = duration.count();
  pub->publish(time_msg);
}

// ==========================================
// Eigen <-> CUDA Type Conversion Helpers
// ==========================================

inline float3 toFloat3(const Eigen::Vector3d& vec)
{
  return make_float3(static_cast<float>(vec.x()), 
                     static_cast<float>(vec.y()), 
                     static_cast<float>(vec.z()));
}

inline Eigen::Vector3d toEigen(const float3& vec)
{
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

inline float4 toFloat4(const Eigen::Quaterniond& q)
{
  return make_float4(static_cast<float>(q.x()), 
                     static_cast<float>(q.y()), 
                     static_cast<float>(q.z()), 
                     static_cast<float>(q.w()));
}

inline Eigen::Quaterniond toEigen(const float4& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

} // namespace mppi_control

#endif // MPPI_CONTROL__UTILS_HPP_
