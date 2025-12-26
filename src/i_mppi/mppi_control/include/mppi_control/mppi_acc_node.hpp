#ifndef MPPI_CONTROL__MPPI_ACC_NODE_HPP_
#define MPPI_CONTROL__MPPI_ACC_NODE_HPP_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <plan_env/sdf_map.h>
#include <cuda_runtime.h>

namespace mppi_control {

// CUDA Kernel Launcher Declaration
extern "C" void launch_mppi_acc_kernel(
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
    uint32_t seed
);

struct MPPIParams {
  int K;         // Number of samples
  int H;         // Horizon steps
  double dt;     // Time step
  double sigma;  // Noise standard deviation
  double lambda; // Temperature parameter for weighting

  double Q_pos_x;  // Position cost weight (x)
  double Q_pos_y;  // Position cost weight (y)
  double Q_pos_z;  // Position cost weight (z)
  double Q_vel_x;  // Velocity cost weight (x)
  double Q_vel_y;  // Velocity cost weight (y)
  double Q_vel_z;  // Velocity cost weight (z)
  double R_x;      // Control effort weight (x)
  double R_y;      // Control effort weight (y)
  double R_z;      // Control effort weight (z)
  double R_rate_x; // Control rate change weight (x)
  double R_rate_y; // Control rate change weight (y)
  double R_rate_z; // Control rate change weight (z)
  double w_obs;    // Obstacle cost weight

  double a_max;       // Max acceleration
  double tilt_max;    // Max tilt angle in radians
  double g;           // Gravity
};

class MPPIAccNode : public rclcpp::Node {
public:
  MPPIAccNode(const rclcpp::NodeOptions& options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void controlLoop();

  // MPPI Core Functions
  void runMPPI();

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State and Data
  Eigen::Vector3d curr_p_, curr_v_;
  double current_yaw_;
  quadrotor_msgs::msg::PositionCommand ref_cmd_;
  bool odom_received_{false};
  bool ref_received_{false};

  MPPIParams params_;
  std::vector<Eigen::Vector3d> u_mean_; // Mean control sequence
  Eigen::Vector3d u_prev_;              // Previous control command (for rate penalty)
  uint32_t seed_{0};
  
  // SDFMap for obstacle costs
  std::shared_ptr<fast_planner::SDFMap> sdf_map_;

  // SO3 Output Params
  double mass_;
  double kR_[3], kOm_[3];
};

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_ACC_NODE_HPP_