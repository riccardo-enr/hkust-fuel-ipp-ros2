#ifndef MPPI_CONTROL__MPPI_CONTROL_NODE_HPP_
#define MPPI_CONTROL__MPPI_CONTROL_NODE_HPP_

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
extern "C" void launch_mppi_kernel(
    const float3* u_mean_host,
    float3 curr_p, float3 curr_v,
    float3 ref_p, float3 ref_v, float3 ref_a,
    int K, int H, float dt, float sigma, float lambda,
    float Q_pos, float Q_vel, float R, float w_obs,
    float a_max, float tilt_max, float g,
    float3* samples_u_host,
    float* costs_host
);

struct MPPIParams {
  int K;         // Number of samples
  int H;         // Horizon steps
  double dt;     // Time step
  double sigma;  // Noise standard deviation
  double lambda; // Temperature parameter for weighting
  
  double Q_pos;  // Position cost weight
  double Q_vel;  // Velocity cost weight
  double R;      // Control effort weight
  double w_obs;  // Obstacle cost weight
  
  double a_max;       // Max acceleration
  double tilt_max;    // Max tilt angle in radians
  double g;           // Gravity
};

class MPPIControlNode : public rclcpp::Node {
public:
  MPPIControlNode(const rclcpp::NodeOptions& options);

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
  
  // SDFMap for obstacle costs
  std::shared_ptr<fast_planner::SDFMap> sdf_map_;

  // SO3 Output Params
  double mass_;
  double kR_[3], kOm_[3];
};

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_CONTROL_NODE_HPP_