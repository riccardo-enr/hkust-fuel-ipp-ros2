#ifndef MPPI_CONTROL__MPPI_TQ_NODE_HPP_
#define MPPI_CONTROL__MPPI_TQ_NODE_HPP_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <plan_env/sdf_map.h>
#include <cuda_runtime.h>

namespace mppi_control {

struct ControlInput {
  float thrust;
  float4 quat;
};

struct ControlSample {
  float thrust;
  float4 quat;
};

// CUDA Kernel Launcher Declaration
extern "C" void launch_mppi_tq_kernel(
    const ControlInput* u_mean_host,
    ControlInput u_prev,
    float3 curr_p, float3 curr_v,
    float3 ref_p, float3 ref_v, float3 ref_a,
    float4 ref_quat, float ref_thrust,
    int K, int H, float dt, float lambda,
    float sigma_thrust, float sigma_quat,
    float Q_pos_x, float Q_pos_y, float Q_pos_z,
    float Q_vel_x, float Q_vel_y, float Q_vel_z,
    float Q_thrust, float R_thrust, float R_rate_thrust,
    float Q_quat, float R_quat, float R_rate_quat,
    float Q_omega,
    float w_obs,
    float thrust_max, float thrust_min, float g,
    ControlSample* samples_u_host,
    float* costs_host,
    unsigned int seed
);

struct MPPITqParams {
  int K;
  int H;
  double dt;
  double lambda;

  double sigma_thrust;
  double sigma_quat;

  double Q_pos_x, Q_pos_y, Q_pos_z;
  double Q_vel_x, Q_vel_y, Q_vel_z;
  double Q_thrust;
  double R_thrust;
  double R_rate_thrust;
  double Q_quat;
  double R_quat;
  double R_rate_quat;
  double Q_omega;
  double w_obs;

  double thrust_max;
  double thrust_min;
  double g;
};

class MPPITqNode : public rclcpp::Node {
public:
  MPPITqNode(const rclcpp::NodeOptions& options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void controlLoop();
  void runMPPI();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::Vector3d curr_p_, curr_v_;
  Eigen::Quaterniond curr_q_;
  double current_yaw_;
  quadrotor_msgs::msg::PositionCommand ref_cmd_;
  bool odom_received_{false};
  bool ref_received_{false};

  MPPITqParams params_;
  std::vector<ControlInput> u_mean_; 
  ControlInput u_prev_;
  uint32_t seed_{0};
  
  std::shared_ptr<fast_planner::SDFMap> sdf_map_;

  double mass_;
  double kR_[3], kOm_[3];
};

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_TQ_NODE_HPP_