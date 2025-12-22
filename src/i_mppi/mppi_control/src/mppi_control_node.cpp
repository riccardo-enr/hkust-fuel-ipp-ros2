#include "mppi_control/mppi_control_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <random>
#include <algorithm>
#include <ctime>

namespace mppi_control {

MPPIControlNode::MPPIControlNode(const rclcpp::NodeOptions& options)
: Node("mppi_control_node", options) {
  
  // Initialize Parameters
  params_.K = this->declare_parameter("mppi/K", 500);
  params_.H = this->declare_parameter("mppi/H", 20);
  params_.dt = this->declare_parameter("mppi/dt", 0.05);
  params_.sigma = this->declare_parameter("mppi/sigma", 1.0);
  params_.lambda = this->declare_parameter("mppi/lambda", 0.1);
  
  params_.Q_pos = this->declare_parameter("mppi/Q_pos", 10.0);
  params_.Q_vel = this->declare_parameter("mppi/Q_vel", 1.0);
  params_.R = this->declare_parameter("mppi/R", 0.1);
  params_.w_obs = this->declare_parameter("mppi/w_obs", 100.0);
  
  params_.a_max = this->declare_parameter("mppi/a_max", 10.0);
  params_.tilt_max = this->declare_parameter("mppi/tilt_max", 0.6); // ~34 deg
  params_.g = this->declare_parameter("mppi/g", 9.81);

  u_mean_.resize(params_.H, Eigen::Vector3d::Zero());

  // Publishers and Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&MPPIControlNode::odomCallback, this, std::placeholders::_1));
  
  pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
    "planning/pos_cmd", 10, std::bind(&MPPIControlNode::posCmdCallback, this, std::placeholders::_1));
    
  mppi_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
    "mppi/pos_cmd", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&MPPIControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "MPPI Control Node Initialized");
}

void MPPIControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  curr_p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  curr_v_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  odom_received_ = true;
}

void MPPIControlNode::posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
  ref_cmd_ = *msg;
  ref_received_ = true;
}

void MPPIControlNode::controlLoop() {
  if (!odom_received_ || !ref_received_) return;

  runMPPI();

  auto out_msg = ref_cmd_;
  out_msg.header.stamp = this->now();
  // Output the optimized acceleration
  out_msg.acceleration.x = u_mean_[0].x();
  out_msg.acceleration.y = u_mean_[0].y();
  out_msg.acceleration.z = u_mean_[0].z();

  mppi_cmd_pub_->publish(out_msg);

  // Shift control sequence for next iteration (Warm start)
  for (int i = 0; i < params_.H - 1; ++i) {
    u_mean_[i] = u_mean_[i + 1];
  }
  u_mean_[params_.H - 1].setZero();
}

void MPPIControlNode::runMPPI() {
  std::vector<float3> samples_u(params_.K * params_.H);
  std::vector<float> costs(params_.K);
  std::vector<float3> u_mean_f3(params_.H);

  for (int i = 0; i < params_.H; ++i) {
    u_mean_f3[i] = make_float3(u_mean_[i].x(), u_mean_[i].y(), u_mean_[i].z());
  }

  float3 cp = make_float3(curr_p_.x(), curr_p_.y(), curr_p_.z());
  float3 cv = make_float3(curr_v_.x(), curr_v_.y(), curr_v_.z());
  float3 rp = make_float3(ref_cmd_.position.x, ref_cmd_.position.y, ref_cmd_.position.z);
  float3 rv = make_float3(ref_cmd_.velocity.x, ref_cmd_.velocity.y, ref_cmd_.velocity.z);
  float3 ra = make_float3(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);

  launch_mppi_kernel(
      u_mean_f3.data(), cp, cv, rp, rv, ra,
      params_.K, params_.H, params_.dt, params_.sigma, params_.lambda,
      params_.Q_pos, params_.Q_vel, params_.R, params_.w_obs,
      params_.a_max, params_.tilt_max, params_.g,
      samples_u.data(), costs.data());

  // Weighting
  float min_cost = *std::min_element(costs.begin(), costs.end());
  double sum_weights = 0.0;
  std::vector<double> weights(params_.K);

  for (int k = 0; k < params_.K; ++k) {
    weights[k] = exp(-(costs[k] - min_cost) / params_.lambda);
    sum_weights += weights[k];
  }

  // Update mean control
  std::vector<Eigen::Vector3d> new_u_mean(params_.H, Eigen::Vector3d::Zero());
  for (int h = 0; h < params_.H; ++h) {
    for (int k = 0; k < params_.K; ++k) {
      float3 ukh = samples_u[k * params_.H + h];
      new_u_mean[h] += weights[k] * Eigen::Vector3d(ukh.x, ukh.y, ukh.z);
    }
    new_u_mean[h] /= (sum_weights + 1e-6);
  }
  u_mean_ = new_u_mean;
}

} // namespace mppi_control

RCLCPP_COMPONENTS_REGISTER_NODE(mppi_control::MPPIControlNode)