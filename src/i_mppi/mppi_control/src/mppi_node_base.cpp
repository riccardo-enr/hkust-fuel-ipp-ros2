#include "mppi_control/mppi_node_base.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mppi_control
{

MPPINodeBase::MPPINodeBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  last_control_time_ = clock_->now();
}

void MPPINodeBase::initBase()
{
  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MPPINodeBase::odomCallback, this, std::placeholders::_1));

  pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "planning/pos_cmd", 10, std::bind(&MPPINodeBase::posCmdCallback, this, std::placeholders::_1));

  // Publishers
  so3_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);
  comp_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("mppi/comp_time", 10);

  // Timer
  double control_period_s = 1.0 / common_params_.ctl_freq;
  auto control_period = std::chrono::duration<double>(control_period_s);
  auto control_period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(control_period);
  timer_ = this->create_wall_timer(
      control_period_ms, std::bind(&MPPINodeBase::controlLoop, this));
}

void MPPINodeBase::validateCommonParameters(const CommonMPPIParams &params)
{
  if (params.K <= 0) RCLCPP_ERROR(this->get_logger(), "K must be > 0");
  if (params.N <= 0.1 || params.N > 10.0) RCLCPP_ERROR(this->get_logger(), "N must be [0.1, 10.0]");
  if (params.ctl_freq < 10.0 || params.ctl_freq > 500.0) RCLCPP_ERROR(this->get_logger(), "ctl_freq must be [10, 500]");
  if (params.lambda <= 0.0) RCLCPP_ERROR(this->get_logger(), "lambda must be > 0");
  
  // Gains validation could be here too
}

void MPPINodeBase::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  curr_p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  curr_v_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

  curr_q_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_yaw_);

  odom_received_ = true;
}

void MPPINodeBase::posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  ref_cmd_ = *msg;
  ref_received_ = true;
}

void MPPINodeBase::checkTiming()
{
  rclcpp::Time current_time = clock_->now();
  double time_since_last = (current_time - last_control_time_).seconds();
  double expected_period = 1.0 / common_params_.ctl_freq;
  double tolerance = 1.5 * expected_period;

  if (time_since_last > tolerance)
  {
    consecutive_slow_cycles_++;
    if (!timing_warning_logged_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000,
                           "Control loop running slow! Actual: %.3fs, Expected: %.3fs (freq: %.1f Hz).",
                           time_since_last, expected_period, common_params_.ctl_freq);
      timing_warning_logged_ = true;
    }
  }
  else
  {
    consecutive_slow_cycles_ = 0;
    timing_warning_logged_ = false;
  }
  last_control_time_ = current_time;
}

bool MPPINodeBase::isDataReady()
{
  if (!odom_received_) return false;
  
  bool ready = ref_received_;
  
  // Failsafe check
  if (consecutive_slow_cycles_ > 10)
  {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock_, 1000,
                            "Control loop severely degraded! Switching to failsafe hover mode.");
      ready = false;
  }

  // Safety check for origin reference
  if (ready && ref_cmd_.position.x == 0 && ref_cmd_.position.y == 0 && ref_cmd_.position.z == 0)
  {
    if (curr_p_.norm() > 1.0)
    {
      ready = false;
    }
  }
  return ready;
}

void MPPINodeBase::publishSO3(const Eigen::Vector3d &force, const Eigen::Quaterniond &orientation)
{
  auto so3_cmd = std::make_shared<quadrotor_msgs::msg::SO3Command>();
  so3_cmd->header.stamp = this->now();
  so3_cmd->header.frame_id = "world";
  so3_cmd->force.x = force.x();
  so3_cmd->force.y = force.y();
  so3_cmd->force.z = force.z();
  so3_cmd->orientation.x = orientation.x();
  so3_cmd->orientation.y = orientation.y();
  so3_cmd->orientation.z = orientation.z();
  so3_cmd->orientation.w = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    so3_cmd->kr[i] = kR_[i];
    so3_cmd->kom[i] = kOm_[i];
  }
  so3_cmd->aux.current_yaw = current_yaw_;
  so3_cmd->aux.enable_motors = true;
  so3_cmd->aux.use_external_yaw = false;

  so3_cmd_pub_->publish(*so3_cmd);
}

} // namespace mppi_control
