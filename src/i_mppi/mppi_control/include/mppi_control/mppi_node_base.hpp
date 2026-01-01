#ifndef MPPI_CONTROL__MPPI_NODE_BASE_HPP_
#define MPPI_CONTROL__MPPI_NODE_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <std_msgs/msg/float32.hpp>
#include <plan_env/sdf_map.h>
#include <Eigen/Eigen>
#include <vector>
#include "mppi_control/utils.hpp"
#include "mppi_control/mppi_common.hpp"

namespace mppi_control
{

struct CommonMPPIParams
{
  int K;            // Number of samples
  int H;            // Horizon steps
  double N;         // Horizon time (s)
  double ctl_freq;  // Control frequency (Hz)
  double dt;        // Time step (s)
  double lambda;    // Temperature
  double w_obs;     // Obstacle weight
  double g;         // Gravity

  bool enable_lpf;
  double lpf_cutoff;
  double lpf_alpha;
};

class MPPINodeBase : public rclcpp::Node
{
public:
  MPPINodeBase(const std::string &node_name, const rclcpp::NodeOptions &options);
  virtual ~MPPINodeBase() = default;

protected:
  // Initialization
  void initBase();
  void validateCommonParameters(const CommonMPPIParams &params);

  // Callbacks
  virtual void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  virtual void posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  virtual void controlLoop() = 0; // Pure virtual

  // Helper methods
  bool isDataReady();
  void publishSO3(const Eigen::Vector3d &force, const Eigen::Quaterniond &orientation);
  void checkTiming();

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr comp_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr objective_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lambda_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  Eigen::Vector3d curr_p_, curr_v_;
  Eigen::Quaterniond curr_q_;
  double current_yaw_;
  quadrotor_msgs::msg::PositionCommand ref_cmd_;
  bool odom_received_{false};
  bool ref_received_{false};

  // SDF Map
  std::shared_ptr<fast_planner::SDFMap> sdf_map_;

  // Timing / Failsafe
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_control_time_;
  bool timing_warning_logged_{false};
  int consecutive_slow_cycles_{0};

  // Common Parameters
  CommonMPPIParams common_params_;
  LambdaAutoTuneParams lambda_params_;
  double mass_;
  double kR_[3], kOm_[3];

  // Helper to init map lazily
  void ensureSDFMap() {
    if (!sdf_map_) {
      sdf_map_.reset(new fast_planner::SDFMap);
      sdf_map_->initMap(this->shared_from_this());
    }
  }
};

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_NODE_BASE_HPP_