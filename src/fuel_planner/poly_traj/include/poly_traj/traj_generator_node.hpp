#ifndef POLY_TRAJ__TRAJ_GENERATOR_NODE_HPP_
#define POLY_TRAJ__TRAJ_GENERATOR_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarmtal_msgs/msg/drone_onboard_command.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "poly_traj/polynomial_traj.h"

namespace poly_traj
{

class TrajGeneratorNode : public rclcpp::Node
{
public:
  explicit TrajGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TrajGeneratorNode() override = default;

  bool hasTrajectory() const { return traj_active_; }
  bool hasOdom() const { return have_odom_; }
  double currentTrajectoryDuration() const { return total_traj_time_; }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void generateTrajectory();
  void commandTimer();
  void displayPathWithColor(
    const std::vector<Eigen::Vector3d> & path, double resolution, const Eigen::Vector4d & color,
    int id);
  void drawState(
    const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, int id,
    const Eigen::Vector4d & color);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr state_pub_;
  rclcpp::Publisher<swarmtal_msgs::msg::DroneOnboardCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_topic_;
  std::string traj_marker_topic_;
  std::string state_marker_topic_;
  std::string command_topic_;
  std::string frame_id_;
  double command_rate_hz_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  bool have_odom_ {false};
  bool traj_active_ {false};

  fast_planner::PolynomialTraj poly_traj_;
  double total_traj_time_ {0.0};
  rclcpp::Time start_time_;
};

}  // namespace poly_traj

#endif  // POLY_TRAJ__TRAJ_GENERATOR_NODE_HPP_
