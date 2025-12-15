#include <algorithm>
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
  explicit TrajGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("traj_generator", options)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/uwb_vicon_odom");
    traj_marker_topic_ = declare_parameter<std::string>("traj_marker_topic", "/traj_generator/traj_vis");
    state_marker_topic_ = declare_parameter<std::string>("state_marker_topic", "/traj_generator/cmd_vis");
    command_topic_ = declare_parameter<std::string>("command_topic", "/drone_commander/onboard_command");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 100.0);

    using std::placeholders::_1;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(), std::bind(&TrajGeneratorNode::odomCallback, this, _1));

    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(traj_marker_topic_, 10);
    state_pub_ = create_publisher<visualization_msgs::msg::Marker>(state_marker_topic_, 10);
    cmd_pub_ = create_publisher<swarmtal_msgs::msg::DroneOnboardCommand>(command_topic_, 10);

    auto period = std::chrono::duration<double>(1.0 / std::max(command_rate_hz_, 1.0));
    timer_ = create_wall_timer(period, std::bind(&TrajGeneratorNode::commandTimer, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->child_frame_id == "X" || msg->child_frame_id == "O") {
      return;
    }

    latest_odom_ = msg;
    if (!have_odom_) {
      have_odom_ = true;
      generateTrajectory();
    }
  }

  void generateTrajectory()
  {
    if (!have_odom_ || !latest_odom_) {
      return;
    }

    Eigen::MatrixXd positions(9, 3);
    positions.row(0) = Eigen::Vector3d(
      latest_odom_->pose.pose.position.x,
      latest_odom_->pose.pose.position.y,
      latest_odom_->pose.pose.position.z);
    positions.row(1) = Eigen::Vector3d(-0.5, 0.5, 1.0);
    positions.row(2) = Eigen::Vector3d(0.0, 0.0, 1.0);
    positions.row(3) = Eigen::Vector3d(0.5, -0.5, 1.0);
    positions.row(4) = Eigen::Vector3d(1.0, 0.0, 1.0);
    positions.row(5) = Eigen::Vector3d(0.5, 0.5, 1.0);
    positions.row(6) = Eigen::Vector3d(0.0, 0.0, 1.0);
    positions.row(7) = Eigen::Vector3d(-0.5, -0.5, 1.0);
    positions.row(8) = Eigen::Vector3d(-1.0, 0.0, 1.0);

    Eigen::VectorXd segment_times(8);
    segment_times << 2.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.0;

    Eigen::Vector3d start_vel(
      latest_odom_->twist.twist.linear.x,
      latest_odom_->twist.twist.linear.y,
      latest_odom_->twist.twist.linear.z);
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    fast_planner::PolynomialTraj::waypointsTraj(
      positions, start_vel, zero, zero, zero, segment_times, poly_traj_);
    total_traj_time_ = poly_traj_.getTotalTime();

    std::vector<Eigen::Vector3d> samples;
    poly_traj_.getSamplePoints(samples);
    displayPathWithColor(samples, 0.05, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 1);

    start_time_ = now();
    traj_active_ = true;
    RCLCPP_INFO(get_logger(), "Trajectory initialized with %.2f s duration", total_traj_time_);
  }

  void commandTimer()
  {
    if (!traj_active_) {
      return;
    }

    const auto current_time = now();
    const double elapsed = (current_time - start_time_).seconds();
    if (elapsed > total_traj_time_) {
      traj_active_ = false;
      RCLCPP_INFO(get_logger(), "Trajectory execution finished");
      return;
    }

    Eigen::Vector3d pos = poly_traj_.evaluate(elapsed, 0);
    Eigen::Vector3d vel = poly_traj_.evaluate(elapsed, 1);
    Eigen::Vector3d acc = poly_traj_.evaluate(elapsed, 2);

    swarmtal_msgs::msg::DroneOnboardCommand cmd;
    cmd.command_type = swarmtal_msgs::msg::DroneOnboardCommand::CTRL_POS_COMMAND;
    cmd.param1 = static_cast<int32_t>(pos.x() * 10000.0);
    cmd.param2 = static_cast<int32_t>(pos.y() * 10000.0);
    cmd.param3 = static_cast<int32_t>(pos.z() * 10000.0);
    cmd.param4 = 666666;
    cmd.param5 = static_cast<int32_t>(vel.x() * 10000.0);
    cmd.param6 = static_cast<int32_t>(vel.y() * 10000.0);
    cmd.param7 = static_cast<int32_t>(vel.z() * 10000.0);
    cmd.param8 = static_cast<int32_t>(acc.x() * 10000.0);
    cmd.param9 = static_cast<int32_t>(acc.y() * 10000.0);
    cmd.param10 = static_cast<int32_t>(acc.z() * 10000.0);

    cmd_pub_->publish(cmd);

    drawState(pos, vel, 0, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0));
    drawState(pos, acc, 1, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0));
  }

  void displayPathWithColor(
    const std::vector<Eigen::Vector3d> & path, double resolution, const Eigen::Vector4d & color,
    int id)
  {
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id_;
    clear.header.stamp = now();
    clear.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    clear.action = visualization_msgs::msg::Marker::DELETE;
    clear.id = id;
    traj_pub_->publish(clear);

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = now();
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.id = id;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    for (const auto & pt_eig : path) {
      geometry_msgs::msg::Point pt;
      pt.x = pt_eig.x();
      pt.y = pt_eig.y();
      pt.z = pt_eig.z();
      mk.points.push_back(pt);
    }
    traj_pub_->publish(mk);
  }

  void drawState(
    const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, int id,
    const Eigen::Vector4d & color)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = now();
    mk.type = visualization_msgs::msg::Marker::ARROW;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.id = id;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.1;
    mk.scale.y = 0.2;
    mk.scale.z = 0.3;

    geometry_msgs::msg::Point start;
    start.x = pos.x();
    start.y = pos.y();
    start.z = pos.z();
    geometry_msgs::msg::Point end;
    end.x = pos.x() + vec.x();
    end.y = pos.y() + vec.y();
    end.z = pos.z() + vec.z();

    mk.points.push_back(start);
    mk.points.push_back(end);

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    state_pub_->publish(mk);
  }

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<poly_traj::TrajGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
