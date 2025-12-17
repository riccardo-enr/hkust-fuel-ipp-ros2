#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <bspline/msg/bspline.hpp>
#include <bspline/non_uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/plan_container.hpp>

using fast_planner::LocalTrajData;
using fast_planner::LocalTrajServer;
using fast_planner::LocalTrajState;
using fast_planner::NonUniformBspline;

namespace fast_planner
{

class TrajServerBackupNode : public rclcpp::Node
{
public:
  explicit TrajServerBackupNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void bsplineCallback(const bspline::msg::Bspline::SharedPtr msg);
  void pauseCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void commandTimer();
  void visualizationTimer();
  void displayTrajWithColor(const std::vector<Eigen::Vector3d> & traj, double resolution,
                            const Eigen::Vector4d & color, int id);
  void drawCmd(const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, int id,
               const Eigen::Vector4d & color);

  rclcpp::Subscription<bspline::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  quadrotor_msgs::msg::PositionCommand cmd_msg_;
  std::shared_ptr<LocalTrajServer> local_traj_;
  std::vector<Eigen::Vector3d> executed_cmd_;

  int pub_traj_id_ {-1};
  std::string frame_id_ {"world"};
  std::string bspline_topic_ {"planning/bspline"};
  std::string pause_topic_ {"planning/pause"};
  std::string command_topic_ {"/position_cmd"};
  std::string cmd_vis_topic_ {"planning/position_cmd_vis"};
  std::string traj_marker_topic_ {"planning/travel_traj"};
};

TrajServerBackupNode::TrajServerBackupNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("traj_server_backup", options)
{
  pub_traj_id_ = declare_parameter<int>("traj_server.pub_traj_id", -1);
  frame_id_ = declare_parameter<std::string>("traj_server.frame_id", "world");
  bspline_topic_ = declare_parameter<std::string>("traj_server.bspline_topic", "planning/bspline");
  pause_topic_ = declare_parameter<std::string>("traj_server.pause_topic", "planning/pause");
  command_topic_ = declare_parameter<std::string>("traj_server.command_topic", "/position_cmd");
  cmd_vis_topic_ = declare_parameter<std::string>(
    "traj_server.cmd_vis_topic", "planning/position_cmd_vis");
  traj_marker_topic_ = declare_parameter<std::string>(
    "traj_server.traj_marker_topic", "planning/travel_traj");

  bspline_sub_ = create_subscription<bspline::msg::Bspline>(
    bspline_topic_, rclcpp::QoS(10),
    std::bind(&TrajServerBackupNode::bsplineCallback, this, std::placeholders::_1));
  pause_sub_ = create_subscription<std_msgs::msg::Empty>(
    pause_topic_, rclcpp::QoS(10),
    std::bind(&TrajServerBackupNode::pauseCallback, this, std::placeholders::_1));

  cmd_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>(cmd_vis_topic_, 10);
  pos_cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(command_topic_, 50);
  traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(traj_marker_topic_, 10);

  cmd_msg_.header.frame_id = frame_id_;
  cmd_msg_.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_msg_.kx = {5.7, 5.7, 6.2};
  cmd_msg_.kv = {3.4, 3.4, 4.0};

  cmd_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&TrajServerBackupNode::commandTimer, this));
  vis_timer_ = create_wall_timer(
    std::chrono::milliseconds(250), std::bind(&TrajServerBackupNode::visualizationTimer, this));

  local_traj_ = std::make_shared<LocalTrajServer>();

  RCLCPP_INFO(get_logger(), "[traj_server_backup]: ready.");
}

void TrajServerBackupNode::bsplineCallback(const bspline::msg::Bspline::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(static_cast<int>(i)) = msg->knots[i];
  }
  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(static_cast<int>(i), 0) = msg->pos_pts[i].x;
    pos_pts(static_cast<int>(i), 1) = msg->pos_pts[i].y;
    pos_pts(static_cast<int>(i), 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (size_t i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(static_cast<int>(i), 0) = msg->yaw_pts[i];
  }
  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  LocalTrajData traj;
  traj.start_time_ = rclcpp::Time(msg->start_time);
  traj.traj_id_ = msg->traj_id;
  traj.position_traj_ = pos_traj;
  traj.velocity_traj_ = traj.position_traj_.getDerivative();
  traj.acceleration_traj_ = traj.velocity_traj_.getDerivative();
  traj.yaw_traj_ = yaw_traj;
  traj.yawdot_traj_ = traj.yaw_traj_.getDerivative();
  traj.duration_ = pos_traj.getTimeSum();
  local_traj_->addTraj(traj);
}

void TrajServerBackupNode::pauseCallback(const std_msgs::msg::Empty::SharedPtr)
{
  local_traj_->resetDuration();
}

void TrajServerBackupNode::commandTimer()
{
  LocalTrajState traj_cmd;
  const rclcpp::Time time_now = now();
  const bool status = local_traj_->evaluate(time_now, traj_cmd);
  if (!status) {
    return;
  }

  cmd_msg_.header.stamp = time_now;
  cmd_msg_.trajectory_id = traj_cmd.id;
  cmd_msg_.position.x = traj_cmd.pos.x();
  cmd_msg_.position.y = traj_cmd.pos.y();
  cmd_msg_.position.z = traj_cmd.pos.z();
  cmd_msg_.velocity.x = traj_cmd.vel.x();
  cmd_msg_.velocity.y = traj_cmd.vel.y();
  cmd_msg_.velocity.z = traj_cmd.vel.z();
  cmd_msg_.acceleration.x = traj_cmd.acc.x();
  cmd_msg_.acceleration.y = traj_cmd.acc.y();
  cmd_msg_.acceleration.z = traj_cmd.acc.z();
  cmd_msg_.yaw = traj_cmd.yaw;
  cmd_msg_.yaw_dot = traj_cmd.yawdot;
  pos_cmd_pub_->publish(cmd_msg_);

  Eigen::Vector3d dir(std::cos(traj_cmd.yaw), std::sin(traj_cmd.yaw), 0.0);
  drawCmd(traj_cmd.pos, 2.0 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

  if (executed_cmd_.empty() || (traj_cmd.pos - executed_cmd_.back()).norm() > 1e-6) {
    executed_cmd_.push_back(traj_cmd.pos);
    if (executed_cmd_.size() > 10000) {
      executed_cmd_.erase(executed_cmd_.begin(), executed_cmd_.begin() + 1000);
    }
  }
}

void TrajServerBackupNode::visualizationTimer()
{
  displayTrajWithColor(executed_cmd_, 0.05, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), pub_traj_id_);
}

void TrajServerBackupNode::displayTrajWithColor(const std::vector<Eigen::Vector3d> & traj,
                                                double resolution, const Eigen::Vector4d & color,
                                                int id)
{
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  traj_pub_->publish(mk);

  if (traj.empty()) {
    return;
  }

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  for (const auto & point : traj) {
    geometry_msgs::msg::Point pt;
    pt.x = point.x();
    pt.y = point.y();
    pt.z = point.z();
    mk.points.push_back(pt);
  }
  traj_pub_->publish(mk);
}

void TrajServerBackupNode::drawCmd(const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, int id,
                                   const Eigen::Vector4d & color)
{
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = frame_id_;
  mk_state.header.stamp = now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::msg::Point start;
  start.x = pos.x();
  start.y = pos.y();
  start.z = pos.z();
  geometry_msgs::msg::Point end;
  end.x = pos.x() + vec.x();
  end.y = pos.y() + vec.y();
  end.z = pos.z() + vec.z();

  mk_state.points.push_back(start);
  mk_state.points.push_back(end);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub_->publish(mk_state);
}

}  // namespace fast_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_planner::TrajServerBackupNode>());
  rclcpp::shutdown();
  return 0;
}
