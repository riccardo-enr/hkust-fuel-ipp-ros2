#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <active_perception/perception_utils.h>
#include <bspline/msg/bspline.hpp>
#include <bspline/non_uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/backward.hpp>

namespace backward {
backward::SignalHandling sh;
}

using fast_planner::NonUniformBspline;
using fast_planner::PerceptionUtils;

namespace fast_planner {

class TrajServerNode : public rclcpp::Node {
public:
  explicit TrajServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void postInit(const rclcpp::Node::SharedPtr & node_handle);

private:
  void bsplineCallback(const bspline::msg::Bspline::SharedPtr msg);
  void replanCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void newCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pgTVioCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void commandTimer();
  void visualizationTimer();

  double calcPathLength(const std::vector<Eigen::Vector3d> & path) const;
  void displayTrajWithColor(const std::vector<Eigen::Vector3d> & path, double resolution,
                            const Eigen::Vector4d & color, int id);
  void drawFOV(const std::vector<Eigen::Vector3d> & list1, const std::vector<Eigen::Vector3d> & list2);
  void drawCmd(const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, int id,
               const Eigen::Vector4d & color);
  void publishInitializationMotion();

  rclcpp::Subscription<bspline::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr new_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr loop_pose_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  quadrotor_msgs::msg::PositionCommand cmd_msg_;
  std::vector<NonUniformBspline> traj_;
  double traj_duration_ {0.0};
  rclcpp::Time traj_start_time_;
  int traj_id_ {0};
  int pub_traj_id_ {-1};

  std::shared_ptr<PerceptionUtils> percep_utils_;
  bool percep_ready_ {false};

  bool receive_traj_ {false};
  double replan_time_ {0.1};

  std::vector<Eigen::Vector3d> traj_cmd_;
  std::vector<Eigen::Vector3d> traj_real_;

  rclcpp::Time stats_start_time_;
  rclcpp::Time stats_end_time_;
  rclcpp::Time stats_last_time_;
  bool stats_started_ {false};
  bool have_last_sample_ {false};
  double energy_ {0.0};

  Eigen::Matrix3d R_loop_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_loop_ = Eigen::Vector3d::Zero();
  bool is_loop_correction_ {false};

  Eigen::Vector3d init_pos_ = Eigen::Vector3d::Zero();
  std::string frame_id_ {"world"};
  std::string bspline_topic_ {"planning/bspline"};
  std::string replan_topic_ {"planning/replan"};
  std::string new_topic_ {"planning/new"};
  std::string odom_topic_ {"/odom_world"};
  std::string loop_pose_topic_ {"/loop_fusion/pg_T_vio"};
  std::string command_topic_ {"/position_cmd"};
  std::string cmd_vis_topic_ {"planning/position_cmd_vis"};
  std::string traj_marker_topic_ {"planning/travel_traj"};
  double command_rate_hz_ {100.0};
};

TrajServerNode::TrajServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("traj_server", options)
{
  using std::placeholders::_1;

  pub_traj_id_ = declare_parameter<int>("traj_server.pub_traj_id", -1);
  replan_time_ = declare_parameter<double>("fsm.replan_time", 0.1);
  is_loop_correction_ = declare_parameter<bool>("loop_correction.isLoopCorrection", false);
  frame_id_ = declare_parameter<std::string>("traj_server.frame_id", "world");
  command_rate_hz_ = declare_parameter<double>("traj_server.command_rate_hz", 100.0);
  bspline_topic_ = declare_parameter<std::string>("traj_server.bspline_topic", "planning/bspline");
  replan_topic_ = declare_parameter<std::string>("traj_server.replan_topic", "planning/replan");
  new_topic_ = declare_parameter<std::string>("traj_server.new_topic", "planning/new");
  odom_topic_ = declare_parameter<std::string>("traj_server.odom_topic", "/odom_world");
  loop_pose_topic_ = declare_parameter<std::string>(
    "traj_server.loop_pose_topic", "/loop_fusion/pg_T_vio");
  command_topic_ = declare_parameter<std::string>("traj_server.command_topic", "/position_cmd");
  cmd_vis_topic_ = declare_parameter<std::string>(
    "traj_server.cmd_vis_topic", "planning/position_cmd_vis");
  traj_marker_topic_ = declare_parameter<std::string>(
    "traj_server.traj_marker_topic", "planning/travel_traj");
  init_pos_.x() = declare_parameter<double>("traj_server.init_x", 0.0);
  init_pos_.y() = declare_parameter<double>("traj_server.init_y", 0.0);
  init_pos_.z() = declare_parameter<double>("traj_server.init_z", 0.0);

  bspline_sub_ = create_subscription<bspline::msg::Bspline>(
    bspline_topic_, rclcpp::QoS(10),
    std::bind(&TrajServerNode::bsplineCallback, this, _1));
  replan_sub_ = create_subscription<std_msgs::msg::Empty>(
    replan_topic_, rclcpp::QoS(10),
    std::bind(&TrajServerNode::replanCallback, this, _1));
  new_sub_ = create_subscription<std_msgs::msg::Empty>(
    new_topic_, rclcpp::QoS(10),
    std::bind(&TrajServerNode::newCallback, this, _1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TrajServerNode::odomCallback, this, _1));
  loop_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
    loop_pose_topic_, rclcpp::QoS(10),
    std::bind(&TrajServerNode::pgTVioCallback, this, _1));

  cmd_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>(cmd_vis_topic_, 10);
  pos_cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(command_topic_, 50);
  traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(traj_marker_topic_, 10);

  auto cmd_period = std::chrono::duration<double>(1.0 / std::max(command_rate_hz_, 1.0));
  cmd_timer_ = create_wall_timer(cmd_period, std::bind(&TrajServerNode::commandTimer, this));
  vis_timer_ = create_wall_timer(
    std::chrono::milliseconds(250),
    std::bind(&TrajServerNode::visualizationTimer, this));

  cmd_msg_.header.frame_id = frame_id_;
  cmd_msg_.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_msg_.trajectory_id = traj_id_;
  cmd_msg_.position.x = init_pos_.x();
  cmd_msg_.position.y = init_pos_.y();
  cmd_msg_.position.z = init_pos_.z();
  cmd_msg_.velocity.x = 0.0;
  cmd_msg_.velocity.y = 0.0;
  cmd_msg_.velocity.z = 0.0;
  cmd_msg_.acceleration.x = 0.0;
  cmd_msg_.acceleration.y = 0.0;
  cmd_msg_.acceleration.z = 0.0;
  cmd_msg_.yaw = 0.0;
  cmd_msg_.yaw_dot = 0.0;
  cmd_msg_.kx = {5.7, 5.7, 6.2};
  cmd_msg_.kv = {3.4, 3.4, 4.0};

  RCLCPP_INFO(get_logger(), "[Traj server]: init...");
}

void TrajServerNode::postInit(const rclcpp::Node::SharedPtr & node_handle)
{
  if (percep_ready_) {
    return;
  }

  percep_utils_ = std::make_shared<PerceptionUtils>(node_handle);
  percep_ready_ = true;

  publishInitializationMotion();

  RCLCPP_INFO(get_logger(), "[Traj server]: ready.");
}

void TrajServerNode::bsplineCallback(const bspline::msg::Bspline::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (msg->traj_id <= traj_id_) {
    RCLCPP_ERROR(get_logger(), "Out-of-order bspline received (id %d <= %d)", msg->traj_id, traj_id_);
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
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);

  traj_start_time_ = rclcpp::Time(msg->start_time);
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.reserve(6);
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_.front().getTimeSum();

  receive_traj_ = true;

  if (!stats_started_) {
    stats_started_ = true;
    stats_start_time_ = now();
  }
}

void TrajServerNode::replanCallback(const std_msgs::msg::Empty::SharedPtr)
{
  if (!receive_traj_) {
    return;
  }

  const double time_out = 0.3;
  const rclcpp::Time time_now = now();
  const double t_stop = (time_now - traj_start_time_).seconds() + time_out + replan_time_;
  traj_duration_ = std::min(t_stop, traj_duration_);
}

void TrajServerNode::newCallback(const std_msgs::msg::Empty::SharedPtr)
{
  traj_cmd_.clear();
  traj_real_.clear();
  energy_ = 0.0;
  have_last_sample_ = false;
}

void TrajServerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg || msg->child_frame_id == "X" || msg->child_frame_id == "O") {
    return;
  }

  traj_real_.emplace_back(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  if (traj_real_.size() > 10000) {
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
  }
}

void TrajServerNode::pgTVioCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  R_loop_ = q.toRotationMatrix();
  T_loop_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
}

void TrajServerNode::visualizationTimer()
{
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), pub_traj_id_);
}

void TrajServerNode::commandTimer()
{
  const rclcpp::Time time_now = now();
  Eigen::Vector3d pos;
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d jer = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yawdot = 0.0;

  if (!receive_traj_) {
    pos = init_pos_;
  } else {
    const double t_cur = (time_now - traj_start_time_).seconds();
    if (t_cur < traj_duration_ && t_cur >= 0.0) {
      pos = traj_[0].evaluateDeBoorT(t_cur);
      vel = traj_[1].evaluateDeBoorT(t_cur);
      acc = traj_[2].evaluateDeBoorT(t_cur);
      yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
      yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
      jer = traj_[5].evaluateDeBoorT(t_cur);
    } else if (t_cur >= traj_duration_) {
      pos = traj_[0].evaluateDeBoorT(traj_duration_);
      vel.setZero();
      acc.setZero();
      jer.setZero();
      yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
      yawdot = 0.0;

      if (stats_started_) {
        const double len = calcPathLength(traj_cmd_);
        const double flight_t = (stats_end_time_ - stats_start_time_).seconds();
        if (flight_t > 1e-3) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Flight time: %.3f s, path length: %.3f m, mean vel: %.3f m/s, energy: %.3f",
            flight_t, len, len / flight_t, energy_);
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[Traj server]: invalid time");
      return;
    }
  }

  if (is_loop_correction_) {
    pos = R_loop_.transpose() * (pos - T_loop_);
    vel = R_loop_.transpose() * vel;
    acc = R_loop_.transpose() * acc;

    Eigen::Vector3d yaw_dir(std::cos(yaw), std::sin(yaw), 0.0);
    yaw_dir = R_loop_.transpose() * yaw_dir;
    yaw = std::atan2(yaw_dir.y(), yaw_dir.x());
  }

  cmd_msg_.header.stamp = time_now;
  cmd_msg_.trajectory_id = traj_id_;
  cmd_msg_.position.x = pos.x();
  cmd_msg_.position.y = pos.y();
  cmd_msg_.position.z = pos.z();
  cmd_msg_.velocity.x = vel.x();
  cmd_msg_.velocity.y = vel.y();
  cmd_msg_.velocity.z = vel.z();
  cmd_msg_.acceleration.x = acc.x();
  cmd_msg_.acceleration.y = acc.y();
  cmd_msg_.acceleration.z = acc.z();
  cmd_msg_.yaw = yaw;
  cmd_msg_.yaw_dot = yawdot;
  pos_cmd_pub_->publish(cmd_msg_);

  if (percep_ready_) {
    percep_utils_->setPose(pos, yaw);
    std::vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawFOV(l1, l2);
  }

  if (traj_cmd_.empty()) {
    traj_cmd_.push_back(pos);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    traj_cmd_.push_back(pos);
    if (traj_cmd_.size() > 100000) {
      traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
    }
    if (have_last_sample_) {
      const double dt = (time_now - stats_last_time_).seconds();
      energy_ += jer.squaredNorm() * dt;
    }
    stats_end_time_ = time_now;
    stats_last_time_ = time_now;
    have_last_sample_ = true;
  }
}

void TrajServerNode::displayTrajWithColor(const std::vector<Eigen::Vector3d> & path, double resolution,
                                          const Eigen::Vector4d & color, int id)
{
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  traj_pub_->publish(mk);

  if (path.empty()) {
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

  for (const auto & pt_eig : path) {
    geometry_msgs::msg::Point pt;
    pt.x = pt_eig.x();
    pt.y = pt_eig.y();
    pt.z = pt_eig.z();
    mk.points.push_back(pt);
  }
  traj_pub_->publish(mk);
}

void TrajServerNode::drawFOV(const std::vector<Eigen::Vector3d> & list1,
                             const std::vector<Eigen::Vector3d> & list2)
{
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  mk.action = visualization_msgs::msg::Marker::DELETE;
  cmd_vis_pub_->publish(mk);

  if (list1.empty() || list1.size() != list2.size()) {
    return;
  }

  mk.points.clear();
  for (size_t i = 0; i < list1.size(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = list1[i].x();
    pt.y = list1[i].y();
    pt.z = list1[i].z();
    mk.points.push_back(pt);

    pt.x = list2[i].x();
    pt.y = list2[i].y();
    pt.z = list2[i].z();
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::msg::Marker::ADD;
  cmd_vis_pub_->publish(mk);
}

void TrajServerNode::drawCmd(const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, int id,
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

  geometry_msgs::msg::Point pt;
  pt.x = pos.x();
  pt.y = pos.y();
  pt.z = pos.z();
  mk_state.points.push_back(pt);

  pt.x = pos.x() + vec.x();
  pt.y = pos.y() + vec.y();
  pt.z = pos.z() + vec.z();
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub_->publish(mk_state);
}

void TrajServerNode::publishInitializationMotion()
{
  const auto sleep_time = std::chrono::milliseconds(10);
  for (int i = 0; i < 100; ++i) {
    cmd_msg_.position.z += 0.01;
    pos_cmd_pub_->publish(cmd_msg_);
    std::this_thread::sleep_for(sleep_time);
  }
  for (int i = 0; i < 100; ++i) {
    cmd_msg_.position.z -= 0.01;
    pos_cmd_pub_->publish(cmd_msg_);
    std::this_thread::sleep_for(sleep_time);
  }
}

double TrajServerNode::calcPathLength(const std::vector<Eigen::Vector3d> & path) const
{
  if (path.size() < 2) {
    return 0.0;
  }
  double len = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

}  // namespace fast_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fast_planner::TrajServerNode>();
  node->postInit(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
