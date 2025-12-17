#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <iostream>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace {
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
std::vector<Eigen::Vector3d> cam1;
std::vector<Eigen::Vector3d> cam2;
Eigen::Vector3d last_cmd_pos = Eigen::Vector3d::Zero();
double last_yaw = 0.0;
double alpha = 0.9;
visualization_msgs::msg::Marker view_marker;
}

void drawLines(const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2,
               double line_width, const Eigen::Vector4d& color, const std::string& ns, int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.ns = ns;
  mk.id = id;
  marker_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::msg::Point pt;
  for (size_t i = 0; i < list1.size(); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  marker_pub->publish(mk);
  rclcpp::sleep_for(1ms);
}

void drawSpheres(const std::vector<Eigen::Vector3d>& points, double resolution, const Eigen::Vector4d& color,
                 const std::string& ns, int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.ns = ns;
  mk.id = id;
  marker_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::msg::Point pt;
  for (const auto& p : points) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    mk.points.push_back(pt);
  }
  marker_pub->publish(mk);
  rclcpp::sleep_for(1ms);
}

void calcNextYaw(double last, double& yaw) {
  double round_last = last;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last + diff;
  } else if (diff > M_PI) {
    yaw = last + diff - 2 * M_PI;
  } else {
    yaw = last + diff + 2 * M_PI;
  }
}

void fovCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (msg->points.empty()) return;

  Eigen::Vector3d p0(msg->points[0].x, msg->points[0].y, msg->points[0].z);
  last_cmd_pos = p0;

  Eigen::Vector3d p1(msg->points[1].x, msg->points[1].y, msg->points[1].z);
  Eigen::Vector3d p5(msg->points[5].x, msg->points[5].y, msg->points[5].z);
  Eigen::Vector3d dir = p1 - p0 + p5 - p0;
  double tmp_yaw = atan2(dir[1], dir[0]);
  calcNextYaw(last_yaw, tmp_yaw);

  double filtered_yaw = (1 - alpha) * tmp_yaw + alpha * last_yaw;
  last_yaw = filtered_yaw;

  Eigen::Matrix3d Rwb;
  Rwb << cos(filtered_yaw), -sin(filtered_yaw), 0,
         sin(filtered_yaw), cos(filtered_yaw), 0,
         0, 0, 1;

  std::vector<Eigen::Vector3d> l1, l2;
  for (size_t i = 0; i < cam1.size(); ++i) {
    l1.push_back(Rwb * cam1[i] + p0);
    l2.push_back(Rwb * cam2[i] + p0);
  }
  drawLines(l1, l2, 0.04, Eigen::Vector4d(0, 0, 0, 1), "fov", 0);
}

void cmdTrajCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (msg->points.empty()) return;

  visualization_msgs::msg::Marker mk = *msg;
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  if (mk.id == 2) {
    mk.color.r = 1;
    mk.color.g = 0;
    mk.color.b = 0;
    mk.ns = "classic";
    for (auto& p : mk.points) {
      p.y = -p.y;
    }
  } else if (mk.id == 3) {
    mk.color.r = 0;
    mk.color.g = 1;
    mk.color.b = 0;
    mk.ns = "rapid";
  } else if (mk.id == 4) {
    mk.color.r = 0;
    mk.color.g = 0;
    mk.color.b = 1;
    mk.ns = "propose";
  }
  mk.header.stamp = g_node->now();
  marker_pub->publish(mk);
}

void planTrajCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (msg->points.empty()) return;

  visualization_msgs::msg::Marker mk = *msg;
  Eigen::Vector3d p0(mk.points[0].x, mk.points[0].y, mk.points[0].z);
  Eigen::Vector3d diff = last_cmd_pos - p0;
  for (auto& p : mk.points) {
    p.x += diff[0];
    p.y += diff[1];
    p.z += diff[2];
  }
  mk.color.a = 0.5;
  mk.ns = "plan_traj";
  mk.id = 0;
  mk.header.stamp = g_node->now();
  marker_pub->publish(mk);

  if (view_marker.points.size() < 6) return;

  Eigen::Vector3d p0_next(view_marker.points[0].x, view_marker.points[0].y, view_marker.points[0].z);
  Eigen::Vector3d p1(view_marker.points[1].x, view_marker.points[1].y, view_marker.points[1].z);
  Eigen::Vector3d p5(view_marker.points[5].x, view_marker.points[5].y, view_marker.points[5].z);
  Eigen::Vector3d dir = p1 + p5 - 2 * p0_next;
  double next_yaw = atan2(dir[1], dir[0]);
  Eigen::Matrix3d Rwb;
  Rwb << cos(next_yaw), -sin(next_yaw), 0,
         sin(next_yaw), cos(next_yaw), 0,
         0, 0, 1;

  auto p_msg_end = mk.points.back();
  Eigen::Vector3d p_end(p_msg_end.x, p_msg_end.y, p_msg_end.z);
  std::vector<Eigen::Vector3d> l1, l2;
  for (size_t i = 0; i < cam1.size(); ++i) {
    l1.push_back(Rwb * cam1[i] + p_end);
    l2.push_back(Rwb * cam2[i] + p_end);
  }
  drawLines(l1, l2, 0.04, Eigen::Vector4d(1, 0, 0, 1), "plan_traj", 1);
}

void viewCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (msg->ns == "global_tour" && msg->points.empty()) {
    visualization_msgs::msg::Marker mk = *msg;
    mk.ns = "plan_traj";
    marker_pub->publish(mk);
    mk.ns = "next_fov";
    marker_pub->publish(mk);
    return;
  }

  if (msg->ns != "refined_view" || msg->points.empty()) return;
  view_marker = *msg;
  if (view_marker.points.size() > 16) {
    view_marker.points.erase(view_marker.points.begin() + 16, view_marker.points.end());
  }
}

void nbvpCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  visualization_msgs::msg::Marker mk = *msg;
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;
  mk.color.r = 1;
  mk.color.g = 0;
  mk.color.b = 1;
  mk.ns = "nbvp";

  for (auto& p : mk.points) {
    double tmpx = p.x;
    double tmpy = p.y;
    p.x = -tmpy;
    p.y = -tmpx;
  }
  mk.header.stamp = g_node->now();
  marker_pub->publish(mk);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("process_msg2");

  alpha = g_node->declare_parameter("process_msg/alpha", 0.9);
  marker_pub = g_node->create_publisher<visualization_msgs::msg::Marker>("/process_msg/marker1", 10);

  auto qos = rclcpp::QoS(10);
  auto fov_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/planning/position_cmd_vis", qos, std::bind(&fovCallback, std::placeholders::_1));
  auto cmd_traj_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/planning/travel_traj", qos, std::bind(&cmdTrajCallback, std::placeholders::_1));
  auto plan_traj_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/planning_vis/trajectory", qos, std::bind(&planTrajCallback, std::placeholders::_1));
  auto view_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/planning_vis/viewpoints", qos, std::bind(&viewCallback, std::placeholders::_1));
  auto nbvp_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/firefly/visualization_marker", qos, std::bind(&nbvpCallback, std::placeholders::_1));
  (void)fov_sub;
  (void)cmd_traj_sub;
  (void)plan_traj_sub;
  (void)view_sub;
  (void)nbvp_sub;

  const double vert_ang = 0.56125;
  const double hor_ang = 0.68901;
  const double cam_scale = 0.8;
  double hor = cam_scale * tan(hor_ang);
  double vert = cam_scale * tan(vert_ang);
  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(cam_scale, hor, vert);
  Eigen::Vector3d left_down(cam_scale, hor, -vert);
  Eigen::Vector3d right_up(cam_scale, -hor, vert);
  Eigen::Vector3d right_down(cam_scale, -hor, -vert);

  cam1 = { origin, origin, origin, origin, left_up, right_up, right_down, left_down };
  cam2 = { left_up, left_down, right_up, right_down, right_up, right_down, left_down, left_up };

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
