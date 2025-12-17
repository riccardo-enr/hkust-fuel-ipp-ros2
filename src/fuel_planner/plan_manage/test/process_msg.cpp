#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace {
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub2;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ewok_pub;

std::vector<Eigen::Vector3d> traj;
std::vector<Eigen::Vector3d> vel;
std::vector<Eigen::Vector3d> yaw1;
std::vector<Eigen::Vector3d> yaw2;
std::vector<double> yaw;
std::vector<Eigen::Vector3d> cam1;
std::vector<Eigen::Vector3d> cam2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pts;
double distance1 = 0.0;
}  // namespace

void displayLineList(const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2,
                     double line_width, const Eigen::Vector4d& color, int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  yaw_pub->publish(mk);

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
  yaw_pub->publish(mk);
  rclcpp::sleep_for(1ms);
}

void displayTrajWithColor(const std::vector<Eigen::Vector3d>& path, double resolution, const Eigen::Vector4d& color,
                          int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  traj_pub->publish(mk);

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
  for (const auto& p : path) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    mk.points.push_back(pt);
  }
  traj_pub->publish(mk);
  rclcpp::sleep_for(1ms);
}

void cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
  static bool started = false;
  if (!started) {
    RCLCPP_INFO(g_node->get_logger(), "start");
    started = true;
  }

  Eigen::Vector3d pt(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector3d v(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  if (!traj.empty()) {
    distance1 += (pt - traj.back()).norm();
  }

  traj.push_back(pt);
  vel.push_back(v);
  displayTrajWithColor(traj, 0.1, Eigen::Vector4d(1, 0, 0, 1), 0);

  yaw.push_back(msg->yaw);
  yaw1.clear();
  yaw2.clear();
  for (int k = 0; k < 4; ++k) {
    int idx = static_cast<int>(yaw.size()) - 1 - 30 * k;
    if (idx < 0) continue;
    double phi_k = yaw[idx];
    Eigen::Vector3d pt_k = traj[idx];
    Eigen::Matrix3d Rwb;
    Rwb << cos(phi_k), -sin(phi_k), 0, sin(phi_k), cos(phi_k), 0, 0, 0, 1;
    for (size_t i = 0; i < cam1.size(); ++i) {
      auto p1 = Rwb * cam1[i] + pt_k;
      auto p2 = Rwb * cam2[i] + pt_k;
      yaw1.push_back(p1);
      yaw2.push_back(p2);
    }
  }

  if (v.norm() < 1e-3) {
    RCLCPP_INFO(g_node->get_logger(), "end, distance: %lf", distance1);
  }
}

void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> pts2;
  pcl::fromROSMsg(*msg, pts2);

  pcl::PointCloud<pcl::PointXYZ> filtered;
  auto new_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  *new_cloud = pts2;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(new_cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(filtered);

  pts = new_cloud;
  pts->points = filtered.points;
  pts->width = pts->points.size();
  pts->height = 1;
  pts->is_dense = true;
  pts->header.frame_id = "world";

  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pts, cloud);
  cloud.header.stamp = g_node->now();
  cloud_pub->publish(cloud);
}

void ewokCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
  if (msg->markers.empty()) return;
  auto marker = msg->markers[0];
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.header.stamp = g_node->now();
  ewok_pub->publish(marker);
}

void travelCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  auto mk = *msg;
  if (mk.id == 5) {
    mk.color.g = 0.8;
  }
  mk.header.stamp = g_node->now();
  traj_pub2->publish(mk);
}

void trajCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (msg->id != 399) return;

  visualization_msgs::msg::Marker mk;
  mk.header = msg->header;
  mk.ns = "traj_copy";
  mk.id = msg->id;
  mk.type = msg->type;
  mk.scale = msg->scale;
  mk.pose = msg->pose;
  mk.points = msg->points;
  mk.colors = msg->colors;
  mk.color = msg->color;
  mk.header.stamp = g_node->now();
  traj_pub2->publish(mk);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("faster");

  auto qos = rclcpp::QoS(10);
  traj_pub = g_node->create_publisher<visualization_msgs::msg::Marker>("/process_msg/execute_traj", qos);
  yaw_pub = g_node->create_publisher<visualization_msgs::msg::Marker>("/process_msg/execute_yaw", qos);
  cloud_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/process_msg/global_cloud", qos);
  ewok_pub = g_node->create_publisher<visualization_msgs::msg::Marker>("/process_msg/ewok", qos);
  traj_pub2 = g_node->create_publisher<visualization_msgs::msg::Marker>("/planning/travel_traj", qos);

  auto cmd_qos = rclcpp::SensorDataQoS();
  auto cmd_sub = g_node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd", cmd_qos, std::bind(&cmdCallback, std::placeholders::_1));
  (void)cmd_sub;
  auto cloud_sub = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sdf_map/occupancy_local", qos, std::bind(&cloudCallback, std::placeholders::_1));
  (void)cloud_sub;
  auto traj_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/planning_vis/trajectory", qos, std::bind(&trajCallback, std::placeholders::_1));
  (void)traj_sub;
  auto ewok_sub = g_node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/firefly/optimal_trajectory", qos, std::bind(&ewokCallback, std::placeholders::_1));
  (void)ewok_sub;
  auto travel_sub = g_node->create_subscription<visualization_msgs::msg::Marker>(
      "/planning/travel_traj", qos, std::bind(&travelCallback, std::placeholders::_1));
  (void)travel_sub;

  pts.reset(new pcl::PointCloud<pcl::PointXYZ>());

  const double vert_ang = 0.56125;
  const double hor_ang = 0.68901;
  const double cam_scale = 0.5;
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
