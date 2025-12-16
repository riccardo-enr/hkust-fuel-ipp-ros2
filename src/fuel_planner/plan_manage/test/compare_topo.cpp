#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>

using namespace std::chrono_literals;

namespace {
pcl::PointCloud<pcl::PointXYZ> latest_cloud;
bool have_map = false;
bool have_goal = false;
Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
Eigen::Vector3d goal_pos = Eigen::Vector3d::Zero();
Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
rclcpp::Node::SharedPtr g_node;
}

void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::fromROSMsg(*msg, latest_cloud);
  have_map = !latest_cloud.points.empty();
  if (have_map) {
    RCLCPP_INFO(g_node->get_logger(), "[compare_topo] received map with %zu points", latest_cloud.points.size());
  }
}

void sgCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg) {
  if (msg->points.size() != 3) {
    RCLCPP_WARN(g_node->get_logger(), "[compare_topo] start/goal message malformed");
    return;
  }
  start_pos = Eigen::Vector3d(msg->points[0].x, msg->points[0].y, msg->points[0].z);
  start_vel = Eigen::Vector3d(msg->points[1].x, msg->points[1].y, msg->points[1].z);
  goal_pos = Eigen::Vector3d(msg->points[2].x, msg->points[2].y, msg->points[2].z);
  have_goal = true;
  RCLCPP_INFO(g_node->get_logger(), "[compare_topo] received start/goal");
}

void evaluateScenario(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr& finish_pub) {
  if (!have_map || !have_goal) {
    return;
  }

  double straight_dist = (goal_pos - start_pos).norm();
  double estimated_time = straight_dist / std::max(0.1, start_vel.norm() + 1.0);
  double density = latest_cloud.points.empty() ? 0.0 : static_cast<double>(latest_cloud.points.size()) / straight_dist;

  RCLCPP_INFO(g_node->get_logger(),
              "[compare_topo] straight distance: %.2f m, est time: %.2f s, obstacle density: %.2f", straight_dist,
              estimated_time, density);

  std_msgs::msg::Empty msg;
  finish_pub->publish(msg);
  have_goal = false;
  have_map = false;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("compare_topo");

  auto map_sub = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/laser_cloud_surround", rclcpp::QoS(1), mapCallback);
  auto sg_sub = g_node->create_subscription<sensor_msgs::msg::PointCloud>(
      "/start_goal", rclcpp::QoS(1), sgCallback);
  (void)map_sub;
  (void)sg_sub;

  auto finish_pub = g_node->create_publisher<std_msgs::msg::Empty>("/finish_test", rclcpp::QoS(1));

  rclcpp::WallRate rate(10);
  while (rclcpp::ok()) {
    rclcpp::spin_some(g_node);
    evaluateScenario(finish_pub);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
