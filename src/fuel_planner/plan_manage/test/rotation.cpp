#include <Eigen/Eigen>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace Eigen;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rotation");

  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/rotation/odom", 10);
  auto mark_pub = node->create_publisher<visualization_msgs::msg::Marker>("/rotation/point", 10);
  rclcpp::sleep_for(1s);

  Matrix3d Rx, Ry, Rz;
  const double a = 0.785;

  Rx << 1.0, 0.0, 0.0, 0.0, cos(a), -sin(a), 0.0, sin(a), cos(a);
  Ry << cos(a), 0.0, sin(a), 0.0, 1.0, 0.0, -sin(a), 0.0, cos(a);
  Rz << cos(a), -sin(a), 0.0, sin(a), cos(a), 0.0, 0.0, 0.0, 1.0;

  Matrix3d R1 = Rz;
  Matrix3d R2 = Rz * Ry;
  Matrix3d R3 = Rz * Ry * Rx;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = node->now();

  auto publish_rotation = [&](const Matrix3d& rot, double x, double y) {
    Quaterniond q(rot);
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.header.stamp = node->now();
    odom_pub->publish(odom);
  };

  publish_rotation(R1, 1.0, 1.0);
  publish_rotation(R2, 2.0, 2.0);
  publish_rotation(R3, 3.0, 3.0);

  Vector3d p0(1, 0, 0);
  Vector3d p1 = R1 * p0;
  Vector3d p2 = R2 * p0;
  Vector3d p3 = Ry * p0;
  Vector3d p4 = Rz * Ry * p0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = node->now();
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.id = 0;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  geometry_msgs::msg::Point pt;
  pt.x = p1(0);
  pt.y = p1(1);
  pt.z = p1(2);
  marker.points.push_back(pt);
  pt.x = p2(0);
  pt.y = p2(1);
  pt.z = p2(2);
  marker.points.push_back(pt);
  pt.x = p3(0);
  pt.y = p3(1);
  pt.z = p3(2);
  marker.points.push_back(pt);
  pt.x = p4(0);
  pt.y = p4(1);
  pt.z = p4(2);
  marker.points.push_back(pt);

  mark_pub->publish(marker);

  rclcpp::sleep_for(1s);
  rclcpp::shutdown();
  return 0;
}
