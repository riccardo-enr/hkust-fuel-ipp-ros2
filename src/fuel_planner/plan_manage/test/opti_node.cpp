#include <Eigen/Eigen>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <poly_traj/polynomial_traj.h>

using namespace std::chrono_literals;
using namespace fast_planner;

namespace {
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr poly_traj_pub;
rclcpp::Node::SharedPtr g_node;
}

void displayPathWithColor(const std::vector<Eigen::Vector3d>& path, double resolution,
                          const Eigen::Vector4d& color, int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  poly_traj_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  geometry_msgs::msg::Point pt;
  for (const auto& p : path) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    mk.points.push_back(pt);
  }
  poly_traj_pub->publish(mk);
  rclcpp::sleep_for(10ms);
}

std::vector<Eigen::Vector3d> samplePolynomial(PolynomialTraj traj, double dt) {
  std::vector<Eigen::Vector3d> result;
  double total_time = traj.getTotalTime();
  for (double t = 0.0; t <= total_time; t += dt) {
    result.push_back(traj.evaluate(t, 0));
  }
  return result;
}

PolynomialTraj generateTrajectory(const std::vector<Eigen::Vector3d>& path) {
  if (path.size() < 2) {
    return PolynomialTraj();
  }

  Eigen::MatrixXd positions(path.size(), 3);
  for (size_t i = 0; i < path.size(); ++i) {
    positions.row(i) = path[i];
  }

  Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();

  Eigen::VectorXd times(path.size() - 1);
  for (size_t i = 0; i < path.size() - 1; ++i) {
    double seg_len = (path[i + 1] - path[i]).norm();
    times(i) = std::max(0.5, seg_len / 2.0);
  }

  PolynomialTraj poly_traj;
  PolynomialTraj::waypointsTraj(positions, start_vel, end_vel, start_acc, end_acc, times, poly_traj);
  return poly_traj;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("opti_node");
  poly_traj_pub = g_node->create_publisher<visualization_msgs::msg::Marker>("/gradient_based/traj", 10);

  rclcpp::sleep_for(500ms);

  std::vector<Eigen::Vector3d> obstacles;
  for (double x = 0.05; x <= 3.0; x += 0.2)
    for (double y = 2.05; y <= 2.7; y += 0.2)
      for (double z = 0.05; z <= 5.0; z += 0.2) {
        obstacles.emplace_back(x, y, z);
      }
  for (double x = 0.05; x >= -3.0; x -= 0.2)
    for (double y = -2.05; y >= -2.7; y -= 0.2)
      for (double z = 0.05; z <= 5.0; z += 0.2) {
        obstacles.emplace_back(x, y, z);
      }

  std::vector<Eigen::Vector3d> init_path = {
      {0, -5, 2},  {1, -4, 2},  {1, -3, 2},  {1, -2, 2},  {1, -1, 2},
      {0, 0, 2},   {-1, 1, 2}, {-1, 2, 2}, {-1, 3, 2}, {-1, 4, 2}, {0, 5, 2} };

  PolynomialTraj poly_traj = generateTrajectory(init_path);
  auto traj_vis = samplePolynomial(poly_traj, 0.05);

  displayPathWithColor(obstacles, 0.5, Eigen::Vector4d(1, 1, 0, 1), 0);
  displayPathWithColor(init_path, 0.3, Eigen::Vector4d(1, 0, 0, 1), 1);
  displayPathWithColor(traj_vis, 0.15, Eigen::Vector4d(0, 0, 1, 1), 2);

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
