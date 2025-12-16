#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <traj_utils/planning_visualization.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace fast_planner;
using namespace std::chrono_literals;

void buildEnv(pcl::PointCloud<pcl::PointXYZ>& map) {
  const double resolution = 0.05;
  pcl::PointXYZ pt;

  for (double x = -0.5; x <= 0.5; x += resolution)
    for (double y = -1.5; y <= 1.5; y += resolution)
      for (double z = 0.0; z <= 2.0; z += resolution) {
        pt.x = x + 1e-2;
        pt.y = y + 1e-2;
        pt.z = z + 1e-2;
        map.push_back(pt);
      }

  std::vector<Eigen::Vector3d> centers = {
      {2.5, -0.5, 0.0}, {2.5, 2.3, 0.0}, {-2.5, 2.3, 0.0}, {0.0, 3.5, 0.0} };

  for (const auto& center : centers) {
    for (double x = -0.27; x <= 0.27; x += resolution)
      for (double y = -0.3; y <= 0.3; y += resolution)
        for (double z = 0.0; z <= 2.0; z += resolution) {
          pt.x = x + center(0) + 1e-2;
          pt.y = y + center(1) + 1e-2;
          pt.z = z + center(2) + 1e-2;
          map.push_back(pt);
        }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("replan_node");

  double limit_vel = node->declare_parameter("bspline/limit_vel", -1.0);
  double limit_acc = node->declare_parameter("bspline/limit_acc", -1.0);
  double limit_ratio = node->declare_parameter("bspline/limit_ratio", 1.0);

  auto map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test_collision/map", rclcpp::QoS(10).transient_local());

  PlanningVisualization::Ptr visualizer(new PlanningVisualization(node));

  auto sdf_map = std::make_shared<SDFMap>();
  sdf_map->initMap(node);

  auto env = std::make_shared<EDTEnvironment>();
  env->setMap(sdf_map);

  auto bspline_opt = std::make_shared<BsplineOptimizer>();
  bspline_opt->setParam(node);
  bspline_opt->setEnvironment(env);

  pcl::PointCloud<pcl::PointXYZ> cloud_map;
  buildEnv(cloud_map);

  for (const auto& pt : cloud_map.points) {
    sdf_map->setOccupied(Eigen::Vector3d(pt.x, pt.y, pt.z));
  }
  sdf_map->updateESDF3d();

  Eigen::Vector3d p0(-5.1, 0.05, 1);
  Eigen::Vector3d v1(2, 0.1, 0);
  Eigen::Vector3d v2(2, -0.1, 0);

  double dt = 0.1;
  std::vector<Eigen::Vector3d> point_set;
  std::vector<Eigen::Vector3d> start_end_derivative;

  for (double t = 0.0; t <= 5.0 + 1e-4; t += dt) {
    Eigen::Vector3d p_t;
    if (t <= 2.5) {
      p_t = p0 + v1 * t;
    } else {
      p_t = p0 + v1 * 2.5 + v2 * (t - 2.5);
    }
    point_set.push_back(p_t);
  }
  start_end_derivative.push_back(v1);
  start_end_derivative.push_back(v2);
  start_end_derivative.push_back(Eigen::Vector3d::Zero());
  start_end_derivative.push_back(Eigen::Vector3d::Zero());

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, 3, ctrl_pts);
  double knot_span = dt;
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  auto t1 = node->now();
  bspline_opt->optimize(ctrl_pts, knot_span, cost_function, 1, 1);
  auto t2 = node->now();
  RCLCPP_INFO(node->get_logger(), "optimization time: %.4f", (t2 - t1).seconds());

  NonUniformBspline traj_opt(ctrl_pts, 3, knot_span);
  traj_opt.setPhysicalLimits(limit_vel, limit_acc);
  traj_opt.lengthenTime(limit_ratio);

  rclcpp::sleep_for(500ms);
  cloud_map.width = cloud_map.points.size();
  cloud_map.height = 1;
  cloud_map.is_dense = true;
  cloud_map.header.frame_id = "world";

  sensor_msgs::msg::PointCloud2 cloud_vis;
  pcl::toROSMsg(cloud_map, cloud_vis);
  cloud_vis.header.frame_id = "world";
  cloud_vis.header.stamp = node->now();
  map_pub->publish(cloud_vis);

  visualizer->drawBspline(traj_opt, 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1), true, 0.12,
                          Eigen::Vector4d(0, 0, 1, 1), 1);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
