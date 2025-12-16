#include <bspline/non_uniform_bspline.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

using namespace fast_planner;
using visualization_msgs::msg::Marker;

namespace {
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<Marker>::SharedPtr pos_pub;
rclcpp::Publisher<Marker>::SharedPtr vel_pub;
rclcpp::Publisher<Marker>::SharedPtr acc_pub;
int pub_id = 0;
}

void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id) {
  Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = Marker::SPHERE_LIST;
  mk.action = Marker::DELETE;
  mk.id = id;

  mk.action = Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::msg::Point pt;
  for (const auto& vec : list) {
    pt.x = vec(0);
    pt.y = vec(1);
    pt.z = vec(2);
    mk.points.push_back(pt);
  }

  if (pub_id == 0) {
    pos_pub->publish(mk);
  } else if (pub_id == 1) {
    vel_pub->publish(mk);
  } else if (pub_id == 2) {
    acc_pub->publish(mk);
  }
}

void drawBspline(NonUniformBspline bspline, double size, const Eigen::Vector4d& color, bool show_ctrl_pts,
                 double ctrl_size, const Eigen::Vector4d& ctrl_color, int id1, int id2) {
  vector<Eigen::Vector3d> traj_pts;
  double t_min, t_max;
  bspline.getTimeSpan(t_min, t_max);
  for (double t = t_min; t <= t_max; t += 0.01) {
    traj_pts.push_back(bspline.evaluateDeBoor(t));
  }
  displaySphereList(traj_pts, size, color, id1);

  if (!show_ctrl_pts) return;

  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
  vector<Eigen::Vector3d> ctrl_vec;
  ctrl_vec.reserve(ctrl_pts.rows());
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_vec.emplace_back(ctrl_pts.row(i));
  }
  displaySphereList(ctrl_vec, ctrl_size, ctrl_color, id2);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("pos_vel_acc");

  pos_pub = g_node->create_publisher<Marker>("/planning_vis/pos", 10);
  vel_pub = g_node->create_publisher<Marker>("/planning_vis/vel", 10);
  acc_pub = g_node->create_publisher<Marker>("/planning_vis/acc", 10);

  rclcpp::sleep_for(std::chrono::seconds(1));

  Eigen::MatrixXd ctrl_pts(8, 3);
  ctrl_pts.row(0) = Eigen::Vector3d(0, 0, 0);
  ctrl_pts.row(1) = Eigen::Vector3d(0.7, 0.9, 0);
  ctrl_pts.row(2) = Eigen::Vector3d(0.9, 2.1, 0);
  ctrl_pts.row(3) = Eigen::Vector3d(2.3, 2.8, 0);
  ctrl_pts.row(4) = Eigen::Vector3d(3.2, 2.8, 0);
  ctrl_pts.row(5) = Eigen::Vector3d(3.7, 1.6, 0);
  ctrl_pts.row(6) = Eigen::Vector3d(5.2, 1.3, 0);
  ctrl_pts.row(7) = Eigen::Vector3d(4.3, 0.6, 0);

  const double ts = 1.5;
  NonUniformBspline pos(ctrl_pts, 3, ts);
  NonUniformBspline vel = pos.getDerivative();
  NonUniformBspline acc = vel.getDerivative();

  pub_id = 0;
  drawBspline(pos, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1.0), true, 0.12,
              Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), 0, 1);
  pub_id = 1;
  drawBspline(vel, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), true, 0.12,
              Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), 0, 1);
  pub_id = 2;
  drawBspline(acc, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1.0), true, 0.12,
              Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), 0, 1);

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
