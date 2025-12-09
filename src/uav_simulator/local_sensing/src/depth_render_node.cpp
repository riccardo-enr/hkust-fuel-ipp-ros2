#include <fstream>
#include <iostream>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <Eigen/Eigen>

using namespace cv;
using namespace std;
using namespace Eigen;

class LocalSensingNode : public rclcpp::Node {
public:
  LocalSensingNode() : Node("pcl_render_node") {
    this->declare_parameter("cam_width", 640);
    this->declare_parameter("cam_height", 480);
    this->declare_parameter("cam_fx", 387.229248046875);
    this->declare_parameter("cam_fy", 387.229248046875);
    this->declare_parameter("cam_cx", 321.04638671875);
    this->declare_parameter("cam_cy", 243.44969177246094);
    this->declare_parameter("sensing_horizon", 5.0);
    this->declare_parameter("sensing_rate", 10.0);
    this->declare_parameter("estimation_rate", 30.0);

    width = this->get_parameter("cam_width").as_int();
    height = this->get_parameter("cam_height").as_int();
    fx = this->get_parameter("cam_fx").as_double();
    fy = this->get_parameter("cam_fy").as_double();
    cx = this->get_parameter("cam_cx").as_double();
    cy = this->get_parameter("cam_cy").as_double();
    sensing_horizon = this->get_parameter("sensing_horizon").as_double();
    sensing_rate = this->get_parameter("sensing_rate").as_double();
    estimation_rate = this->get_parameter("estimation_rate").as_double();

    cam02body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    cam2world = Matrix4d::Identity();

    global_map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "global_map", 1, std::bind(&LocalSensingNode::pointCloudCallBack, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 50, std::bind(&LocalSensingNode::odometryCallbck, this, std::placeholders::_1));

    pub_depth = this->create_publisher<sensor_msgs::msg::Image>("/pcl_render_node/depth", 10);
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pcl_render_node/sensor_pose", 10);

    double sensing_duration = 1.0 / sensing_rate;
    double estimate_duration = 1.0 / estimation_rate;

    local_sensing_timer = this->create_wall_timer(
        std::chrono::duration<double>(sensing_duration), std::bind(&LocalSensingNode::renderSensedPoints, this));
    estimation_timer = this->create_wall_timer(
        std::chrono::duration<double>(estimate_duration), std::bind(&LocalSensingNode::pubCameraPose, this));
  }

private:
  cv::Mat depth_mat;
  int width, height;
  double fx, fy, cx, cy;
  double sensing_horizon, sensing_rate, estimation_rate;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
  rclcpp::TimerBase::SharedPtr local_sensing_timer, estimation_timer;

  bool has_global_map = false;
  bool has_odom = false;

  Matrix4d cam02body;
  Matrix4d cam2world;
  Eigen::Quaterniond cam2world_quat;
  nav_msgs::msg::Odometry odom_;

  rclcpp::Time last_odom_stamp;
  Eigen::Vector3d last_pose_world;
  pcl::PointCloud<pcl::PointXYZ> cloudIn;

  void odometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom) {
    has_odom = true;
    odom_ = *odom;
    Matrix4d Pose_receive = Matrix4d::Identity();

    Eigen::Vector3d request_position;
    Eigen::Quaterniond request_pose;
    request_position.x() = odom->pose.pose.position.x;
    request_position.y() = odom->pose.pose.position.y;
    request_position.z() = odom->pose.pose.position.z;
    request_pose.x() = odom->pose.pose.orientation.x;
    request_pose.y() = odom->pose.pose.orientation.y;
    request_pose.z() = odom->pose.pose.orientation.z;
    request_pose.w() = odom->pose.pose.orientation.w;
    Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
    Pose_receive(0, 3) = request_position(0);
    Pose_receive(1, 3) = request_position(1);
    Pose_receive(2, 3) = request_position(2);

    Matrix4d body_pose = Pose_receive;
    // convert to cam pose
    cam2world = body_pose * cam02body;
    cam2world_quat = cam2world.block<3, 3>(0, 0);
    last_odom_stamp = odom->header.stamp;

    last_pose_world(0) = odom->pose.pose.position.x;
    last_pose_world(1) = odom->pose.pose.position.y;
    last_pose_world(2) = odom->pose.pose.position.z;
  }

  void pubCameraPose() {
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = odom_.header;
    camera_pose.header.frame_id = "/map";
    camera_pose.pose.position.x = cam2world(0, 3);
    camera_pose.pose.position.y = cam2world(1, 3);
    camera_pose.pose.position.z = cam2world(2, 3);
    camera_pose.pose.orientation.w = cam2world_quat.w();
    camera_pose.pose.orientation.x = cam2world_quat.x();
    camera_pose.pose.orientation.y = cam2world_quat.y();
    camera_pose.pose.orientation.z = cam2world_quat.z();
    pub_pose->publish(camera_pose);
  }

  void pointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
    if (has_global_map) return;

    RCLCPP_WARN(this->get_logger(), "Global Pointcloud received..");
    // load global map
    // transform map to point cloud format
    pcl::fromROSMsg(*pointcloud_map, cloudIn);
    printf("global map has points: %ld.\n", cloudIn.points.size());
    has_global_map = true;
  }

  void renderDepth() {
    depth_mat = cv::Mat::zeros(height, width, CV_32FC1);

    Eigen::Matrix4d Tcw = cam2world.inverse();
    Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
    Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);

    Eigen::Vector3d pos = cam2world.block<3, 1>(0, 3);
    for (auto pt : cloudIn.points) {
      Eigen::Vector3d pw(pt.x, pt.y, pt.z);
      if ((pos - pw).norm() > 5.0) continue;

      Eigen::Vector3d pc = Rcw * pw + tcw;

      if (pc[2] <= 0.0) continue;

      float projected_x, projected_y;
      projected_x = pc[0] / pc[2] * fx + cx;
      projected_y = pc[1] / pc[2] * fy + cy;
      if (projected_x < 0 || projected_x >= width || projected_y < 0 || projected_y >= height)
        continue;

      float dist = pc[2];
      int r = 0.0573 * fx / dist + 0.5;
      int min_x = max(int(projected_x - r), 0);
      int max_x = min(int(projected_x + r), width - 1);
      int min_y = max(int(projected_y - r), 0);
      int max_y = min(int(projected_y + r), height - 1);

      for (int to_x = min_x; to_x <= max_x; to_x++)
        for (int to_y = min_y; to_y <= max_y; to_y++) {
          float value = depth_mat.at<float>(to_y, to_x);
          if (value < 1e-3) {
            depth_mat.at<float>(to_y, to_x) = dist;
          } else {
            depth_mat.at<float>(to_y, to_x) = min(value, dist);
          }
        }
    }

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = last_odom_stamp;
    out_msg.header.frame_id = "world";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image = depth_mat.clone();
    pub_depth->publish(*out_msg.toImageMsg());
  }

  void renderSensedPoints() {
    if (!has_global_map) return;
    renderDepth();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalSensingNode>());
  rclcpp::shutdown();
  return 0;
}
