#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

class PCLRenderNode : public rclcpp::Node {
public:
  PCLRenderNode() : Node("pcl_render_node") {
    // Declare and get parameters
    this->declare_parameter("sensing_horizon", 0.0);
    this->declare_parameter("sensing_rate", 0.0);
    this->declare_parameter("estimation_rate", 0.0);
    this->declare_parameter("map/x_size", 0.0);
    this->declare_parameter("map/y_size", 0.0);
    this->declare_parameter("map/z_size", 0.0);

    sensing_horizon_ = this->get_parameter("sensing_horizon").as_double();
    sensing_rate_ = this->get_parameter("sensing_rate").as_double();
    estimation_rate_ = this->get_parameter("estimation_rate").as_double();

    x_size_ = this->get_parameter("map/x_size").as_double();
    y_size_ = this->get_parameter("map/y_size").as_double();
    z_size_ = this->get_parameter("map/z_size").as_double();

    // Subscribers
    global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", 1, std::bind(&PCLRenderNode::rcvGlobalPointCloudCallBack, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 50, std::bind(&PCLRenderNode::rcvOdometryCallbck, this, std::placeholders::_1));

    // Publishers
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_render_node/cloud", 10);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pcl_render_node/sensor_pose", 10);

    // Timers
    double sensing_duration = 1.0 / sensing_rate_;
    double estimate_duration = 1.0 / estimation_rate_;
    
    local_sensing_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sensing_duration), std::bind(&PCLRenderNode::renderSensedPoints, this));
    pose_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(estimate_duration), std::bind(&PCLRenderNode::pubSensorPose, this));

    // Initialize grid variables
    inv_resolution_ = 1.0 / resolution_; // Warning: resolution_ is not initialized from param in original code? 
                                         // In original code: inv_resolution = 1.0 / resolution; but resolution is global var.
                                         // It seems resolution is not set in params of main. 
                                         // Let's check where resolution comes from. 
                                         // It was a global variable 'double resolution, inv_resolution;' but never assigned a value in main from param.
                                         // Assuming 0.1 or similar default if it was missing? 
                                         // Actually in original code it might be uninitialized which is a bug or I missed it.
                                         // Wait, 'gridIndex2coord' uses 'gl_xl', 'resolution'.
                                         // I will add a default resolution parameter to be safe, e.g., 0.1
    
    this->declare_parameter("resolution", 0.1);
    resolution_ = this->get_parameter("resolution").as_double();
    inv_resolution_ = 1.0 / resolution_;

    gl_xl_ = -x_size_ / 2.0;
    gl_yl_ = -y_size_ / 2.0;
    gl_zl_ = 0.0;
    GLX_SIZE_ = (int)(x_size_ * inv_resolution_);
    GLY_SIZE_ = (int)(y_size_ * inv_resolution_);
    GLZ_SIZE_ = (int)(z_size_ * inv_resolution_);

    sensor2body_ << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
    has_global_map_ = false;
    has_local_map_ = false;
    has_odom_ = false;
  }

private:
  void rcvOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom) {
    has_odom_ = true;
    odom_ = *odom;

    Matrix4d body2world = Matrix4d::Identity();

    Eigen::Quaterniond pose;
    pose.x() = odom->pose.pose.orientation.x;
    pose.y() = odom->pose.pose.orientation.y;
    pose.z() = odom->pose.pose.orientation.z;
    pose.w() = odom->pose.pose.orientation.w;
    body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;

    // convert to cam pose
    sensor2world_ = body2world * sensor2body_;
  }

  void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
    if (has_global_map_)
      return;

    RCLCPP_WARN(this->get_logger(), "Global Pointcloud received..");

    pcl::PointCloud<pcl::PointXYZ> cloud_input;
    pcl::fromROSMsg(*pointcloud_map, cloud_input);

    _voxel_sampler_.setLeafSize(0.1f, 0.1f, 0.1f);
    _voxel_sampler_.setInputCloud(cloud_input.makeShared());
    _voxel_sampler_.filter(cloud_all_map_);

    _kdtreeLocalMap_.setInputCloud(cloud_all_map_.makeShared());

    has_global_map_ = true;
  }

  void renderSensedPoints() {
    if (!has_global_map_ || !has_odom_)
      return;

    Eigen::Quaterniond q;
    q.x() = odom_.pose.pose.orientation.x;
    q.y() = odom_.pose.pose.orientation.y;
    q.z() = odom_.pose.pose.orientation.z;
    q.w() = odom_.pose.pose.orientation.w;

    Eigen::Vector3d pos;
    pos << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z;

    Eigen::Matrix3d rot;
    rot = q;
    Eigen::Vector3d yaw_vec = rot.col(0);

    local_map_.points.clear();
    pcl::PointXYZ searchPoint(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
    pointIdxRadiusSearch_.clear();
    pointRadiusSquaredDistance_.clear();

    if (_kdtreeLocalMap_.radiusSearch(searchPoint, sensing_horizon_, pointIdxRadiusSearch_, pointRadiusSquaredDistance_) > 0) {
      for (size_t i = 0; i < pointIdxRadiusSearch_.size(); ++i) {
        auto pt = cloud_all_map_.points[pointIdxRadiusSearch_[i]];
        Eigen::Vector3d pt3;
        pt3[0] = pt.x;
        pt3[1] = pt.y;
        pt3[2] = pt.z;
        auto dir = pt3 - pos;

        if (fabs(dir[2]) > dir.head<2>().norm() * tan(M_PI / 6.0))
          continue;

        if (dir.dot(yaw_vec) < 0)
          continue;

        local_map_.points.push_back(pt);
      }
      local_map_.width = local_map_.points.size();
      local_map_.height = 1;
      local_map_.is_dense = true;

      pcl::toROSMsg(local_map_, local_map_pcd_);
      local_map_pcd_.header = odom_.header;
      pub_cloud_->publish(local_map_pcd_);
    }
  }

  void pubSensorPose() {
    Eigen::Quaterniond q;
    q = sensor2world_.block<3, 3>(0, 0);

    geometry_msgs::msg::PoseStamped sensor_pose;
    sensor_pose.header = odom_.header;
    sensor_pose.header.frame_id = "map";
    sensor_pose.pose.position.x = sensor2world_(0, 3);
    sensor_pose.pose.position.y = sensor2world_(1, 3);
    sensor_pose.pose.position.z = sensor2world_(2, 3);
    sensor_pose.pose.orientation.w = q.w();
    sensor_pose.pose.orientation.x = q.x();
    sensor_pose.pose.orientation.y = q.y();
    sensor_pose.pose.orientation.z = q.z();
    pub_pose_->publish(sensor_pose);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
  rclcpp::TimerBase::SharedPtr local_sensing_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  sensor_msgs::msg::PointCloud2 local_map_pcd_;
  nav_msgs::msg::Odometry odom_;
  Eigen::Matrix4d sensor2body_, sensor2world_;

  bool has_global_map_;
  bool has_local_map_;
  bool has_odom_;

  double sensing_horizon_, sensing_rate_, estimation_rate_;
  double x_size_, y_size_, z_size_;
  double gl_xl_, gl_yl_, gl_zl_;
  double resolution_, inv_resolution_;
  int GLX_SIZE_, GLY_SIZE_, GLZ_SIZE_;

  pcl::PointCloud<pcl::PointXYZ> cloud_all_map_, local_map_;
  pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler_;
  pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap_;
  vector<int> pointIdxRadiusSearch_;
  vector<float> pointRadiusSquaredDistance_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLRenderNode>());
  rclcpp::shutdown();
  return 0;
}
