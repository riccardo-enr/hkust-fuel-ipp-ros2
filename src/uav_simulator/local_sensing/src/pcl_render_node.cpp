#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "depth_render.cuh"

using Eigen::Matrix4d;
using Eigen::Vector3d;

namespace
{
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultCameraFrame[] = "SQ01s/camera";
}  // namespace

class PclRenderNode : public rclcpp::Node
{
public:
  PclRenderNode()
  : rclcpp::Node("pcl_render_node"),
    cam02body_(Matrix4d::Zero()),
    cam2world_(Matrix4d::Identity()),
    cam2world_quat_(Eigen::Quaterniond::Identity()),
    last_pose_world_(Vector3d::Zero()),
    sensing_horizon_(5.0)
  {
    width_ = this->declare_parameter<int>("cam_width", 320);
    height_ = this->declare_parameter<int>("cam_height", 240);
    fx_ = this->declare_parameter<double>("cam_fx", 160.0);
    fy_ = this->declare_parameter<double>("cam_fy", 160.0);
    cx_ = this->declare_parameter<double>("cam_cx", static_cast<double>(width_) / 2.0);
    cy_ = this->declare_parameter<double>("cam_cy", static_cast<double>(height_) / 2.0);
    sensing_horizon_ = this->declare_parameter<double>("sensing_horizon", 5.0);
    sensing_rate_ = this->declare_parameter<double>("sensing_rate", 30.0);
    estimation_rate_ = this->declare_parameter<double>("estimation_rate", 30.0);
    x_size_ = this->declare_parameter<double>("map/x_size", 30.0);
    y_size_ = this->declare_parameter<double>("map/y_size", 30.0);
    z_size_ = this->declare_parameter<double>("map/z_size", 5.0);
    resolution_ = this->declare_parameter<double>("map/resolution", 0.1);
    map_frame_ = this->declare_parameter<std::string>("map_frame", kDefaultMapFrame);
    camera_frame_ = this->declare_parameter<std::string>("camera_frame", kDefaultCameraFrame);

    inv_resolution_ = (resolution_ > 1e-6) ? 1.0 / resolution_ : 0.0;
    gl_xl_ = -x_size_ / 2.0;
    gl_yl_ = -y_size_ / 2.0;
    gl_zl_ = 0.0;
    glx_size_ = static_cast<int>(x_size_ * inv_resolution_);
    gly_size_ = static_cast<int>(y_size_ * inv_resolution_);
    glz_size_ = static_cast<int>(z_size_ * inv_resolution_);

    cam02body_ << 0.0, 0.0, 1.0, 0.0,
                  -1.0, 0.0, 0.0, 0.0,
                  0.0, -1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;

    depth_render_.set_para(fx_, fy_, cx_, cy_, width_, height_);
    depth_buffer_.assign(width_ * height_, 0);

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", sensor_qos,
      std::bind(&PclRenderNode::handleGlobalPointCloud, this, std::placeholders::_1));
    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "local_map", sensor_qos,
      std::bind(&PclRenderNode::handleLocalPointCloud, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", sensor_qos,
      std::bind(&PclRenderNode::handleOdometry, this, std::placeholders::_1));

    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("/pcl_render_node/depth", reliable_qos);
    pub_color_ = this->create_publisher<sensor_msgs::msg::Image>("colordepth", reliable_qos);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pcl_render_node/sensor_pose", reliable_qos);
    pub_pcl_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rendered_pcl", 1);
    pub_caminfo_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", reliable_qos);

    if (sensing_rate_ > 1e-3)
    {
      auto duration = std::chrono::duration<double>(1.0 / sensing_rate_);
      local_sensing_timer_ = this->create_wall_timer(duration, std::bind(&PclRenderNode::renderSensedPoints, this));
    }
    else
    {
      RCLCPP_WARN(get_logger(), "sensing_rate is too small; render timer disabled");
    }

    if (estimation_rate_ > 1e-3)
    {
      auto duration = std::chrono::duration<double>(1.0 / estimation_rate_);
      estimation_timer_ = this->create_wall_timer(duration, std::bind(&PclRenderNode::publishCameraPose, this));
    }
    else
    {
      RCLCPP_WARN(get_logger(), "estimation_rate is too small; pose timer disabled");
    }
  }

private:
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    has_odom_ = true;
    odom_ = *msg;

    Matrix4d pose = Matrix4d::Identity();
    Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation;
    orientation.x() = msg->pose.pose.orientation.x;
    orientation.y() = msg->pose.pose.orientation.y;
    orientation.z() = msg->pose.pose.orientation.z;
    orientation.w() = msg->pose.pose.orientation.w;
    pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    pose(0, 3) = position.x();
    pose(1, 3) = position.y();
    pose(2, 3) = position.z();

    cam2world_ = pose * cam02body_;
    cam2world_quat_ = Eigen::Quaterniond(cam2world_.block<3, 3>(0, 0));

    last_pose_world_ = position;
    last_odom_stamp_ = msg->header.stamp;
  }

  void handleGlobalPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (has_global_map_)
    {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    cloud_data_.reserve(cloud.points.size() * 3);
    for (const auto & pt : cloud.points)
    {
      cloud_data_.push_back(pt.x);
      cloud_data_.push_back(pt.y);
      cloud_data_.push_back(pt.z);
    }

    depth_render_.set_data(cloud_data_);
    depth_buffer_.assign(width_ * height_, 0);
    has_global_map_ = true;
    RCLCPP_INFO(get_logger(), "Global point cloud loaded with %zu points", cloud.points.size());
  }

  void handleLocalPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    if (cloud.empty())
    {
      return;
    }

    for (const auto & pt : cloud.points)
    {
      Eigen::Vector3d pose_pt(pt.x, pt.y, pt.z);
      cloud_data_.push_back(pose_pt(0));
      cloud_data_.push_back(pose_pt(1));
      cloud_data_.push_back(pose_pt(2));
    }

    depth_render_.set_data(cloud_data_);
    depth_buffer_.assign(width_ * height_, 0);
    has_local_map_ = true;
  }

  void publishCameraPose()
  {
    if (!has_odom_)
    {
      return;
    }

    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = odom_.header;
    camera_pose.header.frame_id = map_frame_;
    camera_pose.pose.position.x = cam2world_(0, 3);
    camera_pose.pose.position.y = cam2world_(1, 3);
    camera_pose.pose.position.z = cam2world_(2, 3);
    camera_pose.pose.orientation.w = cam2world_quat_.w();
    camera_pose.pose.orientation.x = cam2world_quat_.x();
    camera_pose.pose.orientation.y = cam2world_quat_.y();
    camera_pose.pose.orientation.z = cam2world_quat_.z();
    pub_pose_->publish(camera_pose);
  }

  void renderSensedPoints()
  {
    if ((!has_global_map_ && !has_local_map_) || !has_odom_)
    {
      return;
    }

    renderCurrentPose();
    renderPointCloud();
  }

  void renderCurrentPose()
  {
    if (depth_buffer_.empty())
    {
      depth_buffer_.assign(width_ * height_, 0);
    }

    Matrix4d cam_pose = cam2world_.inverse();
    double pose[16];
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        pose[j + 4 * i] = cam_pose(i, j);
      }
    }

    depth_render_.render_pose(pose, depth_buffer_.data());

    depth_mat_ = cv::Mat::zeros(height_, width_, CV_32FC1);
    double min_depth = 0.5;
    double max_depth = 1.0;
    for (int row = 0; row < height_; ++row)
    {
      for (int col = 0; col < width_; ++col)
      {
        float depth = static_cast<float>(depth_buffer_[row * width_ + col]) / 1000.0f;
        depth = depth < 500.0f ? depth : 0.0f;
        max_depth = depth > max_depth ? depth : max_depth;
        depth_mat_.at<float>(row, col) = depth;
      }
    }

    cv_bridge::CvImage depth_image;
    depth_image.header.stamp = last_odom_stamp_;
    depth_image.header.frame_id = camera_frame_;
    depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_image.image = depth_mat_.clone();
    pub_depth_->publish(*depth_image.toImageMsg());

    cv::Mat adj_map;
    depth_mat_.convertTo(adj_map, CV_8UC1, 255.0 / 13.0, -min_depth);
    cv::Mat false_colors_map;
    cv::applyColorMap(adj_map, false_colors_map, cv::COLORMAP_RAINBOW);

    cv_bridge::CvImage color_image;
    color_image.header.frame_id = "depthmap";
    color_image.header.stamp = last_odom_stamp_;
    color_image.encoding = sensor_msgs::image_encodings::BGR8;
    color_image.image = false_colors_map;
    pub_color_->publish(*color_image.toImageMsg());

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.stamp = last_odom_stamp_;
    camera_info.header.frame_id = camera_frame_;
    camera_info.width = width_;
    camera_info.height = height_;
    camera_info.k[0] = fx_;
    camera_info.k[4] = fy_;
    camera_info.k[2] = cx_;
    camera_info.k[5] = cy_;
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;
    pub_caminfo_->publish(camera_info);
  }

  void renderPointCloud()
  {
    pcl::PointCloud<pcl::PointXYZ> local_map;

    Eigen::Vector4d pose_in_camera;
    Eigen::Vector4d pose_in_world;
    Vector3d pose_pt;

    for (int u = 0; u < width_; ++u)
    {
      for (int v = 0; v < height_; ++v)
      {
        float depth = depth_mat_.at<float>(v, u);
        if (depth == 0.0f)
        {
          continue;
        }

        pose_in_camera(0) = (u - cx_) * depth / fx_;
        pose_in_camera(1) = (v - cy_) * depth / fy_;
        pose_in_camera(2) = depth;
        pose_in_camera(3) = 1.0;

        pose_in_world = cam2world_ * pose_in_camera;

        if ((pose_in_world.head<3>() - last_pose_world_).norm() > sensing_horizon_)
        {
          continue;
        }

        pose_pt = pose_in_world.head<3>();
        pcl::PointXYZ pt;
        pt.x = pose_pt(0);
        pt.y = pose_pt(1);
        pt.z = pose_pt(2);
        local_map.points.push_back(pt);
      }
    }

    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

    pcl::toROSMsg(local_map, local_map_msg_);
    local_map_msg_.header.frame_id = map_frame_;
    local_map_msg_.header.stamp = last_odom_stamp_;
    pub_pcl_world_->publish(local_map_msg_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_world_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_caminfo_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::TimerBase::SharedPtr local_sensing_timer_;
  rclcpp::TimerBase::SharedPtr estimation_timer_;

  sensor_msgs::msg::PointCloud2 local_map_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  Matrix4d cam02body_;
  Matrix4d cam2world_;
  Eigen::Quaterniond cam2world_quat_;
  Vector3d last_pose_world_;
  nav_msgs::msg::Odometry odom_;
  builtin_interfaces::msg::Time last_odom_stamp_;

  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double sensing_horizon_;
  double sensing_rate_;
  double estimation_rate_;
  double x_size_;
  double y_size_;
  double z_size_;
  double resolution_;
  double inv_resolution_;
  double gl_xl_;
  double gl_yl_;
  double gl_zl_;
  int glx_size_;
  int gly_size_;
  int glz_size_;

  bool has_global_map_ = false;
  bool has_local_map_ = false;
  bool has_odom_ = false;

  std::string map_frame_;
  std::string camera_frame_;

  DepthRender depth_render_;
  cv::Mat depth_mat_;
  std::vector<float> cloud_data_;
  std::vector<int> depth_buffer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclRenderNode>());
  rclcpp::shutdown();
  return 0;
}
