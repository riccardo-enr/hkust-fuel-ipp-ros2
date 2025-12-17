#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "depth_render.cuh"

using Eigen::Matrix3d;
using Eigen::Matrix4d;

class EurocBenchmarkNode : public rclcpp::Node {
public:
  EurocBenchmarkNode();
  ~EurocBenchmarkNode() override = default;

private:
  struct PoseInfo {
    Matrix4d pose;
    double time;
  };

  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                       geometry_msgs::msg::TransformStamped>;

  void declareAndLoadParameters();
  void loadPointCloud(const std::string& path);
  void loadGroundTruth(const std::string& path);
  void setupSubscribers();
  void imagePoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_input,
                         const geometry_msgs::msg::TransformStamped::ConstSharedPtr& pose_input);
  void renderCurrentPose();
  void solvePnp();
  static void imageMouseCallback(int event, int x, int y, int flags, void* userdata);
  static void depthMouseCallback(int event, int x, int y, int flags, void* userdata);
  void handleImageClick(int x, int y);
  void handleDepthClick(int x, int y);

  DepthRender depthrender_;
  std::vector<int> depth_host_buffer_;
  std::vector<float> cloud_data_;

  bool is_distorted_;
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  std::string image_topic_;
  std::string pose_topic_;

  Matrix4d vicon2body_;
  Matrix4d cam02body_;
  Matrix4d cam2world_;
  Matrix4d vicon2leica_;

  cv::Mat cv_K_;
  cv::Mat cv_D_;
  cv::Mat undist_map1_;
  cv::Mat undist_map2_;
  cv::Mat undistorted_image_;
  cv::Mat depth_mat_;

  builtin_interfaces::msg::Time receive_stamp_;

  std::vector<PoseInfo> gt_pose_vect_;
  std::vector<cv::Point3f> pts_3_;
  std::vector<cv::Point2f> pts_2_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<geometry_msgs::msg::TransformStamped> pose_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
};

EurocBenchmarkNode::EurocBenchmarkNode()
  : rclcpp::Node("cloud_benchmark"),
    is_distorted_(false),
    width_(0),
    height_(0),
    fx_(0.0),
    fy_(0.0),
    cx_(0.0),
    cy_(0.0),
    vicon2body_(Matrix4d::Identity()),
    cam02body_(Matrix4d::Identity()),
    cam2world_(Matrix4d::Identity()),
    vicon2leica_(Matrix4d::Identity()) {
  declareAndLoadParameters();
  pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("depth", rclcpp::QoS(10));
  pub_color_ = this->create_publisher<sensor_msgs::msg::Image>("colordepth", rclcpp::QoS(10));
  setupSubscribers();

  cv::namedWindow("bluefox_image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("depth_image", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("bluefox_image", &EurocBenchmarkNode::imageMouseCallback, this);
  cv::setMouseCallback("depth_image", &EurocBenchmarkNode::depthMouseCallback, this);
}

void EurocBenchmarkNode::declareAndLoadParameters() {
  width_ = this->declare_parameter<int>("cam_width", 640);
  height_ = this->declare_parameter<int>("cam_height", 480);
  fx_ = this->declare_parameter<double>("cam_fx", 387.229248046875);
  fy_ = this->declare_parameter<double>("cam_fy", 387.229248046875);
  cx_ = this->declare_parameter<double>("cam_cx", 321.04638671875);
  cy_ = this->declare_parameter<double>("cam_cy", 243.44969177246094);

  double k1 = this->declare_parameter<double>("cam_k1", 0.0);
  double k2 = this->declare_parameter<double>("cam_k2", 0.0);
  double r1 = this->declare_parameter<double>("cam_r1", 0.0);
  double r2 = this->declare_parameter<double>("cam_r2", 0.0);

  image_topic_ = this->declare_parameter<std::string>("image_topic", "/cam0/image_raw");
  pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/vicon/firefly_sbx/firefly_sbx");
  auto cloud_path = this->declare_parameter<std::string>("cloud_path", "");
  auto groundtruth_path =
      this->declare_parameter<std::string>("groundtruth_path", "/home/denny/Downloads/wkx_bag/data.txt");

  if (cloud_path.empty()) {
    throw std::runtime_error("Parameter 'cloud_path' must be set for euroc benchmark node.");
  }

  depthrender_.set_para(static_cast<float>(fx_), static_cast<float>(fy_), static_cast<float>(cx_),
                        static_cast<float>(cy_), width_, height_);
  loadPointCloud(cloud_path);
  loadGroundTruth(groundtruth_path);

  depth_host_buffer_.assign(width_ * height_, 0);
  undistorted_image_.create(height_, width_, CV_8UC1);
  depth_mat_ = cv::Mat::zeros(height_, width_, CV_32FC1);

  cv_K_ = (cv::Mat_<float>(3, 3) << static_cast<float>(fx_), 0.0f, static_cast<float>(cx_), 0.0f,
           static_cast<float>(fy_), static_cast<float>(cy_), 0.0f, 0.0f, 1.0f);

  if (std::fabs(k1) > 1e-6 || std::fabs(k2) > 1e-6 || std::fabs(r1) > 1e-6 || std::fabs(r2) > 1e-6) {
    cv_D_ = (cv::Mat_<float>(1, 4) << static_cast<float>(k1), static_cast<float>(k2), static_cast<float>(r1),
             static_cast<float>(r2));
    cv::initUndistortRectifyMap(cv_K_, cv_D_, cv::Mat_<double>::eye(3, 3), cv_K_, cv::Size(width_, height_),
                                CV_16SC2, undist_map1_, undist_map2_);
    is_distorted_ = true;
  } else {
    is_distorted_ = false;
  }

  vicon2body_ << 0.33638, -0.01749, 0.94156, 0.06901, -0.02078, -0.99972, -0.01114, -0.02781, 0.94150, -0.01582,
      -0.33665, -0.12395, 0.0, 0.0, 0.0, 1.0;
  cam02body_ << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008,
      0.0149672133247, 0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178,
      0.00981073058949, 0.0, 0.0, 0.0, 1.0;
  cam2world_ = Matrix4d::Identity();
  vicon2leica_ = Matrix4d::Identity();
}

void EurocBenchmarkNode::loadPointCloud(const std::string& path) {
  std::ifstream data_file(path);
  if (!data_file.is_open()) {
    throw std::runtime_error("Failed to open cloud_path: " + path);
  }

  double x, y, z, intensity, r, g, b;
  while (data_file >> x >> y >> z >> intensity >> r >> g >> b) {
    cloud_data_.push_back(static_cast<float>(x));
    cloud_data_.push_back(static_cast<float>(y));
    cloud_data_.push_back(static_cast<float>(z));
  }

  if (cloud_data_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud %s is empty.", path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Loaded %zu points from %s.", cloud_data_.size() / 3, path.c_str());
  }

  depthrender_.set_data(cloud_data_);
}

void EurocBenchmarkNode::loadGroundTruth(const std::string& path) {
  std::ifstream gt_file(path);
  if (!gt_file.is_open()) {
    throw std::runtime_error("Failed to open groundtruth_path: " + path);
  }

  double values[8];
  while (gt_file >> values[0]) {
    for (int i = 1; i < 8; ++i) {
      if (!(gt_file >> values[i])) {
        RCLCPP_WARN(this->get_logger(), "Ground truth file truncated, stopping read.");
        break;
      }
    }

    Eigen::Vector3d position(values[1], values[2], values[3]);
    Eigen::Quaterniond orientation(values[4], values[5], values[6], values[7]);

    PoseInfo info;
    info.time = values[0];
    info.pose = Matrix4d::Identity();
    info.pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    info.pose(0, 3) = position.x();
    info.pose(1, 3) = position.y();
    info.pose(2, 3) = position.z();
    gt_pose_vect_.push_back(info);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from %s.", gt_pose_vect_.size(), path.c_str());
}

void EurocBenchmarkNode::setupSubscribers() {
  image_sub_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
  pose_sub_.subscribe(this, pose_topic_, rmw_qos_profile_sensor_data);
  sync_ =
      std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(100), image_sub_, pose_sub_);
  sync_->registerCallback(
      std::bind(&EurocBenchmarkNode::imagePoseCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void EurocBenchmarkNode::imagePoseCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_input,
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr& pose_input) {
  if (gt_pose_vect_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No ground-truth poses loaded.");
    return;
  }

  const double image_time = rclcpp::Time(image_input->header.stamp).seconds();
  double min_time_diff = std::numeric_limits<double>::max();
  size_t min_time_index = 0;

  for (size_t i = 0; i < gt_pose_vect_.size(); ++i) {
    const double diff = std::fabs(image_time - gt_pose_vect_[i].time);
    if (diff < min_time_diff) {
      min_time_diff = diff;
      min_time_index = i;
    }
  }

  Matrix4d pose_receive = gt_pose_vect_[min_time_index].pose;
  Matrix4d body_pose = pose_receive;
  cam2world_ = body_pose * cam02body_ * vicon2leica_;
  receive_stamp_ = pose_input->header.stamp;

  cv_bridge::CvImageConstPtr cv_img_ptr =
      cv_bridge::toCvShare(image_input, sensor_msgs::image_encodings::MONO8);
  cv::Mat mono = cv_img_ptr->image;
  if (is_distorted_) {
    cv::remap(mono, undistorted_image_, undist_map1_, undist_map2_, cv::INTER_LINEAR);
  } else {
    undistorted_image_ = mono;
  }

  renderCurrentPose();
}

void EurocBenchmarkNode::renderCurrentPose() {
  solvePnp();

  const double start_time = this->now().seconds();
  Matrix4d cam_pose = cam2world_.inverse();
  depthrender_.render_pose(cam_pose.data(), depth_host_buffer_.data());

  double min_depth = 0.5;
  double max_depth = 1.0;
  for (int i = 0; i < height_; ++i) {
    for (int j = 0; j < width_; ++j) {
      float depth = static_cast<float>(depth_host_buffer_[i * width_ + j]) / 1000.0f;
      depth = depth < 500.0f ? depth : 0.0f;
      max_depth = depth > max_depth ? depth : max_depth;
      depth_mat_.at<float>(i, j) = depth;
    }
  }

  const double elapsed_ms = (this->now().seconds() - start_time) * 1000.0;
  RCLCPP_INFO(this->get_logger(), "render cost %.2f ms.", elapsed_ms);
  RCLCPP_INFO(this->get_logger(), "max_depth %.2f.", max_depth);

  cv_bridge::CvImage depth_msg;
  depth_msg.header.stamp = receive_stamp_;
  depth_msg.header.frame_id = "depthmap";
  depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg.image = depth_mat_.clone();
  pub_depth_->publish(*depth_msg.toImageMsg());

  cv::Mat adj_map;
  const double range = std::max(1e-3, max_depth - min_depth);
  depth_mat_.convertTo(adj_map, CV_8UC1, 255.0 / range, -min_depth * 255.0 / range);
  cv::Mat false_colors;
  cv::applyColorMap(adj_map, false_colors, cv::COLORMAP_RAINBOW);

  cv::Mat bgr_image;
  cv::cvtColor(undistorted_image_, bgr_image, cv::COLOR_GRAY2BGR);
  cv::addWeighted(bgr_image, 0.2, false_colors, 0.8, 0.0, false_colors);

  cv_bridge::CvImage color_msg;
  color_msg.header.frame_id = "depthmap";
  color_msg.header.stamp = receive_stamp_;
  color_msg.encoding = sensor_msgs::image_encodings::BGR8;
  color_msg.image = false_colors;
  pub_color_->publish(*color_msg.toImageMsg());

  cv::imshow("bluefox_image", bgr_image);
  cv::imshow("depth_image", adj_map);
  cv::waitKey(1);
}

void EurocBenchmarkNode::solvePnp() {
  if (pts_3_.size() < 5 || pts_2_.size() < 5 || pts_3_.size() != pts_2_.size()) {
    return;
  }

  cv::Mat rvec, tvec, rotation;
  cv::solvePnP(pts_3_, pts_2_, cv_K_, cv::Mat::zeros(4, 1, CV_32FC1), rvec, tvec);
  cv::Rodrigues(rvec, rotation);

  Matrix3d rotation_ref;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rotation_ref(i, j) = rotation.at<double>(i, j);
    }
  }

  Matrix4d pnp_result = Matrix4d::Identity();
  pnp_result.block<3, 3>(0, 0) = rotation_ref;
  pnp_result(0, 3) = tvec.at<double>(0, 0);
  pnp_result(1, 3) = tvec.at<double>(1, 0);
  pnp_result(2, 3) = tvec.at<double>(2, 0);

  vicon2leica_ = pnp_result.inverse();
}

void EurocBenchmarkNode::imageMouseCallback(int event, int x, int y, int, void* userdata) {
  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }
  auto* node = static_cast<EurocBenchmarkNode*>(userdata);
  node->handleImageClick(x, y);
}

void EurocBenchmarkNode::depthMouseCallback(int event, int x, int y, int, void* userdata) {
  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }
  auto* node = static_cast<EurocBenchmarkNode*>(userdata);
  node->handleDepthClick(x, y);
}

void EurocBenchmarkNode::handleImageClick(int x, int y) {
  RCLCPP_INFO(this->get_logger(), "image is clicked - position (%d, %d)", x, y);
  pts_2_.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
}

void EurocBenchmarkNode::handleDepthClick(int x, int y) {
  if (x < 0 || x >= depth_mat_.cols || y < 0 || y >= depth_mat_.rows) {
    return;
  }
  double depth = depth_mat_.at<float>(y, x);
  double space_x = (x - cx_) * depth / fx_;
  double space_y = (y - cy_) * depth / fy_;
  double space_z = depth;
  pts_3_.push_back(cv::Point3f(static_cast<float>(space_x), static_cast<float>(space_y),
                               static_cast<float>(space_z)));
  RCLCPP_INFO(this->get_logger(), "depth is clicked - position (%d, %d)", x, y);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EurocBenchmarkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
