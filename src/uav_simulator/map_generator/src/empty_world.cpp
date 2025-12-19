#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

// Constants for default parameters
constexpr double DEFAULT_MAP_X_SIZE = 50.0;
constexpr double DEFAULT_MAP_Y_SIZE = 50.0;
constexpr double DEFAULT_MAP_RESOLUTION = 0.1;
constexpr double DEFAULT_SENSING_RATE = 10.0;
constexpr double GROUND_Z_LEVEL = -0.1;

class EmptyWorldSensing : public rclcpp::Node {
public:
  EmptyWorldSensing() : Node("random_map_sensing") {
    // Publishers
    _local_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/local_cloud", rclcpp::QoS(1));
    _all_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", rclcpp::QoS(1));

    // Subscribers
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(50),
        std::bind(&EmptyWorldSensing::rcvOdometryCallbck, this,
                  std::placeholders::_1));

    // Parameters
    this->declare_parameter("map/x_size", DEFAULT_MAP_X_SIZE);
    this->declare_parameter("map/y_size", DEFAULT_MAP_Y_SIZE);
    this->declare_parameter("map/resolution", DEFAULT_MAP_RESOLUTION);
    this->declare_parameter("sensing/rate", DEFAULT_SENSING_RATE);

    _x_size = this->get_parameter("map/x_size").as_double();
    _y_size = this->get_parameter("map/y_size").as_double();
    _resolution = this->get_parameter("map/resolution").as_double();
    _sense_rate = this->get_parameter("sensing/rate").as_double();

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;
    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    GenerateMap();

    // Create timer for periodic sensing
    auto period = std::chrono::duration<double>(1.0 / _sense_rate);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&EmptyWorldSensing::pubSensedPoints, this));
  }

private:
  void GenerateMap() {
    pcl::PointXYZ pt_random;

    // add ground
    for (double x_pos = _x_l; x_pos < _x_h; x_pos += _resolution) {
      for (double y_pos = _y_l; y_pos < _y_h; y_pos += _resolution) {
        pt_random.x = x_pos;
        pt_random.y = y_pos;
        pt_random.z = GROUND_Z_LEVEL;
        cloudMap_.push_back(pt_random);
      }
    }

    cloudMap_.width = cloudMap_.points.size();
    cloudMap_.height = 1;
    cloudMap_.is_dense = true;

    RCLCPP_WARN(this->get_logger(),
                "Finished generate empty map (ground only)");

    _map_ok = true;
  }

  void rcvOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom) {
    if (odom->child_frame_id == "X" || odom->child_frame_id == "O")
      return;
    _has_odom = true;
  }

  void pubSensedPoints() {
    pcl::toROSMsg(cloudMap_, globalMap_pcd_);
    globalMap_pcd_.header.frame_id = "world";
    globalMap_pcd_.header.stamp = this->now();
    _all_map_pub->publish(globalMap_pcd_);

    // Also publish to local map for compatibility
    _local_map_pub->publish(globalMap_pcd_);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _local_map_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool _map_ok = false;
  bool _has_odom = false;

  // Parameters
  double _x_size, _y_size;
  double _x_l, _x_h, _y_l, _y_h;
  double _resolution, _sense_rate;

  // Point clouds
  sensor_msgs::msg::PointCloud2 globalMap_pcd_;
  pcl::PointCloud<pcl::PointXYZ> cloudMap_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmptyWorldSensing>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
