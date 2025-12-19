#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

class RandomForestSensing : public rclcpp::Node {
public:
  RandomForestSensing() : Node("random_map_sensing") {
    // Publishers
    _local_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/local_cloud", 1);
    _all_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", 1);
    click_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcl_render_node/local_map", 1);

    // Subscribers
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 50,
        std::bind(&RandomForestSensing::rcvOdometryCallbck, this,
                  std::placeholders::_1));

    auto click_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal", 10,
        std::bind(&RandomForestSensing::clickCallback, this,
                  std::placeholders::_1));

    // Parameters
    this->declare_parameter("init_state_x", 0.0);
    this->declare_parameter("init_state_y", 0.0);
    this->declare_parameter("map/x_size", 50.0);
    this->declare_parameter("map/y_size", 50.0);
    this->declare_parameter("map/z_size", 5.0);
    this->declare_parameter("map/obs_num", 30);
    this->declare_parameter("map/resolution", 0.1);
    this->declare_parameter("map/circle_num", 30);
    this->declare_parameter("ObstacleShape/lower_rad", 0.3);
    this->declare_parameter("ObstacleShape/upper_rad", 0.8);
    this->declare_parameter("ObstacleShape/lower_hei", 3.0);
    this->declare_parameter("ObstacleShape/upper_hei", 7.0);
    this->declare_parameter("ObstacleShape/radius_l", 7.0);
    this->declare_parameter("ObstacleShape/radius_h", 7.0);
    this->declare_parameter("ObstacleShape/z_l", 7.0);
    this->declare_parameter("ObstacleShape/z_h", 7.0);
    this->declare_parameter("ObstacleShape/theta", 7.0);
    this->declare_parameter("sensing/radius", 10.0);
    this->declare_parameter("sensing/rate", 10.0);
    this->declare_parameter("ObstacleShape/seed", -1);

    _init_x = this->get_parameter("init_state_x").as_double();
    _init_y = this->get_parameter("init_state_y").as_double();
    _x_size = this->get_parameter("map/x_size").as_double();
    _y_size = this->get_parameter("map/y_size").as_double();
    _z_size = this->get_parameter("map/z_size").as_double();
    _obs_num = this->get_parameter("map/obs_num").as_int();
    _resolution = this->get_parameter("map/resolution").as_double();
    circle_num_ = this->get_parameter("map/circle_num").as_int();
    _w_l = this->get_parameter("ObstacleShape/lower_rad").as_double();
    _w_h = this->get_parameter("ObstacleShape/upper_rad").as_double();
    _h_l = this->get_parameter("ObstacleShape/lower_hei").as_double();
    _h_h = this->get_parameter("ObstacleShape/upper_hei").as_double();
    radius_l_ = this->get_parameter("ObstacleShape/radius_l").as_double();
    radius_h_ = this->get_parameter("ObstacleShape/radius_h").as_double();
    z_l_ = this->get_parameter("ObstacleShape/z_l").as_double();
    z_h_ = this->get_parameter("ObstacleShape/z_h").as_double();
    theta_ = this->get_parameter("ObstacleShape/theta").as_double();
    _sensing_range = this->get_parameter("sensing/radius").as_double();
    _sense_rate = this->get_parameter("sensing/rate").as_double();

    int seed = this->get_parameter("ObstacleShape/seed").as_int();

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;
    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;
    _obs_num = min(_obs_num, (int)_x_size * 10);
    _z_limit = _z_size;

    RCLCPP_INFO(this->get_logger(), "Starting map generation with seed %d", seed);

    // init random device
    if (seed < 0) {
      seed = rd_() % INT32_MAX;
    }
    seed_ = seed;

    eng_ = default_random_engine(seed);

    RandomMapGenerate();

    RCLCPP_INFO(this->get_logger(), "Map generation complete. Cloud size: %zu", cloudMap_.points.size());

    // Create timer for periodic sensing
    auto period = std::chrono::duration<double>(1.0 / _sense_rate);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RandomForestSensing::pubSensedPoints, this));
  }

private:
  void RandomMapGenerate() {
    pcl::PointXYZ pt_random;

    rand_x_ = uniform_real_distribution<double>(_x_l, _x_h);
    rand_y_ = uniform_real_distribution<double>(_y_l, _y_h);
    rand_w_ = uniform_real_distribution<double>(_w_l, _w_h);
    rand_h_ = uniform_real_distribution<double>(_h_l, _h_h);

    rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num; i++) {
      double x, y, w, h;
      x = rand_x_(eng_);
      y = rand_y_(eng_);
      w = rand_w_(eng_);

      if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
        i--;
        continue;
      }

      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w / _resolution);

      for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
          h = rand_h_(eng_);
          int heiNum = ceil(h / _resolution);
          for (int t = -30; t < heiNum; t++) {
            pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
            pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
            pt_random.z = (t + 0.5) * _resolution + 1e-2;
            cloudMap_.points.push_back(pt_random);
          }
        }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
      double x, y, z;
      x = rand_x_(eng_);
      y = rand_y_(eng_);
      z = rand_z_(eng_);

      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;

      Eigen::Vector3d translate(x, y, z);

      double theta = rand_theta_(eng_);
      Eigen::Matrix3d rotate;
      rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
          1;

      double radius1 = rand_radius_(eng_);
      double radius2 = rand_radius2_(eng_);

      // draw a circle centered at (x,y,z)
      Eigen::Vector3d cpt;
      for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
        cpt(0) = 0.0;
        cpt(1) = radius1 * cos(angle);
        cpt(2) = radius2 * sin(angle);

        // inflate
        Eigen::Vector3d cpt_if;
        for (int ifx = -0; ifx <= 0; ++ifx)
          for (int ify = -0; ify <= 0; ++ify)
            for (int ifz = -0; ifz <= 0; ++ifz) {
              cpt_if =
                  cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                        ifz * _resolution);
              cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
              pt_random.x = cpt_if(0);
              pt_random.y = cpt_if(1);
              pt_random.z = cpt_if(2);
              cloudMap_.points.push_back(pt_random);
            }
      }
    }

    // add ground
    for (double x = _x_l; x < _x_h; x += _resolution) {
      for (double y = _y_l; y < _y_h; y += _resolution) {
        pt_random.x = x;
        pt_random.y = y;
        pt_random.z = -0.1;
        cloudMap_.push_back(pt_random);
      }
    }

    cloudMap_.width = cloudMap_.points.size();
    cloudMap_.height = 1;
    cloudMap_.is_dense = true;

    RCLCPP_WARN(this->get_logger(), "Finished generate random map");

    kdtreeLocalMap_.setInputCloud(cloudMap_.makeShared());

    _map_ok = true;
  }

  void rcvOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom) {
    if (odom->child_frame_id == "X" || odom->child_frame_id == "O")
      return;
    _has_odom = true;

    _state = {odom->pose.pose.position.x,
              odom->pose.pose.position.y,
              odom->pose.pose.position.z,
              odom->twist.twist.linear.x,
              odom->twist.twist.linear.y,
              odom->twist.twist.linear.z,
              0.0,
              0.0,
              0.0};
  }

  void pubSensedPoints() {
    pcl::toROSMsg(cloudMap_, globalMap_pcd_);
    globalMap_pcd_.header.frame_id = "world";
    globalMap_pcd_.header.stamp = this->now();
    _all_map_pub->publish(globalMap_pcd_);

    // Throttled warning every 2 seconds
    auto now = this->now();
    if ((now - last_warn_time_).seconds() >= 2.0) {
      RCLCPP_WARN(this->get_logger(), "seed: %d", seed_);
      last_warn_time_ = now;
    }

    return;

    /* ---------- only publish points around current position ---------- */
    if (!_map_ok || !_has_odom)
      return;

    pcl::PointCloud<pcl::PointXYZ> localMap;

    pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
    pointIdxRadiusSearch_.clear();
    pointRadiusSquaredDistance_.clear();

    pcl::PointXYZ pt;

    if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
      return;

    if (kdtreeLocalMap_.radiusSearch(searchPoint, _sensing_range,
                                     pointIdxRadiusSearch_,
                                     pointRadiusSquaredDistance_) > 0) {
      for (size_t i = 0; i < pointIdxRadiusSearch_.size(); ++i) {
        pt = cloudMap_.points[pointIdxRadiusSearch_[i]];
        localMap.points.push_back(pt);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "[Map server] No obstacles .");
      return;
    }

    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    sensor_msgs::msg::PointCloud2 localMap_pcd;
    pcl::toROSMsg(localMap, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    localMap_pcd.header.stamp = this->now();
    _local_map_pub->publish(localMap_pcd);
  }

  void clickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double w = rand_w_(eng_);
    double h;
    pcl::PointXYZ pt_random;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h_(eng_);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          clicked_cloud_.points.push_back(pt_random);
          cloudMap_.points.push_back(pt_random);
        }
      }
    clicked_cloud_.width = clicked_cloud_.points.size();
    clicked_cloud_.height = 1;
    clicked_cloud_.is_dense = true;

    sensor_msgs::msg::PointCloud2 localMap_pcd;
    pcl::toROSMsg(clicked_cloud_, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    localMap_pcd.header.stamp = this->now();
    click_map_pub_->publish(localMap_pcd);

    cloudMap_.width = cloudMap_.points.size();

    return;
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _local_map_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr click_map_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // PCL data structures
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap_;
  vector<int> pointIdxRadiusSearch_;
  vector<float> pointRadiusSquaredDistance_;

  // Random generators
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_x_;
  uniform_real_distribution<double> rand_y_;
  uniform_real_distribution<double> rand_w_;
  uniform_real_distribution<double> rand_h_;
  uniform_real_distribution<double> rand_radius_;
  uniform_real_distribution<double> rand_radius2_;
  uniform_real_distribution<double> rand_theta_;
  uniform_real_distribution<double> rand_z_;

  // State
  vector<double> _state;
  bool _map_ok = false;
  bool _has_odom = false;
  int seed_;

  // Parameters
  int _obs_num;
  double _x_size, _y_size, _z_size;
  double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
  double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
  int circle_num_;
  double radius_l_, radius_h_, z_l_, z_h_;
  double theta_;

  // Point clouds
  sensor_msgs::msg::PointCloud2 globalMap_pcd_;
  pcl::PointCloud<pcl::PointXYZ> cloudMap_;
  pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

  // For throttling
  rclcpp::Time last_warn_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomForestSensing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("random_forest"), "Exception: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("random_forest"), "Unknown exception");
    return 1;
  }
  return 0;
}
