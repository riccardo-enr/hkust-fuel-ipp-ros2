#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <sstream>
#include <eigen3/Eigen/Dense>

using namespace std;

// Helper function to get yaw from quaternion
double getYaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

class WaypointGenerator : public rclcpp::Node {
public:
  WaypointGenerator() : Node("waypoint_generator") {
    this->declare_parameter("waypoint_type", "manual");
    waypoint_type_ = this->get_parameter("waypoint_type").as_string();

    pub1_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
    pub2_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

    sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&WaypointGenerator::odom_callback, this, std::placeholders::_1));
    sub2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 10, std::bind(&WaypointGenerator::goal_callback, this, std::placeholders::_1));
    sub3_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "traj_start_trigger", 10, std::bind(&WaypointGenerator::traj_start_trigger_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waypoint generator initialized with type: %s", waypoint_type_.c_str());
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3_;

  string waypoint_type_ = "manual";
  bool is_odom_ready_ = false;
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path waypoints_;
  std::deque<nav_msgs::msg::Path> waypointSegments_;
  rclcpp::Time trigged_time_;

  void load_seg(int segid, const rclcpp::Time& time_base) {
    std::string seg_str = "seg" + std::to_string(segid) + ".";
    double yaw;
    double time_from_start;
    RCLCPP_INFO(this->get_logger(), "Getting segment %d", segid);

    if (!this->get_parameter(seg_str + "yaw", yaw)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameter: %syaw", seg_str.c_str());
      return;
    }
    if (yaw <= -3.1499999 || yaw >= 3.14999999) {
      RCLCPP_ERROR(this->get_logger(), "yaw=%.3f out of range", yaw);
      return;
    }
    if (!this->get_parameter(seg_str + "time_from_start", time_from_start)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameter: %stime_from_start", seg_str.c_str());
      return;
    }
    if (time_from_start < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "time_from_start must be >= 0.0");
      return;
    }

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    if (!this->get_parameter(seg_str + "x", ptx) ||
        !this->get_parameter(seg_str + "y", pty) ||
        !this->get_parameter(seg_str + "z", ptz)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get waypoint coordinates for segment %d", segid);
      return;
    }

    if (ptx.empty() || ptx.size() != pty.size() || ptx.size() != ptz.size()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid waypoint coordinates for segment %d", segid);
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = rclcpp::Time(time_base.nanoseconds() + static_cast<int64_t>(time_from_start * 1e9));

    double baseyaw = getYaw(odom_.pose.pose.orientation);

    for (size_t k = 0; k < ptx.size(); ++k) {
      geometry_msgs::msg::PoseStamped pt;
      pt.pose.orientation = createQuaternionMsgFromYaw(baseyaw + yaw);
      Eigen::Vector2d dp(ptx.at(k), pty.at(k));
      Eigen::Vector2d rdp;
      rdp.x() = std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
      rdp.y() = -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();
      pt.pose.position.x = rdp.x() + odom_.pose.pose.position.x;
      pt.pose.position.y = rdp.y() + odom_.pose.pose.position.y;
      pt.pose.position.z = ptz.at(k) + odom_.pose.pose.position.z;
      path_msg.poses.push_back(pt);
    }

    waypointSegments_.push_back(path_msg);
  }

  void load_waypoints(const rclcpp::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments_.clear();

    if (!this->get_parameter("segment_cnt", seg_cnt)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameter: segment_cnt");
      return;
    }

    for (int i = 0; i < seg_cnt; ++i) {
      load_seg(i, time_base);
      if (i > 0 && waypointSegments_.size() >= 2) {
        rclcpp::Time t1(waypointSegments_[i - 1].header.stamp);
        rclcpp::Time t2(waypointSegments_[i].header.stamp);
        if (t1 >= t2) {
          RCLCPP_ERROR(this->get_logger(), "Segment timestamps must be strictly increasing");
          return;
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Overall loaded %zu segments", waypointSegments_.size());
  }

  void publish_waypoints() {
    waypoints_.header.frame_id = "world";
    waypoints_.header.stamp = this->now();
    pub1_->publish(waypoints_);
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom_.header;
    init_pose.pose = odom_.pose.pose;
    waypoints_.poses.insert(waypoints_.poses.begin(), init_pose);
    waypoints_.poses.clear();
  }

  void publish_waypoints_vis() {
    nav_msgs::msg::Path wp_vis = waypoints_;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = "world";
    poseArray.header.stamp = this->now();

    {
      geometry_msgs::msg::Pose init_pose;
      init_pose = odom_.pose.pose;
      poseArray.poses.push_back(init_pose);
    }

    for (auto it = waypoints_.poses.begin(); it != waypoints_.poses.end(); ++it) {
      geometry_msgs::msg::Pose p;
      p = it->pose;
      poseArray.poses.push_back(p);
    }
    pub2_->publish(poseArray);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    is_odom_ready_ = true;
    odom_ = *msg;

    if (waypointSegments_.size()) {
      rclcpp::Time expected_time = waypointSegments_.front().header.stamp;
      if (rclcpp::Time(odom_.header.stamp) >= expected_time) {
        waypoints_ = waypointSegments_.front();

        std::stringstream ss;
        ss << "Series send " << (this->now() - trigged_time_).seconds() << " from start:\n";
        for (auto& pose_stamped : waypoints_.poses) {
          ss << "P[" << pose_stamped.pose.position.x << ", "
             << pose_stamped.pose.position.y << ", "
             << pose_stamped.pose.position.z << "] q("
             << pose_stamped.pose.orientation.w << ","
             << pose_stamped.pose.orientation.x << ","
             << pose_stamped.pose.orientation.y << ","
             << pose_stamped.pose.orientation.z << ")\n";
        }
        RCLCPP_INFO_STREAM(this->get_logger(), ss.str());

        publish_waypoints_vis();
        publish_waypoints();

        waypointSegments_.pop_front();
      }
    }
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    trigged_time_ = this->now();

    this->get_parameter("waypoint_type", waypoint_type_);

    if (waypoint_type_ == "circle") {
      waypoints_ = circle();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "eight") {
      waypoints_ = eight();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "point") {
      waypoints_ = point();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "series") {
      load_waypoints(trigged_time_);
    } else if (waypoint_type_ == "manual-lonely-waypoint") {
      if (msg->pose.position.z > -0.1) {
        geometry_msgs::msg::PoseStamped pt = *msg;
        waypoints_.poses.clear();
        waypoints_.poses.push_back(pt);
        publish_waypoints_vis();
        publish_waypoints();
      } else {
        RCLCPP_WARN(this->get_logger(), "invalid goal in manual-lonely-waypoint mode.");
      }
    } else {
      if (msg->pose.position.z > 0) {
        geometry_msgs::msg::PoseStamped pt = *msg;
        if (waypoint_type_ == "noyaw") {
          double yaw = getYaw(odom_.pose.pose.orientation);
          pt.pose.orientation = createQuaternionMsgFromYaw(yaw);
        }
        waypoints_.poses.push_back(pt);
        publish_waypoints_vis();
      } else if (msg->pose.position.z > -1.0) {
        if (waypoints_.poses.size() >= 1) {
          waypoints_.poses.erase(std::prev(waypoints_.poses.end()));
        }
        publish_waypoints_vis();
      } else {
        if (waypoints_.poses.size() >= 1) {
          publish_waypoints_vis();
          publish_waypoints();
        }
      }
    }
  }

  void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!is_odom_ready_) {
      RCLCPP_ERROR(this->get_logger(), "No odom!");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Trigger!");
    trigged_time_ = rclcpp::Time(odom_.header.stamp);

    if (trigged_time_.nanoseconds() <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid trigger time");
      return;
    }

    this->get_parameter("waypoint_type", waypoint_type_);

    RCLCPP_ERROR_STREAM(this->get_logger(), "Pattern " << waypoint_type_ << " generated!");
    if (waypoint_type_ == "free") {
      waypoints_ = point();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "circle") {
      waypoints_ = circle();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "eight") {
      waypoints_ = eight();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "point") {
      waypoints_ = point();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "series") {
      load_waypoints(trigged_time_);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointGenerator>());
  rclcpp::shutdown();
  return 0;
}
