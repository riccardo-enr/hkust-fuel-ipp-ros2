#include "armadillo"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pose_utils.h"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include <iostream>
#include <memory>
#include <string.h>

using namespace arma;
using namespace std;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

class OdomVisualization : public rclcpp::Node {
public:
  OdomVisualization() : Node("odom_visualization") {
    // Declare and get parameters
    this->declare_parameter(
        "mesh_resource",
        "package://odom_visualization/meshes/hummingbird.mesh");
    this->declare_parameter("color/r", 1.0);
    this->declare_parameter("color/g", 0.0);
    this->declare_parameter("color/b", 0.0);
    this->declare_parameter("color/a", 1.0);
    this->declare_parameter("cmd_color/r", 1.0);
    this->declare_parameter("cmd_color/g", 0.0);
    this->declare_parameter("cmd_color/b", 0.0);
    this->declare_parameter("cmd_color/a", 1.0);
    this->declare_parameter("origin", false);
    this->declare_parameter("robot_scale", 2.0);
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("cross_config", false);
    this->declare_parameter("tf45", false);
    this->declare_parameter("covariance_scale", 100.0);
    this->declare_parameter("covariance_position", false);
    this->declare_parameter("covariance_velocity", false);
    this->declare_parameter("covariance_color", false);

    mesh_resource_ = this->get_parameter("mesh_resource").as_string();
    color_r_ = this->get_parameter("color/r").as_double();
    color_g_ = this->get_parameter("color/g").as_double();
    color_b_ = this->get_parameter("color/b").as_double();
    color_a_ = this->get_parameter("color/a").as_double();
    cmd_color_r_ = this->get_parameter("cmd_color/r").as_double();
    cmd_color_g_ = this->get_parameter("cmd_color/g").as_double();
    cmd_color_b_ = this->get_parameter("cmd_color/b").as_double();
    cmd_color_a_ = this->get_parameter("cmd_color/a").as_double();
    origin_ = this->get_parameter("origin").as_bool();
    scale_ = this->get_parameter("robot_scale").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    cross_config_ = this->get_parameter("cross_config").as_bool();
    tf45_ = this->get_parameter("tf45").as_bool();
    cov_scale_ = this->get_parameter("covariance_scale").as_double();
    cov_pos_ = this->get_parameter("covariance_position").as_bool();
    cov_vel_ = this->get_parameter("covariance_velocity").as_bool();
    cov_color_ = this->get_parameter("covariance_color").as_bool();

    // Initialize members
    isOriginSet_ = false;
    poseOrigin_ = colvec(6);

    // Initialize time tracking variables
    prevt_ = this->now();
    prev_cmd_t_ = this->now();
    pt_ = this->now();
    ppose_ = zeros<colvec>(6);

    // Create publishers
    posePub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
    pathPub_ = this->create_publisher<nav_msgs::msg::Path>("path", 100);
    cmdPathPub_ = this->create_publisher<nav_msgs::msg::Path>("cmd_path", 100);
    velPub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "velocity", 100);
    covPub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "covariance", 100);
    covVelPub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "covariance_velocity", 100);
    trajPub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "trajectory", 100);
    sensorPub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("sensor", 100);
    meshPub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("robot", 100);
    cmdVisPub_ = 
        this->create_publisher<visualization_msgs::msg::Marker>("cmd_vis", 100);
    cmdPosePub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("cmd_pose", 100);
    heightPub_ = this->create_publisher<sensor_msgs::msg::Range>("height", 100);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 100,
        std::bind(&OdomVisualization::odom_callback, this,
                  std::placeholders::_1));
    sub_cmd_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "cmd", 100,
        std::bind(&OdomVisualization::cmd_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Odometry visualization node started");
  }

private:
  // Parameters
  string mesh_resource_;
  double color_r_, color_g_, color_b_, color_a_, cov_scale_, scale_;
  double cmd_color_r_, cmd_color_g_, cmd_color_b_, cmd_color_a_;
  bool cross_config_;
  bool tf45_;
  bool cov_pos_;
  bool cov_vel_;
  bool cov_color_;
  bool origin_;
  bool isOriginSet_;
  colvec poseOrigin_;
  string frame_id_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr cmdPathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covVelPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensorPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmdVisPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmdPosePub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr heightPub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr
      sub_cmd_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Message storage
  geometry_msgs::msg::PoseStamped poseROS_;
  nav_msgs::msg::Path pathROS_;
  nav_msgs::msg::Path cmdPathROS_;
  visualization_msgs::msg::Marker velROS_;
  visualization_msgs::msg::Marker covROS_;
  visualization_msgs::msg::Marker covVelROS_;
  visualization_msgs::msg::Marker trajROS_;
  visualization_msgs::msg::Marker sensorROS_;
  visualization_msgs::msg::Marker meshROS_;
  sensor_msgs::msg::Range heightROS_;

  // Time tracking
  rclcpp::Time prevt_;
  rclcpp::Time prev_cmd_t_;
  rclcpp::Time pt_;
  colvec ppose_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg->header.frame_id == string("null"))
      return;
    colvec pose(6);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    colvec q(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
    colvec vel(3);
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    if (origin_ && !isOriginSet_) {
      isOriginSet_ = true;
      poseOrigin_ = pose;
    }
    if (origin_) {
      vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
      pose = pose_update(pose_inverse(poseOrigin_), pose);
      vel = ypr_to_R(pose.rows(3, 5)) * vel;
    }

    // Pose
    poseROS_.header = msg->header;
    poseROS_.header.stamp = msg->header.stamp;
    poseROS_.header.frame_id = string("world");
    poseROS_.pose.position.x = pose(0);
    poseROS_.pose.position.y = pose(1);
    poseROS_.pose.position.z = pose(2);
    q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
    poseROS_.pose.orientation.w = q(0);
    poseROS_.pose.orientation.x = q(1);
    poseROS_.pose.orientation.y = q(2);
    poseROS_.pose.orientation.z = q(3);
    posePub_->publish(poseROS_);

    // Velocity
    colvec yprVel(3);
    yprVel(0) = atan2(vel(1), vel(0));
    yprVel(1) = -atan2(vel(2), norm(vel.rows(0, 1), 2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    velROS_.header.frame_id = string("world");
    velROS_.header.stamp = msg->header.stamp;
    velROS_.ns = string("velocity");
    velROS_.id = 0;
    velROS_.type = visualization_msgs::msg::Marker::ARROW;
    velROS_.action = visualization_msgs::msg::Marker::ADD;
    velROS_.pose.position.x = pose(0);
    velROS_.pose.position.y = pose(1);
    velROS_.pose.position.z = pose(2);
    velROS_.pose.orientation.w = q(0);
    velROS_.pose.orientation.x = q(1);
    velROS_.pose.orientation.y = q(2);
    velROS_.pose.orientation.z = q(3);
    velROS_.scale.x = norm(vel, 2);
    velROS_.scale.y = 0.05;
    velROS_.scale.z = 0.05;
    velROS_.color.a = 1.0;
    velROS_.color.r = color_r_;
    velROS_.color.g = color_g_;
    velROS_.color.b = color_b_;
    velPub_->publish(velROS_);

    // Path
    if ((rclcpp::Time(msg->header.stamp) - prevt_).seconds() > 0.1) {
      prevt_ = rclcpp::Time(msg->header.stamp);
      pathROS_.header = poseROS_.header;
      pathROS_.poses.push_back(poseROS_);
      pathPub_->publish(pathROS_);
    }

    // Covariance color
    double r = 1;
    double g = 1;
    double b = 1;
    bool G = msg->twist.covariance[33];
    bool V = msg->twist.covariance[34];
    bool L = msg->twist.covariance[35];
    if (cov_color_) {
      r = G;
      g = V;
      b = L;
    }

    // Covariance Position
    if (cov_pos_) {
      mat P(6, 6);
      for (int j = 0; j < 6; j++)
        for (int i = 0; i < 6; i++)
          P(i, j) = msg->pose.covariance[i + j * 6];
      colvec eigVal;
      mat eigVec;
      eig_sym(eigVal, eigVec, P.submat(0, 0, 2, 2));
      if (det(eigVec) < 0) {
        for (int k = 0; k < 3; k++) {
          mat eigVecRev = eigVec;
          eigVecRev.col(k) *= -1;
          if (det(eigVecRev) > 0) {
            eigVec = eigVecRev;
            break;
          }
        }
      }
      covROS_.header.frame_id = string("world");
      covROS_.header.stamp = msg->header.stamp;
      covROS_.ns = string("covariance");
      covROS_.id = 0;
      covROS_.type = visualization_msgs::msg::Marker::SPHERE;
      covROS_.action = visualization_msgs::msg::Marker::ADD;
      covROS_.pose.position.x = pose(0);
      covROS_.pose.position.y = pose(1);
      covROS_.pose.position.z = pose(2);
      q = R_to_quaternion(eigVec);
      covROS_.pose.orientation.w = q(0);
      covROS_.pose.orientation.x = q(1);
      covROS_.pose.orientation.y = q(2);
      covROS_.pose.orientation.z = q(3);
      covROS_.scale.x = sqrt(eigVal(0)) * cov_scale_;
      covROS_.scale.y = sqrt(eigVal(1)) * cov_scale_;
      covROS_.scale.z = sqrt(eigVal(2)) * cov_scale_;
      covROS_.color.a = 0.4;
      covROS_.color.r = r * 0.5;
      covROS_.color.g = g * 0.5;
      covROS_.color.b = b * 0.5;
      covPub_->publish(covROS_);
    }

    // Covariance Velocity
    if (cov_vel_) {
      mat P(3, 3);
      for (int j = 0; j < 3; j++)
        for (int i = 0; i < 3; i++)
          P(i, j) = msg->twist.covariance[i + j * 6];
      mat R = ypr_to_R(pose.rows(3, 5));
      P = R * P * trans(R);
      colvec eigVal;
      mat eigVec;
      eig_sym(eigVal, eigVec, P);
      if (det(eigVec) < 0) {
        for (int k = 0; k < 3; k++) {
          mat eigVecRev = eigVec;
          eigVecRev.col(k) *= -1;
          if (det(eigVecRev) > 0) {
            eigVec = eigVecRev;
            break;
          }
        }
      }
      covVelROS_.header.frame_id = string("world");
      covVelROS_.header.stamp = msg->header.stamp;
      covVelROS_.ns = string("covariance_velocity");
      covVelROS_.id = 0;
      covVelROS_.type = visualization_msgs::msg::Marker::SPHERE;
      covVelROS_.action = visualization_msgs::msg::Marker::ADD;
      covVelROS_.pose.position.x = pose(0);
      covVelROS_.pose.position.y = pose(1);
      covVelROS_.pose.position.z = pose(2);
      q = R_to_quaternion(eigVec);
      covVelROS_.pose.orientation.w = q(0);
      covVelROS_.pose.orientation.x = q(1);
      covVelROS_.pose.orientation.y = q(2);
      covVelROS_.pose.orientation.z = q(3);
      covVelROS_.scale.x = sqrt(eigVal(0)) * cov_scale_;
      covVelROS_.scale.y = sqrt(eigVal(1)) * cov_scale_;
      covVelROS_.scale.z = sqrt(eigVal(2)) * cov_scale_;
      covVelROS_.color.a = 0.4;
      covVelROS_.color.r = r;
      covVelROS_.color.g = g;
      covVelROS_.color.b = b;
      covVelPub_->publish(covVelROS_);
    }

    // Color Coded Trajectory
    rclcpp::Time t = rclcpp::Time(msg->header.stamp);
    if ((t - pt_).seconds() > 0.5) {
      trajROS_.header.frame_id = string("world");
      trajROS_.header.stamp = this->now();
      trajROS_.ns = string("trajectory");
      trajROS_.type = visualization_msgs::msg::Marker::LINE_LIST;
      trajROS_.action = visualization_msgs::msg::Marker::ADD;
      trajROS_.pose.position.x = 0;
      trajROS_.pose.position.y = 0;
      trajROS_.pose.position.z = 0;
      trajROS_.pose.orientation.w = 1;
      trajROS_.pose.orientation.x = 0;
      trajROS_.pose.orientation.y = 0;
      trajROS_.pose.orientation.z = 0;
      trajROS_.scale.x = 0.1;
      trajROS_.scale.y = 0;
      trajROS_.scale.z = 0;
      trajROS_.color.r = 0.0;
      trajROS_.color.g = 1.0;
      trajROS_.color.b = 0.0;
      trajROS_.color.a = 0.8;
      geometry_msgs::msg::Point p;
      p.x = ppose_(0);
      p.y = ppose_(1);
      p.z = ppose_(2);
      trajROS_.points.push_back(p);
      p.x = pose(0);
      p.y = pose(1);
      p.z = pose(2);
      trajROS_.points.push_back(p);
      std_msgs::msg::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = 1;
      trajROS_.colors.push_back(color);
      trajROS_.colors.push_back(color);
      ppose_ = pose;
      pt_ = t;
      trajPub_->publish(trajROS_);
    }

    // Sensor availability
    sensorROS_.header.frame_id = string("world");
    sensorROS_.header.stamp = msg->header.stamp;
    sensorROS_.ns = string("sensor");
    sensorROS_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    sensorROS_.action = visualization_msgs::msg::Marker::ADD;
    sensorROS_.pose.position.x = pose(0);
    sensorROS_.pose.position.y = pose(1);
    sensorROS_.pose.position.z = pose(2) + 1.0;
    sensorROS_.pose.orientation.w = q(0);
    sensorROS_.pose.orientation.x = q(1);
    sensorROS_.pose.orientation.y = q(2);
    sensorROS_.pose.orientation.z = q(3);
    string strG = G ? string(" GPS ") : string("");
    string strV = V ? string(" Vision ") : string("");
    string strL = L ? string(" Laser ") : string("");
    sensorROS_.text = "| " + strG + strV + strL + " |";
    sensorROS_.color.a = 1.0;
    sensorROS_.color.r = 1.0;
    sensorROS_.color.g = 1.0;
    sensorROS_.color.b = 1.0;
    sensorROS_.scale.z = 0.5;
    sensorPub_->publish(sensorROS_);

    // Laser height measurement
    double H = msg->twist.covariance[32];
    heightROS_.header.frame_id = string("height");
    heightROS_.header.stamp = msg->header.stamp;
    heightROS_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    heightROS_.field_of_view = 5.0 * M_PI / 180.0;
    heightROS_.min_range = -100;
    heightROS_.max_range = 100;
    heightROS_.range = H;
    heightPub_->publish(heightROS_);

    // Mesh model
    meshROS_.header.frame_id = frame_id_;
    meshROS_.header.stamp = msg->header.stamp;
    meshROS_.ns = "mesh";
    meshROS_.id = 0;
    meshROS_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshROS_.action = visualization_msgs::msg::Marker::ADD;
    meshROS_.pose.position.x = msg->pose.pose.position.x;
    meshROS_.pose.position.y = msg->pose.pose.position.y;
    meshROS_.pose.position.z = msg->pose.pose.position.z;
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    if (cross_config_) {
      colvec ypr = R_to_ypr(quaternion_to_R(q));
      ypr(0) += 45.0 * kPi / 180.0;
      q = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS_.pose.orientation.w = q(0);
    meshROS_.pose.orientation.x = q(1);
    meshROS_.pose.orientation.y = q(2);
    meshROS_.pose.orientation.z = q(3);
    meshROS_.scale.x = scale_;
    meshROS_.scale.y = scale_;
    meshROS_.scale.z = scale_;
    meshROS_.color.a = color_a_;
    meshROS_.color.r = color_r_;
    meshROS_.color.g = color_g_;
    meshROS_.color.b = color_b_;
    meshROS_.mesh_resource = mesh_resource_;
    meshPub_->publish(meshROS_);

    // TF for raw sensor visualization
    if (tf45_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = "world";
      transform.child_frame_id = "/base";
      transform.transform.translation.x = pose(0);
      transform.transform.translation.y = pose(1);
      transform.transform.translation.z = pose(2);
      transform.transform.rotation.w = q(0);
      transform.transform.rotation.x = q(1);
      transform.transform.rotation.y = q(2);
      transform.transform.rotation.z = q(3);
      tf_broadcaster_->sendTransform(transform);

      geometry_msgs::msg::TransformStamped transform45;
      transform45.header.stamp = msg->header.stamp;
      transform45.header.frame_id = "/base";
      transform45.child_frame_id = "/laser";
      transform45.transform.translation.x = 0;
      transform45.transform.translation.y = 0;
      transform45.transform.translation.z = 0;
      colvec y45 = zeros<colvec>(3);
      y45(0) = 45.0 * M_PI / 180;
      colvec q45 = R_to_quaternion(ypr_to_R(y45));
      transform45.transform.rotation.w = q45(0);
      transform45.transform.rotation.x = q45(1);
      transform45.transform.rotation.y = q45(2);
      transform45.transform.rotation.z = q45(3);
      tf_broadcaster_->sendTransform(transform45);

      geometry_msgs::msg::TransformStamped transform45_vision;
      transform45_vision.header.stamp = msg->header.stamp;
      transform45_vision.header.frame_id = "/base";
      transform45_vision.child_frame_id = "/vision";
      transform45_vision.transform.translation.x = 0;
      transform45_vision.transform.translation.y = 0;
      transform45_vision.transform.translation.z = 0;
      transform45_vision.transform.rotation.w = q45(0);
      transform45_vision.transform.rotation.x = q45(1);
      transform45_vision.transform.rotation.y = q45(2);
      transform45_vision.transform.rotation.z = q45(3);
      tf_broadcaster_->sendTransform(transform45_vision);

      geometry_msgs::msg::TransformStamped transform90;
      transform90.header.stamp = msg->header.stamp;
      transform90.header.frame_id = "/base";
      transform90.child_frame_id = "/height";
      transform90.transform.translation.x = 0;
      transform90.transform.translation.y = 0;
      transform90.transform.translation.z = 0;
      colvec p90 = zeros<colvec>(3);
      p90(1) = 90.0 * M_PI / 180;
      colvec q90 = R_to_quaternion(ypr_to_R(p90));
      transform90.transform.rotation.w = q90(0);
      transform90.transform.rotation.x = q90(1);
      transform90.transform.rotation.y = q90(2);
      transform90.transform.rotation.z = q90(3);
      tf_broadcaster_->sendTransform(transform90);
    }
  }

  void cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
    if (cmd->header.frame_id == string("null"))
      return;

    colvec pose(6);
    pose(0) = cmd->position.x;
    pose(1) = cmd->position.y;
    pose(2) = cmd->position.z;
    colvec q(4);
    q(0) = 1.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 0.0;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

    // Mesh model
    meshROS_.header.frame_id = frame_id_;
    meshROS_.header.stamp = cmd->header.stamp;
    meshROS_.ns = "cmd_mesh";
    meshROS_.id = 1;
    meshROS_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshROS_.action = visualization_msgs::msg::Marker::ADD;
    meshROS_.pose.position.x = cmd->position.x;
    meshROS_.pose.position.y = cmd->position.y;
    meshROS_.pose.position.z = cmd->position.z;

    if (cross_config_) {
      colvec ypr = R_to_ypr(quaternion_to_R(q));
      ypr(0) += 45.0 * kPi / 180.0;
      q = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS_.pose.orientation.w = q(0);
    meshROS_.pose.orientation.x = q(1);
    meshROS_.pose.orientation.y = q(2);
    meshROS_.pose.orientation.z = q(3);
    meshROS_.scale.x = 2.0;
    meshROS_.scale.y = 2.0;
    meshROS_.scale.z = 2.0;
    meshROS_.color.a = cmd_color_a_;
    meshROS_.color.r = cmd_color_r_;
    meshROS_.color.g = cmd_color_g_;
    meshROS_.color.b = cmd_color_b_;
    meshROS_.mesh_resource = mesh_resource_;
    cmdVisPub_->publish(meshROS_);

    // Publish Command Pose
    geometry_msgs::msg::PoseStamped cmdPose;
    cmdPose.header = cmd->header;
    cmdPose.pose.position.x = cmd->position.x;
    cmdPose.pose.position.y = cmd->position.y;
    cmdPose.pose.position.z = cmd->position.z;
    cmdPose.pose.orientation.w = q(0);
    cmdPose.pose.orientation.x = q(1);
    cmdPose.pose.orientation.y = q(2);
    cmdPose.pose.orientation.z = q(3);
    cmdPosePub_->publish(cmdPose);

    // Command Path
    if ((rclcpp::Time(cmd->header.stamp) - prev_cmd_t_).seconds() > 0.1) {
      prev_cmd_t_ = rclcpp::Time(cmd->header.stamp);
      cmdPathROS_.header = cmdPose.header;
      cmdPathROS_.header.frame_id = "world";
      cmdPathROS_.poses.push_back(cmdPose);
      if (cmdPathROS_.poses.size() > 1000) {
        cmdPathROS_.poses.erase(cmdPathROS_.poses.begin());
      }
      cmdPathPub_->publish(cmdPathROS_);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomVisualization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
