#include <iostream>
#include <string.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pose_utils.h"

using namespace arma;
using namespace std;

#define CORRECTION_RATE 1

class SO3DisturbanceGenerator : public rclcpp::Node
{
public:
  SO3DisturbanceGenerator() : Node("so3_disturbance_generator")
  {
    // Publishers
    pubo_ = this->create_publisher<nav_msgs::msg::Odometry>("noisy_odom", 10);
    pubc_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("correction", 10);
    pubf_ = this->create_publisher<geometry_msgs::msg::Vector3>("force_disturbance", 10);
    pubm_ = this->create_publisher<geometry_msgs::msg::Vector3>("moment_disturbance", 10);

    // Subscriber
    sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&SO3DisturbanceGenerator::odom_callback, this, std::placeholders::_1));

    // Declare parameters (converting from dynamic_reconfigure)
    this->declare_parameter("enable_drift_odom", false);
    this->declare_parameter("enable_noisy_odom", false);
    this->declare_parameter("vdriftx", 0.0);
    this->declare_parameter("vdrifty", 0.0);
    this->declare_parameter("vdriftz", 0.0);
    this->declare_parameter("vdriftyaw", 0.0);
    this->declare_parameter("stdvdriftxyz", 0.0);
    this->declare_parameter("stdvdriftyaw", 0.0);
    this->declare_parameter("stdxyz", 0.0);
    this->declare_parameter("stdyaw", 0.0);
    this->declare_parameter("stdrp", 0.0);
    this->declare_parameter("stdvxyz", 0.0);
    this->declare_parameter("fxy", 0.0);
    this->declare_parameter("fz", 0.0);
    this->declare_parameter("mrp", 0.0);
    this->declare_parameter("myaw", 0.0);
    this->declare_parameter("stdfxy", 0.0);
    this->declare_parameter("stdfz", 0.0);
    this->declare_parameter("stdmrp", 0.0);
    this->declare_parameter("stdmyaw", 0.0);

    // Load initial parameter values
    load_parameters();

    // Set up parameter callback for runtime parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SO3DisturbanceGenerator::parameters_callback, this, std::placeholders::_1));

    // Timer for disturbance generation (100 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SO3DisturbanceGenerator::set_disturbance, this));
  }

private:
  void load_parameters()
  {
    config_.enable_drift_odom = this->get_parameter("enable_drift_odom").as_bool();
    config_.enable_noisy_odom = this->get_parameter("enable_noisy_odom").as_bool();
    config_.vdriftx = this->get_parameter("vdriftx").as_double();
    config_.vdrifty = this->get_parameter("vdrifty").as_double();
    config_.vdriftz = this->get_parameter("vdriftz").as_double();
    config_.vdriftyaw = this->get_parameter("vdriftyaw").as_double();
    config_.stdvdriftxyz = this->get_parameter("stdvdriftxyz").as_double();
    config_.stdvdriftyaw = this->get_parameter("stdvdriftyaw").as_double();
    config_.stdxyz = this->get_parameter("stdxyz").as_double();
    config_.stdyaw = this->get_parameter("stdyaw").as_double();
    config_.stdrp = this->get_parameter("stdrp").as_double();
    config_.stdvxyz = this->get_parameter("stdvxyz").as_double();
    config_.fxy = this->get_parameter("fxy").as_double();
    config_.fz = this->get_parameter("fz").as_double();
    config_.mrp = this->get_parameter("mrp").as_double();
    config_.myaw = this->get_parameter("myaw").as_double();
    config_.stdfxy = this->get_parameter("stdfxy").as_double();
    config_.stdfz = this->get_parameter("stdfz").as_double();
    config_.stdmrp = this->get_parameter("stdmrp").as_double();
    config_.stdmyaw = this->get_parameter("stdmyaw").as_double();
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
      if (param.get_name() == "enable_drift_odom")
        config_.enable_drift_odom = param.as_bool();
      else if (param.get_name() == "enable_noisy_odom")
        config_.enable_noisy_odom = param.as_bool();
      else if (param.get_name() == "vdriftx")
        config_.vdriftx = param.as_double();
      else if (param.get_name() == "vdrifty")
        config_.vdrifty = param.as_double();
      else if (param.get_name() == "vdriftz")
        config_.vdriftz = param.as_double();
      else if (param.get_name() == "vdriftyaw")
        config_.vdriftyaw = param.as_double();
      else if (param.get_name() == "stdvdriftxyz")
        config_.stdvdriftxyz = param.as_double();
      else if (param.get_name() == "stdvdriftyaw")
        config_.stdvdriftyaw = param.as_double();
      else if (param.get_name() == "stdxyz")
        config_.stdxyz = param.as_double();
      else if (param.get_name() == "stdyaw")
        config_.stdyaw = param.as_double();
      else if (param.get_name() == "stdrp")
        config_.stdrp = param.as_double();
      else if (param.get_name() == "stdvxyz")
        config_.stdvxyz = param.as_double();
      else if (param.get_name() == "fxy")
        config_.fxy = param.as_double();
      else if (param.get_name() == "fz")
        config_.fz = param.as_double();
      else if (param.get_name() == "mrp")
        config_.mrp = param.as_double();
      else if (param.get_name() == "myaw")
        config_.myaw = param.as_double();
      else if (param.get_name() == "stdfxy")
        config_.stdfxy = param.as_double();
      else if (param.get_name() == "stdfz")
        config_.stdfz = param.as_double();
      else if (param.get_name() == "stdmrp")
        config_.stdmrp = param.as_double();
      else if (param.get_name() == "stdmyaw")
        config_.stdmyaw = param.as_double();
    }

    return result;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    noisy_odom_.header = msg->header;
    correction_.header = msg->header;
    
    // Get odom
    colvec pose(6);
    colvec vel(3);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    colvec q = zeros<colvec>(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;
    
    // Drift Odom
    static colvec drift_pose = pose;
    static colvec drift_vel = vel;
    static colvec correction_pose = zeros<colvec>(6);
    static colvec prev_pose = pose;
    static rclcpp::Time prev_pose_t = msg->header.stamp;
    
    if (config_.enable_drift_odom) {
      double dt = (rclcpp::Time(msg->header.stamp) - prev_pose_t).seconds();
      prev_pose_t = msg->header.stamp;
      colvec d = pose_update(pose_inverse(prev_pose), pose);
      prev_pose = pose;
      d(0) += (config_.vdriftx + config_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(1) += (config_.vdrifty + config_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(2) += (config_.vdriftz + config_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(3) += (config_.vdriftyaw + config_.stdvdriftyaw * as_scalar(randn(1))) * dt;
      drift_pose = pose_update(drift_pose, d);
      drift_vel = ypr_to_R(drift_pose.rows(3, 5)) * trans(ypr_to_R(pose.rows(3, 5))) * vel;
      correction_pose = pose_update(pose, pose_inverse(drift_pose));
    } else {
      drift_pose = pose;
      drift_vel = vel;
      correction_pose = zeros<colvec>(6);
    }
    
    // Noisy Odom
    static colvec noisy_pose = drift_pose;
    static colvec noisy_vel = drift_vel;
    
    if (config_.enable_noisy_odom) {
      colvec noise_pose = zeros<colvec>(6);
      colvec noise_vel = zeros<colvec>(3);
      noise_pose(0) = config_.stdxyz * as_scalar(randn(1));
      noise_pose(1) = config_.stdxyz * as_scalar(randn(1));
      noise_pose(2) = config_.stdxyz * as_scalar(randn(1));
      noise_pose(3) = config_.stdyaw * as_scalar(randn(1));
      noise_pose(4) = config_.stdrp * as_scalar(randn(1));
      noise_pose(5) = config_.stdrp * as_scalar(randn(1));
      noise_vel(0) = config_.stdvxyz * as_scalar(randn(1));
      noise_vel(1) = config_.stdvxyz * as_scalar(randn(1));
      noise_vel(2) = config_.stdvxyz * as_scalar(randn(1));
      noisy_pose = drift_pose + noise_pose;
      noisy_vel = drift_vel + noise_vel;
      noisy_odom_.pose.covariance[0 + 0 * 6] = config_.stdxyz * config_.stdxyz;
      noisy_odom_.pose.covariance[1 + 1 * 6] = config_.stdxyz * config_.stdxyz;
      noisy_odom_.pose.covariance[2 + 2 * 6] = config_.stdxyz * config_.stdxyz;
      noisy_odom_.pose.covariance[(0 + 3) + (0 + 3) * 6] = config_.stdyaw * config_.stdyaw;
      noisy_odom_.pose.covariance[(1 + 3) + (1 + 3) * 6] = config_.stdrp * config_.stdrp;
      noisy_odom_.pose.covariance[(2 + 3) + (2 + 3) * 6] = config_.stdrp * config_.stdrp;
      noisy_odom_.twist.covariance[0 + 0 * 6] = config_.stdvxyz * config_.stdvxyz;
      noisy_odom_.twist.covariance[1 + 1 * 6] = config_.stdvxyz * config_.stdvxyz;
      noisy_odom_.twist.covariance[2 + 2 * 6] = config_.stdvxyz * config_.stdvxyz;
    } else {
      noisy_pose = drift_pose;
      noisy_vel = drift_vel;
      noisy_odom_.pose.covariance[0 + 0 * 6] = 0;
      noisy_odom_.pose.covariance[1 + 1 * 6] = 0;
      noisy_odom_.pose.covariance[2 + 2 * 6] = 0;
      noisy_odom_.pose.covariance[(0 + 3) + (0 + 3) * 6] = 0;
      noisy_odom_.pose.covariance[(1 + 3) + (1 + 3) * 6] = 0;
      noisy_odom_.pose.covariance[(2 + 3) + (2 + 3) * 6] = 0;
      noisy_odom_.twist.covariance[0 + 0 * 6] = 0;
      noisy_odom_.twist.covariance[1 + 1 * 6] = 0;
      noisy_odom_.twist.covariance[2 + 2 * 6] = 0;
    }
    
    // Assemble and publish odom
    noisy_odom_.pose.pose.position.x = noisy_pose(0);
    noisy_odom_.pose.pose.position.y = noisy_pose(1);
    noisy_odom_.pose.pose.position.z = noisy_pose(2);
    noisy_odom_.twist.twist.linear.x = noisy_vel(0);
    noisy_odom_.twist.twist.linear.y = noisy_vel(1);
    noisy_odom_.twist.twist.linear.z = noisy_vel(2);
    colvec noisy_q = R_to_quaternion(ypr_to_R(noisy_pose.rows(3, 5)));
    noisy_odom_.pose.pose.orientation.w = noisy_q(0);
    noisy_odom_.pose.pose.orientation.x = noisy_q(1);
    noisy_odom_.pose.pose.orientation.y = noisy_q(2);
    noisy_odom_.pose.pose.orientation.z = noisy_q(3);
    pubo_->publish(noisy_odom_);
    
    // Check time interval and publish correction
    static rclcpp::Time prev_correction_t = msg->header.stamp;
    if ((rclcpp::Time(msg->header.stamp) - prev_correction_t).seconds() > 1.0 / CORRECTION_RATE) {
      prev_correction_t = msg->header.stamp;
      correction_.pose.position.x = correction_pose(0);
      correction_.pose.position.y = correction_pose(1);
      correction_.pose.position.z = correction_pose(2);
      colvec correction_q = R_to_quaternion(ypr_to_R(correction_pose.rows(3, 5)));
      correction_.pose.orientation.w = correction_q(0);
      correction_.pose.orientation.x = correction_q(1);
      correction_.pose.orientation.y = correction_q(2);
      correction_.pose.orientation.z = correction_q(3);
      pubc_->publish(correction_);
    }
  }

  void set_disturbance()
  {
    geometry_msgs::msg::Vector3 f;
    geometry_msgs::msg::Vector3 m;
    f.x = config_.fxy + config_.stdfxy * as_scalar(randn(1));
    f.y = config_.fxy + config_.stdfxy * as_scalar(randn(1));
    f.z = config_.fz + config_.stdfz * as_scalar(randn(1));
    m.x = config_.mrp + config_.stdmrp * as_scalar(randn(1));
    m.y = config_.mrp + config_.stdmrp * as_scalar(randn(1));
    m.z = config_.myaw + config_.stdmyaw * as_scalar(randn(1));
    pubf_->publish(f);
    pubm_->publish(m);
  }

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubo_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubf_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubm_;

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Messages
  nav_msgs::msg::Odometry noisy_odom_;
  geometry_msgs::msg::PoseStamped correction_;

  // Configuration struct (replaces dynamic_reconfigure)
  struct Config {
    bool enable_drift_odom = false;
    bool enable_noisy_odom = false;
    double vdriftx = 0.0;
    double vdrifty = 0.0;
    double vdriftz = 0.0;
    double vdriftyaw = 0.0;
    double stdvdriftxyz = 0.0;
    double stdvdriftyaw = 0.0;
    double stdxyz = 0.0;
    double stdyaw = 0.0;
    double stdrp = 0.0;
    double stdvxyz = 0.0;
    double fxy = 0.0;
    double fz = 0.0;
    double mrp = 0.0;
    double myaw = 0.0;
    double stdfxy = 0.0;
    double stdfz = 0.0;
    double stdmrp = 0.0;
    double stdmyaw = 0.0;
  } config_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SO3DisturbanceGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
