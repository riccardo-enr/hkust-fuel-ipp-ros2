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

class DisturbanceGenerator : public rclcpp::Node
{
public:
  DisturbanceGenerator() : Node("so3_disturbance_generator")
  {
    // Declare all parameters
    declare_parameters();

    // Publishers
    pub_noisy_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/noisy_odom", 10);
    pub_correction_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/correction", 10);
    pub_force_ = create_publisher<geometry_msgs::msg::Vector3>("~/force_disturbance", 10);
    pub_moment_ = create_publisher<geometry_msgs::msg::Vector3>("~/moment_disturbance", 10);

    // Subscriber
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10,
        std::bind(&DisturbanceGenerator::odom_callback, this, std::placeholders::_1));

    // Timer for disturbance generation
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10), // 100 Hz
        std::bind(&DisturbanceGenerator::set_disturbance, this));
  }

private:
  void declare_parameters()
  {
    // External Disturbance
    declare_parameter("fxy", 0.0);
    declare_parameter("stdfxy", 0.0);
    declare_parameter("fz", 0.0);
    declare_parameter("stdfz", 0.0);
    declare_parameter("mrp", 0.0);
    declare_parameter("stdmrp", 0.0);
    declare_parameter("myaw", 0.0);
    declare_parameter("stdmyaw", 0.0);

    // Measurement Noise
    declare_parameter("enable_noisy_odom", false);
    declare_parameter("stdxyz", 0.0);
    declare_parameter("stdvxyz", 0.0);
    declare_parameter("stdrp", 0.0);
    declare_parameter("stdyaw", 0.0);

    // Odom Drifting
    declare_parameter("enable_drift_odom", true);
    declare_parameter("stdvdriftxyz", 0.0);
    declare_parameter("stdvdriftyaw", 0.0);
    declare_parameter("vdriftx", 0.0);
    declare_parameter("vdrifty", 0.0);
    declare_parameter("vdriftz", 0.0);
    declare_parameter("vdriftyaw", 0.0);
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

    bool enable_drift_odom = get_parameter("enable_drift_odom").as_bool();
    if (enable_drift_odom)
    {
      double dt = (rclcpp::Time(msg->header.stamp) - prev_pose_t).seconds();
      prev_pose_t = msg->header.stamp;
      colvec d = pose_update(pose_inverse(prev_pose), pose);
      prev_pose = pose;

      double vdriftx = get_parameter("vdriftx").as_double();
      double vdrifty = get_parameter("vdrifty").as_double();
      double vdriftz = get_parameter("vdriftz").as_double();
      double vdriftyaw = get_parameter("vdriftyaw").as_double();
      double stdvdriftxyz = get_parameter("stdvdriftxyz").as_double();
      double stdvdriftyaw = get_parameter("stdvdriftyaw").as_double();

      d(0) += (vdriftx + stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(1) += (vdrifty + stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(2) += (vdriftz + stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(3) += (vdriftyaw + stdvdriftyaw * as_scalar(randn(1))) * dt;
      drift_pose = pose_update(drift_pose, d);
      drift_vel = ypr_to_R(drift_pose.rows(3, 5)) * trans(ypr_to_R(pose.rows(3, 5))) * vel;
      correction_pose = pose_update(pose, pose_inverse(drift_pose));
    }
    else
    {
      drift_pose = pose;
      drift_vel = vel;
      correction_pose = zeros<colvec>(6);
    }

    // Noisy Odom
    static colvec noisy_pose = drift_pose;
    static colvec noisy_vel = drift_vel;

    bool enable_noisy_odom = get_parameter("enable_noisy_odom").as_bool();
    if (enable_noisy_odom)
    {
      double stdxyz = get_parameter("stdxyz").as_double();
      double stdvxyz = get_parameter("stdvxyz").as_double();
      double stdrp = get_parameter("stdrp").as_double();
      double stdyaw = get_parameter("stdyaw").as_double();

      colvec noise_pose = zeros<colvec>(6);
      colvec noise_vel = zeros<colvec>(3);
      noise_pose(0) = stdxyz * as_scalar(randn(1));
      noise_pose(1) = stdxyz * as_scalar(randn(1));
      noise_pose(2) = stdxyz * as_scalar(randn(1));
      noise_pose(3) = stdyaw * as_scalar(randn(1));
      noise_pose(4) = stdrp * as_scalar(randn(1));
      noise_pose(5) = stdrp * as_scalar(randn(1));
      noise_vel(0) = stdvxyz * as_scalar(randn(1));
      noise_vel(1) = stdvxyz * as_scalar(randn(1));
      noise_vel(2) = stdvxyz * as_scalar(randn(1));
      noisy_pose = drift_pose + noise_pose;
      noisy_vel = drift_vel + noise_vel;
      noisy_odom_.pose.covariance[0 + 0 * 6] = stdxyz * stdxyz;
      noisy_odom_.pose.covariance[1 + 1 * 6] = stdxyz * stdxyz;
      noisy_odom_.pose.covariance[2 + 2 * 6] = stdxyz * stdxyz;
      noisy_odom_.pose.covariance[(0 + 3) + (0 + 3) * 6] = stdyaw * stdyaw;
      noisy_odom_.pose.covariance[(1 + 3) + (1 + 3) * 6] = stdrp * stdrp;
      noisy_odom_.pose.covariance[(2 + 3) + (2 + 3) * 6] = stdrp * stdrp;
      noisy_odom_.twist.covariance[0 + 0 * 6] = stdvxyz * stdvxyz;
      noisy_odom_.twist.covariance[1 + 1 * 6] = stdvxyz * stdvxyz;
      noisy_odom_.twist.covariance[2 + 2 * 6] = stdvxyz * stdvxyz;
    }
    else
    {
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
    pub_noisy_odom_->publish(noisy_odom_);

    // Check time interval and publish correction
    static rclcpp::Time prev_correction_t = msg->header.stamp;
    if ((rclcpp::Time(msg->header.stamp) - prev_correction_t).seconds() > 1.0 / CORRECTION_RATE)
    {
      prev_correction_t = msg->header.stamp;
      correction_.pose.position.x = correction_pose(0);
      correction_.pose.position.y = correction_pose(1);
      correction_.pose.position.z = correction_pose(2);
      colvec correction_q = R_to_quaternion(ypr_to_R(correction_pose.rows(3, 5)));
      correction_.pose.orientation.w = correction_q(0);
      correction_.pose.orientation.x = correction_q(1);
      correction_.pose.orientation.y = correction_q(2);
      correction_.pose.orientation.z = correction_q(3);
      pub_correction_->publish(correction_);
    }
  }

  void set_disturbance()
  {
    geometry_msgs::msg::Vector3 f;
    geometry_msgs::msg::Vector3 m;

    double fxy = get_parameter("fxy").as_double();
    double stdfxy = get_parameter("stdfxy").as_double();
    double fz = get_parameter("fz").as_double();
    double stdfz = get_parameter("stdfz").as_double();
    double mrp = get_parameter("mrp").as_double();
    double stdmrp = get_parameter("stdmrp").as_double();
    double myaw = get_parameter("myaw").as_double();
    double stdmyaw = get_parameter("stdmyaw").as_double();

    f.x = fxy + stdfxy * as_scalar(randn(1));
    f.y = fxy + stdfxy * as_scalar(randn(1));
    f.z = fz + stdfz * as_scalar(randn(1));
    m.x = mrp + stdmrp * as_scalar(randn(1));
    m.y = mrp + stdmrp * as_scalar(randn(1));
    m.z = myaw + stdmyaw * as_scalar(randn(1));
    pub_force_->publish(f);
    pub_moment_->publish(m);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_noisy_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_correction_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_force_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_moment_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry noisy_odom_;
  geometry_msgs::msg::PoseStamped correction_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisturbanceGenerator>());
  rclcpp::shutdown();
  return 0;
}
