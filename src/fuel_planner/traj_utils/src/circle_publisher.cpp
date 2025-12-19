#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

using namespace std::chrono_literals;

class CirclePublisher : public rclcpp::Node
{
public:
  CirclePublisher() : Node("circle_publisher")
  {
    // Parameters
    this->declare_parameter("radius", 5.0); // radius in meters
    this->declare_parameter("omega", 0.5);  // angular velocity in rad/s
    this->declare_parameter("center_x", -18.0);
    this->declare_parameter("center_y", -1.0);
    this->declare_parameter("center_z", 2.0);

    radius_ = this->get_parameter("radius").as_double();
    omega_ = this->get_parameter("omega").as_double();
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    center_z_ = this->get_parameter("center_z").as_double();

    // Publisher
    pos_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
        "/planning/pos_cmd", 10);

    // Timer (100 Hz)
    start_time_ = this->now();
    timer_ = this->create_wall_timer(
        10ms, std::bind(&CirclePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Circle Publisher started.");
  }

private:
  void timer_callback()
  {
    auto current_time = this->now();
    double t = (current_time - start_time_).seconds();

    quadrotor_msgs::msg::PositionCommand msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "world";
    msg.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    msg.trajectory_id = 1;

    // Position
    msg.position.x = center_x_ + radius_ * std::cos(omega_ * t);
    msg.position.y = center_y_ + radius_ * std::sin(omega_ * t);
    msg.position.z = center_z_;

    // Velocity
    msg.velocity.x = -radius_ * omega_ * std::sin(omega_ * t);
    msg.velocity.y = radius_ * omega_ * std::cos(omega_ * t);
    msg.velocity.z = 0.0;

    // Acceleration
    msg.acceleration.x = -radius_ * omega_ * omega_ * std::cos(omega_ * t);
    msg.acceleration.y = -radius_ * omega_ * omega_ * std::sin(omega_ * t);
    msg.acceleration.z = 0.0;

    // Yaw
    msg.yaw = std::atan2(msg.velocity.y, msg.velocity.x);
    msg.yaw_dot = omega_;

    // Gains (default from so3_test.launch.py)
    msg.kx = {5.7, 5.7, 6.2};
    msg.kv = {3.4, 3.4, 4.0};

    pos_cmd_pub_->publish(msg);
  }

  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  double radius_, omega_, center_x_, center_y_, center_z_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CirclePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
