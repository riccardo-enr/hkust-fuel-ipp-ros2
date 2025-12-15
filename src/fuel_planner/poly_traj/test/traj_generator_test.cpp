#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "swarmtal_msgs/msg/drone_onboard_command.hpp"

#include "poly_traj/traj_generator_node.hpp"

using namespace std::chrono_literals;

TEST(TrajGeneratorNodeTest, PublishesCommandAfterReceivingOdometry)
{
  auto context = std::make_shared<rclcpp::Context>();
  try {
    context->init(0, nullptr);
  } catch (const rclcpp::exceptions::RCLError & e) {
    GTEST_SKIP() << "Skipping test; failed to init rcl context: " << e.what();
    return;
  }

  std::shared_ptr<poly_traj::TrajGeneratorNode> traj_node;
  try {
    traj_node = std::make_shared<poly_traj::TrajGeneratorNode>(
      rclcpp::NodeOptions().context(context));
  } catch (const rclcpp::exceptions::RCLError & e) {
    context->shutdown("traj_generator_test skipped: node creation failed");
    GTEST_SKIP() << "Skipping test; failed to create node: " << e.what();
    return;
  }

  auto helper_node = std::make_shared<rclcpp::Node>(
    "traj_generator_test_helper", rclcpp::NodeOptions().context(context));

  std::atomic<size_t> command_count {0};
  std::mutex last_cmd_mutex;
  swarmtal_msgs::msg::DroneOnboardCommand last_cmd;

  auto cmd_sub = helper_node->create_subscription<swarmtal_msgs::msg::DroneOnboardCommand>(
    "/drone_commander/onboard_command", 10,
    [&](const swarmtal_msgs::msg::DroneOnboardCommand::SharedPtr msg) {
      std::scoped_lock<std::mutex> lock(last_cmd_mutex);
      last_cmd = *msg;
      command_count.fetch_add(1);
    });

  auto odom_pub = helper_node->create_publisher<nav_msgs::msg::Odometry>(
    "/uwb_vicon_odom", rclcpp::SensorDataQoS());

  rclcpp::ExecutorOptions exec_options;
  exec_options.context = context;
  rclcpp::executors::SingleThreadedExecutor exec(exec_options);
  exec.add_node(traj_node);
  exec.add_node(helper_node);

  nav_msgs::msg::Odometry odom;
  odom.child_frame_id = "B";  // ensure the callback does not early-return
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 1.0;
  odom.twist.twist.linear.x = 0.1;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  auto publish_and_spin = [&]() {
    odom.header.stamp = helper_node->now();
    odom_pub->publish(odom);
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  };

  // Publish odometry and allow the timer to fire for up to 2 seconds
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline && command_count.load() == 0) {
    publish_and_spin();
  }

  exec.spin_some();

  EXPECT_GT(command_count.load(), 0u) << "traj_generator did not publish commands";
  if (command_count.load() > 0) {
    std::scoped_lock<std::mutex> lock(last_cmd_mutex);
    EXPECT_EQ(last_cmd.command_type, swarmtal_msgs::msg::DroneOnboardCommand::CTRL_POS_COMMAND);
  }

  exec.cancel();
  context->shutdown("traj_generator_test complete");
}
