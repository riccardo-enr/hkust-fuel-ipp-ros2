#include "mppi_control/mppi_acc_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <algorithm>
#include <ctime>

namespace mppi_control
{

  MPPIAccNode::MPPIAccNode(const rclcpp::NodeOptions &options)
      : Node("mppi_acc_node", options)
  {

    // Initialize Parameters
    params_.K = this->declare_parameter("mppi.K", 9000);
    params_.N = this->declare_parameter("mppi.N", 2.0);
    params_.ctl_freq = this->declare_parameter("mppi.ctl_freq", 100.0);
    // Compute dt and H from N and ctl_freq
    params_.dt = 1.0 / params_.ctl_freq;
    params_.H = static_cast<int>(std::round(params_.N * params_.ctl_freq));
    params_.sigma = this->declare_parameter("mppi.sigma", 1.0);
    params_.lambda = this->declare_parameter("mppi.lambda", 0.1);

    params_.Q_pos_x = this->declare_parameter("mppi.Q_pos_x", 10.0);
    params_.Q_pos_y = this->declare_parameter("mppi.Q_pos_y", 10.0);
    params_.Q_pos_z = this->declare_parameter("mppi.Q_pos_z", 10.0);
    params_.Q_vel_x = this->declare_parameter("mppi.Q_vel_x", 1.0);
    params_.Q_vel_y = this->declare_parameter("mppi.Q_vel_y", 1.0);
    params_.Q_vel_z = this->declare_parameter("mppi.Q_vel_z", 1.0);
    params_.R_x = this->declare_parameter("mppi.R_x", 0.1);
    params_.R_y = this->declare_parameter("mppi.R_y", 0.1);
    params_.R_z = this->declare_parameter("mppi.R_z", 0.1);
    params_.R_rate_x = this->declare_parameter("mppi.R_rate_x", 0.5);
    params_.R_rate_y = this->declare_parameter("mppi.R_rate_y", 0.5);
    params_.R_rate_z = this->declare_parameter("mppi.R_rate_z", 0.5);
    params_.w_obs = this->declare_parameter("mppi.w_obs", 100.0);

    params_.a_max = this->declare_parameter("mppi.a_max", 10.0);
    params_.tilt_max = this->declare_parameter("mppi.tilt_max", 0.6); // ~34 deg
    params_.g = this->declare_parameter("mppi.g", 9.81);

    // Low-pass filter parameters
    params_.enable_lpf = this->declare_parameter("mppi.enable_lpf", false);
    params_.lpf_cutoff = this->declare_parameter("mppi.lpf_cutoff", 5.0); // 5 Hz default

    // Compute LPF alpha from cutoff frequency
    // alpha = 2 * pi * fc * dt / (1 + 2 * pi * fc * dt)
    double omega = 2.0 * M_PI * params_.lpf_cutoff;
    params_.lpf_alpha = omega * params_.dt / (1.0 + omega * params_.dt);

    lpf_initialized_ = false;
    acc_filtered_.setZero();

    // Initialize timing monitor
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    last_control_time_ = clock_->now();
    timing_warning_logged_ = false;
    consecutive_slow_cycles_ = 0;

    mass_ = this->declare_parameter("mass", 0.98);
    kR_[0] = this->declare_parameter("gains/rot/x", 1.5);
    kR_[1] = this->declare_parameter("gains/rot/y", 1.5);
    kR_[2] = this->declare_parameter("gains/rot/z", 1.0);
    kOm_[0] = this->declare_parameter("gains/ang/x", 0.13);
    kOm_[1] = this->declare_parameter("gains/ang/y", 0.13);
    kOm_[2] = this->declare_parameter("gains/ang/z", 0.1);

    u_mean_.resize(params_.H, Eigen::Vector3d::Zero());

    // Publishers and Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&MPPIAccNode::odomCallback, this, std::placeholders::_1));

    pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "planning/pos_cmd", 10, std::bind(&MPPIAccNode::posCmdCallback, this, std::placeholders::_1));

    so3_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::SO3Command>(
        "so3_cmd", 10);

    // Create timer with period based on ctl_freq
    double control_period_s = 1.0 / params_.ctl_freq;
    auto control_period = std::chrono::duration<double>(control_period_s);
    auto control_period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(control_period);
    timer_ = this->create_wall_timer(
        control_period_ms, std::bind(&MPPIAccNode::controlLoop, this));

    // Validate and log parameters
    validateAndLogParameters();

    RCLCPP_INFO(this->get_logger(), "MPPI Acc Node Initialized");
  }

  void MPPIAccNode::validateAndLogParameters()
  {
    bool params_valid = true;
    std::string error_msg;

    // Log critical parameters
    RCLCPP_INFO(this->get_logger(), "=== MPPI ACC Configuration ===");
    RCLCPP_INFO(this->get_logger(), "  K (samples): %d", params_.K);
    RCLCPP_INFO(this->get_logger(), "  N (horizon): %.2f s", params_.N);
    RCLCPP_INFO(this->get_logger(), "  ctl_freq: %.1f Hz", params_.ctl_freq);
    RCLCPP_INFO(this->get_logger(), "  dt: %.4f s", params_.dt);
    RCLCPP_INFO(this->get_logger(), "  H (steps): %d", params_.H);
    RCLCPP_INFO(this->get_logger(), "  sigma: %.3f", params_.sigma);
    RCLCPP_INFO(this->get_logger(), "  lambda: %.4f", params_.lambda);
    RCLCPP_INFO(this->get_logger(), "  Q_pos: [%.1f, %.1f, %.1f]",
                params_.Q_pos_x, params_.Q_pos_y, params_.Q_pos_z);
    RCLCPP_INFO(this->get_logger(), "  Q_vel: [%.1f, %.1f, %.1f]",
                params_.Q_vel_x, params_.Q_vel_y, params_.Q_vel_z);
    RCLCPP_INFO(this->get_logger(), "  R_rate: [%.1f, %.1f, %.1f]",
                params_.R_rate_x, params_.R_rate_y, params_.R_rate_z);
    RCLCPP_INFO(this->get_logger(), "  enable_lpf: %s", params_.enable_lpf ? "true" : "false");
    if (params_.enable_lpf)
    {
      RCLCPP_INFO(this->get_logger(), "  lpf_cutoff: %.1f Hz (alpha: %.3f)",
                  params_.lpf_cutoff, params_.lpf_alpha);
    }
    RCLCPP_INFO(this->get_logger(), "==============================");

    // Validate critical parameters
    if (params_.K <= 0)
    {
      error_msg += "K must be > 0; ";
      params_valid = false;
    }
    if (params_.N <= 0.1 || params_.N > 10.0)
    {
      error_msg += "N (horizon) must be in [0.1, 10.0] s; ";
      params_valid = false;
    }
    if (params_.ctl_freq < 10.0 || params_.ctl_freq > 500.0)
    {
      error_msg += "ctl_freq must be in [10, 500] Hz; ";
      params_valid = false;
    }
    if (params_.sigma < 0.0)
    {
      error_msg += "sigma must be >= 0; ";
      params_valid = false;
    }
    if (params_.lambda <= 0.0)
    {
      error_msg += "lambda must be > 0; ";
      params_valid = false;
    }
    if (params_.H < 10)
    {
      error_msg += "H (computed) too small, check N and ctl_freq; ";
      params_valid = false;
    }
    if (params_.H > 500)
    {
      error_msg += "H (computed) too large (>500), may cause performance issues; ";
      params_valid = false;
    }

    if (!params_valid)
    {
      RCLCPP_ERROR(this->get_logger(), "INVALID PARAMETERS: %s", error_msg.c_str());
      RCLCPP_ERROR(this->get_logger(), "Controller may not function correctly! Check YAML configuration.");
      // Note: We don't throw here to allow the node to start, but control may be degraded
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Parameters validated successfully.");
    }
  }

  void MPPIAccNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    curr_p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    curr_v_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    odom_received_ = true;
  }

  void MPPIAccNode::posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    ref_cmd_ = *msg;
    ref_received_ = true;
  }

  void MPPIAccNode::controlLoop()
  {
    if (!odom_received_)
      return;

    // Timing monitor - check if control loop is running slower than expected
    rclcpp::Time current_time = clock_->now();
    double time_since_last = (current_time - last_control_time_).seconds();
    double expected_period = 1.0 / params_.ctl_freq;
    double tolerance = 1.5 * expected_period; // Allow 50% overhead before warning

    if (time_since_last > tolerance)
    {
      consecutive_slow_cycles_++;
      if (!timing_warning_logged_)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000,
                             "Control loop running slow! Actual: %.3fs, Expected: %.3fs (freq: %.1f Hz). "
                             "This may cause instability.",
                             time_since_last, expected_period, params_.ctl_freq);
        timing_warning_logged_ = true;
      }

      // Failsafe: if too many consecutive slow cycles, switch to hover mode
      if (consecutive_slow_cycles_ > 10)
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock_, 1000,
                              "Control loop severely degraded! Switching to failsafe hover mode.");
        // The rest of the function will handle this by setting ready=false
      }
    }
    else
    {
      consecutive_slow_cycles_ = 0;
      timing_warning_logged_ = false;
    }
    last_control_time_ = current_time;

    // Deferred initialization of SDFMap
    if (!sdf_map_)
    {
      sdf_map_.reset(new fast_planner::SDFMap);
      sdf_map_->initMap(this->shared_from_this());
    }

    bool ready = ref_received_;

    // Failsafe: if control loop is severely degraded, force hover mode
    if (consecutive_slow_cycles_ > 10)
    {
      ready = false;
    }

    // Safety: If reference is at origin and drone is not, it's likely an uninitialized reference
    if (ready && ref_cmd_.position.x == 0 && ref_cmd_.position.y == 0 && ref_cmd_.position.z == 0)
    {
      if (curr_p_.norm() > 1.0)
      {
        ready = false;
      }
    }

    Eigen::Vector3d des_acc;

    if (ready)
    {
      runMPPI();
      Eigen::Vector3d ref_acc(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);
      des_acc = u_mean_[0] + ref_acc;
      // Store previous control (delta from reference) for rate penalty
      u_prev_ = u_mean_[0];
    }
    else
    {
      // Hover command if not ready
      des_acc.setZero();
      // Warm start with zero if not ready
      std::fill(u_mean_.begin(), u_mean_.end(), Eigen::Vector3d::Zero());
      u_prev_.setZero();
    }

    // Apply low-pass filter to desired acceleration
    if (params_.enable_lpf)
    {
      if (!lpf_initialized_)
      {
        // Initialize filter with first value
        acc_filtered_ = des_acc;
        lpf_initialized_ = true;
      }
      else
      {
        // Exponential moving average: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
        acc_filtered_ = params_.lpf_alpha * des_acc + (1.0 - params_.lpf_alpha) * acc_filtered_;
      }
    }
    else
    {
      acc_filtered_ = des_acc;
    }

    Eigen::Vector3d force = mass_ * (acc_filtered_ + Eigen::Vector3d(0, 0, params_.g));

    // Use current yaw if reference not ready
    double target_yaw = ready ? ref_cmd_.yaw : current_yaw_;

    Eigen::Vector3d b1d(cos(target_yaw), sin(target_yaw), 0);
    Eigen::Vector3d b3c = force.normalized();
    Eigen::Vector3d b2c = b3c.cross(b1d).normalized();
    Eigen::Vector3d b1c = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;
    Eigen::Quaterniond orientation(R);

    auto so3_cmd = std::make_shared<quadrotor_msgs::msg::SO3Command>();
    so3_cmd->header.stamp = this->now();
    so3_cmd->header.frame_id = "world";
    so3_cmd->force.x = force.x();
    so3_cmd->force.y = force.y();
    so3_cmd->force.z = force.z();
    so3_cmd->orientation.x = orientation.x();
    so3_cmd->orientation.y = orientation.y();
    so3_cmd->orientation.z = orientation.z();
    so3_cmd->orientation.w = orientation.w();
    for (int i = 0; i < 3; i++)
    {
      so3_cmd->kr[i] = kR_[i];
      so3_cmd->kom[i] = kOm_[i];
    }
    so3_cmd->aux.current_yaw = current_yaw_;
    so3_cmd->aux.enable_motors = true;
    so3_cmd->aux.use_external_yaw = false;

    so3_cmd_pub_->publish(*so3_cmd);

    if (ready)
    {
      // Shift control sequence for next iteration (Warm start)
      for (int i = 0; i < params_.H - 1; ++i)
      {
        u_mean_[i] = u_mean_[i + 1];
      }
      // Initialize last step to hover (zero delta from reference)
      u_mean_[params_.H - 1].setZero();
    }
  }

  void MPPIAccNode::runMPPI()
  {
    std::vector<float3> samples_u(params_.K * params_.H);
    std::vector<float> costs(params_.K);
    std::vector<float3> u_mean_f3(params_.H);

    for (int i = 0; i < params_.H; ++i)
    {
      u_mean_f3[i] = make_float3(u_mean_[i].x(), u_mean_[i].y(), u_mean_[i].z());
    }

    float3 cp = make_float3(curr_p_.x(), curr_p_.y(), curr_p_.z());
    float3 cv = make_float3(curr_v_.x(), curr_v_.y(), curr_v_.z());
    float3 rp = make_float3(ref_cmd_.position.x, ref_cmd_.position.y, ref_cmd_.position.z);
    float3 rv = make_float3(ref_cmd_.velocity.x, ref_cmd_.velocity.y, ref_cmd_.velocity.z);
    float3 ra = make_float3(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);
    float3 up = make_float3(u_prev_.x(), u_prev_.y(), u_prev_.z());

    launch_mppi_acc_kernel(
        u_mean_f3.data(), up, cp, cv, rp, rv, ra,
        params_.K, params_.H, params_.dt, params_.sigma, params_.lambda,
        params_.Q_pos_x, params_.Q_pos_y, params_.Q_pos_z,
        params_.Q_vel_x, params_.Q_vel_y, params_.Q_vel_z,
        params_.R_x, params_.R_y, params_.R_z,
        params_.R_rate_x, params_.R_rate_y, params_.R_rate_z,
        params_.w_obs,
        params_.a_max, params_.tilt_max, params_.g,
        samples_u.data(), costs.data(), seed_++);

    // Weighting
    float min_cost = *std::min_element(costs.begin(), costs.end());
    double sum_weights = 0.0;
    std::vector<double> weights(params_.K);

    for (int k = 0; k < params_.K; ++k)
    {
      weights[k] = exp(-(costs[k] - min_cost) / params_.lambda);
      sum_weights += weights[k];
    }

    // Update mean control
    std::vector<Eigen::Vector3d> new_u_mean(params_.H, Eigen::Vector3d::Zero());
    for (int h = 0; h < params_.H; ++h)
    {
      for (int k = 0; k < params_.K; ++k)
      {
        float3 ukh = samples_u[k * params_.H + h];
        new_u_mean[h] += weights[k] * Eigen::Vector3d(ukh.x, ukh.y, ukh.z);
      }
      new_u_mean[h] /= (sum_weights + 1e-6);
    }
    u_mean_ = new_u_mean;
  }

} // namespace mppi_control

RCLCPP_COMPONENTS_REGISTER_NODE(mppi_control::MPPIAccNode)