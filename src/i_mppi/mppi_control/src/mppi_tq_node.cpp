#include "mppi_control/mppi_tq_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <algorithm>
#include <ctime>

namespace mppi_control
{

  // Helper to convert Eigen Quat to float4
  float4 toFloat4(const Eigen::Quaterniond &q)
  {
    return make_float4(q.x(), q.y(), q.z(), q.w());
  }

  // Helper to convert float4 to Eigen Quat
  Eigen::Quaterniond toEigen(const float4 &q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  MPPITqNode::MPPITqNode(const rclcpp::NodeOptions &options)
      : Node("mppi_tq_node", options)
  {
    // Initialize Parameters
    params_.K = this->declare_parameter("mppi.K", 9000);
    params_.N = this->declare_parameter("mppi.N", 2.0);
    params_.ctl_freq = this->declare_parameter("mppi.ctl_freq", 100.0);
    // Compute dt and H from N and ctl_freq
    params_.dt = 1.0 / params_.ctl_freq;
    params_.H = static_cast<int>(std::round(params_.N * params_.ctl_freq));
    params_.lambda = this->declare_parameter("mppi.lambda", 0.1);

    params_.sigma_thrust = this->declare_parameter("mppi.sigma_thrust", 1.0);
    params_.sigma_quat = this->declare_parameter("mppi.sigma_quat", 0.1);

    params_.Q_pos_x = this->declare_parameter("mppi.Q_pos_x", 10.0);
    params_.Q_pos_y = this->declare_parameter("mppi.Q_pos_y", 10.0);
    params_.Q_pos_z = this->declare_parameter("mppi.Q_pos_z", 10.0);
    params_.Q_vel_x = this->declare_parameter("mppi.Q_vel_x", 1.0);
    params_.Q_vel_y = this->declare_parameter("mppi.Q_vel_y", 1.0);
    params_.Q_vel_z = this->declare_parameter("mppi.Q_vel_z", 1.0);

    params_.Q_thrust = this->declare_parameter("mppi.Q_thrust", 0.1);
    params_.R_thrust = this->declare_parameter("mppi.R_thrust", 0.0); // Not used yet in kernel, but good to have
    params_.R_rate_thrust = this->declare_parameter("mppi.R_rate_thrust", 5.0);
    params_.Q_quat = this->declare_parameter("mppi.Q_quat", 10.0);
    params_.R_quat = this->declare_parameter("mppi.R_quat", 0.0);
    params_.R_rate_quat = this->declare_parameter("mppi.R_rate_quat", 5.0);
    params_.w_obs = this->declare_parameter("mppi.w_obs", 10.0);

    params_.thrust_max = this->declare_parameter("mppi.thrust_max", 20.0); // Approx 2g
    params_.thrust_min = this->declare_parameter("mppi.thrust_min", 1.0);
    params_.g = this->declare_parameter("mppi.g", 9.81);

    // Low-pass filter parameters
    params_.enable_lpf = this->declare_parameter("mppi.enable_lpf", false);
    params_.lpf_cutoff = this->declare_parameter("mppi.lpf_cutoff", 5.0); // 5 Hz default

    // Compute LPF alpha from cutoff frequency
    // alpha = 2 * pi * fc * dt / (1 + 2 * pi * fc * dt)
    double omega = 2.0 * M_PI * params_.lpf_cutoff;
    params_.lpf_alpha = omega * params_.dt / (1.0 + omega * params_.dt);

    lpf_initialized_ = false;
    control_filtered_.thrust = params_.g;
    control_filtered_.quat = make_float4(0, 0, 0, 1);

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

    // Initialize mean control with hover
    u_mean_.resize(params_.H);
    for (int i = 0; i < params_.H; ++i)
    {
      u_mean_[i].thrust = params_.g;             // Or mass*g if thrust is force. Using g as acceleration magnitude.
      u_mean_[i].quat = make_float4(0, 0, 0, 1); // Identity
    }
    u_prev_.thrust = params_.g;
    u_prev_.quat = make_float4(0, 0, 0, 1);

    // Publishers and Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&MPPITqNode::odomCallback, this, std::placeholders::_1));

    pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "planning/pos_cmd", 10, std::bind(&MPPITqNode::posCmdCallback, this, std::placeholders::_1));

    so3_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::SO3Command>(
        "so3_cmd", 10);

    // Create timer with period based on ctl_freq
    double control_period_s = 1.0 / params_.ctl_freq;
    auto control_period = std::chrono::duration<double>(control_period_s);
    auto control_period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(control_period);
    timer_ = this->create_wall_timer(
        control_period_ms, std::bind(&MPPITqNode::controlLoop, this));

    // Validate and log parameters
    validateAndLogParameters();

    RCLCPP_INFO(this->get_logger(), "MPPI Tq Node Initialized");
  }

  void MPPITqNode::validateAndLogParameters()
  {
    bool params_valid = true;
    std::string error_msg;

    // Log critical parameters
    RCLCPP_INFO(this->get_logger(), "=== MPPI TQ Configuration ===");
    RCLCPP_INFO(this->get_logger(), "  K (samples): %d", params_.K);
    RCLCPP_INFO(this->get_logger(), "  N (horizon): %.2f s", params_.N);
    RCLCPP_INFO(this->get_logger(), "  ctl_freq: %.1f Hz", params_.ctl_freq);
    RCLCPP_INFO(this->get_logger(), "  dt: %.4f s", params_.dt);
    RCLCPP_INFO(this->get_logger(), "  H (steps): %d", params_.H);
    RCLCPP_INFO(this->get_logger(), "  sigma_thrust: %.3f", params_.sigma_thrust);
    RCLCPP_INFO(this->get_logger(), "  sigma_quat: %.3f", params_.sigma_quat);
    RCLCPP_INFO(this->get_logger(), "  lambda: %.2f", params_.lambda);
    RCLCPP_INFO(this->get_logger(), "  Q_pos: [%.1f, %.1f, %.1f]",
                params_.Q_pos_x, params_.Q_pos_y, params_.Q_pos_z);
    RCLCPP_INFO(this->get_logger(), "  Q_vel: [%.1f, %.1f, %.1f]",
                params_.Q_vel_x, params_.Q_vel_y, params_.Q_vel_z);
    RCLCPP_INFO(this->get_logger(), "  Q_thrust: %.1f, R_rate_thrust: %.1f",
                params_.Q_thrust, params_.R_rate_thrust);
    RCLCPP_INFO(this->get_logger(), "  Q_quat: %.1f, R_rate_quat: %.1f",
                params_.Q_quat, params_.R_rate_quat);
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
    if (params_.sigma_thrust < 0.0 || params_.sigma_quat < 0.0)
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

  void MPPITqNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    curr_p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    curr_v_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    curr_q_ = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

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

  void MPPITqNode::posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    ref_cmd_ = *msg;
    ref_received_ = true;
  }

  void MPPITqNode::controlLoop()
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
      }
    }
    else
    {
      consecutive_slow_cycles_ = 0;
      timing_warning_logged_ = false;
    }
    last_control_time_ = current_time;

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
    if (ready && ref_cmd_.position.x == 0 && ref_cmd_.position.y == 0 && ref_cmd_.position.z == 0)
    {
      if (curr_p_.norm() > 1.0)
        ready = false;
    }

    ControlInput current_control;

    if (ready)
    {
      runMPPI();
      current_control = u_mean_[0];
      u_prev_ = u_mean_[0];
    }
    else
    {
      // Hover
      current_control.thrust = params_.g;             // Acceleration magnitude
      current_control.quat = make_float4(0, 0, 0, 1); // Identity (upright if world frame Z is up)

      // Reset mean
      for (int i = 0; i < params_.H; ++i)
      {
        u_mean_[i].thrust = params_.g;
        u_mean_[i].quat = make_float4(0, 0, 0, 1);
      }
      u_prev_ = current_control;
    }

    // Apply low-pass filter to control output
    if (params_.enable_lpf)
    {
      if (!lpf_initialized_)
      {
        // Initialize filter with first value
        control_filtered_ = current_control;
        lpf_initialized_ = true;
      }
      else
      {
        // Filter thrust: exponential moving average
        control_filtered_.thrust = params_.lpf_alpha * current_control.thrust +
                                   (1.0 - params_.lpf_alpha) * control_filtered_.thrust;

        // Filter quaternion: spherical linear interpolation approximation
        // For small angles, we can filter the quaternion components directly
        // and then re-normalize
        Eigen::Quaterniond q_current = toEigen(current_control.quat);
        Eigen::Quaterniond q_filtered = toEigen(control_filtered_.quat);

        // Ensure shortest path (handle antipodal quaternions)
        if (q_current.dot(q_filtered) < 0)
        {
          q_current.coeffs() = -q_current.coeffs();
        }

        // Linear interpolation in quaternion space and normalize
        Eigen::Quaterniond q_result = q_current.slerp(params_.lpf_alpha, q_filtered);

        control_filtered_.quat = toFloat4(q_result);
      }
    }
    else
    {
      control_filtered_ = current_control;
    }

    // Convert to SO3Command
    // control_filtered_.thrust is acceleration magnitude. Force = mass * acc.
    // The force vector is F = R * [0, 0, force_mag].
    // But simulator takes force in world frame.
    // So we compute F_world = mass * acc_mag * (q * z_hat).

    // Actually, in the kernel I assumed F_world = R * [0, 0, thrust].
    // If thrust is acceleration magnitude, then F_world = mass * R * [0, 0, thrust].
    // And we need to subtract gravity for simulator?
    // In MPPIControlNode (Acc version): force = mass * (des_acc + g).
    // Here we optimize total thrust acceleration directly (including gravity support).
    // So thrust ~ |a + g|.

    Eigen::Quaterniond q_cmd = toEigen(control_filtered_.quat);
    Eigen::Vector3d f_body(0, 0, control_filtered_.thrust * mass_);
    Eigen::Vector3d f_world = q_cmd * f_body;

    // Important: Simulator `quadrotor_simulator_so3` expects:
    // force = cmd.force (Vector3 in World Frame)
    // AND it projects it onto body Z.
    // Also it needs orientation.
    // If we send `f_world` and `q_cmd`, the simulator will do:
    // f_projected = f_world . (R_current * z_hat) (approx)
    // Actually it uses `R` (current attitude) to project `force` input.
    // Wait, let's verify `SO3Command` usage in `quadrotor_simulator_so3`.
    // The plan said: "The simulator projects the commanded force vector onto the current body z-axis".
    // So if we send F_world aligned with q_cmd, and drone is at q_curr ~ q_cmd, then projection preserves magnitude.

    auto so3_cmd = std::make_shared<quadrotor_msgs::msg::SO3Command>();
    so3_cmd->header.stamp = this->now();
    so3_cmd->header.frame_id = "world";
    so3_cmd->force.x = f_world.x();
    so3_cmd->force.y = f_world.y();
    so3_cmd->force.z = f_world.z();
    so3_cmd->orientation.x = q_cmd.x();
    so3_cmd->orientation.y = q_cmd.y();
    so3_cmd->orientation.z = q_cmd.z();
    so3_cmd->orientation.w = q_cmd.w();
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
      // Shift control sequence
      for (int i = 0; i < params_.H - 1; ++i)
      {
        u_mean_[i] = u_mean_[i + 1];
      }
      // Initialize last step with hover condition
      u_mean_[params_.H - 1].thrust = params_.g;
      u_mean_[params_.H - 1].quat = make_float4(0, 0, 0, 1); // Identity quaternion (hover)
    }
  }

  void MPPITqNode::runMPPI()
  {
    std::vector<ControlSample> samples_u(params_.K * params_.H);
    std::vector<float> costs(params_.K);

    float3 cp = make_float3(curr_p_.x(), curr_p_.y(), curr_p_.z());
    float3 cv = make_float3(curr_v_.x(), curr_v_.y(), curr_v_.z());
    float3 rp = make_float3(ref_cmd_.position.x, ref_cmd_.position.y, ref_cmd_.position.z);
    float3 rv = make_float3(ref_cmd_.velocity.x, ref_cmd_.velocity.y, ref_cmd_.velocity.z);
    float3 ra = make_float3(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);

    // Compute reference thrust and quaternion from reference acceleration
    Eigen::Vector3d ref_acc_vec(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);
    Eigen::Vector3d ref_force_vec = (ref_acc_vec + Eigen::Vector3d(0, 0, params_.g));
    float ref_thrust = ref_force_vec.norm();

    // Reference orientation
    Eigen::Vector3d b1d(cos(ref_cmd_.yaw), sin(ref_cmd_.yaw), 0);
    Eigen::Vector3d b3c = ref_force_vec.normalized();
    // Handle singularity when force is zero (free fall) - unlikely but good to check
    if (ref_thrust < 1e-3)
      b3c = Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d b2c = b3c.cross(b1d).normalized();
    Eigen::Vector3d b1c = b2c.cross(b3c).normalized();
    Eigen::Matrix3d R_ref;
    R_ref << b1c, b2c, b3c;
    Eigen::Quaterniond ref_q(R_ref);
    float4 ref_quat = toFloat4(ref_q);

    launch_mppi_tq_kernel(
        u_mean_.data(), u_prev_, cp, cv, rp, rv, ra,
        ref_quat, ref_thrust,
        params_.K, params_.H, params_.dt, params_.lambda,
        params_.sigma_thrust, params_.sigma_quat,
        params_.Q_pos_x, params_.Q_pos_y, params_.Q_pos_z,
        params_.Q_vel_x, params_.Q_vel_y, params_.Q_vel_z,
        params_.Q_thrust, params_.R_thrust, params_.R_rate_thrust,
        params_.Q_quat, params_.R_quat, params_.R_rate_quat,
        params_.w_obs, params_.thrust_max, params_.thrust_min, params_.g,
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
    // Thrust: simple weighted average
    // Quaternion: Weighted average using iterative renormalization or Slerp (Plan mentions Slerp)
    // Since K is large (9000), pair-wise Slerp is expensive.
    // A common approximation for small spread is linear average + normalize.
    // Or just pick the best one? No, MPPI needs average.
    // Let's use the linear average + normalize approximation which is valid for concentrated distributions.
    // Since we are doing local sampling around mean, they should be close.
    // However, the plan proposed Slerp-based.
    // "For antipodal quaternions, use shortest path... weighted_quat = ... normalize"
    // Implementing full sequential Slerp for K samples is O(K*H).
    // Let's stick to Linear Average + Normalize (mean of chords) which is standard for simple quaternion averaging.
    // It minimizes chordal distance sum.

    for (int h = 0; h < params_.H; ++h)
    {
      double w_thrust = 0;

      // Thrust averaging (Simple weighted mean)
      for (int k = 0; k < params_.K; ++k)
      {
        w_thrust += weights[k] * samples_u[k * params_.H + h].thrust;
      }
      u_mean_[h].thrust = w_thrust / (sum_weights + 1e-6);

      // Quaternion Averaging (Iterative Geodesic Mean)
      // Initialize with current mean (warm start)
      Eigen::Quaterniond q_mean = toEigen(u_mean_[h].quat);

      const int max_iters = 3;
      for (int iter = 0; iter < max_iters; ++iter)
      {
        Eigen::Vector3d omega_avg = Eigen::Vector3d::Zero();

        for (int k = 0; k < params_.K; ++k)
        {
          double weight = weights[k];
          ControlSample &s = samples_u[k * params_.H + h];
          Eigen::Quaterniond q_k = toEigen(s.quat);

          // Handle antipodal ambiguity: ensure q_k is close to q_mean
          if (q_mean.dot(q_k) < 0)
            q_k.coeffs() *= -1.0;

          // Error in tangent space (rotation vector)
          // q_k = q_mean * exp(omega)  =>  exp(omega) = q_mean.inv * q_k
          Eigen::Quaterniond q_rel = q_mean.conjugate() * q_k;
          Eigen::AngleAxisd aa(q_rel);
          Eigen::Vector3d log_q = aa.angle() * aa.axis();

          omega_avg += weight * log_q;
        }

        omega_avg /= (sum_weights + 1e-6);

        // Update mean: q_mean = q_mean * exp(omega_avg)
        double angle = omega_avg.norm();
        if (angle > 1e-6)
        {
          Eigen::Vector3d axis = omega_avg.normalized();
          Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle, axis));
          q_mean = q_mean * q_delta;
          q_mean.normalize();
        }
        else
        {
          break; // Converged
        }
      }

      u_mean_[h].quat = toFloat4(q_mean);
    }
  }

} // namespace mppi_control

RCLCPP_COMPONENTS_REGISTER_NODE(mppi_control::MPPITqNode)
