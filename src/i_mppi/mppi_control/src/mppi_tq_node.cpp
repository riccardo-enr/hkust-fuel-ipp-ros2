#include "mppi_control/mppi_tq_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <random>
#include <algorithm>
#include <ctime>

namespace mppi_control
{

  MPPITqNode::MPPITqNode(const rclcpp::NodeOptions &options)
      : MPPINodeBase("mppi_tq_node", options)
  {
    // Initialize Common Parameters
    common_params_.K = this->declare_parameter("mppi.K", 9000);
    common_params_.N = this->declare_parameter("mppi.N", 2.0);
    common_params_.ctl_freq = this->declare_parameter("mppi.ctl_freq", 100.0);
    common_params_.dt = 1.0 / common_params_.ctl_freq;
    common_params_.H = static_cast<int>(std::round(common_params_.N * common_params_.ctl_freq));
    common_params_.lambda = this->declare_parameter("mppi.lambda", 0.1);
    common_params_.w_obs = this->declare_parameter("mppi.w_obs", 10.0); // Tq had 10.0 default vs 100.0 in Acc

    // Low-pass filter common params
    common_params_.enable_lpf = this->declare_parameter("mppi.enable_lpf", false);
    common_params_.lpf_cutoff = this->declare_parameter("mppi.lpf_cutoff", 5.0);
    double omega = 2.0 * M_PI * common_params_.lpf_cutoff;
    common_params_.lpf_alpha = omega * common_params_.dt / (1.0 + omega * common_params_.dt);

    // Tq Specific Params
    tq_params_.sigma_thrust = this->declare_parameter("mppi.sigma_thrust", 1.0);
    tq_params_.sigma_quat = this->declare_parameter("mppi.sigma_quat", 0.1);

    tq_params_.Q_pos_x = this->declare_parameter("mppi.Q_pos_x", 10.0);
    tq_params_.Q_pos_y = this->declare_parameter("mppi.Q_pos_y", 10.0);
    tq_params_.Q_pos_z = this->declare_parameter("mppi.Q_pos_z", 10.0);
    tq_params_.Q_vel_x = this->declare_parameter("mppi.Q_vel_x", 1.0);
    tq_params_.Q_vel_y = this->declare_parameter("mppi.Q_vel_y", 1.0);
    tq_params_.Q_vel_z = this->declare_parameter("mppi.Q_vel_z", 1.0);

    tq_params_.R_thrust = this->declare_parameter("mppi.R_thrust", 0.0);
    tq_params_.R_quat = this->declare_parameter("mppi.R_quat", 0.0);
    tq_params_.R_rate_thrust = this->declare_parameter("mppi.R_rate_thrust", 5.0);
    tq_params_.R_rate_quat = this->declare_parameter("mppi.R_rate_quat", 5.0);

    tq_params_.thrust_max = this->declare_parameter("mppi.thrust_max", 20.0);
    tq_params_.thrust_min = this->declare_parameter("mppi.thrust_min", 1.0);
    common_params_.g = this->declare_parameter("mppi.g", 9.81);

    // Disable auto lambda for TQ version for now
    lambda_params_.auto_lambda = false;

    lpf_initialized_ = false;
    control_filtered_.thrust = common_params_.g;
    control_filtered_.quat = make_float4(0, 0, 0, 1);

    mass_ = this->declare_parameter("mass", 0.98);
    kR_[0] = this->declare_parameter("gains/rot/x", 1.5);
    kR_[1] = this->declare_parameter("gains/rot/y", 1.5);
    kR_[2] = this->declare_parameter("gains/rot/z", 1.0);
    kOm_[0] = this->declare_parameter("gains/ang/x", 0.13);
    kOm_[1] = this->declare_parameter("gains/ang/y", 0.13);
    kOm_[2] = this->declare_parameter("gains/ang/z", 0.1);

    // Initialize mean control with hover
    u_mean_.resize(common_params_.H);
    for (int i = 0; i < common_params_.H; ++i)
    {
      u_mean_[i].thrust = common_params_.g;
      u_mean_[i].quat = make_float4(0, 0, 0, 1);
    }
    u_prev_.thrust = common_params_.g;
    u_prev_.quat = make_float4(0, 0, 0, 1);

    // Initialize Base
    initBase();

    validateAndLogParameters();
    RCLCPP_INFO(this->get_logger(), "MPPI Tq Node Initialized");
  }

  void MPPITqNode::validateAndLogParameters()
  {
    validateCommonParameters(common_params_);

    RCLCPP_INFO(this->get_logger(), "=== MPPI TQ Configuration ===");
    RCLCPP_INFO(this->get_logger(), "  sigma_thrust: %.3f", tq_params_.sigma_thrust);

    if (tq_params_.sigma_thrust < 0.0 || tq_params_.sigma_quat < 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "sigma must be >= 0");
    }
  }

  void MPPITqNode::controlLoop()
  {
    if (!odom_received_)
      return;

    checkTiming();
    auto start_time = std::chrono::high_resolution_clock::now();

    ensureSDFMap();
    bool ready = isDataReady();

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
      current_control.thrust = common_params_.g;
      current_control.quat = make_float4(0, 0, 0, 1);

      // Reset mean
      for (int i = 0; i < common_params_.H; ++i)
      {
        u_mean_[i].thrust = common_params_.g;
        u_mean_[i].quat = make_float4(0, 0, 0, 1);
      }
      u_prev_ = current_control;
    }

    if (common_params_.enable_lpf)
    {
      if (!lpf_initialized_)
      {
        control_filtered_ = current_control;
        lpf_initialized_ = true;
      }
      else
      {
        control_filtered_.thrust = common_params_.lpf_alpha * current_control.thrust +
                                   (1.0 - common_params_.lpf_alpha) * control_filtered_.thrust;

        Eigen::Quaterniond q_current = toEigen(current_control.quat);
        Eigen::Quaterniond q_filtered = toEigen(control_filtered_.quat);

        if (q_current.dot(q_filtered) < 0)
          q_current.coeffs() = -q_current.coeffs();

        Eigen::Quaterniond q_result = q_current.slerp(common_params_.lpf_alpha, q_filtered);
        control_filtered_.quat = toFloat4(q_result);
      }
    }
    else
    {
      control_filtered_ = current_control;
    }

    Eigen::Quaterniond q_cmd = toEigen(control_filtered_.quat);
    Eigen::Vector3d f_body(0, 0, control_filtered_.thrust * mass_);
    Eigen::Vector3d f_world = q_cmd * f_body;

    publishSO3(f_world, q_cmd);

    if (ready)
    {
      publish_execution_time(comp_time_pub_, start_time);

      for (int i = 0; i < common_params_.H - 1; ++i)
      {
        u_mean_[i] = u_mean_[i + 1];
      }
      u_mean_[common_params_.H - 1].thrust = common_params_.g;
      u_mean_[common_params_.H - 1].quat = make_float4(0, 0, 0, 1);
    }
  }

  void MPPITqNode::runMPPI()
  {
    std::vector<ControlSample> samples_u(common_params_.K * common_params_.H);
    std::vector<float> costs(common_params_.K);

    float3 cp = toFloat3(curr_p_);
    float3 cv = toFloat3(curr_v_);
    float3 rp = make_float3(ref_cmd_.position.x, ref_cmd_.position.y, ref_cmd_.position.z);
    float3 rv = make_float3(ref_cmd_.velocity.x, ref_cmd_.velocity.y, ref_cmd_.velocity.z);
    float3 ra = make_float3(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);

    launch_mppi_tq_kernel(
        u_mean_.data(), u_prev_, cp, cv, rp, rv, ra,
        common_params_.K, common_params_.H, common_params_.dt, common_params_.lambda,
        tq_params_.sigma_thrust, tq_params_.sigma_quat,
        tq_params_.Q_pos_x, tq_params_.Q_pos_y, tq_params_.Q_pos_z,
        tq_params_.Q_vel_x, tq_params_.Q_vel_y, tq_params_.Q_vel_z,
        tq_params_.R_thrust, tq_params_.R_quat,
        tq_params_.R_rate_thrust, tq_params_.R_rate_quat,
        common_params_.w_obs, tq_params_.thrust_max, tq_params_.thrust_min, common_params_.g,
        samples_u.data(), costs.data(), seed_++);

    float min_cost = *std::min_element(costs.begin(), costs.end());
    double sum_weights = 0.0;
    std::vector<double> weights(common_params_.K);

    for (int k = 0; k < common_params_.K; ++k)
    {
      weights[k] = exp(-(costs[k] - min_cost) / common_params_.lambda);
      sum_weights += weights[k];
    }

    for (int h = 0; h < common_params_.H; ++h)
    {
      double w_thrust = 0;
      for (int k = 0; k < common_params_.K; ++k)
      {
        w_thrust += weights[k] * samples_u[k * common_params_.H + h].thrust;
      }
      u_mean_[h].thrust = w_thrust / (sum_weights + 1e-6);

      Eigen::Quaterniond q_mean = toEigen(u_mean_[h].quat);

      const int max_iters = 3;
      for (int iter = 0; iter < max_iters; ++iter)
      {
        Eigen::Vector3d omega_avg = Eigen::Vector3d::Zero();

        for (int k = 0; k < common_params_.K; ++k)
        {
          double weight = weights[k];
          ControlSample &s = samples_u[k * common_params_.H + h];
          Eigen::Quaterniond q_k = toEigen(s.quat);

          if (q_mean.dot(q_k) < 0)
            q_k.coeffs() *= -1.0;

          Eigen::Quaterniond q_rel = q_mean.conjugate() * q_k;
          Eigen::AngleAxisd aa(q_rel);
          Eigen::Vector3d log_q = aa.angle() * aa.axis();

          omega_avg += weight * log_q;
        }

        omega_avg /= (sum_weights + 1e-6);

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
          break;
        }
      }
      u_mean_[h].quat = toFloat4(q_mean);
    }
  }

} // namespace mppi_control

RCLCPP_COMPONENTS_REGISTER_NODE(mppi_control::MPPITqNode)