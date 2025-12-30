#include "mppi_control/mppi_acc_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <random>
#include <algorithm>
#include <ctime>

namespace mppi_control
{

  MPPIAccNode::MPPIAccNode(const rclcpp::NodeOptions &options)
      : MPPINodeBase("mppi_acc_node", options)
  {
    // Initialize Common Parameters
    common_params_.K = this->declare_parameter("mppi.K", 9000);
    common_params_.N = this->declare_parameter("mppi.N", 2.0);
    common_params_.ctl_freq = this->declare_parameter("mppi.ctl_freq", 100.0);
    common_params_.dt = 1.0 / common_params_.ctl_freq;
    common_params_.H = static_cast<int>(std::round(common_params_.N * common_params_.ctl_freq));
    common_params_.lambda = this->declare_parameter("mppi.lambda", 0.1);
    common_params_.w_obs = this->declare_parameter("mppi.w_obs", 100.0);
    common_params_.g = this->declare_parameter("mppi.g", 9.81);

    // Low-pass filter common params
    common_params_.enable_lpf = this->declare_parameter("mppi.enable_lpf", false);
    common_params_.lpf_cutoff = this->declare_parameter("mppi.lpf_cutoff", 5.0);
    double omega = 2.0 * M_PI * common_params_.lpf_cutoff;
    common_params_.lpf_alpha = omega * common_params_.dt / (1.0 + omega * common_params_.dt);

    // Acc Specific Parameters
    acc_params_.sigma = this->declare_parameter("mppi.sigma", 1.0);
    acc_params_.Q_pos_x = this->declare_parameter("mppi.Q_pos_x", 10.0);
    acc_params_.Q_pos_y = this->declare_parameter("mppi.Q_pos_y", 10.0);
    acc_params_.Q_pos_z = this->declare_parameter("mppi.Q_pos_z", 10.0);
    acc_params_.Q_vel_x = this->declare_parameter("mppi.Q_vel_x", 1.0);
    acc_params_.Q_vel_y = this->declare_parameter("mppi.Q_vel_y", 1.0);
    acc_params_.Q_vel_z = this->declare_parameter("mppi.Q_vel_z", 1.0);
    acc_params_.R_x = this->declare_parameter("mppi.R_x", 0.1);
    acc_params_.R_y = this->declare_parameter("mppi.R_y", 0.1);
    acc_params_.R_z = this->declare_parameter("mppi.R_z", 0.1);
    acc_params_.R_rate_x = this->declare_parameter("mppi.R_rate_x", 0.5);
    acc_params_.R_rate_y = this->declare_parameter("mppi.R_rate_y", 0.5);
    acc_params_.R_rate_z = this->declare_parameter("mppi.R_rate_z", 0.5);

    acc_params_.a_max = this->declare_parameter("mppi.a_max", 10.0);
    acc_params_.tilt_max = this->declare_parameter("mppi.tilt_max", 0.6);

    // Lambda Auto-tuning Parameters
    lambda_params_.auto_lambda = this->declare_parameter("mppi.auto_lambda", true);
    lambda_params_.target_ess = this->declare_parameter("mppi.target_ess", 50.0);
    lambda_params_.lambda_min = this->declare_parameter("mppi.lambda_min", 0.001);
    lambda_params_.lambda_max = this->declare_parameter("mppi.lambda_max", 1.0);
    lambda_params_.lambda_step = this->declare_parameter("mppi.lambda_step", 0.01);

    lpf_initialized_ = false;
    acc_filtered_.setZero();

    mass_ = this->declare_parameter("mass", 0.98);
    kR_[0] = this->declare_parameter("gains/rot/x", 1.5);
    kR_[1] = this->declare_parameter("gains/rot/y", 1.5);
    kR_[2] = this->declare_parameter("gains/rot/z", 1.0);
    kOm_[0] = this->declare_parameter("gains/ang/x", 0.13);
    kOm_[1] = this->declare_parameter("gains/ang/y", 0.13);
    kOm_[2] = this->declare_parameter("gains/ang/z", 0.1);

    u_mean_.resize(common_params_.H, Eigen::Vector3d::Zero());

    // Initialize Base (Subscribers/Publishers/Timer)
    initBase();

    // Validate
    validateAndLogParameters();

    RCLCPP_INFO(this->get_logger(), "MPPI Acc Node Initialized");
  }

  void MPPIAccNode::validateAndLogParameters()
  {
    validateCommonParameters(common_params_);

    // Log Specifics
    RCLCPP_INFO(this->get_logger(), "=== MPPI ACC Configuration ===");
    RCLCPP_INFO(this->get_logger(), "  sigma: %.3f", acc_params_.sigma);
    RCLCPP_INFO(this->get_logger(), "  Q_pos: [%.1f, %.1f, %.1f]",
                acc_params_.Q_pos_x, acc_params_.Q_pos_y, acc_params_.Q_pos_z);
    RCLCPP_INFO(this->get_logger(), "  Auto Lambda: %s, target_ess: %.1f, step: %.3f",
                lambda_params_.auto_lambda ? "ON" : "OFF",
                lambda_params_.target_ess, lambda_params_.lambda_step);

    if (acc_params_.sigma < 0.0)
      RCLCPP_ERROR(this->get_logger(), "sigma must be >= 0");
  }

  void MPPIAccNode::controlLoop()
  {
    if (!odom_received_)
      return;

    checkTiming();
    auto start_time = std::chrono::high_resolution_clock::now();

    ensureSDFMap();
    bool ready = isDataReady();

    Eigen::Vector3d des_acc;

    if (ready)
    {
      runMPPI();
      des_acc = u_mean_[0];
      u_prev_ = u_mean_[0];
    }
    else
    {
      des_acc.setZero();
      std::fill(u_mean_.begin(), u_mean_.end(), Eigen::Vector3d::Zero());
      u_prev_.setZero();
    }

    if (common_params_.enable_lpf)
    {
      if (!lpf_initialized_)
      {
        acc_filtered_ = des_acc;
        lpf_initialized_ = true;
      }
      else
      {
        acc_filtered_ = common_params_.lpf_alpha * des_acc + (1.0 - common_params_.lpf_alpha) * acc_filtered_;
      }
    }
    else
    {
      acc_filtered_ = des_acc;
    }

    Eigen::Vector3d force = mass_ * (acc_filtered_ + Eigen::Vector3d(0, 0, common_params_.g));
    double target_yaw = ready ? ref_cmd_.yaw : current_yaw_;

    Eigen::Vector3d b1d(cos(target_yaw), sin(target_yaw), 0);
    Eigen::Vector3d b3c = force.normalized();
    Eigen::Vector3d b2c = b3c.cross(b1d).normalized();
    Eigen::Vector3d b1c = b2c.cross(b3c).normalized();
    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;
    Eigen::Quaterniond orientation(R);

    publishSO3(force, orientation);

    if (ready)
    {
      publish_execution_time(comp_time_pub_, start_time);

      // Shift control sequence
      for (int i = 0; i < common_params_.H - 1; ++i)
      {
        u_mean_[i] = u_mean_[i + 1];
      }
      u_mean_[common_params_.H - 1].setZero();
    }
  }

  void MPPIAccNode::runMPPI()
  {
    std::vector<float3> samples_u(common_params_.K * common_params_.H);
    std::vector<float> costs(common_params_.K);
    std::vector<float3> u_mean_f3(common_params_.H);

    for (int i = 0; i < common_params_.H; ++i)
    {
      u_mean_f3[i] = toFloat3(u_mean_[i]);
    }

    float3 cp = toFloat3(curr_p_);
    float3 cv = toFloat3(curr_v_);
    float3 rp = make_float3(ref_cmd_.position.x, ref_cmd_.position.y, ref_cmd_.position.z);
    float3 rv = make_float3(ref_cmd_.velocity.x, ref_cmd_.velocity.y, ref_cmd_.velocity.z);
    float3 ra = make_float3(ref_cmd_.acceleration.x, ref_cmd_.acceleration.y, ref_cmd_.acceleration.z);
    float3 up = toFloat3(u_prev_);

    launch_mppi_acc_kernel(
        u_mean_f3.data(), up, cp, cv, rp, rv, ra,
        common_params_.K, common_params_.H, common_params_.dt, acc_params_.sigma, common_params_.lambda,
        acc_params_.Q_pos_x, acc_params_.Q_pos_y, acc_params_.Q_pos_z,
        acc_params_.Q_vel_x, acc_params_.Q_vel_y, acc_params_.Q_vel_z,
        acc_params_.R_x, acc_params_.R_y, acc_params_.R_z,
        acc_params_.R_rate_x, acc_params_.R_rate_y, acc_params_.R_rate_z,
        common_params_.w_obs,
        acc_params_.a_max, acc_params_.tilt_max, common_params_.g,
        samples_u.data(), costs.data(), seed_++);

    float min_cost = *std::min_element(costs.begin(), costs.end());
    double sum_weights = 0.0;
    std::vector<double> weights(common_params_.K);

    for (int k = 0; k < common_params_.K; ++k)
    {
      weights[k] = exp(-(costs[k] - min_cost) / common_params_.lambda);
      sum_weights += weights[k];
    }

    // ESS and Lambda Auto-tuning
    float ess = update_lambda_ess(weights, sum_weights, common_params_.lambda, lambda_params_);
    
    std_msgs::msg::Float32 ess_msg;
    ess_msg.data = ess;
    ess_pub_->publish(ess_msg);

    std_msgs::msg::Float32 lambda_msg;
    lambda_msg.data = static_cast<float>(common_params_.lambda);
    lambda_pub_->publish(lambda_msg);

    std::vector<Eigen::Vector3d> new_u_mean(common_params_.H, Eigen::Vector3d::Zero());
    for (int h = 0; h < common_params_.H; ++h)
    {
      for (int k = 0; k < common_params_.K; ++k)
      {
        float3 ukh = samples_u[k * common_params_.H + h];
        new_u_mean[h] += weights[k] * toEigen(ukh);
      }
      new_u_mean[h] /= (sum_weights + 1e-6);
    }
    u_mean_ = new_u_mean;
  }

} // namespace mppi_control

RCLCPP_COMPONENTS_REGISTER_NODE(mppi_control::MPPIAccNode)
