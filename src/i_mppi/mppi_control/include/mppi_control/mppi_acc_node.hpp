#ifndef MPPI_CONTROL__MPPI_ACC_NODE_HPP_
#define MPPI_CONTROL__MPPI_ACC_NODE_HPP_

#include "mppi_control/mppi_node_base.hpp"
#include <cuda_runtime.h>

namespace mppi_control {

// CUDA Kernel Launcher Declaration
extern "C" void launch_mppi_acc_kernel(
    const float3* u_mean_host,
    float3 u_prev,
    float3 curr_p, float3 curr_v,
    float3 ref_p, float3 ref_v, float3 ref_a,
    int K, int H, float dt, float sigma, float lambda,
    float Q_pos_x, float Q_pos_y, float Q_pos_z,
    float Q_vel_x, float Q_vel_y, float Q_vel_z,
    float R_x, float R_y, float R_z,
    float R_rate_x, float R_rate_y, float R_rate_z,
    float w_obs,
    float a_max, float tilt_max, float g,
    float3* samples_u_host,
    float* costs_host,
    uint32_t seed
);

struct MPPIAccParams {
  double sigma;
  double Q_pos_x, Q_pos_y, Q_pos_z;
  double Q_vel_x, Q_vel_y, Q_vel_z;
  double R_x, R_y, R_z;
  double R_rate_x, R_rate_y, R_rate_z;
  double a_max;
  double tilt_max;
};

class MPPIAccNode : public MPPINodeBase {
public:
  MPPIAccNode(const rclcpp::NodeOptions& options);

private:
  void controlLoop() override;
  void validateAndLogParameters();
  void runMPPI();

  MPPIAccParams acc_params_;
  std::vector<Eigen::Vector3d> u_mean_; // Mean control sequence
  Eigen::Vector3d u_prev_;              // Previous control command
  uint32_t seed_{0};
  
  // Low-pass filter state
  Eigen::Vector3d acc_filtered_; 
  bool lpf_initialized_;
};

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_ACC_NODE_HPP_