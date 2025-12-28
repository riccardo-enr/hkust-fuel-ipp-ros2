#ifndef MPPI_CONTROL__MPPI_TQ_NODE_HPP_
#define MPPI_CONTROL__MPPI_TQ_NODE_HPP_

#include "mppi_control/mppi_node_base.hpp"
#include <cuda_runtime.h>

namespace mppi_control {

struct ControlInput {
  float thrust;
  float4 quat;
};

struct ControlSample {
  float thrust;
  float4 quat;
};

// CUDA Kernel Launcher Declaration
extern "C" void launch_mppi_tq_kernel(
    const ControlInput* u_mean_host,
    ControlInput u_prev,
    float3 curr_p, float3 curr_v,
    float3 ref_p, float3 ref_v, float3 ref_a,
    float4 ref_quat, float ref_thrust,
    int K, int H, float dt, float lambda,
    float sigma_thrust, float sigma_quat,
    float Q_pos_x, float Q_pos_y, float Q_pos_z,
    float Q_vel_x, float Q_vel_y, float Q_vel_z,
    float Q_thrust, float R_thrust, float R_rate_thrust,
    float Q_quat, float R_quat, float R_rate_quat,
    float w_obs,
    float thrust_max, float thrust_min, float g,
    ControlSample* samples_u_host,
    float* costs_host,
    unsigned int seed
);

struct MPPITqParams {
  double sigma_thrust;
  double sigma_quat;
  double Q_pos_x, Q_pos_y, Q_pos_z;
  double Q_vel_x, Q_vel_y, Q_vel_z;
  double Q_thrust;
  double R_thrust;
  double R_rate_thrust;
  double Q_quat;
  double R_quat;
  double R_rate_quat;
  double thrust_max;
  double thrust_min;
};

class MPPITqNode : public MPPINodeBase {
public:
  MPPITqNode(const rclcpp::NodeOptions& options);

private:
  void controlLoop() override;
  void runMPPI();
  void validateAndLogParameters();

  MPPITqParams tq_params_;
  std::vector<ControlInput> u_mean_; 
  ControlInput u_prev_;
  uint32_t seed_{0};
  
  // Low-pass filter state
  ControlInput control_filtered_; 
  bool lpf_initialized_;
};

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_TQ_NODE_HPP_