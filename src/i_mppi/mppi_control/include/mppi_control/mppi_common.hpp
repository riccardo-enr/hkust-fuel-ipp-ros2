#ifndef MPPI_CONTROL__MPPI_COMMON_HPP_
#define MPPI_CONTROL__MPPI_COMMON_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <vector>

namespace mppi_control
{

  struct LambdaAutoTuneParams
  {
    bool auto_lambda;
    float target_ess;
    float lambda_min;
    float lambda_max;
    float lambda_step;
  };

  /**
   * @brief Computes the Effective Sample Size (ESS) and updates lambda.
   *
   * @param weights Normalized weights of the samples.
   * @param sum_weights Sum of all weights.
   * @param lambda Reference to the lambda parameter to be updated.
   * @param params Auto-tuning parameters.
   * @return float The computed ESS.
   */
  inline float update_lambda_ess(
      const std::vector<double> &weights,
      double sum_weights,
      double &lambda,
      const LambdaAutoTuneParams &params)
  {
    if (weights.empty() || sum_weights < std::numeric_limits<double>::epsilon())
      return 0.0f;

    // Map std::vector to Eigen::VectorXd for efficient computation (zero-copy)
    const Eigen::Map<const Eigen::VectorXd> weights_map(weights.data(), static_cast<Eigen::Index>(weights.size()));

    // ESS = 1 / sum((w/sum_w)^2) = (sum_w)^2 / sum(w^2)
    const double sum_sq_weights = weights_map.squaredNorm();
    const float ess = static_cast<float>((sum_weights * sum_weights) / (sum_sq_weights + std::numeric_limits<double>::epsilon()));

    if (params.auto_lambda)
    {
      // Update law: lambda_{new} = lambda_{old} + step * (target_ess - ESS) / target_ess
      const double delta = static_cast<double>(params.lambda_step) * (static_cast<double>(params.target_ess) - ess) / static_cast<double>(params.target_ess);
      lambda = std::clamp(lambda + delta, static_cast<double>(params.lambda_min), static_cast<double>(params.lambda_max));
    }

    return ess;
  }

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_COMMON_HPP_
