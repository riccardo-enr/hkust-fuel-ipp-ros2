#ifndef MPPI_CONTROL__MPPI_COMMON_HPP_
#define MPPI_CONTROL__MPPI_COMMON_HPP_

#include <vector>
#include <cmath>

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
    const std::vector<double>& weights,
    double sum_weights,
    double& lambda,
    const LambdaAutoTuneParams& params)
{
  if (weights.empty() || sum_weights < 1e-9) return 0.0f;

  double sum_sq_weights = 0.0;
  for (double w : weights)
  {
    double normalized_w = w / sum_weights;
    sum_sq_weights += normalized_w * normalized_w;
  }

  float ess = static_cast<float>(1.0 / (sum_sq_weights + 1e-9));

  if (params.auto_lambda)
  {
    // Update law: lambda_{new} = lambda_{old} + step * (target_ess - ESS) / target_ess
    double delta = params.lambda_step * (params.target_ess - ess) / params.target_ess;
    lambda += delta;
    
    // Clamp
    if (lambda < params.lambda_min) lambda = params.lambda_min;
    if (lambda > params.lambda_max) lambda = params.lambda_max;
  }

  return ess;
}

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_COMMON_HPP_
