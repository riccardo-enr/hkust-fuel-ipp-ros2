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
    float lambda_min;
    float lambda_max;

    // Gradient-based tuning (Jang's method)
    float alpha_fluctuation;  // Weight for fluctuation term
    float beta_performance;    // Weight for performance term
    float learning_rate;       // Gradient descent step size
    float lambda_perturbation; // For finite difference gradient estimation
  };

  /**
   * @brief Jang's gradient-based lambda auto-tuning.
   *
   * Optimizes J(λ) = α * Fluctuation(λ) + β * Performance(λ)
   * where:
   *   - Fluctuation: Variance of weighted mean (low λ → high jitter)
   *   - Performance: Expected cost (high λ → high cost)
   *
   * @param costs Sample costs [K].
   * @param samples Sample trajectories [K x H x dim].
   * @param lambda Reference to lambda parameter.
   * @param params Auto-tuning parameters.
   * @return float The objective value J(λ) for monitoring.
   */
  inline float update_lambda_gradient(
      const std::vector<float> &costs,
      const std::vector<Eigen::Vector3d> &samples,  // [K*H] flattened
      int H,
      double &lambda,
      const LambdaAutoTuneParams &params)
  {
    const int K = static_cast<int>(costs.size());
    if (K == 0 || samples.empty())
      return 0.0f;

    const double eps = std::numeric_limits<double>::epsilon();

    // Helper: compute weights given lambda
    auto compute_weights = [&costs, &eps, K](double lam) {
      float min_cost = *std::min_element(costs.begin(), costs.end());
      std::vector<double> w(K);
      double sum_w = 0.0;
      for (int i = 0; i < K; ++i)
      {
        w[i] = std::exp(-(costs[i] - min_cost) / lam);
        sum_w += w[i];
      }
      // Normalize
      for (int i = 0; i < K; ++i)
      {
        w[i] /= (sum_w + eps);
      }
      return w;
    };

    // Helper: compute objective J(λ) = α * Fluctuation + β * Performance
    auto compute_objective = [&samples, H, K, &eps](
        const std::vector<double> &weights,
        const std::vector<float> &costs_vec) {
      // Compute weighted mean trajectory
      std::vector<Eigen::Vector3d> weighted_mean(H, Eigen::Vector3d::Zero());
      for (int h = 0; h < H; ++h)
      {
        for (int k = 0; k < K; ++k)
        {
          weighted_mean[h] += weights[k] * samples[k * H + h];
        }
      }

      // Fluctuation: variance of samples around weighted mean
      double fluctuation = 0.0;
      for (int k = 0; k < K; ++k)
      {
        for (int h = 0; h < H; ++h)
        {
          Eigen::Vector3d diff = samples[k * H + h] - weighted_mean[h];
          fluctuation += weights[k] * diff.squaredNorm();
        }
      }

      // Performance: expected cost
      double performance = 0.0;
      for (int k = 0; k < K; ++k)
      {
        performance += weights[k] * costs_vec[k];
      }

      return fluctuation;
    };

    if (!params.auto_lambda)
      return 0.0f;

    // Gradient descent using finite differences
    const double h = static_cast<double>(params.lambda_perturbation);

    // Compute weights at current lambda
    std::vector<double> w = compute_weights(lambda);
    double J = compute_objective(w, costs);

    // Compute weights at lambda + h
    std::vector<double> w_perturbed = compute_weights(lambda + h);
    double J_perturbed = compute_objective(w_perturbed, costs);

    // Finite difference gradient
    double grad = (J_perturbed - J) / h;

    // Gradient descent step
    double delta = -static_cast<double>(params.learning_rate) * grad;
    lambda = std::clamp(lambda + delta, static_cast<double>(params.lambda_min), static_cast<double>(params.lambda_max));

    return static_cast<float>(J);
  }

} // namespace mppi_control

#endif // MPPI_CONTROL__MPPI_COMMON_HPP_
