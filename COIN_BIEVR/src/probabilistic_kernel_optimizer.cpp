#include "coin_bievr/probabilistic_kernel_optimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

namespace coin_bievr {

namespace {

constexpr double kVarianceFloor = 1e-12;
constexpr double kWeightFloor = 1e-12;
constexpr double kPi = 3.14159265358979323846;
constexpr int kEvaluationBins = 256;
constexpr int kMaxEmIterations = 50;
constexpr double kEmTolerance = 1e-6;

double clampProbability(double value) {
  return std::max(value, kWeightFloor);
}

}  // namespace

ProbabilisticKernelOptimizer::ProbabilisticKernelOptimizer(
    const ProbabilisticKernelConfig& config)
    : config_(config), reference_delta_(config.max_delta) {
  config_.min_delta = std::max(config_.min_delta, kWeightFloor);
  config_.max_delta = std::max(config_.max_delta, config_.min_delta);
  config_.num_candidates = std::max(config_.num_candidates, 1);
  config_.truncation = std::max(config_.truncation, config_.max_delta);
  config_.gmm_components = std::max(config_.gmm_components, 1);
  config_.gmm_sample_size = std::max(config_.gmm_sample_size, 1);
  reference_delta_ = config_.max_delta;
  candidates_.resize(static_cast<size_t>(config_.num_candidates) + 1);
  for (int i = 0; i <= config_.num_candidates; ++i) {
    const double t = static_cast<double>(i) / config_.num_candidates;
    // Log-like spacing gives the search more resolution near the final robust
    // threshold while still retaining a broad initial GNC scale.
    const double interpolation = (std::pow(100.0, t) - 1.0) / 99.0;
    candidates_[static_cast<size_t>(i)] =
        config_.min_delta + (config_.max_delta - config_.min_delta) * interpolation;
  }
}

void ProbabilisticKernelOptimizer::reset() {
  reference_delta_ = config_.max_delta;
  gmm_weights_.clear();
  gmm_means_.clear();
  gmm_variances_.clear();
}

double ProbabilisticKernelOptimizer::estimate(const std::vector<double>& residuals) {
  std::vector<double> finite_absolute_residuals;
  finite_absolute_residuals.reserve(residuals.size());
  for (const double residual : residuals) {
    if (std::isfinite(residual)) finite_absolute_residuals.push_back(std::abs(residual));
  }

  const size_t minimum_samples =
      static_cast<size_t>(std::max(10, 3 * config_.gmm_components));
  if (finite_absolute_residuals.size() < minimum_samples) return reference_delta_;

  fitGmm(finite_absolute_residuals);

  double best_delta = reference_delta_;
  double best_cost = std::numeric_limits<double>::max();
  for (const double candidate : candidates_) {
    if (candidate > reference_delta_ * (1.0 + 1e-12)) continue;
    const double cost = jsDivergence(candidate);
    if (cost < best_cost) {
      best_cost = cost;
      best_delta = candidate;
    }
  }

  reference_delta_ = std::clamp(best_delta, config_.min_delta, reference_delta_);
  return reference_delta_;
}

void ProbabilisticKernelOptimizer::fitGmm(const std::vector<double>& residuals) {
  const int components = config_.gmm_components;
  const int sample_size = std::min(config_.gmm_sample_size,
                                  static_cast<int>(residuals.size()));

  std::vector<double> sample = residuals;
  std::mt19937 generator(42);
  std::shuffle(sample.begin(), sample.end(), generator);
  sample.resize(static_cast<size_t>(sample_size));
  std::sort(sample.begin(), sample.end());

  gmm_weights_.assign(static_cast<size_t>(components), 1.0 / components);
  gmm_means_.resize(static_cast<size_t>(components));
  gmm_variances_.assign(static_cast<size_t>(components), kVarianceFloor);

  double mean = std::accumulate(sample.begin(), sample.end(), 0.0) / sample.size();
  double variance = 0.0;
  for (const double value : sample) {
    const double difference = value - mean;
    variance += difference * difference;
  }
  variance = std::max(variance / sample.size(), kVarianceFloor);

  for (int component = 0; component < components; ++component) {
    const size_t index = static_cast<size_t>(
        (static_cast<double>(component) + 0.5) * sample.size() / components);
    gmm_means_[static_cast<size_t>(component)] = sample[std::min(index, sample.size() - 1)];
    gmm_variances_[static_cast<size_t>(component)] = variance;
  }

  std::vector<double> responsibilities(
      static_cast<size_t>(sample_size * components), 0.0);
  for (int iteration = 0; iteration < kMaxEmIterations; ++iteration) {
    for (int i = 0; i < sample_size; ++i) {
      double normalization = 0.0;
      for (int component = 0; component < components; ++component) {
        const size_t index = static_cast<size_t>(i * components + component);
        responsibilities[index] =
            std::max(gmm_weights_[static_cast<size_t>(component)], kWeightFloor) *
            gaussianPdf(sample[static_cast<size_t>(i)],
                        gmm_means_[static_cast<size_t>(component)],
                        gmm_variances_[static_cast<size_t>(component)]);
        normalization += responsibilities[index];
      }
      normalization = std::max(normalization, kWeightFloor);
      for (int component = 0; component < components; ++component) {
        responsibilities[static_cast<size_t>(i * components + component)] /= normalization;
      }
    }

    std::vector<double> next_weights(static_cast<size_t>(components));
    std::vector<double> next_means(static_cast<size_t>(components));
    std::vector<double> next_variances(static_cast<size_t>(components));
    double parameter_change = 0.0;
    for (int component = 0; component < components; ++component) {
      double effective_count = 0.0;
      double weighted_mean = 0.0;
      for (int i = 0; i < sample_size; ++i) {
        const double responsibility =
            responsibilities[static_cast<size_t>(i * components + component)];
        effective_count += responsibility;
        weighted_mean += responsibility * sample[static_cast<size_t>(i)];
      }
      effective_count = std::max(effective_count, kWeightFloor);
      next_weights[static_cast<size_t>(component)] = effective_count / sample_size;
      next_means[static_cast<size_t>(component)] = weighted_mean / effective_count;

      double weighted_variance = 0.0;
      for (int i = 0; i < sample_size; ++i) {
        const double difference = sample[static_cast<size_t>(i)] -
                                  next_means[static_cast<size_t>(component)];
        weighted_variance +=
            responsibilities[static_cast<size_t>(i * components + component)] *
            difference * difference;
      }
      next_variances[static_cast<size_t>(component)] =
          std::max(weighted_variance / effective_count, kVarianceFloor);
      parameter_change +=
          std::abs(next_means[static_cast<size_t>(component)] -
                   gmm_means_[static_cast<size_t>(component)]);
    }

    gmm_weights_ = std::move(next_weights);
    gmm_means_ = std::move(next_means);
    gmm_variances_ = std::move(next_variances);
    if (parameter_change < kEmTolerance) break;
  }
}

double ProbabilisticKernelOptimizer::gaussianPdf(double x, double mean,
                                                  double variance) const {
  const double safe_variance = std::max(variance, kVarianceFloor);
  const double difference = x - mean;
  return std::exp(-0.5 * difference * difference / safe_variance) /
         std::sqrt(2.0 * kPi * safe_variance);
}

double ProbabilisticKernelOptimizer::huberWeight(double residual, double delta) const {
  const double absolute_residual = std::abs(residual);
  if (absolute_residual <= delta) return 1.0;
  return delta / std::max(absolute_residual, kWeightFloor);
}

double ProbabilisticKernelOptimizer::partitionFunction(double delta) const {
  const double step = config_.truncation / kEvaluationBins;
  double integral = 0.0;
  for (int i = 0; i < kEvaluationBins; ++i) {
    const double residual = static_cast<double>(i) * step;
    integral += huberWeight(residual, delta) * step;
  }
  return std::max(integral, kWeightFloor);
}

double ProbabilisticKernelOptimizer::jsDivergence(double delta) const {
  const double step = config_.truncation / kEvaluationBins;
  const double partition = partitionFunction(delta);
  std::vector<double> data_mass(kEvaluationBins);
  std::vector<double> kernel_mass(kEvaluationBins);
  double data_total = 0.0;
  double kernel_total = 0.0;

  for (int i = 0; i < kEvaluationBins; ++i) {
    const double residual = static_cast<double>(i) * step;
    double data_density = 0.0;
    for (size_t component = 0; component < gmm_weights_.size(); ++component) {
      data_density += gmm_weights_[component] *
                      gaussianPdf(residual, gmm_means_[component],
                                  gmm_variances_[component]);
    }
    data_mass[static_cast<size_t>(i)] = std::max(data_density * step, kWeightFloor);
    kernel_mass[static_cast<size_t>(i)] =
        std::max(huberWeight(residual, delta) * step / partition, kWeightFloor);
    data_total += data_mass[static_cast<size_t>(i)];
    kernel_total += kernel_mass[static_cast<size_t>(i)];
  }

  double divergence = 0.0;
  for (int i = 0; i < kEvaluationBins; ++i) {
    const double p = clampProbability(data_mass[static_cast<size_t>(i)] / data_total);
    const double q = clampProbability(kernel_mass[static_cast<size_t>(i)] / kernel_total);
    const double midpoint = 0.5 * (p + q);
    divergence += 0.5 * (p * std::log(p / midpoint) + q * std::log(q / midpoint));
  }
  return divergence;
}

}  // namespace coin_bievr
