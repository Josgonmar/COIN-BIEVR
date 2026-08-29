#ifndef COIN_BIEVR_PROBABILISTIC_KERNEL_OPTIMIZER_H_
#define COIN_BIEVR_PROBABILISTIC_KERNEL_OPTIMIZER_H_

#include <vector>

namespace coin_bievr {

// Configuration for the PKO scale search. The delta and truncation values are
// expressed in the same residual units as the samples passed to estimate().
struct ProbabilisticKernelConfig {
  double min_delta = 0.001;
  double max_delta = 1.0;
  int num_candidates = 50;
  // Huber's IRLS weight is not integrable on [0, infinity), so this finite
  // support is part of the Huber-PKO model rather than an accidental cutoff.
  double truncation = 10.0;
  int gmm_components = 3;
  int gmm_sample_size = 1000;
};

// Estimates the Huber IRLS threshold from the current residual distribution.
// One instance should be used for one residual family (for example geometry
// or photometry), because PKO's probability model is defined in residual
// units and must not mix quantities with different scales.
class ProbabilisticKernelOptimizer {
 public:
  explicit ProbabilisticKernelOptimizer(const ProbabilisticKernelConfig& config);

  void reset();

  // Returns a non-increasing scale during one registration. The scale starts
  // at max_delta and is reduced only to candidates below the previous scale,
  // which supplies the graduated-non-convexity schedule.
  double estimate(const std::vector<double>& residuals);

 private:
  void fitGmm(const std::vector<double>& residuals);
  double gaussianPdf(double x, double mean, double variance) const;
  double huberWeight(double residual, double delta) const;
  double partitionFunction(double delta) const;
  double jsDivergence(double delta) const;

  ProbabilisticKernelConfig config_;
  double reference_delta_;
  std::vector<double> candidates_;
  std::vector<double> gmm_weights_;
  std::vector<double> gmm_means_;
  std::vector<double> gmm_variances_;
};

}  // namespace coin_bievr

#endif  // COIN_BIEVR_PROBABILISTIC_KERNEL_OPTIMIZER_H_
