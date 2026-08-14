#ifndef COIN_BIEVR_PRIOR_FACTOR_H_
#define COIN_BIEVR_PRIOR_FACTOR_H_

#include <ceres/ceres.h>

#include "coin_bievr/common.h"

namespace coin_bievr {

class PriorFactor : public ceres::SizedCostFunction<3, 3> {
 public:
  PriorFactor() = delete;
  PriorFactor(const V3& value, const double weight) : prior_(value), weight_(weight) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

 private:
  V3 prior_;
  double weight_;
};

}  // namespace coin_bievr

#endif  // COIN_BIEVR_PRIOR_FACTOR_H_