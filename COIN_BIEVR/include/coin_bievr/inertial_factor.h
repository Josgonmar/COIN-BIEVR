#ifndef COIN_BIEVR_INERTIAL_FACTOR_H_
#define COIN_BIEVR_INERTIAL_FACTOR_H_

#include <ceres/ceres.h>

#include "coin_bievr/common.h"
#include "coin_bievr/imu_integrator.h"

namespace coin_bievr {

class InertialFactor : public ceres::SizedCostFunction<9, 4, 3, 3, 4, 3, 3, 3, 3, 3> {
 public:
  InertialFactor() = delete;
  explicit InertialFactor(ImuIntegratorPtr integrator) : integrator_(integrator) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

 private:
  ImuIntegratorPtr integrator_;
};

}  // namespace coin_bievr

#endif  // COIN_BIEVR_INERTIAL_FACTOR_H_
