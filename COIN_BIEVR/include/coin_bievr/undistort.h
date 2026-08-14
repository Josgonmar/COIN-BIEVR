#ifndef COIN_BIEVR_UNDISTORT_H_
#define COIN_BIEVR_UNDISTORT_H_

#include "coin_bievr/common.h"
#include "coin_bievr/imu_integrator.h"

namespace coin_bievr {

// Undistorts the spatial coordinates using the per-point time offsets (a view onto
// the cloud's time row) and the cloud start stamp. Point order is preserved, so any
// per-point channel (e.g. intensity) kept aside stays aligned with the output by
// index.
void undistortCloud(const ImuIntegratorPtr& imu_integrator, const State& x,
                    const Pointcloud& pointcloud, const TimeView& times, uint64_t stamp,
                    const V3& G, Pointcloud& undistorted_pc);
}  // namespace coin_bievr

#endif  // COIN_BIEVR_UNDISTORT_H_