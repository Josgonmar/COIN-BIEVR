#ifndef COIN_BIEVR_SYNCHRONIZER_H_
#define COIN_BIEVR_SYNCHRONIZER_H_

#include <deque>

#include "coin_bievr/pipeline.h"

namespace coin_bievr {

class Synchronizer {
 public:
  explicit Synchronizer(std::shared_ptr<Pipeline> pipeline) : pipeline_(pipeline) {}
  virtual ~Synchronizer() = default;
  bool addImu(const ImuMeasurement& imu);
  bool addPointcloud(const StampedIntensityPointcloud& cloud);

 private:
  void synchronizeData();
  std::deque<ImuMeasurement> imu_queue_;
  std::deque<StampedIntensityPointcloud> point_queue_;
  std::shared_ptr<Pipeline> pipeline_;
};
}  // namespace coin_bievr

#endif  // COIN_BIEVR_SYNCHRONIZER_H_