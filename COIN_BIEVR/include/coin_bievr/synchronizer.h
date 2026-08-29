#ifndef COIN_BIEVR_SYNCHRONIZER_H_
#define COIN_BIEVR_SYNCHRONIZER_H_

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include "coin_bievr/pipeline.h"

namespace coin_bievr {

class Synchronizer {
 public:
  explicit Synchronizer(std::shared_ptr<Pipeline> pipeline);
  virtual ~Synchronizer();
  bool addImu(const ImuMeasurement& imu);
  bool addPointcloud(const StampedIntensityPointcloud& cloud);
  // Stop after draining all synchronizable frames. This must be called while
  // the Pipeline's publisher owner is still alive.
  void stop();

 private:
  bool synchronizeData();
  void processingWorker();
  std::deque<ImuMeasurement> imu_queue_;
  std::deque<StampedIntensityPointcloud> point_queue_;
  std::shared_ptr<Pipeline> pipeline_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_requested_ = false;
  std::thread processing_thread_;
};
}  // namespace coin_bievr

#endif  // COIN_BIEVR_SYNCHRONIZER_H_
