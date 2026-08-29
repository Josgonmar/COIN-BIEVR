#include "coin_bievr/synchronizer.h"

#include <utility>

#include "coin_bievr/log++.h"
#include "coin_bievr/utils.h"

namespace coin_bievr {

Synchronizer::Synchronizer(std::shared_ptr<Pipeline> pipeline) : pipeline_(std::move(pipeline)) {
  processing_thread_ = std::thread(&Synchronizer::processingWorker, this);
}

Synchronizer::~Synchronizer() { stop(); }

void Synchronizer::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_requested_) return;
    stop_requested_ = true;
  }
  cv_.notify_one();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

bool Synchronizer::addImu(const ImuMeasurement& imu) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!imu_queue_.empty() && imu.stamp < imu_queue_.back().stamp) {
      LOG(W, "IMU measurement at " << imu.stamp << " out of order. Skipping.");
      return false;
    }
    imu_queue_.push_back(imu);
  }

  // Keep a separate output-only IMU queue before synchronization. The IMU
  // worker can propagate this sample while the synchronization worker is
  // processing a LiDAR frame.
  pipeline_->queueImuForOdometry(imu);
  cv_.notify_one();
  return true;
}

bool Synchronizer::addPointcloud(const StampedIntensityPointcloud& cloud) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!point_queue_.empty() && cloud.end_stamp < point_queue_.back().end_stamp) {
      LOG(W, "Pointcloud out of order. Time " << cloud.end_stamp << " smaller than "
                                              << point_queue_.back().end_stamp << ". Skipping.");
      return false;
    }
    point_queue_.push_back(cloud);
  }
  cv_.notify_one();
  return true;
}

bool Synchronizer::synchronizeData() {
  static const uint64_t kOverlapNs = sToNs(0.02);
  std::vector<ImuMeasurement> imu_data;
  StampedIntensityPointcloud pointcloud;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_queue_.size() < 2 || point_queue_.empty()) return false;

    // Clean out point clouds that arrived before IMU.
    while (!point_queue_.empty() &&
           (point_queue_.front().stamp + kOverlapNs) < imu_queue_.front().stamp) {
      LOG(W, "Removed pointcloud: " << point_queue_.front().stamp << " < "
                                     << imu_queue_.front().stamp);
      point_queue_.pop_front();
    }

    if (point_queue_.empty()) return false;

    const uint64_t t_pointcloud = point_queue_.front().end_stamp;
    if (imu_queue_.back().stamp < t_pointcloud) return false;

    imu_data.reserve(imu_queue_.size());
    while (imu_queue_.size() > 1) {
      const ImuMeasurement& imu_curr = imu_queue_[0];
      const ImuMeasurement& imu_next = imu_queue_[1];
      imu_data.push_back(imu_curr);

      if (imu_next.stamp < t_pointcloud) {
        imu_queue_.pop_front();
      } else {
        // Linearly interpolate between the last IMU measurements.
        const uint64_t t_curr = imu_curr.stamp;
        const uint64_t t_next = imu_next.stamp;
        const uint64_t t_diff = t_next - t_curr;
        const double t_ratio = static_cast<double>(t_pointcloud - t_curr) / t_diff;
        ImuMeasurement imu_interpolated;
        imu_interpolated.stamp = t_pointcloud;
        imu_interpolated.acc = imu_curr.acc + t_ratio * (imu_next.acc - imu_curr.acc);
        imu_interpolated.gyro = imu_curr.gyro + t_ratio * (imu_next.gyro - imu_curr.gyro);
        imu_data.push_back(imu_interpolated);
        // Add the interpolated measurement back to the front for the next cloud.
        imu_queue_.front() = imu_interpolated;
        break;
      }
    }

    pointcloud = std::move(point_queue_.front());
    point_queue_.pop_front();
  }

  pipeline_->processFrame(imu_data, pointcloud);
  return true;
}

void Synchronizer::processingWorker() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    cv_.wait(lock, [this] {
      return stop_requested_ || !imu_queue_.empty() || !point_queue_.empty();
    });
    lock.unlock();

    // Drain every frame that is currently synchronizable before waiting again.
    while (synchronizeData()) {
    }

    lock.lock();
    if (stop_requested_) {
      // For bag processing, finish all synchronizable frames before returning.
      // If the final point cloud has no sufficient IMU coverage, it cannot be
      // processed and is intentionally discarded during shutdown.
      lock.unlock();
      while (synchronizeData()) {
      }
      lock.lock();
      if (imu_queue_.empty() || point_queue_.empty()) return;
      // Remaining data is not synchronizable; do not spin forever on shutdown.
      return;
    }
  }
}

}  // namespace coin_bievr
