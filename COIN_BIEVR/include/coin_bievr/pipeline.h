#ifndef COIN_BIEVR_PIPELINE_H_
#define COIN_BIEVR_PIPELINE_H_

#include <deque>
#include <typeindex>

#include "coin_bievr/coin_bievr_map.h"
#include "coin_bievr/imu_integrator.h"
#include "coin_bievr/intensity_processing.h"
#include "coin_bievr/log++.h"
#include "coin_bievr/ls_optimizer.h"
#include "coin_bievr/preprocess.h"
#include "coin_bievr/utils.h"

namespace coin_bievr {

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    IntensityProcessingConfig intensity;
    RegistrationConfig registration;
    COINBIEVRMap::Config map;
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. coin_bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string log_path = "";

    size_t min_points_for_map_init = 100;
    size_t map_size_running_threshold = 5;
    size_t informed_sample_count = 300;
    size_t min_map_size_for_imu_opt = 100;
    size_t min_imu_integrators_for_opt = 5;
    double gravity_prior_weight = 5.0;
  };

  explicit Pipeline(const Config& config);
  virtual ~Pipeline() = default;

  void processFrame(const std::vector<ImuMeasurement>& imu_data,
                    const StampedIntensityPointcloud& pointcloud);

  // Output-only IMU-rate propagation. These samples are not inserted into the
  // LiDAR-time estimator state window or inertial optimization graph.
  void queueImuForOdometry(const ImuMeasurement& imu);
  void propagatePendingImuOdometry();

  template <typename T>
  void registerPublisher(std::function<void(const T&, const Header&, const std::string& topic,
                                            const std::string& child_frame)>
                             func) {
    auto wrapper = [func](const void* val, const Header& header, const std::string& topic,
                          const std::string& child_frame) {
      func(*static_cast<const T*>(val), header, topic, child_frame);
    };
    publishers_[typeid(T)] = wrapper;
  }

 private:
  enum class Phase { NeedBias, NeedMap, Running };

  // Pipeline helpers
  bool initializeBias(const std::vector<ImuMeasurement>& imu_data,
                      const IntensityPointcloud& pointcloud);
  void tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                  const Pointcloud& undistorted, const Intensities& intensities,
                  std::vector<double>& ranges, const Header& header,
                  const ImuMeasurement& anchor_imu);
  void sampleSource(const Pointcloud& undistorted, const Intensities& intensities,
                    const Transform& T_W_I_init, Pointcloud& filtered, Pointcloud& coarse,
                    Pointcloud& fine, IntensityPointcloud& intensity) const;

  // State and optimization management
  bool addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v);
  bool addImuIntegrator(ImuIntegratorPtr imu_integrator);
  bool optimizeInertialWindow();

  // Publishing
  template <typename T>
  void publish(const T& value, const Header& header, const std::string& topic,
               const std::string& child_frame = "") const {
    auto it = publishers_.find(typeid(T));
    if (it != publishers_.end()) {
      it->second(static_cast<const void*>(&value), header, topic, child_frame);
    } else {
      LOG(E, "No publisher registered in pipeline for type.");
    }
  }

  void publishFrame(const Header& header, const Transform& T_W_I, const Pointcloud& full_registered,
                    const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                    const Pointcloud& source_fine, const Pointcloud& undistorted,
                    const Intensities& intensities, const ImuMeasurement& anchor_imu);
  void publishLatestState(const Header& header, const ImuMeasurement& anchor_imu);
  void publishDebugClouds(const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                          const Pointcloud& source_fine, const Pointcloud& undistorted_cloud,
                          const Intensities& intensities, const Transform& T_W_I,
                          const Header& header);

  struct ImuOdometrySample {
    ImuMeasurement imu;
    State predicted_state;
  };

  void resetImuOdometry(const ImuMeasurement& anchor_imu);
  void predictImuOdometrySamples(bool publish);
  void publishImuOdometry(const ImuOdometrySample& sample);
  ImuMeasurement scaledImu(const ImuMeasurement& imu) const;

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::shared_ptr<COINBIEVRMap> map_;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  V3 gravity_dir_ = V3(0, 0, 1);
  // Latest gyro reading, used to report the angular velocity in the odometry twist.
  V3 latest_gyro_ = V3::Zero();
  // Accelerometer scale resolved during bias estimation (1 if raw, g if the IMU
  // reports gravity-normalized accelerations). Applied to all incoming IMU data.
  double imu_acc_scale_ = 1.0;

  // Output-only IMU-rate propagation. The queue is never inserted into states_
  // or imu_integrators_.
  std::deque<ImuOdometrySample> imu_odom_queue_;
  ImuIntegratorPtr imu_odom_integrator_;
  State imu_odom_anchor_;
  uint64_t imu_odom_last_integrated_stamp_ = 0;
  uint64_t imu_odom_last_published_stamp_ = 0;
  size_t imu_odom_seq_counter_ = 0;

  using PublishFunction =
      std::function<void(const void*, const Header&, const std::string&, const std::string&)>;
  std::unordered_map<std::type_index, PublishFunction> publishers_;
  std::shared_ptr<std::ofstream> tum_log_;

  // Accumulated state for the live status dashboard (printDashboard in utils).
  DashboardState dashboard_;
};
}  // namespace coin_bievr

#endif  // COIN_BIEVR_PIPELINE_H_
