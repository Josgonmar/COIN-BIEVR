#ifndef COIN_BIEVR_ROS_COMMON_CONVERSIONS_H_
#define COIN_BIEVR_ROS_COMMON_CONVERSIONS_H_

// Shared message <-> LIO type conversions for both the ROS1 and ROS2 wrappers.
// The ROS1 and ROS2 message structs expose identical field layouts, so the
// conversions are written once here as templates. The handful of genuine
// ROS1-vs-ROS2 differences are resolved generically with `if constexpr` over
// feature-detection concepts:
//   - header stamp -> ns/seconds: ros::Time has toNSec()/toSec(); the ROS2
//     builtin_interfaces::msg::Time exposes .sec/.nanosec fields instead.
//   - std_msgs/Header `seq`: present on ROS1 only.
// The only conversions that can't be written generically are the Livox
// CustomMsg overloads (at the bottom of this header): the two driver
// generations are structurally identical so they can't be told apart
// generically, and they disagree on which field holds the scan base stamp.
//
// Which distro's message headers to include is detected with __has_include, so
// this single header serves both wrappers; a wrapper can override by predefining
// COIN_BIEVR_ROS_COMMON_ROS1 / COIN_BIEVR_ROS_COMMON_ROS2 (e.g. if both distros are
// installed on the include path).

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include "coin_bievr/common.h"
#include "coin_bievr/log++.h"
#include "coin_bievr/timing.h"
#include "coin_bievr/utils.h"

// --- ROS version detection + message headers ------------------------------
#if !defined(COIN_BIEVR_ROS_COMMON_ROS1) && !defined(COIN_BIEVR_ROS_COMMON_ROS2)
#if __has_include(<rclcpp/rclcpp.hpp>)
#define COIN_BIEVR_ROS_COMMON_ROS2
#elif __has_include(<ros/ros.h>)
#define COIN_BIEVR_ROS_COMMON_ROS1
#else
#error \
    "coin_bievr_ros_common/conversions.h: found neither ROS2 (<rclcpp/rclcpp.hpp>) nor ROS1 (<ros/ros.h>)"
#endif
#endif

#ifdef COIN_BIEVR_ROS_COMMON_ROS2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#ifdef COIN_BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#else  // COIN_BIEVR_ROS_COMMON_ROS1
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#ifdef COIN_BIEVR_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#ifdef COIN_BIEVR_WITH_LIVOX2
#include <livox_ros_driver2/CustomMsg.h>
#endif
#endif

namespace coin_bievr {
namespace conv_detail {

// Above this value a "timestamp" field is interpreted as nanoseconds, below it
// as seconds.
constexpr double kTimestampNsThreshold = 2770392337.0;

// Feature-detection concepts. These wrap the requires-expressions in named
// concepts rather than using a requires-expression inline inside `if constexpr`
// (the latter ICEs GCC 9's concepts-TS, the compiler used for the ROS1 build).

// A sensor_msgs/PointCloud2 of either ROS version (used to constrain the generic
// msgToPointcloud overloads so Livox CustomMsg never matches them).
template <typename T>
concept PointCloud2Like = requires(T m) {
  m.fields;
  m.point_step;
  m.data;
};

// geometry_msgs/Pose vs geometry_msgs/Transform (disjoint member names).
template <typename T>
concept PoseLike = requires(T m) {
  m.position;
  m.orientation;
};
template <typename T>
concept TransformLike = requires(T m) {
  m.translation;
  m.rotation;
};

// Stamp/header API differences between ROS1 (ros::Time, std_msgs/Header with
// `seq`) and ROS2 (builtin_interfaces::msg::Time with .sec/.nanosec, no `seq`).
template <typename T>
concept HasToNSec = requires(const T& t) { t.toNSec(); };
template <typename T>
concept HasToSec = requires(const T& t) { t.toSec(); };
template <typename T>
concept HasFromNSec = requires(T& t) { t.fromNSec(std::uint64_t{}); };
template <typename T>
concept HasSeq = requires(T& t) { t.seq; };

// Reads a scalar of type T from a (possibly unaligned) byte address.
template <typename T>
inline T readAt(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

template <typename PointFieldT>
bool isNumericField(const PointFieldT& field) {
  return field.datatype >= PointFieldT::INT8 && field.datatype <= PointFieldT::FLOAT64;
}

template <typename PointFieldT>
double readNumericField(const uint8_t* p, const PointFieldT& field) {
  switch (field.datatype) {
    case PointFieldT::INT8:
      return static_cast<double>(readAt<int8_t>(p));
    case PointFieldT::UINT8:
      return static_cast<double>(readAt<uint8_t>(p));
    case PointFieldT::INT16:
      return static_cast<double>(readAt<int16_t>(p));
    case PointFieldT::UINT16:
      return static_cast<double>(readAt<uint16_t>(p));
    case PointFieldT::INT32:
      return static_cast<double>(readAt<int32_t>(p));
    case PointFieldT::UINT32:
      return static_cast<double>(readAt<uint32_t>(p));
    case PointFieldT::FLOAT32:
      return static_cast<double>(readAt<float>(p));
    case PointFieldT::FLOAT64:
      return readAt<double>(p);
    default:
      return 0.0;
  }
}

// How the per-point time field is encoded in the message.
enum class TimeEncoding {
  kNanosUint32,   // "t": uint32 nanoseconds, relative to scan start (Ouster)
  kSecondsFloat,  // "time": float32 seconds, relative to an arbitrary origin (Velodyne)
  kStampDouble,   // "timestamp": float64 absolute seconds (or ns above a threshold) (Hesai)
};

template <typename FieldT>
const FieldT* findField(const std::vector<FieldT>& fields, const std::string& name) {
  auto it =
      std::find_if(fields.cbegin(), fields.cend(), [&](const FieldT& f) { return f.name == name; });
  return it == fields.cend() ? nullptr : &*it;
}

// Header stamp -> nanoseconds since epoch, for either ROS version.
template <typename StampT>
inline uint64_t stampToNs(const StampT& stamp) {
  if constexpr (HasToNSec<StampT>) {
    return static_cast<uint64_t>(stamp.toNSec());  // ros::Time
  } else {
    return static_cast<uint64_t>(stamp.sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(stamp.nanosec);  // builtin_interfaces::msg::Time
  }
}

// Header stamp -> seconds since epoch, for either ROS version.
template <typename StampT>
inline double stampToS(const StampT& stamp) {
  if constexpr (HasToSec<StampT>) {
    return stamp.toSec();  // ros::Time
  } else {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;  // builtin_interfaces::msg::Time
  }
}

// Livox CustomMsg -> StampedIntensityPointcloud. `base_stamp_ns` is the scan
// base timestamp (per-driver: header stamp for gen1, `timebase` for gen2) and
// `timer_name` is the timing label. Called from the wrappers' Livox overloads.
template <typename LivoxMsgT>
bool livoxToStampedIntensity(const LivoxMsgT& pointcloud_msg, uint64_t base_stamp_ns,
                             const char* timer_name,
                             StampedIntensityPointcloud& stamped_pointcloud) {
  timing::Timer conversion_timer(timer_name);

  stamped_pointcloud.stamp = base_stamp_ns;
  if (stamped_pointcloud.stamp == 0) {
    LOG(W, "Skipping Livox pointcloud without valid timestamp.");
    return false;
  }

  const size_t num_points = pointcloud_msg.point_num;
  if (num_points == 0) {
    LOG(W, "Skipping empty Livox pointcloud with timestamp " << base_stamp_ns << ".");
    return false;
  }

  stamped_pointcloud.resize(num_points);

  size_t counter = 0;
  constexpr double kToSeconds = 1e-9;  // offset_time is in nanoseconds

  for (const auto& pt : pointcloud_msg.points) {
    const double time = static_cast<double>(pt.offset_time) * kToSeconds;  // seconds
    if (time > 0.2) {
      stamped_pointcloud.data().col(counter) << 0, 0, 0, 0, 0;
    } else {
      stamped_pointcloud.data().col(counter) << pt.x, pt.y, pt.z, time,
          static_cast<double>(pt.reflectivity);
    }
    ++counter;
  }

  if (counter != num_points) {
    LOG(E, "Count doesn't match number of points in Livox message!");
  }

  // Adjust timestamps so the first point is at t = 0
  auto times = stamped_pointcloud.times();
  const double min_time = times.minCoeff();
  stamped_pointcloud.stamp -= sToNs(std::abs(min_time));
  times.array() -= min_time;

  const double time_span = stamped_pointcloud.times().maxCoeff();
  if (time_span > 0.5) {
    LOG(W, "Livox pointcloud spans " << time_span
                                     << " s between its earliest and latest point (> 0.5 s)."
                                     << "Seems like the time conversion  is wrong.");
  }

  stamped_pointcloud.end_stamp =
      stamped_pointcloud.stamp + sToNs(stamped_pointcloud.times().maxCoeff());

  return true;
}

}  // namespace conv_detail

// ---------------------------------------------------------------------------
// Message -> LIO
// ---------------------------------------------------------------------------

// PointCloud2 -> StampedIntensityPointcloud.
template <typename PointCloud2T>
  requires conv_detail::PointCloud2Like<PointCloud2T>
bool msgToPointcloud(const PointCloud2T& pointcloud_msg,
                     StampedIntensityPointcloud& stamped_pointcloud) {
  using namespace conv_detail;
  timing::Timer conversion_timer("00_conversion_pc2");

  const uint64_t stamp_ns = stampToNs(pointcloud_msg.header.stamp);
  stamped_pointcloud.stamp = stamp_ns;
  if (stamped_pointcloud.stamp == 0) {
    LOG(W, "Skipping pointcloud without valid timestamp.");
    return false;
  }

  // Skip empty clouds
  const size_t num_points = pointcloud_msg.height * pointcloud_msg.width;
  if (num_points == 0) {
    LOG(W, "Skipping empty pointcloud with timestamp " << stamp_ns << ".");
    return false;
  }

  using PointFieldT = typename PointCloud2T::_fields_type::value_type;

  // Resolve field byte offsets once; the parallel loop below then reads each point
  // directly from the flat buffer. xyz are three contiguous float32 at the x field.
  const auto& fields = pointcloud_msg.fields;
  const PointFieldT* x_field = findField(fields, "x");
  if (x_field == nullptr || (x_field + 1)->name != "y" || (x_field + 2)->name != "z") {
    LOG(W, "Received pointcloud with missing or out-of-order x/y/z fields");
    return false;
  }
  const uint32_t off_x = x_field->offset;

  // Resolve the time field and its encoding.
  TimeEncoding enc;
  uint32_t off_t;
  if (const auto* f = findField(fields, "t")) {
    enc = TimeEncoding::kNanosUint32;
    off_t = f->offset;
  } else if (const auto* f = findField(fields, "time")) {
    enc = TimeEncoding::kSecondsFloat;
    off_t = f->offset;
  } else if (const auto* f = findField(fields, "timestamp")) {
    enc = TimeEncoding::kStampDouble;
    off_t = f->offset;
  } else {
    LOG(E, "Pointcloud has no recognized time field (t/time/timestamp); skipping.");
    return false;
  }

  // COIN-BIEVR requires a numeric intensity channel. Some drivers call the
  // equivalent field "reflectivity", and inexpensive/irregular sensors commonly
  // encode it as an integer rather than float32.
  const PointFieldT* i_field = findField(fields, "intensity");
  if (i_field == nullptr) i_field = findField(fields, "reflectivity");
  if (i_field == nullptr || !isNumericField(*i_field)) {
    LOG(E, "Pointcloud has no numeric 'intensity' or 'reflectivity' field required by "
           "COIN-BIEVR; skipping.");
    return false;
  }
  const uint32_t off_i = i_field->offset;

  stamped_pointcloud.resize(num_points);
  auto& data = stamped_pointcloud.data();
  const uint8_t* base = pointcloud_msg.data.data();
  const size_t step = pointcloud_msg.point_step;
  const double stamp_s = stampToS(pointcloud_msg.header.stamp);

  // Fill xyz, time and intensity in a single parallel pass. Each point writes its
  // own (contiguous, column-major) column, so the writes are disjoint.
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, num_points), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          const uint8_t* p = base + i * step;
          auto col = data.col(i);

          double time;
          bool valid = true;
          switch (enc) {
            case TimeEncoding::kNanosUint32:
              time = static_cast<double>(readAt<uint32_t>(p + off_t)) * 1e-9;
              valid = time <= 0.2;  // reject obviously bogus (wrapped) stamps
              break;
            case TimeEncoding::kSecondsFloat:
              time = static_cast<double>(readAt<float>(p + off_t));
              break;
            case TimeEncoding::kStampDouble: {
              const double raw = readAt<double>(p + off_t);
              time = (raw > kTimestampNsThreshold ? raw * 1e-9 : raw) - stamp_s;
              break;
            }
          }

          if (valid) {
            col(0) = readAt<float>(p + off_x);
            col(1) = readAt<float>(p + off_x + 4);
            col(2) = readAt<float>(p + off_x + 8);
            col(3) = time;
          } else {
            col(0) = col(1) = col(2) = col(3) = 0.0;
          }
          col(4) = valid ? readNumericField(p + off_i, *i_field) : 0.0;
        }
      });

  // A relative "time" field is offset from an arbitrary origin; shift so the first
  // point sits at t = 0 (matching the original behaviour).
  if (enc == TimeEncoding::kSecondsFloat) {
    auto times = stamped_pointcloud.times();
    const double min_time = times.minCoeff();
    stamped_pointcloud.stamp -= sToNs(std::abs(min_time));
    times.array() -= min_time;
  }

  const auto times = stamped_pointcloud.times();
  const double time_span = times.maxCoeff() - times.minCoeff();
  if (time_span > 0.5) {
    LOG(W, "Pointcloud spans " << time_span
                               << " s between its earliest and latest point (> 0.5 s). "
                               << "Seems like the time conversion  is wrong.");
  }

  stamped_pointcloud.end_stamp = stamped_pointcloud.stamp + sToNs(times.maxCoeff());
  return true;
}

// PointCloud2 -> Pointcloud (xyz only).
template <typename PointCloud2T>
  requires conv_detail::PointCloud2Like<PointCloud2T>
bool msgToPointcloud(const PointCloud2T& pointcloud_msg, Pointcloud& pointcloud) {
  using namespace conv_detail;
  timing::Timer conversion_timer("00_conversion_xyz");

  // Skip empty clouds
  const size_t num_points = pointcloud_msg.height * pointcloud_msg.width;
  if (num_points == 0) {
    LOG(W, "Skipping empty pointcloud with timestamp " << stampToNs(pointcloud_msg.header.stamp)
                                                       << ".");
    return false;
  }

  using PointFieldT = typename PointCloud2T::_fields_type::value_type;
  const auto& fields = pointcloud_msg.fields;
  const PointFieldT* x_field = findField(fields, "x");
  if (x_field == nullptr || (x_field + 1)->name != "y" || (x_field + 2)->name != "z") {
    LOG(W, "Received pointcloud with missing or out-of-order x/y/z fields");
    return false;
  }
  const uint32_t off_x = x_field->offset;

  pointcloud.resize(num_points);
  const uint8_t* base = pointcloud_msg.data.data();
  const size_t step = pointcloud_msg.point_step;
  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t* p = base + i * step;
    pointcloud.data().col(i) << readAt<float>(p + off_x), readAt<float>(p + off_x + 4),
        readAt<float>(p + off_x + 8);
  }
  return true;
}

template <typename PointMsgT>
bool msgToPoint(const PointMsgT& msg, Point& point) {
  point.x() = msg.x;
  point.y() = msg.y;
  point.z() = msg.z;
  return true;
}

template <typename ImuMsgT>
bool msgToImuMeasurement(const ImuMsgT& imu_msg, ImuMeasurement& imu) {
  imu.stamp = conv_detail::stampToNs(imu_msg.header.stamp);
  if (imu.stamp == 0) {
    LOG(W, "Skipping imu message without valid timestamp.");
    return false;
  }

  imu.acc << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z;
  imu.gyro << imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z;

  return true;
}

template <typename OdomMsgT>
bool msgToTransform(const OdomMsgT& odom_msg, Transform& transform) {
  transform.translation() << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
      odom_msg.pose.pose.position.z;
  transform.linear() =
      Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
                         odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z)
          .toRotationMatrix();
  return true;
}

// ---------------------------------------------------------------------------
// LIO -> Message
// ---------------------------------------------------------------------------

// Pointcloud -> PointCloud2 (xyz only).
template <typename PointCloud2T>
  requires conv_detail::PointCloud2Like<PointCloud2T>
void pointCloudToMsg(const Pointcloud& pointcloud, PointCloud2T& pointcloud_msg) {
  // Set up the PointCloud2 message metadata
  pointcloud_msg.height = 1;                 // Unordered point cloud (1D list of points)
  pointcloud_msg.width = pointcloud.size();  // Number of points
  pointcloud_msg.is_bigendian = false;
  pointcloud_msg.is_dense = true;  // Assume no NaN values in the input cloud

  // Define fields: x, y, z (each as a float32)
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(pointcloud.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");

  for (size_t i = 0; i < pointcloud.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = static_cast<float>(pointcloud[i].x());
    *iter_y = static_cast<float>(pointcloud[i].y());
    *iter_z = static_cast<float>(pointcloud[i].z());
  }
}

// IntensityPointcloud -> PointCloud2 (xyz + intensity).
template <typename PointCloud2T>
  requires conv_detail::PointCloud2Like<PointCloud2T>
void pointCloudToMsg(const IntensityPointcloud& pointcloud, PointCloud2T& pointcloud_msg) {
  using PointFieldT = typename PointCloud2T::_fields_type::value_type;

  // Set up the PointCloud2 message metadata
  pointcloud_msg.height = 1;                 // Unordered point cloud (1D list of points)
  pointcloud_msg.width = pointcloud.size();  // Number of points
  pointcloud_msg.is_bigendian = false;
  pointcloud_msg.is_dense = true;  // Assume no NaN values in the input cloud

  // Define fields: x, y, z, intensity (each as a float32)
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  modifier.setPointCloud2Fields(4, "x", 1, PointFieldT::FLOAT32, "y", 1, PointFieldT::FLOAT32, "z",
                                1, PointFieldT::FLOAT32, "intensity", 1, PointFieldT::FLOAT32);
  modifier.resize(pointcloud.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(pointcloud_msg, "intensity");

  const auto intensities = pointcloud.intensities();
  for (size_t i = 0; i < pointcloud.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
    *iter_x = static_cast<float>(pointcloud[i].x());
    *iter_y = static_cast<float>(pointcloud[i].y());
    *iter_z = static_cast<float>(pointcloud[i].z());
    *iter_i = static_cast<float>(intensities(i));
  }
}

template <typename HeaderMsgT>
void headerToMsg(const Header& header, HeaderMsgT& header_msg) {
  using StampT = std::remove_reference_t<decltype(header_msg.stamp)>;
  if constexpr (conv_detail::HasFromNSec<StampT>) {
    header_msg.stamp.fromNSec(header.stamp);  // ros::Time
  } else {
    header_msg.stamp.sec = static_cast<std::int32_t>(header.stamp / 1'000'000'000ULL);
    header_msg.stamp.nanosec = static_cast<std::uint32_t>(header.stamp % 1'000'000'000ULL);
  }
  header_msg.frame_id = header.frame;
  if constexpr (conv_detail::HasSeq<HeaderMsgT>) {
    header_msg.seq = header.seq;  // ROS1 std_msgs/Header only
  }
}

// Transform -> geometry_msgs/Pose (position + orientation).
template <typename PoseMsgT>
  requires conv_detail::PoseLike<PoseMsgT>
void transformToMsg(const Transform& transform, PoseMsgT& pose_msg) {
  const auto& translation = transform.translation();
  pose_msg.position.x = translation.x();
  pose_msg.position.y = translation.y();
  pose_msg.position.z = translation.z();

  const Quaternion q = transform.quaternion();
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();
}

// Transform -> geometry_msgs/Transform (translation + rotation).
template <typename TransformMsgT>
  requires conv_detail::TransformLike<TransformMsgT>
void transformToMsg(const Transform& transform, TransformMsgT& transform_msg) {
  const auto& translation = transform.translation();
  transform_msg.translation.x = translation.x();
  transform_msg.translation.y = translation.y();
  transform_msg.translation.z = translation.z();

  const Quaternion q = transform.quaternion();
  transform_msg.rotation.x = q.x();
  transform_msg.rotation.y = q.y();
  transform_msg.rotation.z = q.z();
  transform_msg.rotation.w = q.w();
}

template <typename Vec3MsgT>
void vecToMsg(const V3& vec, Vec3MsgT& msg) {
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
}

// Livox CustomMsg -> StampedIntensityPointcloud. The only per-distro overloads:
// the gen1/gen2 messages are structurally identical (so they can't be told apart
// generically) and disagree on the scan base-stamp field (gen1: header stamp;
// gen2: `timebase`). Gated by the COIN_BIEVR_WITH_LIVOX[2] compile definitions.
#ifdef COIN_BIEVR_ROS_COMMON_ROS2
#ifdef COIN_BIEVR_WITH_LIVOX  // ROS2 has only the gen2 driver (livox_ros_driver2)
inline bool msgToPointcloud(const livox_ros_driver2::msg::CustomMsg& pointcloud_msg,
                            StampedIntensityPointcloud& stamped_pointcloud) {
  return conv_detail::livoxToStampedIntensity(pointcloud_msg, pointcloud_msg.timebase,
                                              "00_conversion_livox2", stamped_pointcloud);
}
#endif
#else                    // COIN_BIEVR_ROS_COMMON_ROS1
#ifdef COIN_BIEVR_WITH_LIVOX  // gen1: base stamp in the message header
inline bool msgToPointcloud(const livox_ros_driver::CustomMsg& pointcloud_msg,
                            StampedIntensityPointcloud& stamped_pointcloud) {
  return conv_detail::livoxToStampedIntensity(pointcloud_msg, pointcloud_msg.header.stamp.toNSec(),
                                              "00_conversion_livox", stamped_pointcloud);
}
#endif
#ifdef COIN_BIEVR_WITH_LIVOX2  // gen2: base stamp in `timebase`
inline bool msgToPointcloud(const livox_ros_driver2::CustomMsg& pointcloud_msg,
                            StampedIntensityPointcloud& stamped_pointcloud) {
  return conv_detail::livoxToStampedIntensity(pointcloud_msg, pointcloud_msg.timebase,
                                              "00_conversion_livox2", stamped_pointcloud);
}
#endif
#endif

}  // namespace coin_bievr

#endif  // COIN_BIEVR_ROS_COMMON_CONVERSIONS_H_
