#include <coin_bievr/common.h>
#include <coin_bievr/log++.h>
#include <coin_bievr/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>

#include "coin_bievr/config_loader.h"
#include "coin_bievr_ros2/publisher.h"
#include "coin_bievr_ros_common/conversions.h"
#ifdef COIN_BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace {
template <typename T>
T deserialize(const rclcpp::SerializedMessage& serialized) {
  T msg;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &msg);
  return msg;
}
}  // namespace

int main(int argc, char** argv) {
  srand(1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("coin_bievr_bag_node");

  coin_bievr::Config config;
  // rclcpp::init does not strip ROS arguments from argv; remove_ros_arguments
  // returns just the application arguments (our config-file flags).
  if (!coin_bievr::loadConfigFromArgs(rclcpp::remove_ros_arguments(argc, argv), config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  auto pipeline = std::make_shared<coin_bievr::Pipeline>(config.pipeline_config);
  auto synchronizer = std::make_shared<coin_bievr::Synchronizer>(pipeline);
  auto lio_pub = std::make_shared<coin_bievr::Publisher>(node, pipeline, "coin_bievr");

  rosbag2_cpp::Reader reader;
  reader.open(config.topic_config.bag_path);

  // Map each topic to its message type (needed to pick the right deserializer).
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  const std::string& pc_topic = config.topic_config.pointcloud_topic;
  const std::string& imu_topic = config.topic_config.imu_topic;

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string& topic = bag_msg->topic_name;
    if (topic != pc_topic && topic != imu_topic) {
      continue;
    }

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    const std::string& type = topic_types[topic];

    if (type == "sensor_msgs/msg/PointCloud2") {
      auto msg = deserialize<sensor_msgs::msg::PointCloud2>(serialized);
      coin_bievr::StampedIntensityPointcloud pointcloud;
      if (coin_bievr::msgToPointcloud(msg, pointcloud)) {
        synchronizer->addPointcloud(pointcloud);
      }
    }
#ifdef COIN_BIEVR_WITH_LIVOX
    else if (type == "livox_ros_driver2/msg/CustomMsg") {
      auto msg = deserialize<livox_ros_driver2::msg::CustomMsg>(serialized);
      coin_bievr::StampedIntensityPointcloud pointcloud;
      if (coin_bievr::msgToPointcloud(msg, pointcloud)) {
        synchronizer->addPointcloud(pointcloud);
      }
    }
#endif
    else if (type == "sensor_msgs/msg/Imu") {
      auto msg = deserialize<sensor_msgs::msg::Imu>(serialized);
      coin_bievr::ImuMeasurement imu;
      coin_bievr::msgToImuMeasurement(msg, imu);
      synchronizer->addImu(imu);
    }
  }

  LOG(I, "Done with bag.");
  reader.close();
  LOG(I, "Bag closed");

  rclcpp::shutdown();
  return 0;
}
