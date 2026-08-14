#include <coin_bievr/common.h>
#include <coin_bievr/log++.h>
#include <coin_bievr/synchronizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include "coin_bievr/config_loader.h"
#include "coin_bievr_ros/publisher.h"
#include "coin_bievr_ros_common/conversions.h"
#ifdef COIN_BIEVR_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#ifdef COIN_BIEVR_WITH_LIVOX2
#include <livox_ros_driver2/CustomMsg.h>
#endif

int main(int argc, char** argv) {
  srand(1);
  // ros::init strips ROS-specific arguments (remappings, __name:=, etc.) from
  // argv in place, leaving our config-file flags for loadConfigFromArgs.
  ros::init(argc, argv, "coin_bievr_bag_node");
  ros::NodeHandle nh;

  coin_bievr::Config config;
  if (!coin_bievr::loadConfigFromArgs({argv, argv + argc}, config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  std::shared_ptr<coin_bievr::Pipeline> pipeline =
      std::make_shared<coin_bievr::Pipeline>(config.pipeline_config);

  coin_bievr::Synchronizer synchronizer(pipeline);

  coin_bievr::Publisher lio_pub(nh, pipeline, "coin_bievr");
  rosbag::Bag bag;
  bag.open(config.topic_config.bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics = {config.topic_config.pointcloud_topic,
                                     config.topic_config.imu_topic};

  rosbag::View bag_view(bag, rosbag::TopicQuery(topics));
  for (const rosbag::MessageInstance& msg : bag_view) {
    if (!ros::ok()) {
      break;
    }

    if ((msg.getDataType() == "sensor_msgs/PointCloud2")) {
      sensor_msgs::PointCloud2::ConstPtr s = msg.instantiate<sensor_msgs::PointCloud2>();
      coin_bievr::StampedIntensityPointcloud pointcloud;
      if (coin_bievr::msgToPointcloud(*s, pointcloud)) {
        synchronizer.addPointcloud(pointcloud);
      }
    }
#ifdef COIN_BIEVR_WITH_LIVOX
    else if ((msg.getDataType() == "livox_ros_driver/CustomMsg")) {
      livox_ros_driver::CustomMsg::ConstPtr s = msg.instantiate<livox_ros_driver::CustomMsg>();
      coin_bievr::StampedIntensityPointcloud pointcloud;
      if (coin_bievr::msgToPointcloud(*s, pointcloud)) {
        synchronizer.addPointcloud(pointcloud);
      }
    }
#endif
#ifdef COIN_BIEVR_WITH_LIVOX2
    else if ((msg.getDataType() == "livox_ros_driver2/CustomMsg")) {
      livox_ros_driver2::CustomMsg::ConstPtr s = msg.instantiate<livox_ros_driver2::CustomMsg>();
      coin_bievr::StampedIntensityPointcloud pointcloud;
      if (coin_bievr::msgToPointcloud(*s, pointcloud)) {
        synchronizer.addPointcloud(pointcloud);
      }
    }
#endif
    else if (msg.getDataType() == "sensor_msgs/Imu") {
      sensor_msgs::Imu::ConstPtr s = msg.instantiate<sensor_msgs::Imu>();
      coin_bievr::ImuMeasurement imu;
      coin_bievr::msgToImuMeasurement(*s, imu);
      synchronizer.addImu(imu);
    }
  }

  LOG(I, "Done with for loop.");
  bag.close();
  LOG(I, "Bag closed");

  return 0;
}
