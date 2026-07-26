#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace autonomy_light
{

struct LidarMergeConfig
{
  std::string target_frame;
  tf2::Vector3 target_to_lidar1_translation{0.0, 0.0, 0.0};
  tf2::Matrix3x3 target_to_lidar1_rotation{tf2::Quaternion::getIdentity()};
  tf2::Vector3 target_to_lidar2_translation{0.0, 0.0, 0.0};
  tf2::Matrix3x3 target_to_lidar2_rotation{tf2::Quaternion::getIdentity()};
  double sync_tolerance_sec{0.005};
  int max_queue_size{8};
  bool publish_lidar1_on_sync_miss{false};
};

// Converts raw LiDAR messages into target_frame and synchronizes two scans.
// It deliberately does not own subscriptions or publishers, so transport and
// process lifecycle remain with AutonomyLightNode.
class LidarMerger
{
public:
  LidarMerger(
    LidarMergeConfig config,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);

  std::vector<sensor_msgs::msg::PointCloud2> ingestCustom(
    int lidar_index,
    const livox_ros_driver2::msg::CustomMsg & message,
    bool * accepted_input = nullptr);

  std::vector<sensor_msgs::msg::PointCloud2> ingestPointCloud(
    int lidar_index,
    const sensor_msgs::msg::PointCloud2 & message,
    bool * accepted_input = nullptr);

private:
  struct TimedPoint
  {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    float intensity{0.0F};
    double time_offset{0.0};
  };

  struct TimedCloud
  {
    rclcpp::Time stamp{0, 0u, RCL_SYSTEM_TIME};
    std::vector<TimedPoint> points;
  };

  TimedPoint transformPoint(
    int lidar_index,
    float x,
    float y,
    float z,
    float intensity,
    double time_offset) const;
  std::vector<sensor_msgs::msg::PointCloud2> pushCloud(int lidar_index, TimedCloud cloud);
  std::vector<sensor_msgs::msg::PointCloud2> tryBuildMergedLocked();
  sensor_msgs::msg::PointCloud2 buildMerged(
    const TimedCloud & lidar1,
    const TimedCloud * lidar2) const;

  LidarMergeConfig config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mutex_;
  std::deque<TimedCloud> lidar1_queue_;
  std::deque<TimedCloud> lidar2_queue_;
};

}  // namespace autonomy_light
