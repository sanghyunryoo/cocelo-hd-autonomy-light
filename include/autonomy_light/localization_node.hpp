#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "autonomy_light/slam/saved_map_localizer.hpp"

namespace autonomy_light
{

class LocalizationNode final : public rclcpp::Node
{
public:
  LocalizationNode();

private:
  struct TimedOdom
  {
    double stamp_seconds{};
    nav_msgs::msg::Odometry message;
  };

  void onOdometry(const nav_msgs::msg::Odometry & message);
  void onCloud(const sensor_msgs::msg::PointCloud2 & message);
  void updateLocalization();
  void broadcastMapToOdom();
  void publishSavedMap();
  [[nodiscard]] bool matchingOdometry(double stamp, nav_msgs::msg::Odometry & output) const;
  [[nodiscard]] static Eigen::Isometry3d poseFromOdometry(const nav_msgs::msg::Odometry & odom);
  [[nodiscard]] static double seconds(const builtin_interfaces::msg::Time & stamp);

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  mutable std::mutex mutex_;
  std::deque<TimedOdom> odom_cache_;
  slam::SavedMapLocalizer localizer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr refined_map_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace autonomy_light
