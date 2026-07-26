#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autonomy_light
{

class ElevationMapNode final : public rclcpp::Node
{
public:
  ElevationMapNode();

private:
  void validateParameters() const;
  void onOdometry(const nav_msgs::msg::Odometry & message);
  void onRegisteredCloud(const sensor_msgs::msg::PointCloud2 & message);
  void publishElevation(
    const pcl::PointCloud<pcl::PointXYZ> & base_cloud,
    const builtin_interfaces::msg::Time & stamp);
  static Eigen::Isometry3f odomPose(const nav_msgs::msg::Odometry & odom);

  std::string registered_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string base_frame_;
  double resolution_{0.05};
  double x_min_{-2.0};
  double x_max_{2.0};
  double y_min_{-2.0};
  double y_max_{2.0};
  double z_min_{-1.5};
  double z_max_{1.0};
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

}  // namespace autonomy_light
