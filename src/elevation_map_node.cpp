#include "autonomy_light/elevation_map_node.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace autonomy_light
{

ElevationMapNode::ElevationMapNode()
: Node("elevation_map")
{
  registered_topic_ = declare_parameter<std::string>("registered_topic", "/cloud_registered");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
  output_topic_ = declare_parameter<std::string>("output_topic", "/autonomy_light/elevation_map");
  base_frame_ = declare_parameter<std::string>("base_frame", "hd/base");
  resolution_ = declare_parameter<double>("resolution", 0.05);
  x_min_ = declare_parameter<double>("x_min", -2.0);
  x_max_ = declare_parameter<double>("x_max", 2.0);
  y_min_ = declare_parameter<double>("y_min", -2.0);
  y_max_ = declare_parameter<double>("y_max", 2.0);
  z_min_ = declare_parameter<double>("z_min", -1.5);
  z_max_ = declare_parameter<double>("z_max", 1.0);
  validateParameters();

  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 2);
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr message) { onOdometry(*message); });
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr message) { onRegisteredCloud(*message); });

  RCLCPP_INFO(
    get_logger(), "Elevation map: registered=%s odom=%s output=%s frame=%s",
    registered_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str());
}

void ElevationMapNode::validateParameters() const
{
  if (resolution_ <= 0.0 || x_max_ <= x_min_ || y_max_ <= y_min_ || z_max_ <= z_min_) {
    throw std::invalid_argument("elevation map bounds and resolution must be positive");
  }
}

void ElevationMapNode::onOdometry(const nav_msgs::msg::Odometry & message)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = message;
  have_odom_ = true;
}

void ElevationMapNode::onRegisteredCloud(const sensor_msgs::msg::PointCloud2 & message)
{
  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      return;
    }
    odom = latest_odom_;
  }

  pcl::PointCloud<pcl::PointXYZ> registered;
  try {
    pcl::fromROSMsg(message, registered);
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Ignoring registered cloud: %s", error.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> base_cloud;
  pcl::transformPointCloud(registered, base_cloud, odomPose(odom).inverse().matrix());
  publishElevation(base_cloud, message.header.stamp);
}

Eigen::Isometry3f ElevationMapNode::odomPose(const nav_msgs::msg::Odometry & odom)
{
  Eigen::Quaternionf orientation(
    static_cast<float>(odom.pose.pose.orientation.w),
    static_cast<float>(odom.pose.pose.orientation.x),
    static_cast<float>(odom.pose.pose.orientation.y),
    static_cast<float>(odom.pose.pose.orientation.z));
  if (orientation.norm() == 0.0F) {
    orientation = Eigen::Quaternionf::Identity();
  } else {
    orientation.normalize();
  }

  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() = orientation.toRotationMatrix();
  transform.translation() = Eigen::Vector3f(
    static_cast<float>(odom.pose.pose.position.x),
    static_cast<float>(odom.pose.pose.position.y),
    static_cast<float>(odom.pose.pose.position.z));
  return transform;
}

void ElevationMapNode::publishElevation(
  const pcl::PointCloud<pcl::PointXYZ> & base_cloud,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto width = static_cast<std::size_t>(std::ceil((x_max_ - x_min_) / resolution_));
  const auto height = static_cast<std::size_t>(std::ceil((y_max_ - y_min_) / resolution_));
  std::vector<float> cells(width * height, std::numeric_limits<float>::quiet_NaN());

  for (const auto & point : base_cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      point.x < x_min_ || point.x >= x_max_ || point.y < y_min_ || point.y >= y_max_ ||
      point.z < z_min_ || point.z > z_max_)
    {
      continue;
    }
    const auto x_index = static_cast<std::size_t>((point.x - x_min_) / resolution_);
    const auto y_index = static_cast<std::size_t>((point.y - y_min_) / resolution_);
    auto & cell = cells[y_index * width + x_index];
    if (!std::isfinite(cell) || point.z < cell) {
      cell = point.z;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> elevation;
  elevation.reserve(cells.size());
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const float z = cells[y * width + x];
      if (std::isfinite(z)) {
        elevation.push_back(pcl::PointXYZ(
          static_cast<float>(x_min_ + (static_cast<double>(x) + 0.5) * resolution_),
          static_cast<float>(y_min_ + (static_cast<double>(y) + 0.5) * resolution_), z));
      }
    }
  }
  elevation.width = static_cast<std::uint32_t>(elevation.size());
  elevation.height = 1;
  elevation.is_dense = true;

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(elevation, output);
  output.header.stamp = stamp;
  output.header.frame_id = base_frame_;
  publisher_->publish(output);
}

}  // namespace autonomy_light
