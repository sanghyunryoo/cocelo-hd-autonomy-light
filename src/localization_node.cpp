#include "autonomy_light/localization_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace autonomy_light
{

namespace
{
slam::SavedMapLocalizerConfig readLocalizerConfig(rclcpp::Node & node)
{
  slam::SavedMapLocalizerConfig config;
  config.scan_voxel_size = node.declare_parameter<double>("localization.scan_voxel_size", 0.2);
  config.map_voxel_size = node.declare_parameter<double>("localization.map_voxel_size", 0.2);
  config.feature_voxel_size = node.declare_parameter<double>("localization.feature_voxel_size", 0.5);
  config.initial_submap_seconds = node.declare_parameter<double>("localization.initial_submap_seconds", 2.0);
  config.minimum_points = static_cast<std::size_t>(node.declare_parameter<int>("localization.minimum_points", 500));
  config.maximum_initial_points = static_cast<std::size_t>(node.declare_parameter<int>("localization.maximum_initial_points", 30000));
  config.maximum_feature_points = static_cast<std::size_t>(node.declare_parameter<int>("localization.maximum_feature_points", 6000));
  config.normal_radius = node.declare_parameter<double>("localization.normal_radius", 0.8);
  config.feature_radius = node.declare_parameter<double>("localization.feature_radius", 1.2);
  config.target_radius = node.declare_parameter<double>("localization.target_radius", 20.0);
  config.update_period_seconds = node.declare_parameter<double>("localization.update_period_seconds", 1.0);
  config.filter_alpha = node.declare_parameter<double>("localization.filter_alpha", 0.2);
  config.max_translation_step = node.declare_parameter<double>("localization.max_translation_step", 1.0);
  config.max_rotation_step_radians = node.declare_parameter<double>("localization.max_rotation_step_radians", 0.35);
  return config;
}

slam::NanoGicpRegistration readRegistration(rclcpp::Node & node)
{
  return {node.declare_parameter<double>("registration.voxel_size", 0.4),
    node.declare_parameter<double>("registration.max_correspondence_distance", 3.0),
    static_cast<int>(node.declare_parameter<int>("registration.correspondence_randomness", 20)),
    static_cast<int>(node.declare_parameter<int>("registration.max_iterations", 32)),
    node.declare_parameter<double>("registration.max_fitness", 0.5)};
}
}  // namespace

LocalizationNode::LocalizationNode()
: Node("saved_map_localization"),
  odom_topic_(declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init")),
  cloud_topic_(declare_parameter<std::string>("cloud_topic", "/cloud_registered")),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  localizer_(readLocalizerConfig(*this), readRegistration(*this))
{
  const std::string map_file = declare_parameter<std::string>("saved_map_file", "");
  if (map_file.empty()) {
    throw std::invalid_argument("LOCALIZATION requires a non-empty saved_map_file");
  }
  localizer_.load(map_file);
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr message) {onOdometry(*message);});
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {onCloud(*message);});
  map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_lio/global_map", rclcpp::QoS(1).transient_local());
  refined_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_lio/global_map_refined", rclcpp::QoS(1).transient_local());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const double tf_publish_hz = declare_parameter<double>("tf.publish_hz", 20.0);
  if (tf_publish_hz <= 0.0) {
    throw std::invalid_argument("tf.publish_hz must be positive");
  }
  localization_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {updateLocalization();});
  tf_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / tf_publish_hz)),
    [this] {broadcastMapToOdom();});
  publishSavedMap();
  RCLCPP_INFO(get_logger(), "LOCALIZATION loaded %zu map points from %s", localizer_.map().size(), map_file.c_str());
}

void LocalizationNode::onOdometry(const nav_msgs::msg::Odometry & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_cache_.push_back({seconds(message.header.stamp), message});
  while (odom_cache_.size() > 200U) {odom_cache_.pop_front();}
}

void LocalizationNode::onCloud(const sensor_msgs::msg::PointCloud2 & message)
{
  nav_msgs::msg::Odometry odom;
  const double stamp = seconds(message.header.stamp);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!matchingOdometry(stamp, odom)) {return;}
  }
  slam::Cloud cloud;
  try {
    pcl::fromROSMsg(message, cloud);
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Ignoring Point-LIO cloud: %s", error.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  localizer_.addScan(cloud, poseFromOdometry(odom));
}

void LocalizationNode::updateLocalization()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto result = localizer_.update();
  if (result && !result->converged) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Saved-map localization rejected (fitness=%.3f)", result->fitness);
  }
}

void LocalizationNode::broadcastMapToOdom()
{
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (localizer_.initialized()) {
      correction = localizer_.mapFromOdom();
    }
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.translation.x = correction.translation().x();
  transform.transform.translation.y = correction.translation().y();
  transform.transform.translation.z = correction.translation().z();
  const Eigen::Quaterniond orientation(correction.rotation());
  transform.transform.rotation.w = orientation.w();
  transform.transform.rotation.x = orientation.x();
  transform.transform.rotation.y = orientation.y();
  transform.transform.rotation.z = orientation.z();
  tf_broadcaster_->sendTransform(transform);
}

void LocalizationNode::publishSavedMap()
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(localizer_.map(), message);
  message.header.stamp = now();
  message.header.frame_id = map_frame_;
  map_publisher_->publish(message);
  refined_map_publisher_->publish(message);
}

bool LocalizationNode::matchingOdometry(const double stamp, nav_msgs::msg::Odometry & output) const
{
  if (odom_cache_.empty()) {return false;}
  const auto iterator = std::min_element(odom_cache_.begin(), odom_cache_.end(),
    [stamp](const TimedOdom & lhs, const TimedOdom & rhs) {
      return std::abs(lhs.stamp_seconds - stamp) < std::abs(rhs.stamp_seconds - stamp);
    });
  if (std::abs(iterator->stamp_seconds - stamp) > 0.05) {return false;}
  output = iterator->message;
  return true;
}

Eigen::Isometry3d LocalizationNode::poseFromOdometry(const nav_msgs::msg::Odometry & odom)
{
  Eigen::Quaterniond orientation(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  orientation = orientation.squaredNorm() < 1e-6 ? Eigen::Quaterniond::Identity() : orientation.normalized();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = orientation.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  return pose;
}

double LocalizationNode::seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace autonomy_light
