#include "autonomy_light/full_slam_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace autonomy_light
{

FullSlamNode::FullSlamNode()
: Node("full_slam"),
  odom_topic_(declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init")),
  cloud_topic_(declare_parameter<std::string>("cloud_topic", "/cloud_registered")),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  keyframe_distance_(declare_parameter<double>("keyframe.distance", 1.0)),
  keyframe_angle_radians_(declare_parameter<double>("keyframe.angle_radians", 0.35)),
  loop_exclude_recent_(static_cast<std::size_t>(declare_parameter<int>("loop.exclude_recent", 30))),
  loop_candidate_count_(static_cast<std::size_t>(declare_parameter<int>("loop.candidate_count", 10))),
  loop_descriptor_distance_(declare_parameter<double>("loop.max_descriptor_distance", 0.18)),
  submap_neighbor_count_(static_cast<std::size_t>(declare_parameter<int>("loop.submap_neighbors", 3))),
  map_voxel_size_(declare_parameter<double>("map.voxel_size", 0.15)),
  raw_map_file_(declare_parameter<std::string>("map.raw_pcd_file", "maps/point_lio_global_raw.pcd")),
  refined_map_file_(declare_parameter<std::string>("map.refined_pcd_file", "maps/point_lio_global_refined.pcd")),
  map_save_period_seconds_(declare_parameter<double>("map.save_period_seconds", 30.0)),
  map_builder_({map_voxel_size_, declare_parameter<double>("map.refined_voxel_size", 0.03),
      static_cast<int>(declare_parameter<int>("map.refinement.mean_k", 16)),
      declare_parameter<double>("map.refinement.stddev_multiplier", 0.8)}),
  place_index_(
    declare_parameter<int>("place_recognition.rings", 20),
    declare_parameter<int>("place_recognition.sectors", 60),
    declare_parameter<double>("place_recognition.max_radius", 80.0)),
  registration_(
    declare_parameter<double>("registration.voxel_size", 0.4),
    declare_parameter<double>("registration.max_correspondence_distance", 3.0),
    declare_parameter<int>("registration.correspondence_randomness", 20),
    declare_parameter<int>("registration.max_iterations", 32),
    declare_parameter<double>("registration.max_fitness", 0.5))
{
  if (keyframe_distance_ <= 0.0 || keyframe_angle_radians_ <= 0.0 ||
    loop_exclude_recent_ == 0U || loop_candidate_count_ == 0U || map_voxel_size_ <= 0.0 ||
    map_save_period_seconds_ < 0.0)
  {
    throw std::invalid_argument("full SLAM parameters must be positive");
  }
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr message) {onOdometry(*message);});
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {onCloud(*message);});
  map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_lio/global_map", rclcpp::QoS(1).transient_local());
  refined_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_lio/global_map_refined", rclcpp::QoS(1).transient_local());
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("/point_lio/optimized_path", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const double period = declare_parameter<double>("loop.period_seconds", 1.0);
  const double tf_publish_hz = declare_parameter<double>("tf.publish_hz", 20.0);
  if (period <= 0.0 || tf_publish_hz <= 0.0) {
    throw std::invalid_argument("loop.period_seconds and tf.publish_hz must be positive");
  }
  loop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(period)),
    [this] {processOneLoop();});
  tf_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / tf_publish_hz)),
    [this] {broadcastMapToOdom();});
  RCLCPP_INFO(get_logger(), "FULL_SLAM consumes %s and %s; Point-LIO remains odom TF owner",
    odom_topic_.c_str(), cloud_topic_.c_str());
}

FullSlamNode::~FullSlamNode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (keyframes_.empty()) {return;}
  try {
    saveMapProducts(buildMapProducts());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Final global-map save failed: %s", error.what());
  }
}

void FullSlamNode::onOdometry(const nav_msgs::msg::Odometry & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_cache_.push_back({seconds(message.header.stamp), message});
  while (odom_cache_.size() > 200U) {
    odom_cache_.pop_front();
  }
}

void FullSlamNode::onCloud(const sensor_msgs::msg::PointCloud2 & message)
{
  nav_msgs::msg::Odometry odom;
  const double stamp = seconds(message.header.stamp);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!matchingOdometry(stamp, odom)) {
      return;
    }
  }
  slam::CloudPtr cloud(new slam::Cloud);
  try {
    pcl::fromROSMsg(message, *cloud);
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Ignoring Point-LIO cloud: %s", error.what());
    return;
  }
  if (cloud->empty()) {
    return;
  }
  const Eigen::Isometry3d odom_from_body = poseFromOdometry(odom);
  pcl::transformPointCloud(*cloud, *cloud, odom_from_body.inverse().matrix().cast<float>());

  std::lock_guard<std::mutex> lock(mutex_);
  if (!shouldCreateKeyframe(odom_from_body)) {
    return;
  }
  slam::Keyframe keyframe;
  keyframe.id = keyframes_.size();
  keyframe.stamp_seconds = stamp;
  keyframe.odom_from_body = odom_from_body;
  keyframe.local_cloud = cloud;
  pose_graph_.addOdometryKeyframe(keyframe.id, keyframe.odom_from_body);
  place_index_.add(*keyframe.local_cloud);
  keyframes_.push_back(std::move(keyframe));
  pending_loop_ids_.push_back(keyframes_.back().id);
  map_dirty_ = true;
}

void FullSlamNode::processOneLoop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_loop_ids_.empty()) {
    const std::size_t current_id = pending_loop_ids_.front();
    pending_loop_ids_.pop_front();
    if (current_id >= loop_exclude_recent_) {
      const auto candidate = place_index_.findCandidate(
        *keyframes_[current_id].local_cloud, loop_exclude_recent_, loop_candidate_count_,
        loop_descriptor_distance_);
      if (candidate && accepted_loops_.count({candidate->keyframe_id, current_id}) == 0U) {
        const slam::Cloud target = buildTargetSubmap(candidate->keyframe_id);
        const Eigen::Isometry3d initial_guess = mapFromOdom() * keyframes_[current_id].odom_from_body;
        const auto result = registration_.align(*keyframes_[current_id].local_cloud, target, initial_guess);
        if (result.converged) {
          const Eigen::Isometry3d candidate_from_current =
            pose_graph_.optimizedPose(candidate->keyframe_id).inverse() * result.target_from_source;
          try {
            pose_graph_.addLoopConstraint(
              candidate->keyframe_id, current_id, candidate_from_current,
              std::max(1e-4, result.fitness));
            accepted_loops_.insert({candidate->keyframe_id, current_id});
            map_dirty_ = true;
            RCLCPP_INFO(get_logger(), "Loop accepted %zu -> %zu (descriptor=%.3f fitness=%.3f)",
              candidate->keyframe_id, current_id, candidate->distance, result.fitness);
          } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "Loop rejected %zu -> %zu: %s",
              candidate->keyframe_id, current_id, error.what());
          }
        }
      }
    }
  }
  publishState();
}

void FullSlamNode::publishState()
{
  if (keyframes_.empty()) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_;
  for (const auto & keyframe : keyframes_) {
    const Eigen::Isometry3d pose = pose_graph_.optimizedPose(keyframe.id);
    geometry_msgs::msg::PoseStamped point;
    point.header = path.header;
    point.pose.position.x = pose.translation().x();
    point.pose.position.y = pose.translation().y();
    point.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond orientation(pose.rotation());
    point.pose.orientation.w = orientation.w();
    point.pose.orientation.x = orientation.x();
    point.pose.orientation.y = orientation.y();
    point.pose.orientation.z = orientation.z();
    path.poses.push_back(std::move(point));
  }
  path_publisher_->publish(path);

  if (!map_dirty_) {
    return;
  }
  const auto products = buildMapProducts();
  sensor_msgs::msg::PointCloud2 raw_message;
  pcl::toROSMsg(products.raw, raw_message);
  raw_message.header = path.header;
  map_publisher_->publish(raw_message);
  sensor_msgs::msg::PointCloud2 refined_message;
  pcl::toROSMsg(products.refined, refined_message);
  refined_message.header = path.header;
  refined_map_publisher_->publish(refined_message);
  const auto save_now = std::chrono::steady_clock::now();
  if (map_save_period_seconds_ > 0.0 &&
    (last_map_save_.time_since_epoch().count() == 0 ||
    std::chrono::duration<double>(save_now - last_map_save_).count() >= map_save_period_seconds_))
  {
    try {
      saveMapProducts(products);
      last_map_save_ = save_now;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Global-map save failed: %s", error.what());
    }
  }
  map_dirty_ = false;
}

void FullSlamNode::broadcastMapToOdom()
{
  Eigen::Isometry3d map_from_odom = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!keyframes_.empty()) {
      map_from_odom = mapFromOdom();
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.translation.x = map_from_odom.translation().x();
  transform.transform.translation.y = map_from_odom.translation().y();
  transform.transform.translation.z = map_from_odom.translation().z();
  const Eigen::Quaterniond rotation(map_from_odom.rotation());
  transform.transform.rotation.w = rotation.w();
  transform.transform.rotation.x = rotation.x();
  transform.transform.rotation.y = rotation.y();
  transform.transform.rotation.z = rotation.z();
  tf_broadcaster_->sendTransform(transform);
}

bool FullSlamNode::shouldCreateKeyframe(const Eigen::Isometry3d & pose) const
{
  if (keyframes_.empty()) {
    return true;
  }
  const auto & previous = keyframes_.back().odom_from_body;
  const double translation = (pose.translation() - previous.translation()).norm();
  const double angle = Eigen::AngleAxisd(previous.rotation().transpose() * pose.rotation()).angle();
  return translation >= keyframe_distance_ || angle >= keyframe_angle_radians_;
}

bool FullSlamNode::matchingOdometry(
  const double stamp_seconds, nav_msgs::msg::Odometry & output) const
{
  if (odom_cache_.empty()) {
    return false;
  }
  const auto iterator = std::min_element(
    odom_cache_.begin(), odom_cache_.end(),
    [stamp_seconds](const TimedOdom & lhs, const TimedOdom & rhs) {
      return std::abs(lhs.stamp_seconds - stamp_seconds) < std::abs(rhs.stamp_seconds - stamp_seconds);
    });
  if (std::abs(iterator->stamp_seconds - stamp_seconds) > 0.05) {
    return false;
  }
  output = iterator->message;
  return true;
}

slam::Cloud FullSlamNode::buildTargetSubmap(const std::size_t center_id) const
{
  slam::Cloud target;
  const std::size_t first = center_id > submap_neighbor_count_ ? center_id - submap_neighbor_count_ : 0U;
  const std::size_t last = std::min(keyframes_.size(), center_id + submap_neighbor_count_ + 1U);
  for (std::size_t id = first; id < last; ++id) {
    slam::Cloud transformed;
    pcl::transformPointCloud(
      *keyframes_[id].local_cloud, transformed, pose_graph_.optimizedPose(id).matrix().cast<float>());
    target += transformed;
  }
  return target;
}

Eigen::Isometry3d FullSlamNode::mapFromOdom() const
{
  if (keyframes_.empty()) {
    return Eigen::Isometry3d::Identity();
  }
  const auto & newest = keyframes_.back();
  return pose_graph_.optimizedPose(newest.id) * newest.odom_from_body.inverse();
}

slam::MapProducts FullSlamNode::buildMapProducts() const
{
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(keyframes_.size());
  for (const auto & keyframe : keyframes_) {
    poses.push_back(pose_graph_.optimizedPose(keyframe.id));
  }
  return map_builder_.build(keyframes_, poses);
}

void FullSlamNode::saveMapProducts(const slam::MapProducts & products) const
{
  map_builder_.save(products, raw_map_file_, refined_map_file_);
}

Eigen::Isometry3d FullSlamNode::poseFromOdometry(const nav_msgs::msg::Odometry & odom)
{
  Eigen::Quaterniond orientation(
    odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  orientation = orientation.squaredNorm() < 1e-6 ? Eigen::Quaterniond::Identity() : orientation.normalized();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = orientation.toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  return result;
}

double FullSlamNode::seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace autonomy_light
