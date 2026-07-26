#include "autonomy_light/lidar_merger.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace autonomy_light
{
namespace
{

const sensor_msgs::msg::PointField * findPointField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(),
    cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) { return field.name == name; });
  return it == cloud.fields.end() ? nullptr : &(*it);
}

template<typename T>
T readPointFieldAs(const std::uint8_t * ptr)
{
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

double readPointFieldNumeric(
  const std::uint8_t * base,
  const sensor_msgs::msg::PointField * field,
  const double fallback = 0.0)
{
  if (field == nullptr) {
    return fallback;
  }
  const auto * ptr = base + field->offset;
  switch (field->datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(readPointFieldAs<std::int8_t>(ptr));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(readPointFieldAs<std::uint8_t>(ptr));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(readPointFieldAs<std::int16_t>(ptr));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(readPointFieldAs<std::uint16_t>(ptr));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(readPointFieldAs<std::int32_t>(ptr));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(readPointFieldAs<std::uint32_t>(ptr));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(readPointFieldAs<float>(ptr));
    case sensor_msgs::msg::PointField::FLOAT64:
      return readPointFieldAs<double>(ptr);
    default:
      return fallback;
  }
}

double stampSeconds(const rclcpp::Time & stamp)
{
  return static_cast<double>(stamp.nanoseconds()) * 1.0e-9;
}

}  // namespace

LidarMerger::LidarMerger(
  LidarMergeConfig config,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
: config_(std::move(config)), logger_(std::move(logger)), clock_(std::move(clock))
{
  config_.max_queue_size = std::max(1, config_.max_queue_size);
  config_.sync_tolerance_sec = std::max(0.0, config_.sync_tolerance_sec);
}

LidarMerger::TimedPoint LidarMerger::transformPoint(
  const int lidar_index,
  const float x,
  const float y,
  const float z,
  const float intensity,
  const double time_offset) const
{
  const auto & rotation = lidar_index == 0 ?
    config_.target_to_lidar1_rotation : config_.target_to_lidar2_rotation;
  const auto & translation = lidar_index == 0 ?
    config_.target_to_lidar1_translation : config_.target_to_lidar2_translation;
  const tf2::Vector3 p_target = translation + rotation * tf2::Vector3(x, y, z);
  return {
    static_cast<float>(p_target.x()),
    static_cast<float>(p_target.y()),
    static_cast<float>(p_target.z()),
    intensity,
    time_offset};
}

std::vector<sensor_msgs::msg::PointCloud2> LidarMerger::ingestCustom(
  const int lidar_index,
  const livox_ros_driver2::msg::CustomMsg & message,
  bool * accepted_input)
{
  if (accepted_input != nullptr) {
    *accepted_input = false;
  }
  TimedCloud cloud;
  cloud.stamp = rclcpp::Time(message.header.stamp);
  cloud.points.reserve(message.points.size());
  for (const auto & point : message.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    cloud.points.push_back(transformPoint(
      lidar_index,
      point.x,
      point.y,
      point.z,
      static_cast<float>(point.reflectivity),
      static_cast<double>(point.offset_time) * 1.0e-9));
  }
  if (accepted_input != nullptr) {
    *accepted_input = !cloud.points.empty();
  }
  return pushCloud(lidar_index, std::move(cloud));
}

std::vector<sensor_msgs::msg::PointCloud2> LidarMerger::ingestPointCloud(
  const int lidar_index,
  const sensor_msgs::msg::PointCloud2 & message,
  bool * accepted_input)
{
  if (accepted_input != nullptr) {
    *accepted_input = false;
  }
  const auto * x_field = findPointField(message, "x");
  const auto * y_field = findPointField(message, "y");
  const auto * z_field = findPointField(message, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr || message.point_step == 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "LiDAR merge input cloud missing xyz fields");
    return {};
  }
  auto * intensity_field = findPointField(message, "intensity");
  if (intensity_field == nullptr) {
    intensity_field = findPointField(message, "reflectivity");
  }
  const auto * time_field = findPointField(message, "timestamp");
  if (time_field == nullptr) {
    time_field = findPointField(message, "time");
  }
  if (time_field == nullptr) {
    time_field = findPointField(message, "t");
  }
  if (time_field == nullptr) {
    time_field = findPointField(message, "offset_time");
  }

  TimedCloud cloud;
  cloud.stamp = rclcpp::Time(message.header.stamp);
  const auto point_count = static_cast<std::size_t>(message.width) * message.height;
  if (point_count == 0) {
    return {};
  }
  cloud.points.reserve(point_count);
  const auto * data = message.data.data();
  const double first_time = time_field != nullptr ?
    readPointFieldNumeric(data, time_field, 0.0) : 0.0;

  for (std::size_t i = 0; i < point_count; ++i) {
    const auto * base = data + i * message.point_step;
    const auto x = static_cast<float>(readPointFieldNumeric(base, x_field));
    const auto y = static_cast<float>(readPointFieldNumeric(base, y_field));
    const auto z = static_cast<float>(readPointFieldNumeric(base, z_field));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const auto intensity = static_cast<float>(readPointFieldNumeric(base, intensity_field, 0.0));
    double time_offset = 0.0;
    if (time_field != nullptr) {
      time_offset = readPointFieldNumeric(base, time_field, first_time) - first_time;
      if (std::abs(time_offset) > 1.0) {
        time_offset *= 1.0e-9;
      }
    }
    cloud.points.push_back(transformPoint(lidar_index, x, y, z, intensity, time_offset));
  }
  if (accepted_input != nullptr) {
    *accepted_input = !cloud.points.empty();
  }
  return pushCloud(lidar_index, std::move(cloud));
}

std::vector<sensor_msgs::msg::PointCloud2> LidarMerger::pushCloud(
  const int lidar_index,
  TimedCloud cloud)
{
  if (cloud.points.empty()) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto & queue = lidar_index == 0 ? lidar1_queue_ : lidar2_queue_;
  queue.push_back(std::move(cloud));
  while (static_cast<int>(queue.size()) > config_.max_queue_size) {
    queue.pop_front();
  }
  return tryBuildMergedLocked();
}

std::vector<sensor_msgs::msg::PointCloud2> LidarMerger::tryBuildMergedLocked()
{
  std::vector<sensor_msgs::msg::PointCloud2> merged;
  while (!lidar1_queue_.empty()) {
    if (lidar2_queue_.empty()) {
      return merged;
    }

    const auto & lidar1 = lidar1_queue_.front();
    const double lidar1_stamp = stampSeconds(lidar1.stamp);
    auto best_it = lidar2_queue_.begin();
    double best_diff = std::abs(stampSeconds(best_it->stamp) - lidar1_stamp);
    for (auto it = std::next(lidar2_queue_.begin()); it != lidar2_queue_.end(); ++it) {
      const double diff = std::abs(stampSeconds(it->stamp) - lidar1_stamp);
      if (diff < best_diff) {
        best_diff = diff;
        best_it = it;
      }
    }

    if (best_diff <= config_.sync_tolerance_sec) {
      merged.push_back(buildMerged(lidar1, &(*best_it)));
      lidar2_queue_.erase(lidar2_queue_.begin(), std::next(best_it));
      lidar1_queue_.pop_front();
      continue;
    }

    const double newest_lidar2_stamp = stampSeconds(lidar2_queue_.back().stamp);
    if (newest_lidar2_stamp + config_.sync_tolerance_sec < lidar1_stamp) {
      lidar2_queue_.pop_front();
      continue;
    }
    if (lidar1_stamp + config_.sync_tolerance_sec < newest_lidar2_stamp) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "LiDAR merge sync miss: nearest dt=%.4fs tolerance=%.4fs",
        best_diff, config_.sync_tolerance_sec);
      if (config_.publish_lidar1_on_sync_miss) {
        merged.push_back(buildMerged(lidar1, nullptr));
      }
      lidar1_queue_.pop_front();
      continue;
    }
    return merged;
  }
  return merged;
}

sensor_msgs::msg::PointCloud2 LidarMerger::buildMerged(
  const TimedCloud & lidar1,
  const TimedCloud * lidar2) const
{
  const std::size_t lidar2_size = lidar2 == nullptr ? 0 : lidar2->points.size();
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = lidar1.stamp;
  cloud.header.frame_id = config_.target_frame;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(lidar1.points.size() + lidar2_size);

  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float> time_it(cloud, "time");

  const double base_stamp = stampSeconds(lidar1.stamp);
  auto write_point = [&](const TimedCloud & source, const TimedPoint & point) {
      *x_it = point.x;
      *y_it = point.y;
      *z_it = point.z;
      *intensity_it = point.intensity;
      const double source_delta = stampSeconds(source.stamp) - base_stamp;
      *time_it = static_cast<float>(std::max(0.0, source_delta + point.time_offset) * 1.0e9);
      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
      ++time_it;
    };

  for (const auto & point : lidar1.points) {
    write_point(lidar1, point);
  }
  if (lidar2 != nullptr) {
    for (const auto & point : lidar2->points) {
      write_point(*lidar2, point);
    }
  }
  return cloud;
}

}  // namespace autonomy_light
