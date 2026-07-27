#include "autonomy_light/elevation/live_scan_buffer.hpp"

#include <cmath>
#include <vector>

namespace autonomy_light::elevation
{

LiveScanBuffer::LiveScanBuffer(const double history_seconds)
: history_seconds_(history_seconds)
{
}

void LiveScanBuffer::add(const Cloud & cloud, const double stamp_seconds)
{
  if (cloud.empty() || !std::isfinite(stamp_seconds)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  scans_.push_back({stamp_seconds, cloud});
  trim(stamp_seconds);
}

ElevationGrid LiveScanBuffer::build(
  const GridBuilder & builder, const Eigen::Isometry3f & gravity_from_odom,
  const double reference_stamp_seconds) const
{
  std::vector<Eigen::Vector3f> points;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & scan : scans_) {
      if (reference_stamp_seconds - scan.stamp_seconds > history_seconds_) {
        continue;
      }
      points.reserve(points.size() + scan.cloud.size());
      for (const auto & point : scan.cloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          points.push_back(gravity_from_odom * Eigen::Vector3f(point.x, point.y, point.z));
        }
      }
    }
  }
  return builder.build(points);
}

void LiveScanBuffer::trim(const double newest_stamp_seconds)
{
  while (!scans_.empty() && newest_stamp_seconds - scans_.front().stamp_seconds > history_seconds_) {
    scans_.pop_front();
  }
}

}  // namespace autonomy_light::elevation
