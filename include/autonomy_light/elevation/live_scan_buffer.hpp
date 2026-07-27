#pragma once

#include <deque>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "autonomy_light/elevation/grid.hpp"
#include "autonomy_light/elevation/grid_builder.hpp"

namespace autonomy_light::elevation
{

class LiveScanBuffer final
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  explicit LiveScanBuffer(double history_seconds);

  void add(const Cloud & cloud, double stamp_seconds);
  [[nodiscard]] ElevationGrid build(
    const GridBuilder & builder, const Eigen::Isometry3f & gravity_from_odom,
    double reference_stamp_seconds) const;

private:
  struct Scan
  {
    double stamp_seconds;
    Cloud cloud;
  };

  void trim(double newest_stamp_seconds);

  double history_seconds_;
  mutable std::mutex mutex_;
  std::deque<Scan> scans_;
};

}  // namespace autonomy_light::elevation
