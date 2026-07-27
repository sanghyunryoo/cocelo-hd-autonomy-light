#pragma once

#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "autonomy_light/elevation/grid.hpp"
#include "autonomy_light/elevation/grid_builder.hpp"

namespace autonomy_light::elevation
{

class PriorMap final
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  void set(Cloud::Ptr cloud, std::string frame);
  [[nodiscard]] bool ready() const;
  [[nodiscard]] std::string frame() const;
  [[nodiscard]] ElevationGrid build(
    const GridBuilder & builder, const Eigen::Isometry3f & gravity_from_map,
    const Eigen::Vector3f & gravity_origin_in_map, float lidar_offset_xy,
    float query_radius_margin) const;

private:
  mutable std::mutex mutex_;
  Cloud::ConstPtr cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  std::string frame_;
};

}  // namespace autonomy_light::elevation
