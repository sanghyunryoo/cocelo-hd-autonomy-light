#include "autonomy_light/elevation/prior_map.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace autonomy_light::elevation
{

void PriorMap::set(Cloud::Ptr cloud, std::string frame)
{
  if (!cloud || cloud->empty() || frame.empty()) {
    return;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  std::lock_guard<std::mutex> lock(mutex_);
  cloud_ = std::move(cloud);
  tree_ = std::move(tree);
  frame_ = std::move(frame);
}

bool PriorMap::ready() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cloud_ && tree_ && !frame_.empty();
}

std::string PriorMap::frame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_;
}

ElevationGrid PriorMap::build(
  const GridBuilder & builder, const Eigen::Isometry3f & gravity_from_map,
  const Eigen::Vector3f & gravity_origin_in_map, const float lidar_offset_xy,
  const float query_radius_margin) const
{
  Cloud::ConstPtr cloud;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud = cloud_;
    tree = tree_;
  }
  if (!cloud || !tree) {
    return {};
  }

  const auto & geometry = builder.geometry();
  const float radius = std::hypot(
    static_cast<float>(0.5 * geometry.x_length), static_cast<float>(0.5 * geometry.y_length)) +
    lidar_offset_xy + query_radius_margin + static_cast<float>(geometry.resolution);
  std::vector<int> indices;
  std::vector<float> distances;
  tree->radiusSearch(
    pcl::PointXYZ(gravity_origin_in_map.x(), gravity_origin_in_map.y(), gravity_origin_in_map.z()),
    radius, indices, distances);
  std::vector<Eigen::Vector3f> points;
  points.reserve(indices.size());
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      points.push_back(gravity_from_map * Eigen::Vector3f(point.x, point.y, point.z));
    }
  }
  return builder.build(points);
}

}  // namespace autonomy_light::elevation
