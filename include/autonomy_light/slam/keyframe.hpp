#pragma once

#include <cstddef>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autonomy_light::slam
{

// Nano-GICP 1.0 ships an explicit PointXYZI instantiation, matching Point-LIO output.
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

struct Keyframe
{
  std::size_t id{};
  double stamp_seconds{};
  Eigen::Isometry3d odom_from_body{Eigen::Isometry3d::Identity()};
  CloudPtr local_cloud{new Cloud};
};

}  // namespace autonomy_light::slam
