#include "autonomy_light/slam/map_product_builder.hpp"

#include <filesystem>
#include <stdexcept>

#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

namespace autonomy_light::slam
{

MapProductBuilder::MapProductBuilder(MapProductConfig config)
: config_(config)
{
  if (config_.raw_voxel_size <= 0.0 || config_.refined_voxel_size <= 0.0 ||
    config_.outlier_mean_k < 2 || config_.outlier_stddev_multiplier <= 0.0)
  {
    throw std::invalid_argument("invalid global-map product settings");
  }
}

MapProducts MapProductBuilder::build(
  const std::vector<Keyframe> & keyframes,
  const std::vector<Eigen::Isometry3d> & optimized_poses) const
{
  if (keyframes.size() != optimized_poses.size()) {
    throw std::invalid_argument("keyframe and optimized-pose counts differ");
  }
  Cloud raw_full;
  Cloud refined_full;
  for (std::size_t index = 0; index < keyframes.size(); ++index) {
    Cloud transformed;
    pcl::transformPointCloud(
      *keyframes[index].local_cloud, transformed,
      optimized_poses[index].matrix().cast<float>());
    raw_full += transformed;
    refined_full += removeOutliers(transformed);
  }
  return {downsample(raw_full, config_.raw_voxel_size),
    downsample(refined_full, config_.refined_voxel_size)};
}

void MapProductBuilder::save(
  const MapProducts & products, const std::string & raw_path,
  const std::string & refined_path) const
{
  savePcd(products.raw, raw_path);
  savePcd(products.refined, refined_path);
}

Cloud MapProductBuilder::downsample(const Cloud & cloud, const double voxel_size) const
{
  if (cloud.empty()) {
    return {};
  }
  pcl::ApproximateVoxelGrid<Cloud::PointType> filter;
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.setInputCloud(cloud.makeShared());
  Cloud result;
  filter.filter(result);
  return result;
}

Cloud MapProductBuilder::removeOutliers(const Cloud & cloud) const
{
  if (cloud.size() < static_cast<std::size_t>(config_.outlier_mean_k)) {
    return cloud;
  }
  pcl::StatisticalOutlierRemoval<Cloud::PointType> filter;
  filter.setInputCloud(cloud.makeShared());
  filter.setMeanK(config_.outlier_mean_k);
  filter.setStddevMulThresh(config_.outlier_stddev_multiplier);
  Cloud result;
  filter.filter(result);
  return result;
}

void MapProductBuilder::savePcd(const Cloud & cloud, const std::string & path)
{
  if (path.empty() || cloud.empty()) {
    return;
  }
  const std::filesystem::path output(path);
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
    throw std::runtime_error("failed to save map PCD: " + path);
  }
}

}  // namespace autonomy_light::slam
