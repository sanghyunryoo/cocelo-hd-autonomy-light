#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "autonomy_light/slam/keyframe.hpp"

namespace autonomy_light::slam
{

struct MapProductConfig
{
  double raw_voxel_size{0.15};
  double refined_voxel_size{0.03};
  int outlier_mean_k{16};
  double outlier_stddev_multiplier{0.8};
};

struct MapProducts
{
  Cloud raw;
  Cloud refined;
};

class MapProductBuilder final
{
public:
  explicit MapProductBuilder(MapProductConfig config);

  [[nodiscard]] MapProducts build(
    const std::vector<Keyframe> & keyframes,
    const std::vector<Eigen::Isometry3d> & optimized_poses) const;
  void save(const MapProducts & products, const std::string & raw_path,
    const std::string & refined_path) const;

private:
  [[nodiscard]] Cloud downsample(const Cloud & cloud, double voxel_size) const;
  [[nodiscard]] Cloud removeOutliers(const Cloud & cloud) const;
  static void savePcd(const Cloud & cloud, const std::string & path);

  MapProductConfig config_;
};

}  // namespace autonomy_light::slam
