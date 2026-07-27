#pragma once

#include <chrono>
#include <deque>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "autonomy_light/slam/keyframe.hpp"
#include "autonomy_light/slam/nano_gicp_registration.hpp"

namespace autonomy_light::slam
{

struct SavedMapLocalizerConfig
{
  double scan_voxel_size{0.2};
  double map_voxel_size{0.2};
  double feature_voxel_size{0.5};
  double initial_submap_seconds{2.0};
  std::size_t minimum_points{500};
  std::size_t maximum_initial_points{30000};
  std::size_t maximum_feature_points{6000};
  double normal_radius{0.8};
  double feature_radius{1.2};
  double target_radius{20.0};
  double update_period_seconds{1.0};
  double filter_alpha{0.2};
  double max_translation_step{1.0};
  double max_rotation_step_radians{0.35};
};

class SavedMapLocalizer final
{
public:
  SavedMapLocalizer(SavedMapLocalizerConfig config, NanoGicpRegistration registration);

  void load(const std::string & pcd_path);
  void addScan(const Cloud & registered_in_odom, const Eigen::Isometry3d & odom_from_body);
  [[nodiscard]] std::optional<RegistrationResult> update();
  [[nodiscard]] bool initialized() const;
  [[nodiscard]] const Cloud & map() const;
  [[nodiscard]] Eigen::Isometry3d mapFromOdom() const;

private:
  struct TimedCloud
  {
    std::chrono::steady_clock::time_point received_at;
    Cloud cloud;
  };

  [[nodiscard]] Cloud downsample(const Cloud & cloud, double voxel_size) const;
  [[nodiscard]] Cloud cap(const Cloud & cloud, std::size_t limit) const;
  [[nodiscard]] Cloud rollingSubmap() const;
  [[nodiscard]] Cloud localTarget() const;
  [[nodiscard]] std::optional<Eigen::Isometry3d> globalInitialGuess(const Cloud & source) const;
  [[nodiscard]] static Eigen::Isometry3d interpolate(
    const Eigen::Isometry3d & current, const Eigen::Isometry3d & measurement, double alpha);

  SavedMapLocalizerConfig config_;
  NanoGicpRegistration registration_;
  Cloud map_;
  Cloud registration_map_;
  Cloud feature_map_;
  Cloud initial_submap_;
  std::deque<TimedCloud> rolling_submap_;
  std::chrono::steady_clock::time_point initial_started_at_{};
  std::chrono::steady_clock::time_point last_update_at_{};
  Eigen::Isometry3d map_from_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d latest_odom_from_body_{Eigen::Isometry3d::Identity()};
  bool initialized_{false};
};

}  // namespace autonomy_light::slam
