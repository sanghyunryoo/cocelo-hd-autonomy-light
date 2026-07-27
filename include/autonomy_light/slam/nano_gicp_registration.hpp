#pragma once

#include <Eigen/Geometry>

#include "autonomy_light/slam/keyframe.hpp"

namespace autonomy_light::slam
{

struct RegistrationResult
{
  bool converged{false};
  double fitness{0.0};
  Eigen::Isometry3d target_from_source{Eigen::Isometry3d::Identity()};
};

class NanoGicpRegistration final
{
public:
  NanoGicpRegistration(
    double voxel_size, double max_correspondence_distance, int correspondence_randomness,
    int max_iterations, double fitness_threshold);

  [[nodiscard]] RegistrationResult align(
    const Cloud & source, const Cloud & target, const Eigen::Isometry3d & initial_guess) const;

private:
  double voxel_size_;
  double max_correspondence_distance_;
  int correspondence_randomness_;
  int max_iterations_;
  double fitness_threshold_;
};

}  // namespace autonomy_light::slam
