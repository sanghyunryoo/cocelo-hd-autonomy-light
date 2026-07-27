#include "autonomy_light/slam/nano_gicp_registration.hpp"

#include <stdexcept>

#include <nano_gicp/nano_gicp.hpp>
#include <pcl/filters/approximate_voxel_grid.h>

namespace autonomy_light::slam
{

NanoGicpRegistration::NanoGicpRegistration(
  const double voxel_size, const double max_correspondence_distance,
  const int correspondence_randomness, const int max_iterations, const double fitness_threshold)
: voxel_size_(voxel_size),
  max_correspondence_distance_(max_correspondence_distance),
  correspondence_randomness_(correspondence_randomness),
  max_iterations_(max_iterations),
  fitness_threshold_(fitness_threshold)
{
  if (voxel_size_ <= 0.0 || max_correspondence_distance_ <= 0.0 ||
    correspondence_randomness_ <= 0 || max_iterations_ <= 0 || fitness_threshold_ <= 0.0)
  {
    throw std::invalid_argument("invalid Nano-GICP settings");
  }
}

RegistrationResult NanoGicpRegistration::align(
  const Cloud & source, const Cloud & target, const Eigen::Isometry3d & initial_guess) const
{
  if (source.empty() || target.empty()) {
    return {};
  }
  pcl::ApproximateVoxelGrid<Cloud::PointType> filter;
  filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  Cloud source_filtered;
  Cloud target_filtered;
  filter.setInputCloud(source.makeShared());
  filter.filter(source_filtered);
  filter.setInputCloud(target.makeShared());
  filter.filter(target_filtered);
  if (source_filtered.empty() || target_filtered.empty()) {
    return {};
  }

  nano_gicp::NanoGICP<Cloud::PointType, Cloud::PointType> registration;
  registration.setNumThreads(0);
  registration.setCorrespondenceRandomness(correspondence_randomness_);
  registration.setMaximumIterations(max_iterations_);
  registration.setMaxCorrespondenceDistance(max_correspondence_distance_);
  registration.setTransformationEpsilon(1e-3);
  registration.setEuclideanFitnessEpsilon(1e-3);
  registration.setInputSource(source_filtered.makeShared());
  registration.calculateSourceCovariances();
  registration.setInputTarget(target_filtered.makeShared());
  registration.calculateTargetCovariances();

  Cloud aligned;
  registration.align(aligned, initial_guess.matrix().cast<float>());
  RegistrationResult result;
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore();
  result.target_from_source.matrix() = registration.getFinalTransformation().cast<double>();
  if (!result.converged || !std::isfinite(result.fitness) || result.fitness > fitness_threshold_) {
    result.converged = false;
  }
  return result;
}

}  // namespace autonomy_light::slam
