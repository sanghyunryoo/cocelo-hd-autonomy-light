#include "autonomy_light/slam/saved_map_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>

namespace autonomy_light::slam
{

namespace
{
using Normals = pcl::PointCloud<pcl::Normal>;
using Features = pcl::PointCloud<pcl::FPFHSignature33>;

std::optional<Features::Ptr> featuresFor(
  const Cloud::ConstPtr & cloud, const double normal_radius, const double feature_radius)
{
  if (!cloud || cloud->empty()) {
    return std::nullopt;
  }
  auto tree = pcl::search::KdTree<Cloud::PointType>::Ptr(new pcl::search::KdTree<Cloud::PointType>());
  pcl::NormalEstimationOMP<Cloud::PointType, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setRadiusSearch(normal_radius);
  auto normals = Normals::Ptr(new Normals());
  normal_estimation.compute(*normals);
  pcl::FPFHEstimationOMP<Cloud::PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normals);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(feature_radius);
  auto features = Features::Ptr(new Features());
  fpfh.compute(*features);
  return features->empty() ? std::nullopt : std::optional<Features::Ptr>(features);
}
}  // namespace

SavedMapLocalizer::SavedMapLocalizer(
  SavedMapLocalizerConfig config, NanoGicpRegistration registration)
: config_(config), registration_(std::move(registration))
{
  if (config_.scan_voxel_size <= 0.0 || config_.map_voxel_size <= 0.0 ||
    config_.feature_voxel_size <= 0.0 || config_.initial_submap_seconds <= 0.0 ||
    config_.minimum_points == 0U || config_.target_radius <= 0.0 ||
    config_.update_period_seconds <= 0.0 || config_.filter_alpha <= 0.0 ||
    config_.filter_alpha > 1.0 || config_.max_translation_step <= 0.0 ||
    config_.max_rotation_step_radians <= 0.0)
  {
    throw std::invalid_argument("invalid saved-map localization settings");
  }
}

void SavedMapLocalizer::load(const std::string & pcd_path)
{
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(pcd_path, blob) < 0) {
    throw std::runtime_error("cannot load saved map: " + pcd_path);
  }
  pcl::fromPCLPointCloud2(blob, map_);
  if (map_.empty()) {
    throw std::runtime_error("saved map has no compatible XYZ points: " + pcd_path);
  }
  registration_map_ = downsample(map_, config_.map_voxel_size);
  feature_map_ = cap(downsample(map_, config_.feature_voxel_size), config_.maximum_feature_points);
  if (registration_map_.size() < config_.minimum_points || feature_map_.size() < config_.minimum_points) {
    throw std::runtime_error("saved map is too small for localization");
  }
}

void SavedMapLocalizer::addScan(
  const Cloud & registered_in_odom, const Eigen::Isometry3d & odom_from_body)
{
  if (registered_in_odom.empty()) {
    return;
  }
  latest_odom_from_body_ = odom_from_body;
  const Cloud scan = downsample(registered_in_odom, config_.scan_voxel_size);
  if (scan.empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!initialized_) {
    if (initial_submap_.empty()) {
      initial_started_at_ = now;
    }
    initial_submap_ += scan;
    initial_submap_ = cap(downsample(initial_submap_, config_.scan_voxel_size), config_.maximum_initial_points);
  }
  rolling_submap_.push_back({now, scan});
  const auto cutoff = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.initial_submap_seconds));
  while (!rolling_submap_.empty() && rolling_submap_.front().received_at < cutoff) {
    rolling_submap_.pop_front();
  }
}

std::optional<RegistrationResult> SavedMapLocalizer::update()
{
  if (map_.empty()) {
    throw std::logic_error("saved map has not been loaded");
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_update_at_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double>(now - last_update_at_).count() < config_.update_period_seconds)
  {
    return std::nullopt;
  }
  Cloud source;
  Eigen::Isometry3d initial_guess = map_from_odom_;
  if (!initialized_) {
    if (initial_submap_.size() < config_.minimum_points ||
      std::chrono::duration<double>(now - initial_started_at_).count() < config_.initial_submap_seconds)
    {
      return std::nullopt;
    }
    const auto global_guess = globalInitialGuess(initial_submap_);
    initial_submap_.clear();
    if (!global_guess) {
      last_update_at_ = now;
      return std::nullopt;
    }
    source = rollingSubmap();
    initial_guess = *global_guess;
  } else {
    source = rollingSubmap();
  }
  if (source.size() < config_.minimum_points) {
    return std::nullopt;
  }
  const Cloud target = initialized_ ? localTarget() : registration_map_;
  if (target.size() < config_.minimum_points) {
    return std::nullopt;
  }
  RegistrationResult result = registration_.align(source, target, initial_guess);
  last_update_at_ = now;
  if (!result.converged) {
    return result;
  }
  if (initialized_) {
    const double translation = (result.target_from_source.translation() - map_from_odom_.translation()).norm();
    const double angle = Eigen::AngleAxisd(
      map_from_odom_.rotation().transpose() * result.target_from_source.rotation()).angle();
    if (translation > config_.max_translation_step || angle > config_.max_rotation_step_radians) {
      result.converged = false;
      return result;
    }
    map_from_odom_ = interpolate(map_from_odom_, result.target_from_source, config_.filter_alpha);
  } else {
    map_from_odom_ = result.target_from_source;
    initialized_ = true;
  }
  return result;
}

bool SavedMapLocalizer::initialized() const {return initialized_;}
const Cloud & SavedMapLocalizer::map() const {return map_;}
Eigen::Isometry3d SavedMapLocalizer::mapFromOdom() const {return map_from_odom_;}

Cloud SavedMapLocalizer::downsample(const Cloud & cloud, const double voxel_size) const
{
  pcl::ApproximateVoxelGrid<Cloud::PointType> filter;
  filter.setInputCloud(cloud.makeShared());
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  Cloud result;
  filter.filter(result);
  return result;
}

Cloud SavedMapLocalizer::cap(const Cloud & cloud, const std::size_t limit) const
{
  if (cloud.size() <= limit) {
    return cloud;
  }
  Cloud result;
  result.reserve(limit);
  const std::size_t stride = (cloud.size() + limit - 1U) / limit;
  for (std::size_t index = 0; index < cloud.size(); index += stride) {
    result.push_back(cloud[index]);
  }
  return result;
}

Cloud SavedMapLocalizer::rollingSubmap() const
{
  Cloud result;
  for (const auto & entry : rolling_submap_) {
    result += entry.cloud;
  }
  return downsample(result, config_.scan_voxel_size);
}

Cloud SavedMapLocalizer::localTarget() const
{
  const Eigen::Vector3d center = map_from_odom_ * latest_odom_from_body_.translation();
  const double radius_squared = config_.target_radius * config_.target_radius;
  Cloud result;
  for (const auto & point : registration_map_) {
    const double dx = point.x - center.x();
    const double dy = point.y - center.y();
    if (dx * dx + dy * dy <= radius_squared) {
      result.push_back(point);
    }
  }
  return result.size() >= config_.minimum_points ? result : registration_map_;
}

std::optional<Eigen::Isometry3d> SavedMapLocalizer::globalInitialGuess(const Cloud & source) const
{
  const Cloud feature_source = cap(downsample(source, config_.feature_voxel_size), config_.maximum_feature_points);
  if (feature_source.size() < config_.minimum_points) {
    return std::nullopt;
  }
  const auto source_features = featuresFor(
    feature_source.makeShared(), config_.normal_radius, config_.feature_radius);
  const auto target_features = featuresFor(
    feature_map_.makeShared(), config_.normal_radius, config_.feature_radius);
  if (!source_features || !target_features) {
    return std::nullopt;
  }
  pcl::SampleConsensusPrerejective<Cloud::PointType, Cloud::PointType, pcl::FPFHSignature33> aligner;
  aligner.setInputSource(feature_source.makeShared());
  aligner.setSourceFeatures(*source_features);
  aligner.setInputTarget(feature_map_.makeShared());
  aligner.setTargetFeatures(*target_features);
  aligner.setNumberOfSamples(3);
  aligner.setCorrespondenceRandomness(5);
  aligner.setSimilarityThreshold(0.9F);
  aligner.setMaxCorrespondenceDistance(static_cast<float>(config_.target_radius));
  aligner.setInlierFraction(0.2F);
  aligner.setMaximumIterations(1500);
  Cloud aligned;
  aligner.align(aligned);
  if (!aligner.hasConverged()) {
    return std::nullopt;
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = aligner.getFinalTransformation().cast<double>();
  return result;
}

Eigen::Isometry3d SavedMapLocalizer::interpolate(
  const Eigen::Isometry3d & current, const Eigen::Isometry3d & measurement, const double alpha)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(current.rotation()).slerp(
    alpha, Eigen::Quaterniond(measurement.rotation())).toRotationMatrix();
  result.translation() = (1.0 - alpha) * current.translation() + alpha * measurement.translation();
  return result;
}

}  // namespace autonomy_light::slam
