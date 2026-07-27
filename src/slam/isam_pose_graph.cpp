#include "autonomy_light/slam/isam_pose_graph.hpp"

#include <stdexcept>

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace autonomy_light::slam
{

IsamPoseGraph::IsamPoseGraph()
: isam_([] {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    return parameters;
  }())
{
}

void IsamPoseGraph::addOdometryKeyframe(
  const std::size_t id, const Eigen::Isometry3d & odom_from_body)
{
  if (id != raw_poses_.size()) {
    throw std::logic_error("keyframe ids must be contiguous");
  }
  if (id == 0U) {
    const auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    pending_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(0, toGtsam(odom_from_body), noise));
  } else {
    const Eigen::Isometry3d relative = raw_poses_.back().inverse() * odom_from_body;
    const auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector6() << 5e-4, 5e-4, 5e-4, 2e-2, 2e-2, 2e-2).finished());
    pending_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(id - 1U, id, toGtsam(relative), noise));
  }
  pending_values_.insert(id, toGtsam(odom_from_body));
  raw_poses_.push_back(odom_from_body);
  update();
}

void IsamPoseGraph::addLoopConstraint(
  const std::size_t from_id, const std::size_t to_id, const Eigen::Isometry3d & from_to,
  const double variance)
{
  if (!contains(from_id) || !contains(to_id) || from_id == to_id || variance <= 0.0) {
    throw std::invalid_argument("invalid loop constraint");
  }
  pending_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
    from_id, to_id, toGtsam(from_to), gtsam::noiseModel::Isotropic::Variance(6, variance)));
  update();
}

bool IsamPoseGraph::contains(const std::size_t id) const
{
  return id < raw_poses_.size() && estimate_.exists(id);
}

Eigen::Isometry3d IsamPoseGraph::optimizedPose(const std::size_t id) const
{
  if (!contains(id)) {
    throw std::out_of_range("pose graph key is absent");
  }
  return fromGtsam(estimate_.at<gtsam::Pose3>(id));
}

void IsamPoseGraph::update()
{
  isam_.update(pending_factors_, pending_values_);
  isam_.update();
  pending_factors_.resize(0);
  pending_values_.clear();
  estimate_ = isam_.calculateEstimate();
}

gtsam::Pose3 IsamPoseGraph::toGtsam(const Eigen::Isometry3d & transform)
{
  return gtsam::Pose3(
    gtsam::Rot3(transform.rotation()),
    gtsam::Point3(
      transform.translation().x(), transform.translation().y(), transform.translation().z()));
}

Eigen::Isometry3d IsamPoseGraph::fromGtsam(const gtsam::Pose3 & transform)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = transform.rotation().matrix();
  result.translation() = transform.translation();
  return result;
}

}  // namespace autonomy_light::slam
