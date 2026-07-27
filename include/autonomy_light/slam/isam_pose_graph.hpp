#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace autonomy_light::slam
{

class IsamPoseGraph final
{
public:
  IsamPoseGraph();

  void addOdometryKeyframe(std::size_t id, const Eigen::Isometry3d & odom_from_body);
  void addLoopConstraint(
    std::size_t from_id, std::size_t to_id, const Eigen::Isometry3d & from_to,
    double variance);
  [[nodiscard]] bool contains(std::size_t id) const;
  [[nodiscard]] Eigen::Isometry3d optimizedPose(std::size_t id) const;

private:
  void update();
  [[nodiscard]] static gtsam::Pose3 toGtsam(const Eigen::Isometry3d & transform);
  [[nodiscard]] static Eigen::Isometry3d fromGtsam(const gtsam::Pose3 & transform);

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph pending_factors_;
  gtsam::Values pending_values_;
  gtsam::Values estimate_;
  std::vector<Eigen::Isometry3d> raw_poses_;
};

}  // namespace autonomy_light::slam
