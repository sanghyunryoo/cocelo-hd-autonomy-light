#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace autonomy_light
{

struct TransformFrames
{
  std::string target_frame;
  std::string height_map_frame;
  std::string map_frame;
  std::string odom_frame;
};

// Owns the TF broadcasters used by autonomy-light's external-output domain.
// Point-LIO remains the sole authority for odom -> target_frame.
class TransformPublisher
{
public:
  TransformPublisher(rclcpp::Node * output_node, TransformFrames frames);

  void publishStaticLidar(
    const std::string & child_frame,
    const tf2::Vector3 & translation,
    const tf2::Quaternion & rotation,
    const rclcpp::Time & stamp);

  void publishHeightMapFrame(
    const nav_msgs::msg::Odometry & odom,
    double height_origin_z,
    const rclcpp::Time & fallback_stamp);

  void publishMapToOdom(
    const Eigen::Matrix4f & map_from_odom,
    bool localized,
    const rclcpp::Time & stamp);

private:
  class TransformPublisherImpl;

  TransformFrames frames_;
  std::shared_ptr<TransformPublisherImpl> impl_;
};

}  // namespace autonomy_light
