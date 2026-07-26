#include "autonomy_light/transform_publisher.hpp"
#include "autonomy_light/transform_math.hpp"

#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace autonomy_light
{

class TransformPublisher::TransformPublisherImpl
{
public:
  explicit TransformPublisherImpl(rclcpp::Node * output_node)
  : static_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(output_node)),
    dynamic_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(output_node))
  {
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};

TransformPublisher::TransformPublisher(rclcpp::Node * output_node, TransformFrames frames)
: frames_(std::move(frames))
{
  if (output_node == nullptr) {
    throw std::invalid_argument("TransformPublisher requires an output node");
  }
  // The implementation is stored through the shared pointer below to keep the
  // ROS broadcaster headers out of the public interface.
  impl_ = std::make_shared<TransformPublisherImpl>(output_node);
}

void TransformPublisher::publishStaticLidar(
  const std::string & child_frame,
  const tf2::Vector3 & translation,
  const tf2::Quaternion & rotation,
  const rclcpp::Time & stamp)
{
  if (child_frame.empty() || frames_.target_frame.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frames_.target_frame;
  msg.child_frame_id = child_frame;
  msg.transform.translation.x = translation.x();
  msg.transform.translation.y = translation.y();
  msg.transform.translation.z = translation.z();
  msg.transform.rotation = tf2::toMsg(rotation);
  impl_->static_broadcaster_->sendTransform(msg);
}

void TransformPublisher::publishHeightMapFrame(
  const nav_msgs::msg::Odometry & odom,
  const double height_origin_z,
  const rclcpp::Time & fallback_stamp)
{
  if (frames_.height_map_frame.empty() || frames_.target_frame.empty() ||
    frames_.height_map_frame == frames_.target_frame)
  {
    return;
  }

  tf2::Quaternion q_map_target;
  tf2::fromMsg(odom.pose.pose.orientation, q_map_target);
  q_map_target.normalize();
  const tf2::Quaternion q_map_height = yawOnlyQuaternion(odom.pose.pose.orientation);
  const tf2::Vector3 p_map_target(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z);
  const tf2::Vector3 p_map_height(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    height_origin_z);
  tf2::Quaternion q_target_height = q_map_target.inverse() * q_map_height;
  q_target_height.normalize();
  const tf2::Vector3 p_target_height = tf2::quatRotate(
    q_map_target.inverse(), p_map_height - p_map_target);

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = odom.header.stamp;
  if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
    msg.header.stamp = fallback_stamp;
  }
  msg.header.frame_id = frames_.target_frame;
  msg.child_frame_id = frames_.height_map_frame;
  msg.transform.translation.x = p_target_height.x();
  msg.transform.translation.y = p_target_height.y();
  msg.transform.translation.z = p_target_height.z();
  msg.transform.rotation = tf2::toMsg(q_target_height);
  impl_->dynamic_broadcaster_->sendTransform(msg);
}

void TransformPublisher::publishMapToOdom(
  const Eigen::Matrix4f & map_from_odom,
  const bool localized,
  const rclcpp::Time & stamp)
{
  if (!localized || frames_.map_frame.empty() || frames_.odom_frame.empty() ||
    frames_.map_frame == frames_.odom_frame)
  {
    return;
  }

  Eigen::Quaternionf rotation(map_from_odom.block<3, 3>(0, 0));
  rotation.normalize();
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frames_.map_frame;
  msg.child_frame_id = frames_.odom_frame;
  msg.transform.translation.x = map_from_odom(0, 3);
  msg.transform.translation.y = map_from_odom(1, 3);
  msg.transform.translation.z = map_from_odom(2, 3);
  msg.transform.rotation.x = rotation.x();
  msg.transform.rotation.y = rotation.y();
  msg.transform.rotation.z = rotation.z();
  msg.transform.rotation.w = rotation.w();
  impl_->dynamic_broadcaster_->sendTransform(msg);
}

}  // namespace autonomy_light
