#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace autonomy_light
{

inline tf2::Quaternion yawOnlyQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, yaw);
  q_yaw.normalize();
  return q_yaw;
}

}  // namespace autonomy_light
