// ROSのメッセージとPose2Dの変換

#pragma once

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omni_kinematics/omni_kinematics.hpp"

namespace omni_kinematics {

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline Pose2D to_pose2d(const geometry_msgs::msg::Pose& pose) {
  return Pose2D{pose.position.x, pose.position.y,
                yaw_from_quaternion(pose.orientation)};
}

inline Pose2D to_pose2d(const geometry_msgs::msg::TransformStamped& transform) {
  return Pose2D{transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(transform.transform.rotation)};
}

inline geometry_msgs::msg::Quaternion to_quaternion(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

inline geometry_msgs::msg::Pose to_pose_msg(const Pose2D& pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.orientation = to_quaternion(pose.yaw);
  return msg;
}

inline geometry_msgs::msg::Transform to_transform_msg(const Pose2D& pose) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = pose.x;
  msg.translation.y = pose.y;
  msg.rotation = to_quaternion(pose.yaw);
  return msg;
}

}  // namespace omni_kinematics
