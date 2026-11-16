// ROSのメッセージとPose2Dの変換

#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omni_kinematics/omni_kinematics.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace omni_kinematics {

inline Pose2D to_pose2d(const geometry_msgs::msg::Pose& pose) {
  return Pose2D{pose.position.x, pose.position.y,
                tf2::getYaw(pose.orientation)};
}

inline Pose2D to_pose2d(const geometry_msgs::msg::TransformStamped& transform) {
  return Pose2D{transform.transform.translation.x,
                transform.transform.translation.y,
                tf2::getYaw(transform.transform.rotation)};
}

// z軸まわりにyawだけ回転したクォータニオン
inline geometry_msgs::msg::Quaternion to_quaternion(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
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
