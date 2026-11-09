// オムニ向けのPure Pursuit
//
// 差動二輪と違い、機体の向きを変えずに先読み点へ直接進める。
// 向きは、先読み点の姿勢へ独立に合わせる。
#ifndef OMNI_NAVIGATION__PURE_PURSUIT_HPP_
#define OMNI_NAVIGATION__PURE_PURSUIT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "omni_kinematics/omni_kinematics.hpp"

namespace omni_navigation
{

struct PurePursuitConfig
{
  double lookahead_distance{0.30};
  double max_linear_speed{0.50};
  double max_angular_speed{1.00};
  double position_gain{1.5};
  double yaw_gain{2.0};
};

inline double distance_between(
  const omni_kinematics::Pose2D & a, const omni_kinematics::Pose2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

// 最寄りの経路点を探す。経路を逆走しないよう、前回の位置より先だけを見る
inline std::size_t find_nearest_index(
  const std::vector<omni_kinematics::Pose2D> & path, const omni_kinematics::Pose2D & pose,
  std::size_t index)
{
  while (index + 1 < path.size() &&
    distance_between(pose, path[index + 1]) <= distance_between(pose, path[index]))
  {
    ++index;
  }
  return index;
}

// 最寄り点より先で、先読み距離以上離れた最初の経路点を探す。なければ終点を返す
// 最寄り点は機体の後ろにあることもあるので、目標には選ばない
inline std::size_t find_target_index(
  const std::vector<omni_kinematics::Pose2D> & path, const omni_kinematics::Pose2D & pose,
  const std::size_t nearest_index, const double lookahead_distance)
{
  std::size_t index = std::min(nearest_index + 1, path.size() - 1);
  while (index + 1 < path.size() && distance_between(pose, path[index]) < lookahead_distance) {
    ++index;
  }
  return index;
}

// 機体座標系の速度指令を計算する
inline omni_kinematics::BodyMotion compute_velocity(
  const omni_kinematics::Pose2D & pose, const omni_kinematics::Pose2D & target,
  const omni_kinematics::Pose2D & goal, const PurePursuitConfig & config)
{
  omni_kinematics::BodyMotion velocity;

  const double target_distance = distance_between(pose, target);
  if (target_distance > 1e-6) {
    const double speed = std::min(
      config.max_linear_speed, config.position_gain * distance_between(pose, goal));
    const double world_x = speed * (target.x - pose.x) / target_distance;
    const double world_y = speed * (target.y - pose.y) / target_distance;
    velocity.x = std::cos(pose.yaw) * world_x + std::sin(pose.yaw) * world_y;
    velocity.y = -std::sin(pose.yaw) * world_x + std::cos(pose.yaw) * world_y;
  }

  const double yaw_error = std::remainder(target.yaw - pose.yaw, 2.0 * M_PI);
  velocity.yaw = std::clamp(
    config.yaw_gain * yaw_error, -config.max_angular_speed, config.max_angular_speed);
  return velocity;
}

}  // namespace omni_navigation

#endif  // OMNI_NAVIGATION__PURE_PURSUIT_HPP_
