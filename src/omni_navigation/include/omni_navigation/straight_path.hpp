// 開始姿勢からゴールまでを直線で結ぶ経路の計算

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "omni_kinematics/omni_kinematics.hpp"

namespace omni_navigation {

// 位置と姿勢の両方が指定の間隔を超えないよう、同じ割合で補間する
inline std::vector<omni_kinematics::Pose2D> make_straight_path(
    const omni_kinematics::Pose2D& start, const omni_kinematics::Pose2D& goal,
    const double waypoint_spacing, const double angular_spacing) {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double dyaw = std::remainder(goal.yaw - start.yaw, 2.0 * M_PI);

  const int steps = std::max(
      {1, static_cast<int>(std::ceil(std::hypot(dx, dy) / waypoint_spacing)),
       static_cast<int>(std::ceil(std::abs(dyaw) / angular_spacing))});

  std::vector<omni_kinematics::Pose2D> path;
  path.reserve(steps + 1);
  for (int i = 0; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / steps;
    path.push_back(omni_kinematics::Pose2D{
        start.x + ratio * dx, start.y + ratio * dy,
        std::remainder(start.yaw + ratio * dyaw, 2.0 * M_PI)});
  }
  return path;
}

}  // namespace omni_navigation
