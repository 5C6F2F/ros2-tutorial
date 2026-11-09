#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "omni_navigation/pure_pursuit.hpp"
#include "omni_navigation/straight_path.hpp"

using omni_kinematics::Pose2D;
using omni_navigation::PurePursuitConfig;

namespace {
constexpr double kTolerance = 1e-9;
}  // namespace

TEST(StraightPath, KeepsSpacingAndEndpoints) {
  const auto path = omni_navigation::make_straight_path(
      Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 0.0}, 0.05, 0.10);
  ASSERT_EQ(path.size(), 21u);
  EXPECT_NEAR(path.front().x, 0.0, kTolerance);
  EXPECT_NEAR(path.back().x, 1.0, kTolerance);
  EXPECT_NEAR(path[1].x - path[0].x, 0.05, kTolerance);
}

TEST(StraightPath, RotationAloneCreatesWaypoints) {
  const auto path = omni_navigation::make_straight_path(
      Pose2D{0.0, 0.0, 0.0}, Pose2D{0.0, 0.0, 1.0}, 0.05, 0.10);
  ASSERT_EQ(path.size(), 11u);
  EXPECT_NEAR(path.back().yaw, 1.0, kTolerance);
}

TEST(StraightPath, TurnsThroughShortestArc) {
  const auto path = omni_navigation::make_straight_path(
      Pose2D{0.0, 0.0, 3.0}, Pose2D{0.0, 0.0, -3.0}, 0.05, 0.10);
  EXPECT_EQ(path.size(), 4u);
  EXPECT_NEAR(path.back().yaw, -3.0, kTolerance);
}

TEST(StraightPath, IdenticalPosesStillHaveTwoPoints) {
  const auto path = omni_navigation::make_straight_path(
      Pose2D{1.0, 2.0, 0.5}, Pose2D{1.0, 2.0, 0.5}, 0.05, 0.10);
  EXPECT_EQ(path.size(), 2u);
}

TEST(PurePursuit, NearestIndexDoesNotMoveBackward) {
  const std::vector<Pose2D> path{
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  EXPECT_EQ(omni_navigation::find_nearest_index(path, Pose2D{0.0, 0.0, 0.0}, 2),
            2u);
}

TEST(PurePursuit, TargetIsFirstPointBeyondLookahead) {
  const auto path = omni_navigation::make_straight_path(
      Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 0.0}, 0.05, 0.10);
  EXPECT_EQ(
      omni_navigation::find_target_index(path, Pose2D{0.0, 0.0, 0.0}, 0, 0.30),
      6u);
  EXPECT_EQ(
      omni_navigation::find_target_index(path, Pose2D{0.9, 0.0, 0.0}, 18, 0.30),
      20u);
}

TEST(PurePursuit, TargetStaysAheadOnSparsePath) {
  // 経路点の間隔が先読み距離より広くても、後ろの点へ戻らない
  const std::vector<Pose2D> path{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const Pose2D pose{0.4, 0.0, 0.0};
  const auto nearest = omni_navigation::find_nearest_index(path, pose, 0);
  EXPECT_EQ(nearest, 0u);
  EXPECT_EQ(omni_navigation::find_target_index(path, pose, nearest, 0.30), 1u);
}

TEST(PurePursuit, SinglePointPathTargetsThatPoint) {
  const std::vector<Pose2D> path{{1.0, 0.0, 0.0}};
  EXPECT_EQ(omni_navigation::find_target_index(path, Pose2D{}, 0, 0.30), 0u);
}

TEST(PurePursuit, MovesSidewaysWithoutTurning) {
  // 機体が前方を向いたまま、左にある目標へ平行移動する
  PurePursuitConfig config;
  const Pose2D pose{0.0, 0.0, 0.0};
  const Pose2D target{0.0, 1.0, 0.0};
  const auto velocity =
      omni_navigation::compute_velocity(pose, target, target, config);
  EXPECT_NEAR(velocity.x, 0.0, kTolerance);
  EXPECT_NEAR(velocity.y, config.max_linear_speed, kTolerance);
  EXPECT_NEAR(velocity.yaw, 0.0, kTolerance);
}

TEST(PurePursuit, ConvertsWorldDirectionIntoBodyFrame) {
  // 左を向いた機体にとって、ワールドのx正方向は右側にある
  PurePursuitConfig config;
  const Pose2D pose{0.0, 0.0, M_PI / 2.0};
  const Pose2D target{1.0, 0.0, M_PI / 2.0};
  const auto velocity =
      omni_navigation::compute_velocity(pose, target, target, config);
  EXPECT_NEAR(velocity.x, 0.0, kTolerance);
  EXPECT_NEAR(velocity.y, -config.max_linear_speed, kTolerance);
}

TEST(PurePursuit, SlowsDownNearGoal) {
  PurePursuitConfig config;
  const Pose2D pose{0.0, 0.0, 0.0};
  const Pose2D goal{0.1, 0.0, 0.0};
  const auto velocity =
      omni_navigation::compute_velocity(pose, goal, goal, config);
  EXPECT_NEAR(velocity.x, config.position_gain * 0.1, kTolerance);
}

TEST(PurePursuit, LimitsAngularSpeed) {
  PurePursuitConfig config;
  const Pose2D pose{0.0, 0.0, 0.0};
  const Pose2D target{0.0, 0.0, 3.0};
  const auto velocity =
      omni_navigation::compute_velocity(pose, target, target, config);
  EXPECT_NEAR(velocity.yaw, config.max_angular_speed, kTolerance);
}
