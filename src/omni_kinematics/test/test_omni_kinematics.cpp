#include <gtest/gtest.h>

#include <cmath>

#include "omni_kinematics/omni_kinematics.hpp"

using omni_kinematics::BodyMotion;
using omni_kinematics::Pose2D;
using omni_kinematics::WheelMotion;

namespace
{
constexpr double kBaseRadius = 0.15;
constexpr double kTolerance = 1e-12;
}  // namespace

TEST(BodyToWheels, ForwardMotionDoesNotTurnFrontWheel)
{
  const auto wheels = omni_kinematics::body_to_wheels(BodyMotion{1.0, 0.0, 0.0}, kBaseRadius);
  EXPECT_NEAR(wheels.front, 0.0, kTolerance);
  EXPECT_NEAR(wheels.rear_left, -std::sqrt(3.0) / 2.0, kTolerance);
  EXPECT_NEAR(wheels.rear_right, std::sqrt(3.0) / 2.0, kTolerance);
}

TEST(BodyToWheels, RotationDrivesAllWheelsEqually)
{
  const auto wheels = omni_kinematics::body_to_wheels(BodyMotion{0.0, 0.0, 2.0}, kBaseRadius);
  EXPECT_NEAR(wheels.front, 0.3, kTolerance);
  EXPECT_NEAR(wheels.rear_left, 0.3, kTolerance);
  EXPECT_NEAR(wheels.rear_right, 0.3, kTolerance);
}

TEST(WheelsToBody, InvertsBodyToWheels)
{
  const BodyMotion body{0.4, -0.3, 0.7};
  const auto result = omni_kinematics::wheels_to_body(
    omni_kinematics::body_to_wheels(body, kBaseRadius), kBaseRadius);
  EXPECT_NEAR(result.x, body.x, kTolerance);
  EXPECT_NEAR(result.y, body.y, kTolerance);
  EXPECT_NEAR(result.yaw, body.yaw, kTolerance);
}

TEST(IntegrateMidpoint, MovesAlongCurrentHeading)
{
  const auto pose = omni_kinematics::integrate_midpoint(
    Pose2D{1.0, 2.0, M_PI / 2.0}, BodyMotion{0.5, 0.0, 0.0});
  EXPECT_NEAR(pose.x, 1.0, kTolerance);
  EXPECT_NEAR(pose.y, 2.5, kTolerance);
  EXPECT_NEAR(pose.yaw, M_PI / 2.0, kTolerance);
}

TEST(IntegrateMidpoint, TraversesCircle)
{
  // 半径1 mの円を1周すると原点へ戻る
  constexpr int steps = 1000;
  const double dyaw = 2.0 * M_PI / steps;
  Pose2D pose;
  for (int i = 0; i < steps; ++i) {
    pose = omni_kinematics::integrate_midpoint(pose, BodyMotion{dyaw, 0.0, dyaw});
  }
  EXPECT_NEAR(pose.x, 0.0, 1e-5);
  EXPECT_NEAR(pose.y, 0.0, 1e-5);
  EXPECT_NEAR(pose.yaw, 0.0, 1e-9);
}

TEST(IntegrateMidpoint, WrapsYawIntoPlusMinusPi)
{
  const auto pose = omni_kinematics::integrate_midpoint(
    Pose2D{0.0, 0.0, 3.0}, BodyMotion{0.0, 0.0, 1.0});
  EXPECT_NEAR(pose.yaw, 4.0 - 2.0 * M_PI, kTolerance);
}

TEST(Compose, AppliesParentRotation)
{
  const auto pose = omni_kinematics::compose(Pose2D{1.0, 0.0, M_PI / 2.0}, Pose2D{1.0, 0.0, 0.5});
  EXPECT_NEAR(pose.x, 1.0, kTolerance);
  EXPECT_NEAR(pose.y, 1.0, kTolerance);
  EXPECT_NEAR(pose.yaw, M_PI / 2.0 + 0.5, kTolerance);
}

TEST(Inverse, ComposeWithInverseIsIdentity)
{
  const Pose2D pose{0.7, -1.2, 2.5};
  const auto identity = omni_kinematics::compose(pose, omni_kinematics::inverse(pose));
  EXPECT_NEAR(identity.x, 0.0, kTolerance);
  EXPECT_NEAR(identity.y, 0.0, kTolerance);
  EXPECT_NEAR(identity.yaw, 0.0, kTolerance);
}

TEST(Interpolate, TurnsThroughShortestArc)
{
  const auto pose = omni_kinematics::interpolate(Pose2D{0.0, 0.0, 3.0}, Pose2D{2.0, 4.0, -3.0}, 0.5);
  EXPECT_NEAR(pose.x, 1.0, kTolerance);
  EXPECT_NEAR(pose.y, 2.0, kTolerance);
  EXPECT_NEAR(std::abs(pose.yaw), M_PI, kTolerance);
}
