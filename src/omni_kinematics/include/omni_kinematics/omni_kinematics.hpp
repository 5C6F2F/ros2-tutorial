// 3輪オムニの運動学
//
// 測定輪は機体中心から半径Lの円周上に、前方を0度として0度、120度、240度で並ぶ。
// 値の正方向は、機体を反時計回りに回す向き。
// 線形変換なので、速度にも微小変位にも同じ関数を使える。
#ifndef OMNI_KINEMATICS__OMNI_KINEMATICS_HPP_
#define OMNI_KINEMATICS__OMNI_KINEMATICS_HPP_

#include <cmath>

namespace omni_kinematics
{

// 機体座標系の動き。x前方、y左方、yaw反時計回り
struct BodyMotion
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 各測定輪の接線方向の動き
struct WheelMotion
{
  double front{0.0};
  double rear_left{0.0};
  double rear_right{0.0};
};

// ワールド座標系の姿勢
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 逆運動学: 機体の動き → 各測定輪の動き
inline WheelMotion body_to_wheels(const BodyMotion & body, const double base_radius)
{
  const double sqrt3_2 = std::sqrt(3.0) / 2.0;
  const double rotation = base_radius * body.yaw;
  return WheelMotion{
    body.y + rotation,
    -sqrt3_2 * body.x - 0.5 * body.y + rotation,
    sqrt3_2 * body.x - 0.5 * body.y + rotation};
}

// 順運動学: 各測定輪の動き → 機体の動き
inline BodyMotion wheels_to_body(const WheelMotion & wheels, const double base_radius)
{
  return BodyMotion{
    (wheels.rear_right - wheels.rear_left) / std::sqrt(3.0),
    (2.0 * wheels.front - wheels.rear_left - wheels.rear_right) / 3.0,
    (wheels.front + wheels.rear_left + wheels.rear_right) / (3.0 * base_radius)};
}

// 機体座標系の微小変位をワールド座標系へ積算する。向きには区間の中点の値を使う
inline Pose2D integrate_midpoint(const Pose2D & pose, const BodyMotion & delta)
{
  const double yaw_mid = pose.yaw + 0.5 * delta.yaw;
  const double cos_yaw = std::cos(yaw_mid);
  const double sin_yaw = std::sin(yaw_mid);
  return Pose2D{
    pose.x + cos_yaw * delta.x - sin_yaw * delta.y,
    pose.y + sin_yaw * delta.x + cos_yaw * delta.y,
    std::remainder(pose.yaw + delta.yaw, 2.0 * M_PI)};
}

// 姿勢の合成: aの座標系から見たbを、aの親の座標系で表す
inline Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  const double cos_yaw = std::cos(a.yaw);
  const double sin_yaw = std::sin(a.yaw);
  return Pose2D{
    a.x + cos_yaw * b.x - sin_yaw * b.y,
    a.y + sin_yaw * b.x + cos_yaw * b.y,
    std::remainder(a.yaw + b.yaw, 2.0 * M_PI)};
}

// 逆変換: 親から見た子の姿勢を、子から見た親の姿勢へ変える
inline Pose2D inverse(const Pose2D & pose)
{
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  return Pose2D{
    -cos_yaw * pose.x - sin_yaw * pose.y,
    sin_yaw * pose.x - cos_yaw * pose.y,
    std::remainder(-pose.yaw, 2.0 * M_PI)};
}

// aからbへ、割合ratioだけ近づける。角度は近い向きへ回す
inline Pose2D interpolate(const Pose2D & a, const Pose2D & b, const double ratio)
{
  return Pose2D{
    a.x + ratio * (b.x - a.x),
    a.y + ratio * (b.y - a.y),
    std::remainder(a.yaw + ratio * std::remainder(b.yaw - a.yaw, 2.0 * M_PI), 2.0 * M_PI)};
}

}  // namespace omni_kinematics

#endif  // OMNI_KINEMATICS__OMNI_KINEMATICS_HPP_
