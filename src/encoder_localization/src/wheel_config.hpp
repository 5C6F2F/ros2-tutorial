#pragma once
#include <units.h>

using namespace units::length;
using namespace units::angle;

struct WheelConfig {
  meter_t x;       // ロボット中心からの位置 x [m]
  meter_t y;       // ロボット中心からの位置 y [m]
  radian_t theta;  // 車輪の取り付け角度 [rad] (前方が0, 反時計回り正)
  meter_t radius;  // 車輪半径 [m]
};
