#pragma once
#include <cmath>

namespace controller
{

struct VehicleParams
{
  double wheel_base{1.614};      // m，轴距
  double max_steer_angle{28.0};  // deg，前轮最大转角
};

struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};        // rad
  double velocity{0.0};   // m/s，实测车速
  double vx{0.0};         // m/s，车体纵向速度
  double vy{0.0};         // m/s，车体横向速度
};

// 速度航向角：低速用 yaw，高速叠加侧偏角 atan2(vy, vx)
inline double velocityCourse(const VehicleState & state, double speed_threshold)
{
  const double speed = std::hypot(state.vx, state.vy);
  return speed > speed_threshold
    ? state.yaw + std::atan2(state.vy, state.vx)
    : state.yaw;
}

}  // namespace controller
