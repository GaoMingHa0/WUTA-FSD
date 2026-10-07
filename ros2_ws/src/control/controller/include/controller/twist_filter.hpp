#pragma once
#include "controller/vehicle_state.hpp"

namespace controller
{

// 控制指令安全滤波：速度平滑 + 转向限幅与限速。
class TwistFilter
{
public:
  struct Config
  {
    double accel_alpha{0.1};  // 加速平滑系数(0~1)，out = (1-α)·last + α·input
    double decel_alpha{0.7};  // 减速响应系数(0~1)
  };

  TwistFilter(const VehicleParams & params, int control_rate_hz,
              double max_steering_rate_deg_s, const Config & cfg);

  struct FilteredCommand
  {
    double steering_angle{0.0};  // deg，已限幅限速
    double velocity{0.0};        // m/s，已平滑
  };

  FilteredCommand filter(double raw_angle, double raw_velocity);

  void reset() { last_velocity_ = 0.0; last_steering_angle_ = 0.0; }

private:
  VehicleParams params_;
  Config cfg_;
  int control_rate_hz_{50};
  double max_steering_rate_deg_s_{180.0};
  double last_velocity_{0.0};
  double last_steering_angle_{0.0};
};

}  // namespace controller
