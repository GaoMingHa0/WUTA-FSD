#pragma once
#include <algorithm>

namespace controller
{
struct SpeedPidConfig
{
  double kp;
  double ki;
  double kd;
  double integral_limit;
  double max_throttle;
  double max_brake;
};

// 正负开度分别限幅；积分防饱和使用实际驱动/制动边界。
inline double speedPidEffort(double error, double dt, double previous_error,
  const SpeedPidConfig & config, double & integral)
{
  const double derivative = (error - previous_error) / dt;
  const double candidate_integral = std::clamp(
    integral + config.ki * error * dt, -config.integral_limit, config.integral_limit);
  const double candidate_output = config.kp * error + candidate_integral +
    config.kd * derivative;
  if (!((candidate_output > config.max_throttle && error > 0) ||
    (candidate_output < -config.max_brake && error < 0))) {
    integral = candidate_integral;
  }
  return std::clamp(config.kp * error + integral + config.kd * derivative,
    -config.max_brake, config.max_throttle);
}
}  // namespace controller
