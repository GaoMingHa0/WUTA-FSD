#include "controller/twist_filter.hpp"
#include <algorithm>
#include <cmath>

namespace controller
{

TwistFilter::TwistFilter(const VehicleParams & params, int control_rate_hz,
                         double max_steering_rate_deg_s, const Config & cfg)
: params_(params), cfg_(cfg), control_rate_hz_(std::max(1, control_rate_hz)),
  max_steering_rate_deg_s_(std::max(0.0, max_steering_rate_deg_s)) {}

TwistFilter::FilteredCommand TwistFilter::filter(
  double raw_angle, double raw_velocity)
{
  FilteredCommand out;

  // 速度一阶平滑：加速用 accel_alpha，减速用 decel_alpha
  const double alpha = raw_velocity >= last_velocity_
    ? cfg_.accel_alpha : cfg_.decel_alpha;
  out.velocity = (1.0 - alpha) * last_velocity_ + alpha * raw_velocity;
  last_velocity_ = out.velocity;

  // 转向先限幅到最大转角，再按每周期最大变化量限速
  const double bounded_angle = std::clamp(
    raw_angle, -params_.max_steer_angle, params_.max_steer_angle);
  const double max_step = max_steering_rate_deg_s_ / control_rate_hz_;
  out.steering_angle = std::clamp(
    bounded_angle,
    last_steering_angle_ - max_step,
    last_steering_angle_ + max_step);
  last_steering_angle_ = out.steering_angle;

  return out;
}

}  // namespace controller
