#include "controller/twist_filter.hpp"
#include <algorithm>
#include <cmath>

namespace controller
{

TwistFilter::TwistFilter(const VehicleParams & params, int control_rate_hz,
                         double max_steering_rate_deg_s)
: params_(params), control_rate_hz_(std::max(1, control_rate_hz)),
  max_steering_rate_deg_s_(std::max(0.0, max_steering_rate_deg_s)) {}

TwistFilter::FilteredCommand TwistFilter::filter(
  double raw_angle, double raw_velocity)
{
  FilteredCommand out;

  // --- Velocity smoothing (from HRT-D TwistFilter) ---
  if (raw_velocity >= last_velocity_) {
    // Accelerating: gentle ramp to avoid wheel spin
    out.velocity = 0.9 * last_velocity_ + 0.1 * raw_velocity;
  } else {
    // Decelerating: faster response for safety
    out.velocity = 0.3 * last_velocity_ + 0.7 * raw_velocity;
  }
  last_velocity_ = out.velocity;

  // --- Steering clamp and rate limit ---
  const double bounded_angle = std::clamp(
    raw_angle,
    -params_.max_steer_angle,
     params_.max_steer_angle);
  // Rate limiting suppresses command chatter caused by localization noise
  // without changing the path geometry.
  const double max_step = max_steering_rate_deg_s_ / control_rate_hz_;
  out.steering_angle = std::clamp(
    bounded_angle,
    last_steering_angle_ - max_step,
    last_steering_angle_ + max_step);
  last_steering_angle_ = out.steering_angle;

  return out;
}

}  // namespace controller
