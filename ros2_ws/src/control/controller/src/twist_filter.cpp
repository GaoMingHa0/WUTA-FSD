#include "controller/twist_filter.hpp"
#include <algorithm>
#include <cmath>

namespace controller
{

TwistFilter::TwistFilter(const VehicleParams & params)
: params_(params) {}

TwistFilter::FilteredCommand TwistFilter::filter(
  double raw_angle, double raw_velocity, bool emergency)
{
  FilteredCommand out;
  out.emergency = emergency;

  // --- Emergency: stop immediately ---
  if (emergency) {
    out.velocity = 0.0;
    out.steering_angle = 0.0;
    last_velocity_ = 0.0;
    return out;
  }

  // --- Velocity smoothing (from HRT-D TwistFilter) ---
  if (raw_velocity >= last_velocity_) {
    // Accelerating: gentle ramp to avoid wheel spin
    out.velocity = 0.9 * last_velocity_ + 0.1 * raw_velocity;
  } else {
    // Decelerating: faster response for safety
    out.velocity = 0.3 * last_velocity_ + 0.7 * raw_velocity;
  }
  last_velocity_ = out.velocity;

  // --- Steering clamp ---
  out.steering_angle = std::clamp(
    raw_angle,
    -params_.max_steer_angle,
     params_.max_steer_angle);

  return out;
}

}  // namespace controller
