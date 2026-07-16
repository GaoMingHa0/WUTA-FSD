#pragma once
#include "controller/vehicle_state.hpp"

namespace controller
{

/**
 * Safety filter for control commands (from HRT-D TwistFilter, stripped of HPL).
 *
 * Velocity:
 *   - Acceleration: slow exponential ramp   (0.9×last + 0.1×input)
 *   - Deceleration: faster response         (0.3×last + 0.7×input)
 *
 * Steering:
 *   - Hard clamp to ±max_steer_angle
 *
 * Emergency:
 *   - If emergency flag set → velocity = 0
 */
class TwistFilter
{
public:
  TwistFilter(const VehicleParams & params, int control_rate_hz,
              double max_steering_rate_deg_s);

  struct FilteredCommand
  {
    double steering_angle{0.0};  // degrees, clamped
    double velocity{0.0};        // m/s, smoothed
    bool   emergency{false};
  };

  FilteredCommand filter(double raw_angle, double raw_velocity,
                         bool emergency = false);

  void reset() { last_velocity_ = 0.0; last_steering_angle_ = 0.0; }

private:
  VehicleParams params_;
  int control_rate_hz_{50};
  double max_steering_rate_deg_s_{180.0};
  double last_velocity_{0.0};
  double last_steering_angle_{0.0};
};

}  // namespace controller
