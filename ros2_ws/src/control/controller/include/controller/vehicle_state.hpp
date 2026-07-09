#pragma once
#include <cmath>

namespace controller
{

struct VehicleParams
{
  double wheel_base{1.53};      // m — front to rear axle
  double lf{0.8};               // m — CoM to front axle
  double max_steer_angle{25.0}; // degrees
};

struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};          // rad
  double velocity{0.0};     // m/s, current measured speed
};

}  // namespace controller
