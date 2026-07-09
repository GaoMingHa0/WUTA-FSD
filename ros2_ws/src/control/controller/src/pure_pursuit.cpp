#include "controller/pure_pursuit.hpp"
#include <algorithm>
#include <limits>

namespace controller
{

PurePursuit::PurePursuit(const VehicleParams & params, const Config & cfg)
: params_(params), cfg_(cfg) {}

ControlCommand PurePursuit::compute(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints)
{
  ControlCommand cmd;
  if (waypoints.empty()) return cmd;

  // 1. Compute lookahead distance — velocity-proportional, clamped
  lookahead_dist_ = std::clamp(
    std::abs(state.velocity) * cfg_.ld_ratio,
    cfg_.min_lookahead,
    cfg_.max_lookahead);

  // 2. Find target waypoint
  target_idx_ = findTargetIndex(state, waypoints, lookahead_dist_);
  if (target_idx_ < 0) {
    target_idx_ = static_cast<int>(waypoints.size()) - 1;
  }

  const auto & target = waypoints[target_idx_];
  const double tx = target.pose.pose.position.x;
  const double ty = target.pose.pose.position.y;

  // 3. Distance to target
  const double dist = planeDist(tx, ty, state.x, state.y);
  if (dist < 1e-6) return cmd;

  // 4. Lateral offset in vehicle body frame (x_body = how far left/right target is)
  const double x_body = lateralOffset(tx, ty, state.x, state.y, state.yaw);

  // Numerical stabilization: amplify very small lateral offset (straight-ahead case)
  double numerator = 2.0 * x_body;
  if (std::abs(numerator) < 0.1) {
    numerator = 10.0 * std::copysign(1.0, numerator) * numerator;
  }

  // 5. Curvature: kappa = 2·x_body / dist²
  const double kappa = numerator / (dist * dist);

  // 6. Steering angle (Ackermann bicycle model): δ = atan(L × kappa)
  cmd.steering_angle = std::atan(params_.wheel_base * kappa) * 180.0 / M_PI;

  // 7. Velocity from target waypoint
  cmd.velocity = target.twist.twist.linear.x;

  cmd.valid = true;
  return cmd;
}

int PurePursuit::findTargetIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double ld) const
{
  // Search backwards from end of path to find first waypoint within LD
  // (HRT-D FindTheTargetNormal logic)
  for (int i = static_cast<int>(waypoints.size()) - 1; i >= 1; --i) {
    const double d = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    if (d < ld) return i;
  }
  return static_cast<int>(waypoints.size()) - 1;
}

double PurePursuit::lateralOffset(
  double target_x, double target_y,
  double car_x,    double car_y, double car_yaw)
{
  const double dx = target_x - car_x;
  const double dy = target_y - car_y;
  // Body frame x = lateral (left positive), y = longitudinal (forward positive)
  // x_body = -dx·sin(yaw) + dy·cos(yaw)
  return -dx * std::sin(car_yaw) + dy * std::cos(car_yaw);
}

double PurePursuit::planeDist(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace controller
