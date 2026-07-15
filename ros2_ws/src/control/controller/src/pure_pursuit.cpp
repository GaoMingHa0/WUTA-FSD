#include "controller/pure_pursuit.hpp"
#include <algorithm>
#include <limits>

namespace controller
{

PurePursuit::PurePursuit(const VehicleParams & params, const Config & cfg)
: params_(params), cfg_(cfg) {}

void PurePursuit::reset()
{
  lookahead_dist_ = 0.0;
  target_idx_ = 0;
  progress_idx_ = 0;
}

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

  // 2. Advance monotonically along the path, then look ahead from that point.
  // This is essential for self-intersecting/overlapping paths such as skidpad:
  // selecting the last geometrically-close waypoint would jump to a later lap.
  progress_idx_ = std::max(
    progress_idx_, findNearestForwardIndex(state, waypoints));
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

  // 7. Velocity follows the current path progress rather than the geometric
  // lookahead point.  This lets the planned skidpad exit brake at the stop
  // line instead of commanding zero speed one lookahead distance too early.
  cmd.velocity = waypoints[progress_idx_].twist.twist.linear.x;

  cmd.valid = true;
  return cmd;
}

int PurePursuit::findTargetIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double ld) const
{
  // First point at or beyond the lookahead distance after current progress.
  for (int i = progress_idx_; i < static_cast<int>(waypoints.size()); ++i) {
    const double d = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    if (d >= ld) return i;
  }
  return static_cast<int>(waypoints.size()) - 1;
}

int PurePursuit::findNearestForwardIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints) const
{
  int nearest = std::min(progress_idx_, static_cast<int>(waypoints.size()) - 1);
  double nearest_distance = std::numeric_limits<double>::max();
  // Only inspect the locally reachable part of the route. A figure-8 has
  // overlapping crossings and an exit line that can be geometrically closer
  // than the active circle; a global search would skip directly to that exit.
  const int last_candidate = std::min(
    static_cast<int>(waypoints.size()) - 1,
    nearest + std::max(1, cfg_.max_progress_advance));
  for (int i = nearest; i <= last_candidate; ++i) {
    const double distance = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    // Keep the first index for ties: repeated crossing points must resolve to
    // the current lap, not an identical point in a future lap.
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  return nearest;
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
