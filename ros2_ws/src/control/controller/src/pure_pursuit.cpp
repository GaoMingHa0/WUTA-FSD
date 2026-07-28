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
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double lookahead_override)
{
  ControlCommand cmd;
  if (waypoints.empty()) return cmd;

  // 1. Compute lookahead distance — velocity-proportional, clamped
  lookahead_dist_ = lookahead_override > 0.0
    ? lookahead_override
    : std::clamp(
        std::abs(state.velocity) * cfg_.ld_ratio,
        cfg_.min_lookahead,
        cfg_.max_lookahead);

  // 2. Advance monotonically along the path, then look ahead from that point.
  // This is essential for self-intersecting/overlapping paths such as skidpad:
  // selecting the last geometrically-close waypoint would jump to a later lap.
  // Trackdrive local paths may also be re-ordered, so targets behind the car are rejected.
  progress_idx_ = std::max(
    progress_idx_, findNearestForwardIndex(state, waypoints));
  target_idx_ = findTargetIndex(state, waypoints, lookahead_dist_);
  if (target_idx_ < 0) {
    target_idx_ = static_cast<int>(waypoints.size()) - 1;
  }

  const auto & target = waypoints[target_idx_];
  if (longitudinalOffset(
      target.pose.pose.position.x, target.pose.pose.position.y,
      state.x, state.y, state.yaw) <= 0.0) {
    return cmd;
  }
  const double tx = target.pose.pose.position.x;
  const double ty = target.pose.pose.position.y;

  // 3. Distance to target
  const double dist = planeDist(tx, ty, state.x, state.y);
  if (dist < 1e-6) return cmd;

  // 4. Lateral offset in vehicle body frame (x_body = how far left/right target is)
  const double x_body = lateralOffset(tx, ty, state.x, state.y, state.yaw);

  // 5. Curvature: kappa = 2·x_body / dist²
  // Keep the pure-pursuit relationship continuous around x_body = 0.  The
  // former small-error amplification introduced a 10x jump at its threshold,
  // which appeared as severe steering chatter in the driven trajectory.
  const double kappa = (2.0 * x_body) / (dist * dist);

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
  // First point at or beyond the lookahead distance after current progress,
  // but only if it is in front of the vehicle.  A locally re-planned
  // Trackdrive lane can occasionally arrive in the reverse order; following a
  // behind-car target makes the vehicle turn around and circle.
  int furthest_forward_idx = -1;
  double furthest_forward = 0.0;
  for (int i = progress_idx_; i < static_cast<int>(waypoints.size()); ++i) {
    const double forward = longitudinalOffset(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y, state.yaw);
    if (forward <= 0.0) continue;
    if (forward > furthest_forward) {
      furthest_forward = forward;
      furthest_forward_idx = i;
    }
    const double d = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    if (d >= ld) return i;
  }
  return furthest_forward_idx;
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
    const bool is_zero_speed_terminal =
      i == static_cast<int>(waypoints.size()) - 1 &&
      std::abs(waypoints[i].twist.twist.linear.x) < 1e-6;
    if (is_zero_speed_terminal) {
      if (distance > std::max(0.0, cfg_.terminal_progress_distance)) {
        // Keep commanding the final positive-speed waypoint until the vehicle
        // is close enough to stop. This prevents pose noise from selecting the
        // zero-speed endpoint several metres early on ordered stopping paths.
        continue;
      }
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = i;
      }
      continue;
    }
    const double forward = longitudinalOffset(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y, state.yaw);
    if (forward < -0.5) continue;
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

double PurePursuit::longitudinalOffset(
  double target_x, double target_y,
  double car_x,    double car_y, double car_yaw)
{
  const double dx = target_x - car_x;
  const double dy = target_y - car_y;
  // Body frame y = longitudinal, positive in front of the vehicle.
  return dx * std::cos(car_yaw) + dy * std::sin(car_yaw);
}

double PurePursuit::planeDist(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace controller
