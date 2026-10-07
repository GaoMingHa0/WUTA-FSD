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

  const double course = velocityCourse(state, cfg_.course_speed_threshold);

  // 1. 前视距离：外部覆盖优先，否则按车速动态计算并限幅
  lookahead_dist_ = lookahead_override > 0.0
    ? lookahead_override
    : std::clamp(
        std::abs(state.velocity) * cfg_.ld_ratio,
        cfg_.min_lookahead,
        cfg_.max_lookahead);

  // 2. 单调推进进度点，再从该点起向前选目标点
  progress_idx_ = std::max(
    progress_idx_, findNearestForwardIndex(state, waypoints, course));
  target_idx_ = findTargetIndex(state, waypoints, lookahead_dist_, course);
  if (target_idx_ < 0) {
    target_idx_ = static_cast<int>(waypoints.size()) - 1;
  }

  const auto & target = waypoints[target_idx_];
  if (longitudinalOffset(
      target.pose.pose.position.x, target.pose.pose.position.y,
      state.x, state.y, course) <= 0.0) {
    return cmd;
  }
  const double tx = target.pose.pose.position.x;
  const double ty = target.pose.pose.position.y;

  // 3. 目标点距离
  const double dist = planeDist(tx, ty, state.x, state.y);
  if (dist < 1e-6) return cmd;

  // 4. 目标点在车体横向的偏移（左正）
  const double x_body = lateralOffset(tx, ty, state.x, state.y, course);

  // 5. 曲率 kappa = 2·x_body / dist²
  const double kappa = (2.0 * x_body) / (dist * dist);

  // 6. 前轮转角 δ = atan(L × kappa)
  cmd.steering_angle = std::atan(params_.wheel_base * kappa) * 180.0 / M_PI;

  // 7. 目标速度取进度点速度
  cmd.velocity = waypoints[progress_idx_].twist.twist.linear.x;

  cmd.valid = true;
  return cmd;
}

int PurePursuit::findTargetIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double ld, double course) const
{
  // 从进度点起找首个「在车前且距离 ≥ ld」的点；没有则取最远的前向点
  int furthest_forward_idx = -1;
  double furthest_forward = 0.0;
  for (int i = progress_idx_; i < static_cast<int>(waypoints.size()); ++i) {
    const double forward = longitudinalOffset(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y, course);
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
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double course) const
{
  int nearest = std::min(progress_idx_, static_cast<int>(waypoints.size()) - 1);
  double nearest_distance = std::numeric_limits<double>::max();
  // 只搜索进度点起 max_progress_advance 个点，避免自交路径跳到后续圈
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
        // 未进入终点容差前，跳过零速终点，保持倒数正速度点
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
      state.x, state.y, course);
    if (forward < -cfg_.forward_margin) continue;
    // 并列时保留靠前的索引，避免自交点解析到后续圈
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  return nearest;
}

double PurePursuit::lateralOffset(
  double target_x, double target_y,
  double car_x,    double car_y, double course_angle)
{
  const double dx = target_x - car_x;
  const double dy = target_y - car_y;
  return -dx * std::sin(course_angle) + dy * std::cos(course_angle);
}

double PurePursuit::longitudinalOffset(
  double target_x, double target_y,
  double car_x,    double car_y, double course_angle)
{
  const double dx = target_x - car_x;
  const double dy = target_y - car_y;
  return dx * std::cos(course_angle) + dy * std::sin(course_angle);
}

double PurePursuit::planeDist(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace controller
