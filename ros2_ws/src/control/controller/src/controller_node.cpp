#include "controller/controller_node.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace controller
{

using MissionState = wuta_msgs::msg::MissionState;

namespace
{
double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  // --- Vehicle parameters ---
  VehicleParams vp;
  vp.wheel_base       = declare_parameter("wheel_base",       vp.wheel_base);
  vp.lf               = declare_parameter("lf",               vp.lf);
  vp.max_steer_angle  = declare_parameter("max_steer_angle",  vp.max_steer_angle);

  // --- Pure Pursuit config ---
  PurePursuit::Config pp_cfg;
  pp_cfg.ld_ratio       = declare_parameter("ld_ratio",       pp_cfg.ld_ratio);
  pp_cfg.min_lookahead  = declare_parameter("min_lookahead",  pp_cfg.min_lookahead);
  pp_cfg.max_lookahead  = declare_parameter("max_lookahead",  pp_cfg.max_lookahead);
  pp_cfg.max_progress_advance = declare_parameter(
    "max_progress_advance", pp_cfg.max_progress_advance);
  skidpad_lookahead_ = declare_parameter("skidpad_lookahead", skidpad_lookahead_);
  trackdrive_lookahead_ = declare_parameter(
    "trackdrive_lookahead", trackdrive_lookahead_);
  trackdrive_dynamic_lookahead_ = declare_parameter(
    "trackdrive_dynamic_lookahead", trackdrive_dynamic_lookahead_);
  trackdrive_min_lookahead_ = declare_parameter(
    "trackdrive_min_lookahead", trackdrive_min_lookahead_);
  trackdrive_curvature_preview_distance_ = declare_parameter(
    "trackdrive_curvature_preview_distance", trackdrive_curvature_preview_distance_);
  trackdrive_straight_curvature_ = declare_parameter(
    "trackdrive_straight_curvature", trackdrive_straight_curvature_);
  trackdrive_corner_curvature_ = declare_parameter(
    "trackdrive_corner_curvature", trackdrive_corner_curvature_);
  trackdrive_lookahead_rate_limit_ = declare_parameter(
    "trackdrive_lookahead_rate_limit", trackdrive_lookahead_rate_limit_);
  trackdrive_min_lookahead_ = std::min(trackdrive_min_lookahead_, trackdrive_lookahead_);
  trackdrive_corner_curvature_ = std::max(
    trackdrive_corner_curvature_, trackdrive_straight_curvature_ + 1e-6);
  filtered_trackdrive_lookahead_ = trackdrive_lookahead_;
  trackdrive_target_loss_hold_time_ = declare_parameter(
    "trackdrive_target_loss_hold_time", trackdrive_target_loss_hold_time_);
  trackdrive_target_loss_hold_speed_ = declare_parameter(
    "trackdrive_target_loss_hold_speed", trackdrive_target_loss_hold_speed_);
  trackdrive_start_speed_ = declare_parameter(
    "trackdrive_start_speed", trackdrive_start_speed_);
  trackdrive_start_speed_duration_ = declare_parameter(
    "trackdrive_start_speed_duration", trackdrive_start_speed_duration_);

  // --- Control loop rate ---
  const int rate_hz = declare_parameter("control_rate_hz", 50);
  const double max_steering_rate_deg_s = declare_parameter(
    "max_steering_rate_deg_s", 180.0);
  finish_position_tolerance_ = declare_parameter(
    "finish_position_tolerance", finish_position_tolerance_);
  finish_speed_threshold_ = declare_parameter(
    "finish_speed_threshold", finish_speed_threshold_);
  pp_cfg.terminal_progress_distance = finish_position_tolerance_;

  pure_pursuit_ = std::make_unique<PurePursuit>(vp, pp_cfg);
  twist_filter_ = std::make_unique<TwistFilter>(
    vp, rate_hz, max_steering_rate_deg_s);

  // --- Subscribers ---
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&ControllerNode::onPose, this, std::placeholders::_1));

  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/localization/velocity", 10,
    std::bind(&ControllerNode::onVelocity, this, std::placeholders::_1));

  waypoints_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/final_waypoints", 10,
    std::bind(&ControllerNode::onWaypoints, this, std::placeholders::_1));

  mission_sub_ = create_subscription<MissionState>(
    "/system/mission_state", 10,
    std::bind(&ControllerNode::onMissionState, this, std::placeholders::_1));

  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency", 10,
    std::bind(&ControllerNode::onEmergency, this, std::placeholders::_1));

  // --- Publishers ---
  cmd_pub_ = create_publisher<autoware_msgs::msg::Command>("/control/command", 10);
  mission_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/mission_complete", 10);
  target_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/control/target_viz", 10);

  // --- Control loop timer ---
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / rate_hz),
    std::bind(&ControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "ControllerNode ready. rate=%dHz, LD_ratio=%.1f",
    rate_hz, pp_cfg.ld_ratio);
}

void ControllerNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  vehicle_state_.x = msg->pose.position.x;
  vehicle_state_.y = msg->pose.position.y;

  const auto & q = msg->pose.orientation;
  vehicle_state_.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  pose_ready_ = true;
}

void ControllerNode::onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Magnitude of velocity vector
  const double vx = msg->twist.linear.x;
  const double vy = msg->twist.linear.y;
  vehicle_state_.vx = vx;
  vehicle_state_.vy = vy;
  vehicle_state_.velocity = std::sqrt(vx * vx + vy * vy);
}

void ControllerNode::onWaypoints(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  const bool changed = !isSamePath(msg->waypoints);
  waypoints_ = msg->waypoints;
  waypoints_ready_ = !waypoints_.empty();
  if (changed) {
    pure_pursuit_->reset();
    mission_complete_ = false;
  }
}

void ControllerNode::onMissionState(const MissionState::SharedPtr msg)
{
  mission_mode_ = msg->mission_mode;
  enabled_ = (
    msg->state == MissionState::EXPLORE ||
    msg->state == MissionState::MAPPING_DONE ||
    msg->state == MissionState::RACE);

  if (!enabled_) {
    twist_filter_->reset();
    last_valid_trackdrive_cmd_ready_ = false;
    trackdrive_start_speed_started_ = false;
    // Publish stop command
    publishZeroCommand();
  }
}

void ControllerNode::onEmergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_ = msg->data;
}

void ControllerNode::controlLoop()
{
  // 急停：停止一切控制输出，持续发布全零命令直至解除
  if (emergency_) {
    twist_filter_->reset();
    last_valid_trackdrive_cmd_ready_ = false;
    publishZeroCommand();
    return;
  }

  if (!enabled_ || !pose_ready_ || !waypoints_ready_) return;

  if (mission_complete_) return;
  const auto loop_time = now();

  // 1. Pure Pursuit
  // At 5 m/s the generic LD=v*2 would preview 10 m, almost one skidpad
  // radius. At the entry, circle transition, and exit this selects a point
  // from the following path segment and makes the bicycle model cut inward or
  // unload steering before the crossing.
  double lookahead_override = 0.0;
  if (mission_mode_ == MissionState::MISSION_SKIDPAD) {
    lookahead_override = skidpad_lookahead_;
  } else if (mission_mode_ == MissionState::MISSION_TRACKDRIVE) {
    lookahead_override = trackdriveLookahead(loop_time);
  }
  auto raw_cmd = pure_pursuit_->compute(
    vehicle_state_, waypoints_, lookahead_override);
  if (mission_mode_ == MissionState::MISSION_TRACKDRIVE && raw_cmd.valid) {
    // Trackdrive receives a freshly rebuilt local centerline on every map
    // update. Its progress index therefore restarts at the vehicle-origin
    // waypoint, whose curvature and speed are normally zero/maximum. Use the
    // same forward target selected for lateral Pure Pursuit so the curvature
    // speed profile is effective before entering the bend. Skidpad and
    // Acceleration keep progress-point speed for their ordered stop paths.
    const int target_index = pure_pursuit_->targetIndex();
    if (target_index >= 0 &&
        target_index < static_cast<int>(waypoints_.size())) {
      raw_cmd.velocity = waypoints_[target_index].twist.twist.linear.x;
    }
    last_valid_trackdrive_cmd_ = raw_cmd;
    last_valid_trackdrive_cmd_time_ = loop_time;
    last_valid_trackdrive_cmd_ready_ = true;
  }

  const bool stopping_mission =
    mission_mode_ == MissionState::MISSION_SKIDPAD ||
    mission_mode_ == MissionState::MISSION_ACCELERATION;
  if (stopping_mission &&
      pure_pursuit_->progressIndex() == static_cast<int>(waypoints_.size()) - 1 &&
      std::hypot(
        waypoints_.back().pose.pose.position.x - vehicle_state_.x,
        waypoints_.back().pose.pose.position.y - vehicle_state_.y) <= finish_position_tolerance_ &&
      vehicle_state_.velocity <= finish_speed_threshold_)
  {
    publishMissionComplete();
    return;
  }

  if (!raw_cmd.valid) {
    const bool can_hold_trackdrive_cmd =
      mission_mode_ == MissionState::MISSION_TRACKDRIVE &&
      last_valid_trackdrive_cmd_ready_ &&
      (loop_time - last_valid_trackdrive_cmd_time_).seconds() <=
        std::max(0.0, trackdrive_target_loss_hold_time_);

    if (can_hold_trackdrive_cmd) {
      raw_cmd = last_valid_trackdrive_cmd_;
      raw_cmd.velocity = std::min(
        raw_cmd.velocity, std::max(0.0, trackdrive_target_loss_hold_speed_));
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "No forward waypoint target available; holding last Trackdrive command at %.2f m/s.",
        raw_cmd.velocity);
    } else {
      last_valid_trackdrive_cmd_ready_ = false;
      twist_filter_->reset();
      publishZeroCommand();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "No forward waypoint target available; publishing stop command.");
      return;
    }
  }

  if (mission_mode_ == MissionState::MISSION_TRACKDRIVE && raw_cmd.valid &&
      trackdrive_start_speed_duration_ > 0.0)
  {
    // Start this interval only when planning first offers a forward target.
    // Starting it at EXPLORE would consume the protection while mapping is
    // still empty and the vehicle is stationary.
    if (!trackdrive_start_speed_started_) {
      trackdrive_start_speed_started_ = true;
      trackdrive_start_speed_time_ = loop_time;
      RCLCPP_INFO(
        get_logger(),
        "Trackdrive launch speed fixed at %.2f m/s for %.2f s after first valid target.",
        trackdrive_start_speed_, trackdrive_start_speed_duration_);
    }
    const double elapsed = (loop_time - trackdrive_start_speed_time_).seconds();
    if (elapsed < trackdrive_start_speed_duration_) {
      raw_cmd.velocity = std::max(0.0, trackdrive_start_speed_);
    }
  }

  // 2. Safety filter
  auto filtered = twist_filter_->filter(raw_cmd.steering_angle, raw_cmd.velocity);

  // 3. Publish command
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = loop_time;
  cmd.header.frame_id = "base_link";
  cmd.speed    = filtered.velocity;
  cmd.angle    = filtered.steering_angle;
  cmd_pub_->publish(cmd);

  // DEBUG: throttled to 2 Hz
  {
    static auto last_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count() >= 500) {
      last_log = now;
      int tgt = pure_pursuit_->targetIndex();
      int prog = pure_pursuit_->progressIndex();
      double ld = pure_pursuit_->lookaheadDistance();
      RCLCPP_INFO(get_logger(),
        "state=(%.2f,%.2f) yaw=%.2f° v=%.2f | waypoints=%zu | target=%d prog=%d/%zu ld=%.2f "
        "| raw(angle=%.1f° vel=%.1f) | cmd(angle=%.1f° vel=%.1f)",
        vehicle_state_.x, vehicle_state_.y, vehicle_state_.yaw * 180.0 / M_PI,
        vehicle_state_.velocity, waypoints_.size(), tgt, prog, waypoints_.size(), ld,
        raw_cmd.steering_angle, raw_cmd.velocity,
        filtered.steering_angle, filtered.velocity);
    }
  }

  // 4. Visualization (target waypoint marker)
  if (target_viz_pub_->get_subscription_count() > 0 &&
      pure_pursuit_->targetIndex() < static_cast<int>(waypoints_.size()))
  {
    const auto & wp = waypoints_[pure_pursuit_->targetIndex()];
    publishVisualization(
      wp.pose.pose.position.x,
      wp.pose.pose.position.y);
  }
}

double ControllerNode::trackdriveLookahead(const rclcpp::Time & loop_time)
{
  if (!trackdrive_dynamic_lookahead_ || waypoints_.size() < 3) {
    filtered_trackdrive_lookahead_ = trackdrive_lookahead_;
    last_trackdrive_lookahead_time_ = loop_time;
    return filtered_trackdrive_lookahead_;
  }

  // The local Trackdrive lane starts near the vehicle and is rebuilt as the
  // map changes. Inspect its forward near-horizon geometry: the robust
  // percentile rejects one noisy point, while a portion of the maximum still
  // detects the onset of a genuinely sharp upcoming turn.
  std::vector<double> curvatures;
  double inspected_distance = 0.0;
  // 曲率预览用速度航向角，与 pure pursuit 横向几何保持一致（低速退化为 yaw）
  const double speed = std::hypot(vehicle_state_.vx, vehicle_state_.vy);
  const double course = speed > 0.5
    ? vehicle_state_.yaw + std::atan2(vehicle_state_.vy, vehicle_state_.vx)
    : vehicle_state_.yaw;
  for (size_t i = 0; i + 2 < waypoints_.size(); ++i) {
    const auto & p0 = waypoints_[i].pose.pose.position;
    const auto & p1 = waypoints_[i + 1].pose.pose.position;
    const auto & p2 = waypoints_[i + 2].pose.pose.position;
    const double forward =
      (p1.x - vehicle_state_.x) * std::cos(course) +
      (p1.y - vehicle_state_.y) * std::sin(course);
    if (forward < -0.5) continue;

    const double ds0 = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const double ds1 = std::hypot(p2.x - p1.x, p2.y - p1.y);
    if (ds0 < 1e-3 || ds1 < 1e-3) continue;
    inspected_distance += ds0;
    if (inspected_distance > trackdrive_curvature_preview_distance_) break;

    const double heading0 = std::atan2(p1.y - p0.y, p1.x - p0.x);
    const double heading1 = std::atan2(p2.y - p1.y, p2.x - p1.x);
    curvatures.push_back(std::abs(normalizeAngle(heading1 - heading0)) /
                         (0.5 * (ds0 + ds1)));
  }

  double effective_curvature = 0.0;
  if (!curvatures.empty()) {
    std::sort(curvatures.begin(), curvatures.end());
    const size_t percentile_index = static_cast<size_t>(
      0.75 * static_cast<double>(curvatures.size() - 1));
    effective_curvature = std::max(
      curvatures[percentile_index], 0.6 * curvatures.back());
  }
  const double curvature_ratio = std::clamp(
    (effective_curvature - trackdrive_straight_curvature_) /
      (trackdrive_corner_curvature_ - trackdrive_straight_curvature_),
    0.0, 1.0);
  const double desired = trackdrive_lookahead_ - curvature_ratio *
    (trackdrive_lookahead_ - trackdrive_min_lookahead_);

  if (last_trackdrive_lookahead_time_.nanoseconds() != 0) {
    const double dt = std::max(0.0, (loop_time - last_trackdrive_lookahead_time_).seconds());
    const double max_change = trackdrive_lookahead_rate_limit_ * dt;
    filtered_trackdrive_lookahead_ += std::clamp(
      desired - filtered_trackdrive_lookahead_, -max_change, max_change);
  } else {
    filtered_trackdrive_lookahead_ = desired;
  }
  last_trackdrive_lookahead_time_ = loop_time;
  return std::clamp(
    filtered_trackdrive_lookahead_, trackdrive_min_lookahead_, trackdrive_lookahead_);
}

bool ControllerNode::isSamePath(
  const std::vector<autoware_msgs::msg::Waypoint> & candidate) const
{
  if (candidate.size() != waypoints_.size()) return false;
  for (size_t i = 0; i < candidate.size(); ++i) {
    const auto & lhs = candidate[i].pose.pose.position;
    const auto & rhs = waypoints_[i].pose.pose.position;
    if (std::abs(lhs.x - rhs.x) > 1e-6 || std::abs(lhs.y - rhs.y) > 1e-6 ||
        std::abs(lhs.z - rhs.z) > 1e-6) {
      return false;
    }
  }
  return true;
}

void ControllerNode::publishMissionComplete()
{
  mission_complete_ = true;
  enabled_ = false;
  twist_filter_->reset();
  last_valid_trackdrive_cmd_ready_ = false;

  publishZeroCommand();

  std_msgs::msg::Bool complete;
  complete.data = true;
  mission_complete_pub_->publish(complete);
  RCLCPP_INFO(
    get_logger(),
    "Mission complete: mode=%u progress=%d/%zu pose=(%.3f, %.3f) speed=%.3f m/s",
    mission_mode_,
    pure_pursuit_->progressIndex(), waypoints_.size() - 1,
    vehicle_state_.x, vehicle_state_.y, vehicle_state_.velocity);
}

void ControllerNode::publishVisualization(double target_x, double target_y)
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp    = now();
  m.ns     = "pp_target";
  m.id     = 0;
  m.type   = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x  = target_x;
  m.pose.position.y  = target_y;
  m.pose.position.z  = 0.5;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
  m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 1.0f;
  arr.markers.push_back(m);

  // Lookahead circle
  visualization_msgs::msg::Marker circle;
  circle.header = m.header;
  circle.ns   = "pp_lookahead";
  circle.id   = 1;
  circle.type = visualization_msgs::msg::Marker::CYLINDER;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.pose.position.x = vehicle_state_.x;
  circle.pose.position.y = vehicle_state_.y;
  circle.pose.position.z = 0.0;
  circle.pose.orientation.w = 1.0;
  const double ld = pure_pursuit_->lookaheadDistance();
  circle.scale.x = ld * 2; circle.scale.y = ld * 2; circle.scale.z = 0.05;
  circle.color.r = 0.0f; circle.color.g = 0.6f; circle.color.b = 1.0f; circle.color.a = 0.3f;
  arr.markers.push_back(circle);

  target_viz_pub_->publish(arr);
}

void ControllerNode::publishZeroCommand()
{
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.speed = 0.0;
  cmd.angle = 0.0;
  cmd_pub_->publish(cmd);
}

}  // namespace controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
