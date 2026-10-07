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

  // --- 车检模式（INSPECTION）参数 ---
  inspection_speed_ = declare_parameter("inspection_speed", inspection_speed_);
  // 车检恒定纵向开度：车举升/拆胎时唯一的反馈（华测车速）恒为 0，速度环不可观测，
  // 故车检不走 PID，只发这一个常量。钳到 [0,1]：车检只允许驱动方向，不允许制动。
  inspection_throttle_ = std::clamp(
    declare_parameter<double>("inspection_throttle", inspection_throttle_), 0.0, 1.0);
  if (inspection_throttle_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
      "inspection_throttle=%.3f 会让驱动系统不转；规则 2.8.3 要求其旋转，请调大。",
      inspection_throttle_);
  }
  inspection_steer_amp_ = declare_parameter("inspection_steer_amp", inspection_steer_amp_);
  // 转向周期（s）优先：比频率直观（写 6.0 而不是 0.1666667）；>0 时覆盖 freq
  inspection_steer_period_ = declare_parameter(
    "inspection_steer_period", inspection_steer_period_);
  inspection_steer_freq_ = declare_parameter("inspection_steer_freq", inspection_steer_freq_);
  if (inspection_steer_period_ > 0.0) {
    inspection_steer_freq_ = 1.0 / inspection_steer_period_;
  }
  inspection_duration_ = declare_parameter("inspection_duration", inspection_duration_);
  // 收尾相位校验：车检结束那一帧的正弦值必须接近 0——finishInspection() 直接发 angle=0，
  // 末帧若停在幅值附近就会形成单帧大跳变（实测 27.0s/0.4Hz → 末帧 -14.3°@20ms ≈ 715°/s，
  // 是 180°/s 限幅的 4 倍）。注意半整数周期同样过零（sin(2π(n+0.5))=0），
  // 所以判据看"末帧角度"，而不是"周期数是否整数"。
  if (inspection_steer_freq_ > 0.0) {
    const double cycles = inspection_duration_ * inspection_steer_freq_;
    const double end_phase = cycles - std::floor(cycles);
    const double end_angle = std::abs(
      inspection_steer_amp_ * std::sin(2.0 * M_PI * end_phase));
    if (end_angle > 5.0) {
      RCLCPP_WARN(get_logger(),
        "车检收尾停在 %.2f 个周期处（相位 %.2f），末帧转向 %.1f° > 5°：回中会有单帧跳变；"
        "请让 inspection_duration 与转向周期成 0.5 的整数倍关系。",
        cycles, end_phase, end_angle);
    }
  }

  // --- 速度 PID（纵向开度）---
  pid_speed_kp_ = declare_parameter("pid_speed_kp", pid_speed_kp_);
  pid_speed_ki_ = declare_parameter("pid_speed_ki", pid_speed_ki_);
  pid_speed_kd_ = declare_parameter("pid_speed_kd", pid_speed_kd_);
  pid_stop_clear_eps_ = declare_parameter("pid_stop_clear_eps", pid_stop_clear_eps_);

  pure_pursuit_ = std::make_unique<PurePursuit>(vp, pp_cfg);
  twist_filter_ = std::make_unique<TwistFilter>(
    vp, rate_hz, max_steering_rate_deg_s);

  // --- Subscribers ---
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&ControllerNode::onPose, this, std::placeholders::_1));

  // 车速（华测 INS）：同时供 Pure Pursuit 前视与纵向 PID 反馈
  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/chcnav/velocity", 10,
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
  speed_feedback_available_ = true;  // 华测速度到达，PID 反馈就绪
}

double ControllerNode::computeSpeedPid(double target_speed)
{
  const rclcpp::Time t = now();
  const double err = target_speed - vehicle_state_.velocity;
  if (pid_last_time_.nanoseconds() == 0) {
    pid_last_time_ = t;
    pid_prev_err_ = err;
    return 0.0;  // 首拍只初始化，不输出
  }
  const double dt = std::max(0.001, (t - pid_last_time_).seconds());

  // 停车消积分：目标 0 且车速已接近停稳时清零积分，防止巡航期残留
  // 的正积分在停车后输出驱动开度导致溜车。
  // TwistFilter 的减速输出渐近趋 0（0.3 倍衰减）但永不为 0，若用严格
  // `target_speed <= 0.0` 判定则永不成立、积分残留，故引入小阈值。
  if (target_speed <= pid_stop_clear_eps_ && vehicle_state_.velocity < 0.5) {
    pid_integral_ = 0.0;
  }

  // 防积分饱和：积分项与输出同量纲，钳位到 [-1,1]
  pid_integral_ = std::clamp(
    pid_integral_ + pid_speed_ki_ * err * dt, -1.0, 1.0);
  const double deriv = (err - pid_prev_err_) / dt;
  const double x = std::clamp(
    pid_speed_kp_ * err + pid_integral_ + pid_speed_kd_ * deriv, -1.0, 1.0);

  pid_prev_err_ = err;
  pid_last_time_ = t;
  return x;
}

double ControllerNode::computeThrottleBrake(double target_speed)
{
  if (emergency_ || !speed_feedback_available_) {
    // 急停或速度反馈未就绪：纵向开度输出 0（保守，不驱动）
    pid_integral_ = 0.0;
    pid_prev_err_ = 0.0;
    pid_last_time_ = rclcpp::Time();
    return 0.0;
  }
  return computeSpeedPid(target_speed);
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
  const uint8_t prev_state = state_;
  state_ = msg->state;
  mission_mode_ = msg->mission_mode;
  enabled_ = (
    msg->state == MissionState::EXPLORE ||
    msg->state == MissionState::MAPPING_DONE ||
    msg->state == MissionState::RACE);

  // 零指令只在真的没有别的发布者时才发。车检是例外：它由 runInspection() 以 50Hz
  // 独占发布正弦转向 + PID 开度，若这里再按「未使能」插一脚，10Hz 零指令会和它抢
  // 同一个 /control/command（vcan0 实测：13% 的帧横向被拉回中位，纵向在满驱动与
  // 满制动之间跳），而且两条路径共用同一套 PID 状态、互相当成对方的导数。
  if (!enabled_ && state_ != MissionState::INSPECTION) {
    twist_filter_->reset();
    last_valid_trackdrive_cmd_ready_ = false;
    trackdrive_start_speed_started_ = false;
    // Publish stop command
    publishZeroCommand(" [inactive state]");
  }

  // 车检模式：进入 INSPECTION 启动演示，离开时复位
  if (state_ == MissionState::INSPECTION &&
      prev_state != MissionState::INSPECTION) {
    // 清掉上一模式残留的控制状态：否则 PID 误差从 0 突变到 +1，导数项会让第一拍
    // 直接顶到限幅。
    twist_filter_->reset();
    pid_integral_ = 0.0;
    pid_prev_err_ = 0.0;
    pid_last_time_ = rclcpp::Time();
    inspection_start_time_ = now();
    inspection_done_published_ = false;
    const double steer_period = inspection_steer_freq_ > 0.0
      ? 1.0 / inspection_steer_freq_ : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "Inspection started: throttle=%.2f (constant, no PID) steer=%.1f deg @%.2fs (%.3f Hz) for %.1f s",
      inspection_throttle_, inspection_steer_amp_,
      steer_period, inspection_steer_freq_, inspection_duration_);
  } else if (state_ != MissionState::INSPECTION &&
             prev_state == MissionState::INSPECTION) {
    inspection_done_published_ = false;
  }
}

void ControllerNode::onEmergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 急停不可恢复：只置位，不因上游 false 解除（与 mission_manager 终态语义一致）
  if (msg->data) emergency_ = true;
}

void ControllerNode::controlLoop()
{
  // 急停：停止一切控制输出，持续发布全零命令直至解除
  if (emergency_) {
    twist_filter_->reset();
    last_valid_trackdrive_cmd_ready_ = false;
    publishZeroCommand(" [EMERGENCY zero hold]");
    return;
  }

  // 车检模式：慢速转驱动 + 正弦波转转向
  if (state_ == MissionState::INSPECTION) {
    runInspection();
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
    mission_mode_ == MissionState::MISSION_ACCELERATION ||
    mission_mode_ == MissionState::MISSION_EBS_TEST;  // EBS 复用同一终点完成判定
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
      publishZeroCommand(" [no forward target]");
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
  const double throttle_brake = computeThrottleBrake(filtered.velocity);  // 速度 PID → 纵向开度
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = loop_time;
  cmd.header.frame_id = "base_link";
  cmd.speed    = filtered.velocity;
  cmd.angle    = filtered.steering_angle;
  cmd.throttle_brake = throttle_brake;
  cmd_pub_->publish(cmd);

  // 调试日志（2Hz，独立节流点）：与零指令路径共用 controlLine，格式逐行对齐
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s",
    controlLine(raw_cmd.steering_angle, raw_cmd.velocity,
      cmd.angle, cmd.speed, throttle_brake, "").c_str());

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

void ControllerNode::runInspection()
{
  if (inspection_done_published_) return;
  const double t = (now() - inspection_start_time_).seconds();

  // 演示时长到达：停零并回报完成（mission_manager 切 FINISH）
  if (t >= inspection_duration_) {
    finishInspection();
    return;
  }

  // 正弦波转转向：仍经 TwistFilter 做速率限制（与正常赛项同一套滤波）
  const double steer_deg = inspection_steer_amp_ *
    std::sin(2.0 * M_PI * inspection_steer_freq_ * t);
  const auto filtered = twist_filter_->filter(steer_deg, inspection_speed_);

  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.speed = filtered.velocity;          // 仅记录：车举升无车速反馈，不代表实际转速
  cmd.angle = filtered.steering_angle;
  // 车检纵向：**恒定开度，不走 PID**。车举升/拆胎后唯一的反馈（华测车速）恒为 0，
  // 误差恒 +1 → 积分必然顶到限幅：速度环在此场景没有可观测的被控量。直接发常量反而
  // 确定、可复现；「转速快慢」只由 inspection_throttle 一个旋钮决定（先低后调）。
  cmd.throttle_brake = inspection_throttle_;
  cmd_pub_->publish(cmd);
}

void ControllerNode::finishInspection()
{
  inspection_done_published_ = true;
  // 车检期间已不跑 PID，仍清一次状态：保证回零第一拍从零开始，也不把任何残留误差
  // 带进后续赛项（旧的 PID 方案收尾实测有 2 帧 raw=10 满制动，约 40ms）。
  pid_integral_ = 0.0;
  pid_prev_err_ = 0.0;
  pid_last_time_ = rclcpp::Time();
  publishZeroCommand();

  std_msgs::msg::Bool complete;
  complete.data = true;
  mission_complete_pub_->publish(complete);
  RCLCPP_INFO(get_logger(), "Inspection complete.");
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

void ControllerNode::publishZeroCommand(const char * tag)
{
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.speed = 0.0;
  cmd.angle = 0.0;
  cmd.throttle_brake = computeThrottleBrake(0.0);
  cmd_pub_->publish(cmd);

  // 零指令路径（急停/未使能/无目标/完成）同样周期打印，确认仍在持续发布全零；
  // 独立节流点（2Hz），不被正常路径日志顶掉。
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s",
    controlLine(0.0, 0.0, cmd.angle, cmd.speed, cmd.throttle_brake, tag).c_str());
}

std::string ControllerNode::controlLine(
  double raw_angle, double raw_vel,
  double cmd_angle, double cmd_vel, double cmd_thr,
  const char * tag)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "state=(%.2f,%.2f) yaw=%.2f° v=%.2f | waypoints=%zu | target=%d prog=%d/%zu ld=%.2f "
    "| raw(angle=%.1f° vel=%.1f) | cmd(angle=%.1f° vel=%.1f thr=%.2f)%s",
    vehicle_state_.x, vehicle_state_.y, vehicle_state_.yaw * 180.0 / M_PI,
    vehicle_state_.velocity, waypoints_.size(),
    pure_pursuit_->targetIndex(), pure_pursuit_->progressIndex(),
    waypoints_.size(), pure_pursuit_->lookaheadDistance(),
    raw_angle, raw_vel, cmd_angle, cmd_vel, cmd_thr, tag);
  return std::string(buf);
}

}  // namespace controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
