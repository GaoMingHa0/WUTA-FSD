#include "controller/controller_node.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <stdexcept>
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
  // 车辆参数
  VehicleParams vp;
  vp.wheel_base      = declare_parameter("vehicle.wheel_base",       vp.wheel_base);
  vp.max_steer_angle = declare_parameter("vehicle.max_steer_angle",  vp.max_steer_angle);
  if (!std::isfinite(vp.wheel_base) || vp.wheel_base <= 0 ||
    !std::isfinite(vp.max_steer_angle) || vp.max_steer_angle <= 0) {
    throw std::invalid_argument("Vehicle wheel_base and max_steer_angle must be finite and positive");
  }

  // 控制回路与安全滤波（所有赛项共用）
  const int rate_hz = declare_parameter("control.rate_hz", 50);
  const double max_steering_rate_deg_s = declare_parameter(
    "control.max_steering_rate_deg_s", 180.0);
  TwistFilter::Config filter_cfg;
  filter_cfg.accel_alpha = declare_parameter("control.accel_alpha", filter_cfg.accel_alpha);
  filter_cfg.decel_alpha = declare_parameter("control.decel_alpha", filter_cfg.decel_alpha);
  if (rate_hz <= 0 || !std::isfinite(max_steering_rate_deg_s) || max_steering_rate_deg_s < 0 ||
    !std::isfinite(filter_cfg.accel_alpha) || filter_cfg.accel_alpha < 0 || filter_cfg.accel_alpha > 1 ||
    !std::isfinite(filter_cfg.decel_alpha) || filter_cfg.decel_alpha < 0 || filter_cfg.decel_alpha > 1) {
    throw std::invalid_argument("Invalid control rate, steering rate or smoothing alpha");
  }

  // 各赛项独立调参
  loadMissionConfig();
  brake_bench_mode_ = declare_parameter("brake_bench_mode", false);
  bench_delay_sec_ = declare_parameter("acceleration.bench_brake.delay_sec", 5.0);
  bench_hold_sec_ = declare_parameter("acceleration.bench_brake.hold_sec", 10.0);
  bench_opening_ = declare_parameter("acceleration.bench_brake.opening", 0.5);
  if (!std::isfinite(bench_delay_sec_) || bench_delay_sec_ < 0 ||
    !std::isfinite(bench_hold_sec_) || bench_hold_sec_ <= 0 ||
    !std::isfinite(bench_opening_) || bench_opening_ <= 0 || bench_opening_ > 1 ||
    (brake_bench_mode_ && bench_opening_ > accel_cfg_.brake_max_opening)) {
    throw std::invalid_argument("Invalid acceleration.bench_brake timing/opening or brake limit");
  }

  // 车检参数
  inspection_throttle_ = std::clamp(
    declare_parameter<double>("inspection.throttle", inspection_throttle_), 0.0, 1.0);
  inspection_steer_amp_ = declare_parameter("inspection.steer_amplitude", inspection_steer_amp_);
  inspection_steer_period_ = declare_parameter("inspection.steer_period", inspection_steer_period_);
  inspection_duration_ = declare_parameter("inspection.duration", inspection_duration_);
  // 由周期与时长推出频率：取整数个半周期，保证收尾正弦过零、回中无跳变
  if (inspection_steer_period_ > 0.0 && inspection_duration_ > 0.0) {
    const double half_cycles = std::max(
      1.0, std::round(2.0 * inspection_duration_ / inspection_steer_period_));
    inspection_steer_freq_ = half_cycles / (2.0 * inspection_duration_);
  }

  pure_pursuit_ = std::make_unique<PurePursuit>(vp, accel_cfg_.pp);
  twist_filter_ = std::make_unique<TwistFilter>(
    vp, rate_hz, max_steering_rate_deg_s, filter_cfg);
  applyMissionConfig(mission_mode_);

  // 订阅
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&ControllerNode::onPose, this, std::placeholders::_1));
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
    "/system/emergency", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ControllerNode::onEmergency, this, std::placeholders::_1));

  // 发布
  cmd_pub_ = create_publisher<autoware_msgs::msg::Command>("/control/command", 10);
  mission_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/mission_complete", 10);
  target_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/control/target_viz", 10);

  // 控制定时器
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_hz)),
    std::bind(&ControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "ControllerNode ready. rate=%dHz", rate_hz);

}

// 从参数读入各赛项的横向/纵向/终点配置
void ControllerNode::loadMissionConfig()
{
  auto load_pp_common = [this](const std::string & ns, PurePursuit::Config & c) {
    c.max_progress_advance = declare_parameter(
      ns + ".max_progress_advance", c.max_progress_advance);
    c.terminal_progress_distance = declare_parameter(
      ns + ".terminal_progress_distance", c.terminal_progress_distance);
    c.course_speed_threshold = declare_parameter(
      ns + ".course_speed_threshold", c.course_speed_threshold);
    c.forward_margin = declare_parameter(
      ns + ".forward_margin", c.forward_margin);
  };
  auto load_pid = [this](const std::string & ns, MissionConfig & m) {
    m.pid_kp = declare_parameter(ns + ".pid.kp", m.pid_kp);
    m.pid_ki = declare_parameter(ns + ".pid.ki", m.pid_ki);
    m.pid_kd = declare_parameter(ns + ".pid.kd", m.pid_kd);
    m.pid_stop_clear_eps = declare_parameter(
      ns + ".pid.stop_clear_eps", m.pid_stop_clear_eps);
    m.pid_stop_clear_speed = declare_parameter(
      ns + ".pid.stop_clear_speed", m.pid_stop_clear_speed);
    // 油门/制动限幅属于执行开度参数，按赛项分别设置。
    if (get_node_parameters_interface()->get_parameter_overrides().count(ns + ".pid.output_limit")) {
      throw std::invalid_argument(ns + ": 旧pid.output_limit已移除，请将原值分别迁移到throttle.max_opening和brake.max_opening");
    }
    m.throttle_max_opening = declare_parameter(
      ns + ".throttle.max_opening", m.throttle_max_opening);
    m.brake_max_opening = declare_parameter(
      ns + ".brake.max_opening", m.brake_max_opening);
    m.pid_integral_limit = declare_parameter(
      ns + ".pid.integral_limit", m.pid_integral_limit);
    for (double value : {m.pid_kp, m.pid_ki, m.pid_kd, m.pid_stop_clear_eps,
      m.pid_stop_clear_speed, m.pid_integral_limit}) {
      if (!std::isfinite(value) || value < 0) {
        throw std::invalid_argument(ns + ": PID参数必须为非负有限值");
      }
    }
    for (double opening : {m.throttle_max_opening, m.brake_max_opening}) {
      if (!std::isfinite(opening) || opening < 0 || opening > 1) {
        throw std::invalid_argument(ns + ": throttle/brake.max_opening必须为[0,1]内的有限值");
      }
    }
  };
  auto load_finish = [this](const std::string & ns, MissionConfig & m) {
    m.has_finish = true;
    m.finish_position_tolerance = declare_parameter(
      ns + ".finish.position_tolerance", m.finish_position_tolerance);
    m.finish_speed_threshold = declare_parameter(
      ns + ".finish.speed_threshold", m.finish_speed_threshold);
  };

  // 直线加速（动态前视）
  accel_cfg_.pp.ld_ratio = declare_parameter(
    "acceleration.pure_pursuit.lookahead_ratio", accel_cfg_.pp.ld_ratio);
  accel_cfg_.pp.min_lookahead = declare_parameter(
    "acceleration.pure_pursuit.min_lookahead", accel_cfg_.pp.min_lookahead);
  accel_cfg_.pp.max_lookahead = declare_parameter(
    "acceleration.pure_pursuit.max_lookahead", accel_cfg_.pp.max_lookahead);
  load_pp_common("acceleration.pure_pursuit", accel_cfg_.pp);
  load_pid("acceleration", accel_cfg_);
  load_finish("acceleration", accel_cfg_);

  // EBS 测试（动态前视）
  ebs_cfg_.pp.ld_ratio = declare_parameter(
    "ebs.pure_pursuit.lookahead_ratio", ebs_cfg_.pp.ld_ratio);
  ebs_cfg_.pp.min_lookahead = declare_parameter(
    "ebs.pure_pursuit.min_lookahead", ebs_cfg_.pp.min_lookahead);
  ebs_cfg_.pp.max_lookahead = declare_parameter(
    "ebs.pure_pursuit.max_lookahead", ebs_cfg_.pp.max_lookahead);
  load_pp_common("ebs.pure_pursuit", ebs_cfg_.pp);
  load_pid("ebs", ebs_cfg_);
  load_finish("ebs", ebs_cfg_);

  // 八字绕环（固定前视）
  skidpad_lookahead_ = declare_parameter(
    "skidpad.pure_pursuit.lookahead", skidpad_lookahead_);
  load_pp_common("skidpad.pure_pursuit", skidpad_cfg_.pp);
  load_pid("skidpad", skidpad_cfg_);
  load_finish("skidpad", skidpad_cfg_);

  // 高速循迹（曲率自适应前视 + 起步/丢目标）
  trackdrive_lookahead_ = declare_parameter(
    "trackdrive.pure_pursuit.max_lookahead", trackdrive_lookahead_);
  trackdrive_min_lookahead_ = declare_parameter(
    "trackdrive.pure_pursuit.min_lookahead", trackdrive_min_lookahead_);
  trackdrive_min_lookahead_ = std::min(trackdrive_min_lookahead_, trackdrive_lookahead_);
  trackdrive_dynamic_lookahead_ = declare_parameter(
    "trackdrive.pure_pursuit.dynamic_lookahead", trackdrive_dynamic_lookahead_);
  trackdrive_curvature_preview_distance_ = declare_parameter(
    "trackdrive.pure_pursuit.curvature_preview_distance",
    trackdrive_curvature_preview_distance_);
  trackdrive_straight_curvature_ = declare_parameter(
    "trackdrive.pure_pursuit.straight_curvature", trackdrive_straight_curvature_);
  trackdrive_corner_curvature_ = declare_parameter(
    "trackdrive.pure_pursuit.corner_curvature", trackdrive_corner_curvature_);
  trackdrive_corner_curvature_ = std::max(
    trackdrive_corner_curvature_, trackdrive_straight_curvature_ + 1e-6);
  trackdrive_lookahead_rate_limit_ = declare_parameter(
    "trackdrive.pure_pursuit.lookahead_rate_limit", trackdrive_lookahead_rate_limit_);
  curvature_percentile_ = declare_parameter(
    "trackdrive.pure_pursuit.curvature_percentile", curvature_percentile_);
  curvature_peak_factor_ = declare_parameter(
    "trackdrive.pure_pursuit.curvature_peak_factor", curvature_peak_factor_);
  trackdrive_cfg_.pp.min_lookahead = trackdrive_min_lookahead_;
  trackdrive_cfg_.pp.max_lookahead = trackdrive_lookahead_;
  load_pp_common("trackdrive.pure_pursuit", trackdrive_cfg_.pp);
  load_pid("trackdrive", trackdrive_cfg_);
  trackdrive_target_loss_hold_time_ = declare_parameter(
    "trackdrive.target_loss_hold_time", trackdrive_target_loss_hold_time_);
  filtered_trackdrive_lookahead_ = trackdrive_lookahead_;
  for (const auto * cfg : {&accel_cfg_, &ebs_cfg_, &skidpad_cfg_, &trackdrive_cfg_}) {
    if (!std::isfinite(cfg->pp.ld_ratio) || cfg->pp.ld_ratio <= 0 ||
      !std::isfinite(cfg->pp.min_lookahead) || cfg->pp.min_lookahead <= 0 ||
      !std::isfinite(cfg->pp.max_lookahead) || cfg->pp.max_lookahead < cfg->pp.min_lookahead) {
      throw std::invalid_argument("前视距离参数必须有限、为正且上下限有序");
    }
  }
  if (!std::isfinite(curvature_percentile_) || curvature_percentile_ < 0 || curvature_percentile_ > 1 ||
    !std::isfinite(curvature_peak_factor_) || curvature_peak_factor_ < 0 ||
    !std::isfinite(trackdrive_lookahead_rate_limit_) || trackdrive_lookahead_rate_limit_ < 0 ||
    !std::isfinite(trackdrive_straight_curvature_) || trackdrive_straight_curvature_ < 0 ||
    !std::isfinite(trackdrive_corner_curvature_) || trackdrive_corner_curvature_ <= trackdrive_straight_curvature_) {
    throw std::invalid_argument("曲率前视参数无效");
  }
}

const ControllerNode::MissionConfig & ControllerNode::configFor(uint8_t mission_mode) const
{
  switch (mission_mode) {
    case MissionState::MISSION_SKIDPAD:      return skidpad_cfg_;
    case MissionState::MISSION_ACCELERATION: return accel_cfg_;
    case MissionState::MISSION_EBS_TEST:     return ebs_cfg_;
    case MissionState::MISSION_TRACKDRIVE:
    default:                                 return trackdrive_cfg_;
  }
}

void ControllerNode::applyMissionConfig(uint8_t mission_mode)
{
  const MissionConfig & cfg = configFor(mission_mode);
  pure_pursuit_->setConfig(cfg.pp);
  pid_speed_kp_ = cfg.pid_kp;
  pid_speed_ki_ = cfg.pid_ki;
  pid_speed_kd_ = cfg.pid_kd;
  pid_stop_clear_eps_ = cfg.pid_stop_clear_eps;
  pid_stop_clear_speed_ = cfg.pid_stop_clear_speed;
  throttle_max_opening_ = cfg.throttle_max_opening;
  brake_max_opening_ = cfg.brake_max_opening;
  pid_integral_limit_ = cfg.pid_integral_limit;
}

void ControllerNode::resetSpeedPid()
{
  pid_integral_ = 0.0;
  pid_prev_err_ = 0.0;
  pid_last_time_ = rclcpp::Time();
}

void ControllerNode::resetControlPipeline()
{
  twist_filter_->reset();
  last_valid_trackdrive_cmd_ready_ = false;
}

void ControllerNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & p = msg->pose.position;
  const auto & rotation = msg->pose.orientation;
  const double norm2 = rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm2) || norm2 < 1e-12) {
    pose_ready_ = false;
    return;
  }
  vehicle_state_.x = msg->pose.position.x;
  vehicle_state_.y = msg->pose.position.y;

  const auto & q = msg->pose.orientation;
  vehicle_state_.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    norm2 - 2.0 * (q.y * q.y + q.z * q.z));

  pose_ready_ = true;
  last_pose_time_ = now();
}

void ControllerNode::onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const double vx = msg->twist.linear.x;
  const double vy = msg->twist.linear.y;
  const double speed = std::hypot(vx, vy);
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(speed)) {
    speed_feedback_available_ = false;
    return;
  }
  vehicle_state_.vx = vx;
  vehicle_state_.vy = vy;
  vehicle_state_.velocity = speed;
  speed_feedback_available_ = true;
  last_speed_feedback_time_ = now();
}

double ControllerNode::computeSpeedPid(double target_speed)
{
  const rclcpp::Time t = now();
  const double err = target_speed - vehicle_state_.velocity;
  if (pid_last_time_.nanoseconds() == 0) {
    pid_last_time_ = t;
    pid_prev_err_ = err;
    return 0.0;  // 首拍只初始化时间
  }
  const double dt = std::max(0.001, (t - pid_last_time_).seconds());

  // 目标速度接近 0 且车速已接近停稳时清零积分
  if (target_speed <= pid_stop_clear_eps_ && vehicle_state_.velocity < pid_stop_clear_speed_) {
    pid_integral_ = 0.0;
  }

  const SpeedPidConfig cfg{
    pid_speed_kp_, pid_speed_ki_, pid_speed_kd_, pid_integral_limit_,
    throttle_max_opening_, brake_max_opening_};
  const double x = speedPidEffort(err, dt, pid_prev_err_, cfg, pid_integral_);

  pid_prev_err_ = err;
  pid_last_time_ = t;
  return x;
}

double ControllerNode::computeThrottleBrake(double target_speed)
{
  // 赛项PID要求有效车速反馈；固定0.5秒超时保护，不提供额外调参入口。
  const double feedback_age = speed_feedback_available_
    ? (now() - last_speed_feedback_time_).seconds() : 0.0;
  const bool feedback_fresh = speed_feedback_available_ && std::isfinite(feedback_age) &&
    feedback_age >= 0 && feedback_age <= 0.5;
  if (emergency_ || !feedback_fresh ||
    !std::isfinite(target_speed) || target_speed < 0) {
    if (!emergency_ && !feedback_fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "车速反馈未就绪、无效或超时，纵向开度归零（不代表主动急停）。");
    }
    resetSpeedPid();
    return 0.0;
  }
  const double effort = computeSpeedPid(target_speed);
  // 零速目标不能因噪声微分项或残余积分重新给出正驱动。
  return target_speed <= pid_stop_clear_eps_ ? std::min(0.0, effort) : effort;
}

void ControllerNode::onWaypoints(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  for (const auto & wp : msg->waypoints) {
    if (!std::isfinite(wp.pose.pose.position.x) || !std::isfinite(wp.pose.pose.position.y) ||
      !std::isfinite(wp.twist.twist.linear.x) || wp.twist.twist.linear.x < 0) {
      waypoints_ready_ = false;
      waypoints_.clear();
      return;
    }
  }
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
  const uint8_t prev_mission = mission_mode_;
  state_ = msg->state;
  mission_mode_ = msg->mission_mode;
  bench_state_matches_ = msg->brake_bench_mode == brake_bench_mode_;
  bench_state_received_at_ = std::chrono::steady_clock::now();
  // 双向核对运行模式；任何一端为台架时都禁止落入正常驾驶。
  if (brake_bench_mode_ || msg->brake_bench_mode) {
    enabled_ = false;
    resetSpeedPid();
    resetControlPipeline();
    if (!bench_state_matches_) {
      bench_aborted_ = true;
      publishBenchCommand();
      return;
    }
    if (!bench_started_ && !bench_aborted_ &&
      (state_ == MissionState::IDLE || state_ == MissionState::READY)) {
      bench_armed_ = true;
    }
    if (state_ == MissionState::EXPLORE &&
      mission_mode_ == MissionState::MISSION_ACCELERATION && !bench_started_ &&
      !bench_aborted_ && bench_armed_ &&
      (prev_state == MissionState::IDLE || prev_state == MissionState::READY)) {
      bench_started_ = true;
      bench_started_at_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(), "Brake bench started: delay=%.2fs hold=%.2fs opening=%.2f",
        bench_delay_sec_, bench_hold_sec_, bench_opening_);
    }
    if (bench_started_ && (state_ != MissionState::EXPLORE ||
      mission_mode_ != MissionState::MISSION_ACCELERATION)) {
      bench_aborted_ = true;
    }
    if (state_ != MissionState::EXPLORE) publishBenchCommand();
    return;
  }
  enabled_ = (
    msg->state == MissionState::EXPLORE ||
    msg->state == MissionState::MAPPING_DONE ||
    msg->state == MissionState::RACE);

  // 赛项切换：应用对应赛项的横向/纵向调参
  if (mission_mode_ != prev_mission) {
    applyMissionConfig(mission_mode_);
    resetSpeedPid();
  }

  // 未使能且非车检：复位控制状态并发布零指令
  if (!enabled_ && state_ != MissionState::INSPECTION) {
    resetControlPipeline();
    publishZeroCommand(" [inactive state]");
  }

  // 进入车检：复位状态并记录起始时间
  if (state_ == MissionState::INSPECTION &&
      prev_state != MissionState::INSPECTION) {
    resetControlPipeline();
    resetSpeedPid();
    inspection_start_time_ = now();
    inspection_done_published_ = false;
    const double steer_period = inspection_steer_freq_ > 0.0
      ? 1.0 / inspection_steer_freq_ : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "Inspection started: throttle=%.2f steer=%.1f deg @%.2fs (%.3f Hz) for %.1f s",
      inspection_throttle_, inspection_steer_amp_,
      steer_period, inspection_steer_freq_, inspection_duration_);
  } else if (state_ != MissionState::INSPECTION &&
             prev_state == MissionState::INSPECTION) {
    inspection_done_published_ = false;
  }
}

void ControllerNode::onEmergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 急停不可恢复：只置位
  if (msg->data) emergency_ = true;
}

void ControllerNode::controlLoop()
{
  if (brake_bench_mode_ || bench_aborted_) {
    runBrakeBench();
    return;
  }
  // 急停：持续输出全零
  if (emergency_) {
    resetControlPipeline();
    publishZeroCommand(" [EMERGENCY zero hold]");
    return;
  }

  // 车检：独立流程
  if (state_ == MissionState::INSPECTION) {
    runInspection();
    return;
  }

  const double pose_age = pose_ready_ ? (now() - last_pose_time_).seconds() : 0.0;
  if (!enabled_ || !pose_ready_ || !waypoints_ready_ || pose_age < 0 || pose_age > pose_timeout_sec_) {
    resetControlPipeline();
    publishZeroCommand(" [inactive/missing/stale input]");
    return;
  }
  if (mission_complete_) return;

  const auto loop_time = now();

  // 前视距离：各赛项覆盖（八字用固定值，循迹用曲率自适应）
  double lookahead_override = 0.0;
  if (mission_mode_ == MissionState::MISSION_SKIDPAD) {
    lookahead_override = skidpad_lookahead_;
  } else if (mission_mode_ == MissionState::MISSION_TRACKDRIVE) {
    lookahead_override = trackdriveLookahead(loop_time);
  }
  auto raw_cmd = pure_pursuit_->compute(
    vehicle_state_, waypoints_, lookahead_override);

  // 循迹用前视目标点的速度，使其在入弯前采用弯道限速
  if (mission_mode_ == MissionState::MISSION_TRACKDRIVE && raw_cmd.valid) {
    const int target_index = pure_pursuit_->targetIndex();
    if (target_index >= 0 &&
        target_index < static_cast<int>(waypoints_.size())) {
      raw_cmd.velocity = waypoints_[target_index].twist.twist.linear.x;
    }
    last_valid_trackdrive_cmd_ = raw_cmd;
    last_valid_trackdrive_cmd_time_ = loop_time;
    last_valid_trackdrive_cmd_ready_ = true;
  }

  // 停车类赛项终点判定
  const MissionConfig & cfg = configFor(mission_mode_);
  if (cfg.has_finish &&
      pure_pursuit_->progressIndex() == static_cast<int>(waypoints_.size()) - 1 &&
      std::hypot(
        waypoints_.back().pose.pose.position.x - vehicle_state_.x,
        waypoints_.back().pose.pose.position.y - vehicle_state_.y) <=
        cfg.finish_position_tolerance &&
      vehicle_state_.velocity <= cfg.finish_speed_threshold)
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
      // 直接回放上一条有效指令（其速度已由规划侧限速，控制侧不再二次封顶）
      raw_cmd = last_valid_trackdrive_cmd_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "No forward waypoint target available; holding last Trackdrive command at %.2f m/s.",
        raw_cmd.velocity);
    } else {
      resetControlPipeline();
      publishZeroCommand(" [no forward target]");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "No forward waypoint target available; publishing stop command.");
      return;
    }
  }

  // 安全滤波
  if (!std::isfinite(raw_cmd.steering_angle) || !std::isfinite(raw_cmd.velocity)) {
    resetControlPipeline();
    publishZeroCommand(" [non-finite control]");
    return;
  }
  auto filtered = twist_filter_->filter(raw_cmd.steering_angle, raw_cmd.velocity);

  // 发布命令
  const double throttle_brake = computeThrottleBrake(filtered.velocity);
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = loop_time;
  cmd.header.frame_id = "base_link";
  cmd.speed    = filtered.velocity;
  cmd.angle    = filtered.steering_angle;
  cmd.throttle_brake = throttle_brake;
  cmd_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s",
    controlLine(raw_cmd.steering_angle, raw_cmd.velocity,
      cmd.angle, cmd.speed, throttle_brake, "").c_str());

  // 可视化目标点与前视圆
  if (target_viz_pub_->get_subscription_count() > 0 &&
      pure_pursuit_->targetIndex() < static_cast<int>(waypoints_.size()))
  {
    const auto & wp = waypoints_[pure_pursuit_->targetIndex()];
    publishVisualization(
      wp.pose.pose.position.x,
      wp.pose.pose.position.y);
  }
}

void ControllerNode::publishBenchCommand(double opening)
{
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.speed = 0.0;
  cmd.angle = 0.0;
  cmd.throttle_brake = -opening;
  cmd_pub_->publish(cmd);
}

void ControllerNode::runBrakeBench()
{
  const auto t = std::chrono::steady_clock::now();
  const double state_age = std::chrono::duration<double>(t - bench_state_received_at_).count();
  if (emergency_ || state_ == MissionState::EMERGENCY || !bench_state_matches_ ||
    (bench_started_ && state_age > 0.5)) bench_aborted_ = true;
  if (!bench_started_ || bench_aborted_ || bench_done_ ||
    state_ != MissionState::EXPLORE || mission_mode_ != MissionState::MISSION_ACCELERATION) {
    publishBenchCommand();
    return;
  }
  const double elapsed = std::chrono::duration<double>(t - bench_started_at_).count();
  if (elapsed < bench_delay_sec_) {
    publishBenchCommand();
  } else if (elapsed < bench_delay_sec_ + bench_hold_sec_) {
    publishBenchCommand(bench_opening_);
  } else {
    bench_done_ = true;
    publishBenchCommand();
    std_msgs::msg::Bool complete;
    complete.data = true;
    mission_complete_pub_->publish(complete);
    RCLCPP_INFO(get_logger(), "Brake bench complete: neutral hold, mission_complete published.");
  }
}

double ControllerNode::trackdriveLookahead(const rclcpp::Time & loop_time)
{
  if (!trackdrive_dynamic_lookahead_ || waypoints_.size() < 3) {
    filtered_trackdrive_lookahead_ = trackdrive_lookahead_;
    last_trackdrive_lookahead_time_ = loop_time;
    return filtered_trackdrive_lookahead_;
  }

  // 统计前方局部中心线的曲率：分位数抗噪，峰值项保留弯道入口预判
  std::vector<double> curvatures;
  double inspected_distance = 0.0;
  const double course = velocityCourse(
    vehicle_state_, trackdrive_cfg_.pp.course_speed_threshold);
  for (size_t i = 0; i + 2 < waypoints_.size(); ++i) {
    const auto & p0 = waypoints_[i].pose.pose.position;
    const auto & p1 = waypoints_[i + 1].pose.pose.position;
    const auto & p2 = waypoints_[i + 2].pose.pose.position;
    const double forward =
      (p1.x - vehicle_state_.x) * std::cos(course) +
      (p1.y - vehicle_state_.y) * std::sin(course);
    if (forward < -trackdrive_cfg_.pp.forward_margin) continue;

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
      curvature_percentile_ * static_cast<double>(curvatures.size() - 1));
    effective_curvature = std::max(
      curvatures[percentile_index], curvature_peak_factor_ * curvatures.back());
  }
  const double curvature_ratio = std::clamp(
    (effective_curvature - trackdrive_straight_curvature_) /
      (trackdrive_corner_curvature_ - trackdrive_straight_curvature_),
    0.0, 1.0);
  const double desired = trackdrive_lookahead_ - curvature_ratio *
    (trackdrive_lookahead_ - trackdrive_min_lookahead_);

  // 限制前视变化率，避免地图刷新造成目标点突变
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

  // 演示时长到达：停零并回报完成
  if (t >= inspection_duration_) {
    finishInspection();
    return;
  }

  // 正弦转向，经 TwistFilter 做限幅与限速
  const double steer_deg = inspection_steer_amp_ *
    std::sin(2.0 * M_PI * inspection_steer_freq_ * t);
  const auto filtered = twist_filter_->filter(steer_deg, 0.0);

  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.speed = 0.0;
  cmd.angle = filtered.steering_angle;
  cmd.throttle_brake = inspection_throttle_;  // 恒定开度，不走 PID
  cmd_pub_->publish(cmd);
}

void ControllerNode::finishInspection()
{
  inspection_done_published_ = true;
  resetSpeedPid();
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
  resetControlPipeline();

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

  // 前视圆
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
  if (brake_bench_mode_ || bench_aborted_) {
    publishBenchCommand();
    return;
  }
  autoware_msgs::msg::Command cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.speed = 0.0;
  cmd.angle = 0.0;
  cmd.throttle_brake = computeThrottleBrake(0.0);
  cmd_pub_->publish(cmd);

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
