#include "path_generator/path_generator_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace path_generator
{
namespace
{
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double longitudinalOffset(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  const geometry_msgs::msg::Point & point)
{
  const double dx = point.x - pose.pose.position.x;
  const double dy = point.y - pose.pose.position.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}
}  // namespace

using State = wuta_msgs::msg::MissionState;

PathGeneratorNode::PathGeneratorNode(const rclcpp::NodeOptions & options)
: Node("path_generator_node", options)
{
  // EXPLORE（第1圈，建图/探索圈）速度配置
  trackdrive_explore_max_velocity_ = declare_parameter(
    "trackdrive.explore.max_velocity", trackdrive_explore_max_velocity_);
  trackdrive_explore_min_velocity_ = declare_parameter(
    "trackdrive.explore.min_velocity", trackdrive_explore_min_velocity_);
  trackdrive_explore_lateral_accel_limit_ = declare_parameter(
    "trackdrive.explore.lateral_accel_limit", trackdrive_explore_lateral_accel_limit_);
  // RACE（比赛圈）速度配置
  trackdrive_race_lap2_max_velocity_ = declare_parameter(
    "trackdrive.race.lap2_max_velocity", trackdrive_race_lap2_max_velocity_);
  trackdrive_race_lap3_max_velocity_ = declare_parameter(
    "trackdrive.race.lap3_max_velocity", trackdrive_race_lap3_max_velocity_);
  trackdrive_race_min_velocity_ = declare_parameter(
    "trackdrive.race.min_velocity", trackdrive_race_min_velocity_);
  trackdrive_race_lateral_accel_limit_ = declare_parameter(
    "trackdrive.race.lateral_accel_limit", trackdrive_race_lateral_accel_limit_);
  // 通用处理（所有状态共用）
  trackdrive_resample_spacing_ = declare_parameter(
    "trackdrive.resample_spacing", trackdrive_resample_spacing_);
  trackdrive_min_forward_target_ = declare_parameter(
    "trackdrive.min_forward_target", trackdrive_min_forward_target_);
  trackdrive_full_speed_forward_distance_ = declare_parameter(
    "trackdrive.full_speed_forward_distance", trackdrive_full_speed_forward_distance_);
  // 降级限速（短中心线 / 低置信度共用）
  trackdrive_degraded_velocity_ = declare_parameter(
    "trackdrive.degraded_velocity", trackdrive_degraded_velocity_);
  trackdrive_short_centerline_points_ = declare_parameter(
    "trackdrive.short_centerline_points", trackdrive_short_centerline_points_);
  // 置信度 → 速度映射
  trackdrive_confidence_slow_threshold_ = declare_parameter(
    "trackdrive.confidence.slow_threshold", trackdrive_confidence_slow_threshold_);
  trackdrive_confidence_full_threshold_ = declare_parameter(
    "trackdrive.confidence.full_threshold", trackdrive_confidence_full_threshold_);
  trackdrive_confidence_timeout_sec_ = declare_parameter(
    "trackdrive.confidence.timeout_sec", trackdrive_confidence_timeout_sec_);
  // 全局冻结中心线
  trackdrive_global_horizon_distance_ = declare_parameter(
    "trackdrive.global.horizon_distance", trackdrive_global_horizon_distance_);
  trackdrive_global_search_points_ = declare_parameter(
    "trackdrive.global.search_points", trackdrive_global_search_points_);
  trackdrive_global_min_points_ = declare_parameter(
    "trackdrive.global.min_points", trackdrive_global_min_points_);
  trackdrive_global_publish_period_sec_ = declare_parameter(
    "trackdrive.global.publish_period_sec", trackdrive_global_publish_period_sec_);
  skidpad_radius_         = declare_parameter("skidpad_radius",         skidpad_radius_);
  skidpad_velocity_       = declare_parameter("skidpad_velocity",       skidpad_velocity_);
  skidpad_points_         = declare_parameter("skidpad_points",         skidpad_points_);
  skidpad_start_x_        = declare_parameter("skidpad_start_x",        skidpad_start_x_);
  skidpad_start_y_        = declare_parameter("skidpad_start_y",        skidpad_start_y_);
  skidpad_start_yaw_      = declare_parameter("skidpad_start_yaw",      skidpad_start_yaw_);
  skidpad_entry_x_        = declare_parameter("skidpad_entry_x",        skidpad_entry_x_);
  skidpad_entry_y_        = declare_parameter("skidpad_entry_y",        skidpad_entry_y_);
  skidpad_exit_length_    = declare_parameter("skidpad_exit_length",    skidpad_exit_length_);
  skidpad_braking_distance_ = declare_parameter(
    "skidpad_braking_distance", skidpad_braking_distance_);
  skidpad_csv_path_       = declare_parameter("skidpad_csv_path",       skidpad_csv_path_);
  driven_trajectory_smoothing_alpha_ = declare_parameter(
    "driven_trajectory_smoothing_alpha", driven_trajectory_smoothing_alpha_);
  driven_trajectory_min_distance_ = declare_parameter(
    "driven_trajectory_min_distance", driven_trajectory_min_distance_);
  driven_trajectory_max_step_ = declare_parameter(
    "driven_trajectory_max_step", driven_trajectory_max_step_);
  acceleration_start_x_ = declare_parameter("acceleration_start_x", acceleration_start_x_);
  acceleration_start_y_ = declare_parameter("acceleration_start_y", acceleration_start_y_);
  acceleration_start_yaw_ = declare_parameter(
    "acceleration_start_yaw", acceleration_start_yaw_);
  acceleration_timing_start_x_ = declare_parameter(
    "acceleration_timing_start_x", acceleration_timing_start_x_);
  acceleration_length_    = declare_parameter("acceleration_length",    acceleration_length_);
  acceleration_stopping_distance_ = declare_parameter(
    "acceleration_stopping_distance", acceleration_stopping_distance_);
  acceleration_velocity_  = declare_parameter("acceleration_velocity",  acceleration_velocity_);

  // EBS 测试参数（赛规 7.5），结构复用 acceleration
  ebs_start_x_ = declare_parameter("ebs_start_x", ebs_start_x_);
  ebs_start_y_ = declare_parameter("ebs_start_y", ebs_start_y_);
  ebs_start_yaw_ = declare_parameter("ebs_start_yaw", ebs_start_yaw_);
  ebs_timing_start_x_ = declare_parameter("ebs_timing_start_x", ebs_timing_start_x_);
  ebs_length_ = declare_parameter("ebs_length", ebs_length_);
  ebs_stopping_distance_ = declare_parameter(
    "ebs_stopping_distance", ebs_stopping_distance_);
  ebs_velocity_ = declare_parameter("ebs_velocity", ebs_velocity_);

  // Subscribers
  mission_sub_ = create_subscription<State>(
    "/system/mission_state", 10,
    std::bind(&PathGeneratorNode::onMissionState, this, std::placeholders::_1));

  centerline_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/centerline", 10,
    std::bind(&PathGeneratorNode::onCenterline, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&PathGeneratorNode::onPose, this, std::placeholders::_1));

  const auto status_qos = rclcpp::QoS(1).reliable().transient_local();
  global_centerline_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/planning/global_centerline_ready", status_qos,
    std::bind(
      &PathGeneratorNode::onGlobalCenterlineReady, this, std::placeholders::_1));
  path_confidence_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/planning/path_confidence", status_qos,
    std::bind(&PathGeneratorNode::onPathConfidence, this, std::placeholders::_1));
  localization_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/localization_ready", 10,
    std::bind(&PathGeneratorNode::onLocalizationReady, this, std::placeholders::_1));
  localization_confidence_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/system/localization_confidence", 10,
    std::bind(
      &PathGeneratorNode::onLocalizationConfidence, this, std::placeholders::_1));
  lap_count_sub_ = create_subscription<std_msgs::msg::UInt32>(
    "/system/lap_count", status_qos,
    std::bind(&PathGeneratorNode::onLapCount, this, std::placeholders::_1));

  // Publisher — final_waypoints consumed by controller
  waypoints_pub_ = create_publisher<autoware_msgs::msg::Lane>("/planning/final_waypoints", 10);

  // Visualization — LINE_STRIP through planned waypoints
  viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/final_waypoints_viz", 10);

  // Visualization — driven trajectory growing behind the vehicle
  trajectory_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/driven_trajectory_viz", 10);

  RCLCPP_INFO(get_logger(), "PathGeneratorNode ready.");
}

void PathGeneratorNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & position = msg->pose.position;
  const auto & orientation = msg->pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring non-finite localization pose.");
    return;
  }

  // 急停后车辆仍在制动滑行，继续记录轨迹；FINISH 为任务终态则停止追加
  if (system_state_ == State::FINISH) return;

  current_pose_ = *msg;
  pose_ready_ = true;
  last_pose_received_at_ = now();

  // Smooth and spatially decimate the visualization history.  The raw pose
  // still reaches the controller unchanged; this only makes the RViz line
  // readable when the simulated INS supplies independent measurement noise.
  geometry_msgs::msg::Point pt;
  pt.x = msg->pose.position.x;
  pt.y = msg->pose.position.y;
  pt.z = msg->pose.position.z;

  const bool trajectory_jump = trajectory_filter_ready_ &&
      std::hypot(pt.x - filtered_trajectory_point_.x,
        pt.y - filtered_trajectory_point_.y) >
        std::max(0.1, driven_trajectory_max_step_);
  if (trajectory_jump)
  {
    // 重新同步滤波器，避免高速下永久自锁
    filtered_trajectory_point_ = pt;
    last_trajectory_point_ = pt;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring localization jump in driven-trajectory visualization.");
  }

  if (!trajectory_jump) {
    const double alpha = std::clamp(driven_trajectory_smoothing_alpha_, 0.0, 1.0);
    if (!trajectory_filter_ready_)
    {
      filtered_trajectory_point_ = pt;
      trajectory_filter_ready_ = true;
    }
    else
    {
      filtered_trajectory_point_.x += alpha * (pt.x - filtered_trajectory_point_.x);
      filtered_trajectory_point_.y += alpha * (pt.y - filtered_trajectory_point_.y);
      filtered_trajectory_point_.z += alpha * (pt.z - filtered_trajectory_point_.z);
    }

    const double min_distance = std::max(0.0, driven_trajectory_min_distance_);
    if (trajectory_.empty() || std::hypot(
          filtered_trajectory_point_.x - last_trajectory_point_.x,
          filtered_trajectory_point_.y - last_trajectory_point_.y) >= min_distance)
    {
      trajectory_.push_back(filtered_trajectory_point_);
      last_trajectory_point_ = filtered_trajectory_point_;

      // Publish every few points so RViz can discover the topic before subscribing
      if (trajectory_.size() % 3 == 0)
      {
        publishTrajectory();
      }
    }
  }

  if (global_centerline_ready_ && global_trackdrive_lane_ready_ &&
      trackdriveStateActive())
  {
    const auto current_time = now();
    if (last_global_publish_at_.nanoseconds() == 0 ||
        (current_time - last_global_publish_at_).seconds() >=
          std::max(0.02, trackdrive_global_publish_period_sec_))
    {
      publishGlobalTrackdriveHorizon();
      last_global_publish_at_ = current_time;
    }
  }
}

void PathGeneratorNode::onGlobalCenterlineReady(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  global_centerline_ready_ = msg->data;
  if (!global_centerline_ready_) {
    global_trackdrive_lane_ready_ = false;
    global_progress_ready_ = false;
  }
}

void PathGeneratorNode::onPathConfidence(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  path_confidence_ = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
}

void PathGeneratorNode::onLocalizationReady(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  localization_ready_ = msg->data;
}

void PathGeneratorNode::onLocalizationConfidence(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  localization_confidence_ = std::clamp(
    static_cast<double>(msg->data), 0.0, 1.0);
}

void PathGeneratorNode::onLapCount(
  const std_msgs::msg::UInt32::SharedPtr msg)
{
  lap_count_ = msg->data;
}

void PathGeneratorNode::onMissionState(const State::SharedPtr msg)
{
  const bool mission_changed = msg->mission_mode != mission_mode_;
  mission_mode_  = msg->mission_mode;
  system_state_  = msg->state;

  if (mission_changed) {
    skidpad_path_ready_ = false;
    acceleration_path_ready_ = false;
    ebs_path_ready_ = false;
    last_trackdrive_lane_ready_ = false;
    global_trackdrive_lane_ready_ = false;
    global_progress_ready_ = false;
  }

  // Trigger non-trackdrive paths when system is active
  if (!trackdriveStateActive()) return;

  if (mission_mode_ == State::MISSION_SKIDPAD) {
    if (!skidpad_path_ready_) {
      skidpad_path_ = generateSkidpadPath();
      skidpad_path_ready_ = true;
    }
    auto lane = skidpad_path_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
    publishVisualization(lane, 0.0f, 1.0f, 1.0f);  // cyan for skidpad
  } else if (mission_mode_ == State::MISSION_ACCELERATION) {
    // This route is fixed by acceleration.yaml. Regenerating it from the
    // moving localization pose would shift the finish line forward on every
    // MissionState update, so the controller could never reach its stop.
    if (!acceleration_path_ready_) {
      acceleration_path_ = generateAccelerationPath();
      acceleration_path_ready_ = true;
    }
    auto lane = acceleration_path_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
    publishVisualization(lane, 1.0f, 0.5f, 0.0f);  // orange for acceleration
  } else if (mission_mode_ == State::MISSION_EBS_TEST) {
    // 固定路径（同 acceleration），避免跟随定位位姿重生成导致终点漂移
    if (!ebs_path_ready_) {
      ebs_path_ = generateEbsTestPath();
      ebs_path_ready_ = true;
    }
    auto lane = ebs_path_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
    publishVisualization(lane, 0.8f, 0.8f, 0.0f);  // yellow for EBS
  }
  // TRACKDRIVE: forwarded by onCenterline callback
}

void PathGeneratorNode::onCenterline(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  // Only forward trackdrive centerline
  if (mission_mode_ != State::MISSION_TRACKDRIVE) return;
  if (!trackdriveStateActive()) return;

  const bool short_centerline = msg->waypoints.size() <=
    static_cast<std::size_t>(std::max(2, trackdrive_short_centerline_points_));
  const bool closed_map_lane =
    system_state_ != State::EXPLORE &&
    msg->waypoints.size() >=
      static_cast<std::size_t>(std::max(3, trackdrive_global_min_points_));
  if (global_centerline_ready_ || closed_map_lane) {
    global_trackdrive_lane_ = *msg;
    global_trackdrive_lane_ready_ = msg->waypoints.size() >= 3;
    if (global_trackdrive_lane_ready_ && pose_ready_) {
      publishGlobalTrackdriveHorizon();
    }
    return;
  }

  publishTrackdriveLane(*msg, short_centerline);
}

void PathGeneratorNode::publishTrackdriveLane(
  const autoware_msgs::msg::Lane & source, bool short_source)
{
  auto lane = resampleTrackdriveLane(source);
  applyTrackdriveSpeedProfile(lane);
  const double max_velocity = activeTrackdriveMaxVelocity();
  if (short_source) {
    const double velocity_cap = std::clamp(
      trackdrive_degraded_velocity_, 0.0, std::max(0.0, max_velocity));
    for (auto & waypoint : lane.waypoints) {
      waypoint.twist.twist.linear.x = std::min(waypoint.twist.twist.linear.x, velocity_cap);
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Short Trackdrive centerline (%zu source waypoints); capping speed to %.2f m/s",
      source.waypoints.size(), velocity_cap);
  }

  double path_length = 0.0;
  for (std::size_t i = 1; i < lane.waypoints.size(); ++i) {
    const auto & previous = lane.waypoints[i - 1].pose.pose.position;
    const auto & current = lane.waypoints[i].pose.pose.position;
    path_length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  const double distance_ratio = std::clamp(
    path_length / std::max(1.0, trackdrive_full_speed_forward_distance_), 0.0, 1.0);
  const double distance_cap =
    activeTrackdriveMinVelocity() +
    distance_ratio * (max_velocity - activeTrackdriveMinVelocity());

  const double confidence = currentTrackdriveConfidence();
  const double slow_threshold = std::clamp(
    trackdrive_confidence_slow_threshold_, 0.0, 1.0);
  const double full_threshold = std::max(
    slow_threshold + 1e-3,
    std::clamp(trackdrive_confidence_full_threshold_, 0.0, 1.0));
  const double confidence_scale = std::clamp(
    (confidence - slow_threshold) / (full_threshold - slow_threshold), 0.0, 1.0);
  const double confidence_cap =
    std::clamp(trackdrive_degraded_velocity_, 0.0, max_velocity) +
    confidence_scale * (
      max_velocity - std::clamp(
        trackdrive_degraded_velocity_, 0.0, max_velocity));
  const double safety_cap = std::min(distance_cap, confidence_cap);
  for (auto & waypoint : lane.waypoints) {
    waypoint.twist.twist.linear.x =
      std::min(waypoint.twist.twist.linear.x, safety_cap);
  }

  if (!trackdriveLaneHasForwardTarget(lane)) {
    if (last_trackdrive_lane_ready_ &&
        trackdriveLaneHasForwardTarget(last_trackdrive_lane_))
    {
      auto cached_lane = last_trackdrive_lane_;
      cached_lane.header.stamp = now();
      waypoints_pub_->publish(cached_lane);
      publishVisualization(cached_lane, 0.0f, 0.8f, 0.2f);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejected reverse/no-forward Trackdrive lane (%zu waypoints); holding last valid lane",
        lane.waypoints.size());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejected reverse/no-forward Trackdrive lane (%zu waypoints); no valid cached lane",
        lane.waypoints.size());
    }
    return;
  }
  last_trackdrive_lane_ = lane;
  last_trackdrive_lane_ready_ = true;
  waypoints_pub_->publish(lane);
  publishVisualization(lane, 0.0f, 1.0f, 0.0f);  // green for trackdrive
}

autoware_msgs::msg::Lane PathGeneratorNode::extractGlobalTrackdriveHorizon()
{
  autoware_msgs::msg::Lane horizon;
  horizon.header = global_trackdrive_lane_.header;
  horizon.header.stamp = now();
  horizon.header.frame_id = "map";
  const auto & waypoints = global_trackdrive_lane_.waypoints;
  if (!pose_ready_ || waypoints.size() < 3) {
    return horizon;
  }

  const double yaw = yawFromPose(current_pose_);
  const double heading_x = std::cos(yaw);
  const double heading_y = std::sin(yaw);
  const auto candidate_score = [this, &waypoints, heading_x, heading_y](
      std::size_t index) {
      const auto & point = waypoints[index].pose.pose.position;
      const auto & next = waypoints[(index + 1) % waypoints.size()].pose.pose.position;
      const double distance = std::hypot(
        point.x - current_pose_.pose.position.x,
        point.y - current_pose_.pose.position.y);
      const double segment_length = std::hypot(next.x - point.x, next.y - point.y);
      const double alignment = segment_length < 1e-6
        ? -1.0
        : ((next.x - point.x) * heading_x + (next.y - point.y) * heading_y) /
          segment_length;
      return distance + 3.0 * (1.0 - alignment);
    };

  std::size_t best_index = 0;
  double best_score = std::numeric_limits<double>::max();
  if (!global_progress_ready_) {
    for (std::size_t index = 0; index < waypoints.size(); ++index) {
      const double score = candidate_score(index);
      if (score < best_score) {
        best_score = score;
        best_index = index;
      }
    }
  } else {
    const int backwards = 3;
    const int forwards = std::max(3, trackdrive_global_search_points_);
    const int count = static_cast<int>(waypoints.size());
    for (int offset = -backwards; offset <= forwards; ++offset) {
      const int wrapped =
        (static_cast<int>(global_progress_index_) + offset + count) % count;
      const auto index = static_cast<std::size_t>(wrapped);
      const double score = candidate_score(index);
      if (score < best_score) {
        best_score = score;
        best_index = index;
      }
    }
  }
  global_progress_index_ = best_index;
  global_progress_ready_ = true;

  horizon.waypoints.push_back(waypoints[best_index]);
  double accumulated_distance = 0.0;
  std::size_t index = best_index;
  const double horizon_distance = std::max(5.0, trackdrive_global_horizon_distance_);
  for (std::size_t step = 1; step < waypoints.size(); ++step) {
    const std::size_t next_index = (index + 1) % waypoints.size();
    const auto & previous = waypoints[index].pose.pose.position;
    const auto & next = waypoints[next_index].pose.pose.position;
    accumulated_distance += std::hypot(next.x - previous.x, next.y - previous.y);
    horizon.waypoints.push_back(waypoints[next_index]);
    index = next_index;
    if (accumulated_distance >= horizon_distance && horizon.waypoints.size() >= 3) {
      break;
    }
  }
  return horizon;
}

void PathGeneratorNode::publishGlobalTrackdriveHorizon()
{
  auto horizon = extractGlobalTrackdriveHorizon();
  if (horizon.waypoints.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Frozen global centerline has no usable local horizon.");
    return;
  }
  publishTrackdriveLane(horizon, false);
}

bool PathGeneratorNode::trackdriveStateActive() const
{
  return system_state_ == State::EXPLORE ||
         system_state_ == State::MAPPING_DONE ||
         system_state_ == State::RACE;
}

double PathGeneratorNode::activeTrackdriveMaxVelocity() const
{
  if (system_state_ != State::RACE) {
    return std::max(0.0, trackdrive_explore_max_velocity_);
  }
  return std::max(
    0.0,
    lap_count_ <= 1 ? trackdrive_race_lap2_max_velocity_ : trackdrive_race_lap3_max_velocity_);
}

double PathGeneratorNode::activeTrackdriveMinVelocity() const
{
  const double max_velocity = activeTrackdriveMaxVelocity();
  const double requested = system_state_ == State::RACE
    ? trackdrive_race_min_velocity_
    : trackdrive_explore_min_velocity_;
  return std::clamp(requested, 0.0, max_velocity);
}

double PathGeneratorNode::activeTrackdriveLateralAccelLimit() const
{
  return system_state_ == State::RACE
    ? std::max(0.1, trackdrive_race_lateral_accel_limit_)
    : std::max(0.1, trackdrive_explore_lateral_accel_limit_);
}

double PathGeneratorNode::currentTrackdriveConfidence() const
{
  if (!pose_ready_ || !localization_ready_ ||
      last_pose_received_at_.nanoseconds() == 0)
  {
    return 0.0;
  }
  const double pose_age = (now() - last_pose_received_at_).seconds();
  if (pose_age > std::max(0.05, trackdrive_confidence_timeout_sec_)) {
    return 0.0;
  }
  return std::min(
    std::clamp(path_confidence_, 0.0, 1.0),
    std::clamp(localization_confidence_, 0.0, 1.0));
}

autoware_msgs::msg::Lane PathGeneratorNode::resampleTrackdriveLane(
  const autoware_msgs::msg::Lane & input) const
{
  if (input.waypoints.size() < 2 || trackdrive_resample_spacing_ <= 0.05) {
    return input;
  }

  autoware_msgs::msg::Lane output;
  output.header = input.header;
  const double spacing = std::max(0.2, trackdrive_resample_spacing_);

  const auto append_point = [&output, this](
      const autoware_msgs::msg::Waypoint & source,
      double x, double y, double z, double yaw) {
    autoware_msgs::msg::Waypoint wp = source;
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.y = y;
    wp.pose.pose.position.z = z;
    wp.pose.pose.orientation.x = 0.0;
    wp.pose.pose.orientation.y = 0.0;
    wp.pose.pose.orientation.z = std::sin(yaw * 0.5);
    wp.pose.pose.orientation.w = std::cos(yaw * 0.5);
    wp.twist.twist.linear.x = trackdrive_explore_max_velocity_;
    output.waypoints.push_back(wp);
  };

  for (std::size_t i = 0; i + 1 < input.waypoints.size(); ++i) {
    const auto & from = input.waypoints[i];
    const auto & to = input.waypoints[i + 1];
    const double x0 = from.pose.pose.position.x;
    const double y0 = from.pose.pose.position.y;
    const double z0 = from.pose.pose.position.z;
    const double x1 = to.pose.pose.position.x;
    const double y1 = to.pose.pose.position.y;
    const double z1 = to.pose.pose.position.z;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double length = std::hypot(dx, dy);
    if (length < 1e-3) continue;

    const double yaw = std::atan2(dy, dx);
    const int segments = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int step = 0; step < segments; ++step) {
      if (!output.waypoints.empty() && step == 0) continue;
      const double ratio = static_cast<double>(step) / segments;
      append_point(from, x0 + dx * ratio, y0 + dy * ratio, z0 + dz * ratio, yaw);
    }
  }

  const auto & last = input.waypoints.back();
  const auto & previous = input.waypoints[input.waypoints.size() - 2];
  const double yaw = std::atan2(
    last.pose.pose.position.y - previous.pose.pose.position.y,
    last.pose.pose.position.x - previous.pose.pose.position.x);
  append_point(last, last.pose.pose.position.x, last.pose.pose.position.y,
    last.pose.pose.position.z, yaw);
  return output;
}

void PathGeneratorNode::applyTrackdriveSpeedProfile(autoware_msgs::msg::Lane & lane) const
{
  if (lane.waypoints.empty()) return;

  const double max_velocity = activeTrackdriveMaxVelocity();
  const double min_velocity = activeTrackdriveMinVelocity();
  const double lateral_accel = activeTrackdriveLateralAccelLimit();

  if (lane.waypoints.size() < 3 || max_velocity <= 0.0) {
    for (auto & wp : lane.waypoints) {
      wp.twist.twist.linear.x = max_velocity;
    }
    return;
  }

  for (std::size_t i = 0; i < lane.waypoints.size(); ++i) {
    const std::size_t prev_index = (i == 0) ? 0 : i - 1;
    const std::size_t next_index = (i + 1 >= lane.waypoints.size())
      ? lane.waypoints.size() - 1
      : i + 1;

    const auto & prev = lane.waypoints[prev_index].pose.pose.position;
    const auto & curr = lane.waypoints[i].pose.pose.position;
    const auto & next = lane.waypoints[next_index].pose.pose.position;

    const double ax = curr.x - prev.x;
    const double ay = curr.y - prev.y;
    const double bx = next.x - curr.x;
    const double by = next.y - curr.y;
    const double cx = next.x - prev.x;
    const double cy = next.y - prev.y;

    const double a = std::hypot(ax, ay);
    const double b = std::hypot(bx, by);
    const double c = std::hypot(cx, cy);
    double target_velocity = max_velocity;

    if (a > 1e-3 && b > 1e-3 && c > 1e-3) {
      const double double_area = std::abs(ax * cy - ay * cx);
      const double curvature = 2.0 * double_area / (a * b * c);
      if (curvature > 1e-4) {
        target_velocity = std::sqrt(lateral_accel / curvature);
      }
    }

    lane.waypoints[i].twist.twist.linear.x =
      std::clamp(target_velocity, min_velocity, max_velocity);
  }
}

bool PathGeneratorNode::trackdriveLaneHasForwardTarget(
  const autoware_msgs::msg::Lane & lane) const
{
  if (!pose_ready_ || lane.waypoints.empty()) {
    return true;
  }

  const double yaw = yawFromPose(current_pose_);
  const double min_forward = std::max(0.0, trackdrive_min_forward_target_);
  for (const auto & wp : lane.waypoints) {
    if (longitudinalOffset(current_pose_, yaw, wp.pose.pose.position) > min_forward) {
      return true;
    }
  }
  return false;
}

void PathGeneratorNode::exportSkidpadCsv(const std::vector<SkidpadCsvRow> & rows) const
{
  if (skidpad_csv_path_.empty()) return;

  namespace fs = std::filesystem;
  fs::path output_path(skidpad_csv_path_);
  if (output_path.is_relative()) {
    try {
      // <WUTA-FSD>/ros2_ws/install/path_generator/share/path_generator
      // is the package share path in this workspace installation.
      fs::path fsd_root = ament_index_cpp::get_package_share_directory("path_generator");
      for (int i = 0; i < 5; ++i) fsd_root = fsd_root.parent_path();
      output_path = fsd_root / output_path;
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Cannot resolve WUTA-FSD output root: %s", exception.what());
    }
  }

  std::error_code error;
  fs::create_directories(output_path.parent_path(), error);
  if (error) {
    RCLCPP_ERROR(get_logger(), "Unable to create skidpad CSV directory %s: %s",
      output_path.parent_path().c_str(), error.message().c_str());
    return;
  }

  std::ofstream stream(output_path);
  if (!stream.is_open()) {
    RCLCPP_ERROR(get_logger(), "Unable to write skidpad CSV: %s", output_path.c_str());
    return;
  }

  stream << "index,phase,lap,x_m,y_m,yaw_rad,target_speed_mps\n";
  stream << std::fixed << std::setprecision(6);
  for (std::size_t index = 0; index < rows.size(); ++index) {
    const auto & row = rows[index];
    stream << index << ',' << row.phase << ',' << row.lap << ','
           << row.x << ',' << row.y << ',' << row.yaw << ',' << row.velocity << '\n';
  }
  RCLCPP_INFO(get_logger(), "Skidpad trajectory CSV: %s (%zu rows)",
    output_path.c_str(), rows.size());
}

autoware_msgs::msg::Lane PathGeneratorNode::generateSkidpadPath() const
{
  autoware_msgs::msg::Lane lane;
  std::vector<SkidpadCsvRow> csv_rows;

  // The track is fixed in map, not regenerated from the moving vehicle pose.
  // At yaw=0 the crossing is (0, 0), the right circle is below it and the
  // left circle above it, matching perception_simulation/tracks/skidpad.yaml.
  const double c = std::cos(skidpad_start_yaw_);
  const double s = std::sin(skidpad_start_yaw_);
  const auto to_map = [this, c, s](double local_x, double local_y, double local_yaw,
                                    autoware_msgs::msg::Waypoint & wp) {
    wp.pose.pose.position.x = skidpad_start_x_ + local_x * c - local_y * s;
    wp.pose.pose.position.y = skidpad_start_y_ + local_x * s + local_y * c;
    wp.pose.pose.position.z = 0.0;
    const double yaw = skidpad_start_yaw_ + local_yaw;
    wp.pose.pose.orientation.z = std::sin(yaw * 0.5);
    wp.pose.pose.orientation.w = std::cos(yaw * 0.5);
  };

  const auto append_waypoint = [&lane, &csv_rows, &to_map, this](
    double local_x, double local_y, double local_yaw, double velocity,
    const std::string & phase, int lap) {
      autoware_msgs::msg::Waypoint wp;
      to_map(local_x, local_y, local_yaw, wp);
      wp.twist.twist.linear.x = velocity;
      lane.waypoints.push_back(wp);
      csv_rows.push_back({phase, lap, wp.pose.pose.position.x, wp.pose.pose.position.y,
        skidpad_start_yaw_ + local_yaw, velocity});
    };

  const int circle_points = std::max(8, skidpad_points_);
  const double d_theta = 2.0 * M_PI / circle_points;

  // FSAC: the vehicle starts 15 m before the timing line and enters in the
  // same direction as the eventual exit.  Include the straight explicitly so
  // the controller never shortcuts from the staging point to a circle.
  const double entry_length = std::hypot(skidpad_entry_x_, skidpad_entry_y_);
  const int entry_segments = std::max(1, static_cast<int>(std::ceil(entry_length)));
  for (int i = 0; i <= entry_segments; ++i) {
    const double ratio = static_cast<double>(i) / entry_segments;
    append_waypoint(skidpad_entry_x_ * (1.0 - ratio),
      skidpad_entry_y_ * (1.0 - ratio), skidpad_entry_y_ == 0.0 ? 0.0 :
      std::atan2(-skidpad_entry_y_, -skidpad_entry_x_), skidpad_velocity_, "entry", 0);
  }

  // The first right lap establishes steering, the second is timed.  Start at
  // i=1 because the entry already contributes the crossing waypoint; each
  // subsequent phase similarly reuses only the preceding phase's endpoint.
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 1; i <= circle_points; ++i) {
      const double theta = M_PI_2 - i * d_theta;  // clockwise, starts at crossing
      append_waypoint(skidpad_radius_ * std::cos(theta),
        -skidpad_radius_ + skidpad_radius_ * std::sin(theta),
        std::atan2(-std::cos(theta), std::sin(theta)), skidpad_velocity_,
        "right_circle", lap + 1);
    }
  }

  // Third lap enters the left circle; the fourth is timed.  Counter-clockwise
  // travel preserves the +x crossing direction.
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 1; i <= circle_points; ++i) {
      const double theta = -M_PI_2 + i * d_theta;  // counter-clockwise
      append_waypoint(skidpad_radius_ * std::cos(theta),
        skidpad_radius_ + skidpad_radius_ * std::sin(theta),
        std::atan2(std::cos(theta), -std::sin(theta)), skidpad_velocity_,
        "left_circle", lap + 3);
    }
  }

  // Leave the crossing in the same direction as entry and stop at 25 m.
  // The final braking segment gives the controller a decreasing speed target.
  for (int i = 1; i <= static_cast<int>(std::ceil(skidpad_exit_length_)); ++i) {
    const double distance = std::min(static_cast<double>(i), skidpad_exit_length_);
    const double remaining = skidpad_exit_length_ - distance;
    const double velocity = remaining < skidpad_braking_distance_
      ? skidpad_velocity_ * remaining / skidpad_braking_distance_
      : skidpad_velocity_;
    append_waypoint(distance, 0.0, 0.0, velocity, "exit", 0);
  }

  exportSkidpadCsv(csv_rows);

  RCLCPP_INFO(get_logger(),
    "Fixed skidpad path generated: %.1f m entry, right lap 1/2, left lap 3/4, %.1f m exit (%zu waypoints)",
    entry_length, skidpad_exit_length_, lane.waypoints.size());
  return lane;
}

autoware_msgs::msg::Lane PathGeneratorNode::generateStraightRun(
  double start_x, double start_y, double start_yaw,
  double timing_start_x, double length, double stopping_distance,
  double velocity) const
{
  autoware_msgs::msg::Lane lane;

  const double finish_x = timing_start_x + length;
  const double stop_end_x = finish_x + std::max(1e-6, stopping_distance);
  const double braking_deceleration =
    velocity * velocity / (2.0 * std::max(1e-6, stopping_distance));
  const auto append_waypoint = [&lane, start_y, start_yaw](double x, double v) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.y = start_y;
    wp.pose.pose.position.z = 0.0;
    wp.pose.pose.orientation.z = std::sin(start_yaw * 0.5);
    wp.pose.pose.orientation.w = std::cos(start_yaw * 0.5);
    wp.twist.twist.linear.x = v;
    lane.waypoints.push_back(wp);
  };

  // 恒定速度通过计时线，之后按 v²=2aΔx 递减到停车终点（有限时间停车）
  append_waypoint(start_x, velocity);
  append_waypoint(timing_start_x, velocity);
  for (int x = static_cast<int>(std::ceil(timing_start_x)) + 1;
       x <= static_cast<int>(std::ceil(stop_end_x)); ++x) {
    const double waypoint_x = std::min(static_cast<double>(x), stop_end_x);
    const double wp_velocity = waypoint_x <= finish_x
      ? velocity
      : std::sqrt(2.0 * braking_deceleration * std::max(0.0, stop_end_x - waypoint_x));
    append_waypoint(waypoint_x, wp_velocity);
  }

  RCLCPP_INFO(get_logger(),
    "Fixed straight run path generated: start=%.2f m, timing finish=%.2f m, stop=%.2f m (%zu waypoints)",
    start_x, finish_x, stop_end_x, lane.waypoints.size());
  return lane;
}

autoware_msgs::msg::Lane PathGeneratorNode::generateAccelerationPath() const
{
  return generateStraightRun(
    acceleration_start_x_, acceleration_start_y_, acceleration_start_yaw_,
    acceleration_timing_start_x_, acceleration_length_,
    acceleration_stopping_distance_, acceleration_velocity_);
}

autoware_msgs::msg::Lane PathGeneratorNode::generateEbsTestPath() const
{
  // EBS 测试（赛规 7.5）：起点后 0.3m → 25m 测速点 ≥40km/h(11.11) → RES 急停 → ≤10m 停车
  return generateStraightRun(
    ebs_start_x_, ebs_start_y_, ebs_start_yaw_,
    ebs_timing_start_x_, ebs_length_,
    ebs_stopping_distance_, ebs_velocity_);
}

void PathGeneratorNode::publishVisualization(
  const autoware_msgs::msg::Lane & lane,
  float r, float g, float b)
{
  visualization_msgs::msg::MarkerArray arr;

  // LINE_STRIP through all waypoints — ADD with same ns/id replaces in place
  visualization_msgs::msg::Marker line;
  line.header = lane.header;
  line.ns     = "planned_path";
  line.id     = 0;
  line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.08;  // line width
  line.color.r = r;
  line.color.g = g;
  line.color.b = b;
  line.color.a = 0.9f;

  for (const auto & wp : lane.waypoints) {
    geometry_msgs::msg::Point p;
    p.x = wp.pose.pose.position.x;
    p.y = wp.pose.pose.position.y;
    p.z = wp.pose.pose.position.z;
    line.points.push_back(p);
  }
  arr.markers.push_back(line);
  viz_pub_->publish(arr);
}

void PathGeneratorNode::publishTrajectory()
{
  if (trajectory_.size() < 2) return;

  visualization_msgs::msg::MarkerArray arr;

  // LINE_STRIP of driven positions — ADD with same ns/id replaces previous marker
  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp    = now();
  line.ns     = "driven_trajectory";
  line.id     = 0;
  line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.06;  // slightly thinner than planned path
  line.color.r = 1.0f;
  line.color.g = 0.85f;
  line.color.b = 0.0f;
  line.color.a = 0.9f;
  line.points = trajectory_;

  arr.markers.push_back(line);
  trajectory_viz_pub_->publish(arr);
}

}  // namespace path_generator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_generator::PathGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
