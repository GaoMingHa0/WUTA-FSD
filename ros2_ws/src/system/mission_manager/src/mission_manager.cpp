#include "mission_manager/mission_manager.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace mission_manager
{

using State = wuta_msgs::msg::MissionState;

MissionManager::MissionManager(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  // Default mission mode from parameter
  const std::string mode_str = declare_parameter<std::string>("mission_mode", "trackdrive");
  if (mode_str == "skidpad")       mission_mode_ = State::MISSION_SKIDPAD;
  else if (mode_str == "acceleration") mission_mode_ = State::MISSION_ACCELERATION;
  else                             mission_mode_ = State::MISSION_TRACKDRIVE;

  min_blue_cones_ = declare_parameter("min_blue_cones", min_blue_cones_);
  min_yellow_cones_ = declare_parameter("min_yellow_cones", min_yellow_cones_);
  min_map_average_confidence_ = declare_parameter(
    "min_map_average_confidence", min_map_average_confidence_);
  min_map_color_balance_ = declare_parameter(
    "min_map_color_balance", min_map_color_balance_);
  min_localization_confidence_ = declare_parameter(
    "min_localization_confidence", min_localization_confidence_);
  localization_timeout_sec_ = declare_parameter(
    "localization_timeout_sec", localization_timeout_sec_);
  trackdrive_finish_laps_ = declare_parameter(
    "trackdrive_finish_laps", trackdrive_finish_laps_);
  lap_min_duration_sec_ = declare_parameter(
    "lap_min_duration_sec", lap_min_duration_sec_);
  lap_min_distance_ = declare_parameter("lap_min_distance", lap_min_distance_);
  lap_arm_distance_ = declare_parameter("lap_arm_distance", lap_arm_distance_);
  lap_line_half_width_ = declare_parameter(
    "lap_line_half_width", lap_line_half_width_);
  lap_heading_tolerance_deg_ = declare_parameter(
    "lap_heading_tolerance_deg", lap_heading_tolerance_deg_);
  use_ndt_race_localization_ = declare_parameter(
    "use_ndt_race_localization", use_ndt_race_localization_);

  // 开机传感器自检（心跳监控）
  check_lidar_ = declare_parameter("check_lidar", check_lidar_);
  check_imu_ = declare_parameter("check_imu", check_imu_);
  check_camera_ = declare_parameter("check_camera", check_camera_);
  sensor_timeout_sec_ = declare_parameter(
    "sensor_timeout_sec", sensor_timeout_sec_);
  selfcheck_interval_sec_ = declare_parameter(
    "selfcheck_interval_sec", selfcheck_interval_sec_);
  selfcheck_grace_sec_ = declare_parameter(
    "selfcheck_grace_sec", selfcheck_grace_sec_);
  startup_time_ = now();

  // Publishers
  state_pub_ = create_publisher<State>("/system/mission_state", 10);
  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  lap_count_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "/system/lap_count", latched_qos);
  devices_inspection_pub_ = create_publisher<wuta_msgs::msg::DevicesInspection>(
    "/system/devices_inspection", 10);

  // Subscribers — normal mission
  cone_map_sub_ = create_subscription<wuta_msgs::msg::ConeMap>(
    "/mapping/cone_map", 10,
    std::bind(&MissionManager::onConeMap, this, std::placeholders::_1));

  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency", 10,
    std::bind(&MissionManager::onEmergency, this, std::placeholders::_1));

  mission_mode_sub_ = create_subscription<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10,
    std::bind(&MissionManager::onMissionModeCmd, this, std::placeholders::_1));

  start_command_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/start_command", 10,
    std::bind(&MissionManager::onStartCommand, this, std::placeholders::_1));

  mission_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/mission_complete", 10,
    std::bind(&MissionManager::onMissionComplete, this, std::placeholders::_1));

  map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/ndt/map_ready", 10,
    std::bind(&MissionManager::onMapReady, this, std::placeholders::_1));

  global_centerline_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/planning/global_centerline_ready", latched_qos,
    std::bind(
      &MissionManager::onGlobalCenterlineReady, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&MissionManager::onPose, this, std::placeholders::_1));

  localization_confidence_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/system/localization_confidence", 10,
    std::bind(
      &MissionManager::onLocalizationConfidence, this, std::placeholders::_1));

  lidar_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/lidar_ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      lidar_ready_ = msg->data;
      advanceWhenReady();
    });

  localization_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/localization_ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      localization_ready_ = msg->data;
      advanceWhenReady();
      advanceRaceWhenReady();
    });

  // ---------------------------------------------------------------------------
  // 开机传感器自检：订阅设备数据流，心跳超时即判故障
  // ---------------------------------------------------------------------------
  lidar_data_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    declare_parameter("lidar_topic", "/rslidar_points"), 10,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      (void)msg;
      lidar_last_seen_ = now();
    });
  imu_data_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter("imu_topic", "/chcnav/odometry"), 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      (void)msg;
      imu_last_seen_ = now();
    });
  camera_data_sub_ = create_subscription<sensor_msgs::msg::Image>(
    declare_parameter("camera_topic", "/zed2i/zed_node/left/image_rect_color"), 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      (void)msg;
      camera_last_seen_ = now();
    });

  // Periodic state broadcast at 10 Hz
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionManager::publishState, this));

  // 开机自检定时器
  selfcheck_timer_ = create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int64_t>(std::max(0.1, selfcheck_interval_sec_) * 1000.0)),
    std::bind(&MissionManager::selfCheckTick, this));

  RCLCPP_INFO(get_logger(), "Mission Manager initialized. mode=%s state=IDLE",
    mode_str.c_str());
  publishLapCount();
}

void MissionManager::advanceWhenReady()
{
  if (lidar_ready_ && localization_ready_ && current_state_ == State::IDLE) {
    transitionTo(State::READY);
  }
  if (start_requested_ && current_state_ == State::READY) {
    transitionTo(State::EXPLORE);
  }
}

void MissionManager::advanceRaceWhenReady()
{
  if (current_state_ != State::MAPPING_DONE) return;

  const bool pose_fresh =
    last_pose_received_at_.nanoseconds() != 0 &&
    (now() - last_pose_received_at_).seconds() <=
      std::max(0.05, localization_timeout_sec_);
  const bool localization_gate =
    localization_ready_ && pose_fresh &&
    localization_confidence_ >=
      std::clamp(min_localization_confidence_, 0.0, 1.0) &&
    (!use_ndt_race_localization_ || ndt_map_ready_);
  const bool first_lap_complete = lap_count_ >= 1;
  if (map_closed_ && map_quality_ok_ && global_centerline_ready_ &&
      localization_gate && first_lap_complete)
  {
    RCLCPP_INFO(
      get_logger(),
      "Race gates passed: map_closed=true map_quality=true "
      "localization=true global_centerline=true first_lap=true.");
    transitionTo(State::RACE);
  }
}

void MissionManager::transitionTo(uint8_t new_state)
{
  const auto state_name = [](uint8_t s) -> std::string {
    switch (s) {
      case State::IDLE:         return "IDLE";
      case State::READY:        return "READY";
      case State::INSPECTION:   return "INSPECTION";
      case State::EXPLORE:      return "EXPLORE";
      case State::MAPPING_DONE: return "MAPPING_DONE";
      case State::RACE:         return "RACE";
      case State::FINISH:       return "FINISH";
      case State::EMERGENCY:    return "EMERGENCY";
      default:                  return "UNKNOWN";
    }
  };

  RCLCPP_INFO(get_logger(), "State: %s → %s",
    state_name(current_state_).c_str(), state_name(new_state).c_str());

  current_state_ = new_state;

  if (new_state == State::RACE && use_ndt_race_localization_) {
    localization_mode_ = State::LOC_NDT;
    RCLCPP_INFO(get_logger(), "Localization: NDT map matching");
  } else if (new_state == State::RACE || new_state == State::EXPLORE) {
    localization_mode_ = State::LOC_KISS_ICP;
    RCLCPP_INFO(
      get_logger(), "Localization: KISS-ICP + EKF%s",
      new_state == State::RACE ? " retained for RACE" : "");
  }

  if (new_state == State::EXPLORE && mission_mode_ == State::MISSION_TRACKDRIVE) {
    lap_reference_ready_ = false;
    previous_lap_pose_ready_ = false;
    lap_line_armed_ = false;
    lap_count_ = 0;
    lap_traveled_distance_ = 0.0;
    publishLapCount();
  }

  publishState();
}

void MissionManager::publishState()
{
  State msg;
  msg.header.stamp    = now();
  msg.state           = current_state_;
  msg.mission_mode    = mission_mode_;
  msg.localization_mode = localization_mode_;
  msg.description = "lap=" + std::to_string(lap_count_);
  state_pub_->publish(msg);
}

void MissionManager::onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg)
{
  if (mission_mode_ != State::MISSION_TRACKDRIVE || !msg->is_closed) return;

  map_closed_ = true;
  map_quality_ok_ = coneMapQualityPasses(*msg);
  if (current_state_ == State::EXPLORE) {
    RCLCPP_INFO(
      get_logger(),
      "Cone map closed. blue=%zu yellow=%zu quality=%s",
      msg->blue_cones.size(), msg->yellow_cones.size(),
      map_quality_ok_ ? "PASS" : "FAIL");
    transitionTo(State::MAPPING_DONE);
  }
  advanceRaceWhenReady();
}

void MissionManager::onMapReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  ndt_map_ready_ = msg->data;
  advanceRaceWhenReady();
}

void MissionManager::onGlobalCenterlineReady(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  global_centerline_ready_ = msg->data;
  advanceRaceWhenReady();
}

void MissionManager::onLocalizationConfidence(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  localization_confidence_ = std::clamp(
    static_cast<double>(msg->data), 0.0, 1.0);
  advanceRaceWhenReady();
}

bool MissionManager::coneMapQualityPasses(
  const wuta_msgs::msg::ConeMap & map) const
{
  if (!map.is_closed ||
      map.blue_cones.size() < static_cast<std::size_t>(std::max(0, min_blue_cones_)) ||
      map.yellow_cones.size() < static_cast<std::size_t>(std::max(0, min_yellow_cones_)))
  {
    return false;
  }

  const double larger_side = static_cast<double>(
    std::max(map.blue_cones.size(), map.yellow_cones.size()));
  const double smaller_side = static_cast<double>(
    std::min(map.blue_cones.size(), map.yellow_cones.size()));
  const double color_balance = larger_side <= 0.0 ? 0.0 : smaller_side / larger_side;

  double confidence_sum = 0.0;
  std::size_t confidence_count = 0;
  const auto accumulate = [&confidence_sum, &confidence_count](const auto & cones) {
      for (const auto & cone : cones) {
        confidence_sum += std::clamp(static_cast<double>(cone.confidence), 0.0, 1.0);
        ++confidence_count;
      }
    };
  accumulate(map.blue_cones);
  accumulate(map.yellow_cones);
  const double average_confidence = confidence_count == 0
    ? 0.0
    : confidence_sum / static_cast<double>(confidence_count);

  const bool passed =
    color_balance >= std::clamp(min_map_color_balance_, 0.0, 1.0) &&
    average_confidence >= std::clamp(min_map_average_confidence_, 0.0, 1.0);
  if (!passed) {
    RCLCPP_ERROR(
      get_logger(),
      "Cone map quality failed: balance=%.3f (min=%.3f), confidence=%.3f (min=%.3f)",
      color_balance, min_map_color_balance_,
      average_confidence, min_map_average_confidence_);
  }
  return passed;
}

void MissionManager::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pose_received_at_ = now();
  updateLapCounter(*msg);
  advanceRaceWhenReady();
}

void MissionManager::updateLapCounter(
  const geometry_msgs::msg::PoseStamped & pose)
{
  const bool active_trackdrive =
    mission_mode_ == State::MISSION_TRACKDRIVE &&
    (current_state_ == State::EXPLORE ||
     current_state_ == State::MAPPING_DONE ||
     current_state_ == State::RACE);
  if (!active_trackdrive) {
    previous_lap_pose_ready_ = false;
    return;
  }

  if (!lap_reference_ready_) {
    lap_reference_pose_ = pose;
    previous_lap_pose_ = pose;
    lap_reference_ready_ = true;
    previous_lap_pose_ready_ = true;
    lap_line_armed_ = false;
    lap_traveled_distance_ = 0.0;
    lap_started_at_ = now();
    RCLCPP_INFO(
      get_logger(), "Trackdrive lap line initialized at (%.3f, %.3f).",
      pose.pose.position.x, pose.pose.position.y);
    return;
  }

  if (!previous_lap_pose_ready_) {
    previous_lap_pose_ = pose;
    previous_lap_pose_ready_ = true;
    return;
  }

  const double step = std::hypot(
    pose.pose.position.x - previous_lap_pose_.pose.position.x,
    pose.pose.position.y - previous_lap_pose_.pose.position.y);
  if (step <= 5.0) {
    lap_traveled_distance_ += step;
  }

  const auto & q = lap_reference_pose_.pose.orientation;
  const double reference_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double heading_x = std::cos(reference_yaw);
  const double heading_y = std::sin(reference_yaw);
  const auto signed_offsets = [this, heading_x, heading_y](
      const geometry_msgs::msg::PoseStamped & sample) {
      const double dx =
        sample.pose.position.x - lap_reference_pose_.pose.position.x;
      const double dy =
        sample.pose.position.y - lap_reference_pose_.pose.position.y;
      return std::pair<double, double>{
        dx * heading_x + dy * heading_y,
        -dx * heading_y + dy * heading_x};
    };

  const auto current_offsets = signed_offsets(pose);
  if (!lap_line_armed_) {
    const double distance_from_reference = std::hypot(
      pose.pose.position.x - lap_reference_pose_.pose.position.x,
      pose.pose.position.y - lap_reference_pose_.pose.position.y);
    if (distance_from_reference >= std::max(0.0, lap_arm_distance_)) {
      lap_line_armed_ = true;
      RCLCPP_INFO(get_logger(), "Trackdrive lap line armed.");
    }
  }

  if (lap_line_armed_) {
    const auto previous_offsets = signed_offsets(previous_lap_pose_);
    const bool crosses_line =
      previous_offsets.first < 0.0 && current_offsets.first >= 0.0;
    if (crosses_line) {
      const double denominator = current_offsets.first - previous_offsets.first;
      const double ratio = std::abs(denominator) < 1e-9
        ? 0.0
        : -previous_offsets.first / denominator;
      const double crossing_lateral = previous_offsets.second + ratio *
        (current_offsets.second - previous_offsets.second);
      const double elapsed = (now() - lap_started_at_).seconds();
      const auto & current_q = pose.pose.orientation;
      const double current_yaw = std::atan2(
        2.0 * (current_q.w * current_q.z + current_q.x * current_q.y),
        1.0 - 2.0 * (current_q.y * current_q.y + current_q.z * current_q.z));
      const double heading_error = std::abs(std::atan2(
        std::sin(current_yaw - reference_yaw),
        std::cos(current_yaw - reference_yaw)));
      const double heading_tolerance =
        std::clamp(lap_heading_tolerance_deg_, 1.0, 180.0) * M_PI / 180.0;
      if (std::abs(crossing_lateral) <= std::max(0.1, lap_line_half_width_) &&
          lap_traveled_distance_ >= std::max(0.0, lap_min_distance_) &&
          elapsed >= std::max(0.0, lap_min_duration_sec_) &&
          heading_error <= heading_tolerance)
      {
        ++lap_count_;
        publishLapCount();
        RCLCPP_INFO(
          get_logger(),
          "Trackdrive lap %u/%d complete: %.2f s, %.1f m.",
          lap_count_, std::max(1, trackdrive_finish_laps_),
          elapsed, lap_traveled_distance_);

        lap_line_armed_ = false;
        lap_traveled_distance_ = 0.0;
        lap_started_at_ = now();
        advanceRaceWhenReady();
        if (lap_count_ >= static_cast<uint32_t>(
            std::max(1, trackdrive_finish_laps_)))
        {
          transitionTo(State::FINISH);
        }
      }
    }
  }

  previous_lap_pose_ = pose;
}

void MissionManager::publishLapCount()
{
  std_msgs::msg::UInt32 msg;
  msg.data = lap_count_;
  lap_count_pub_->publish(msg);
}

void MissionManager::onEmergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 仅负责 mission_state 状态切换；刹车动作（控制输出归零）由 controller_node 负责
  if (msg->data) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY triggered!");
    transitionTo(State::EMERGENCY);
  }
}

void MissionManager::onMissionModeCmd(const std_msgs::msg::String::SharedPtr msg)
{
  if (current_state_ != State::IDLE && current_state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Cannot change mission mode in state %d", current_state_);
    return;
  }
  if (msg->data == "trackdrive")    mission_mode_ = State::MISSION_TRACKDRIVE;
  else if (msg->data == "skidpad")  mission_mode_ = State::MISSION_SKIDPAD;
  else if (msg->data == "acceleration") mission_mode_ = State::MISSION_ACCELERATION;
  else if (msg->data == "inspection")   mission_mode_ = State::MISSION_INSPECTION;
  else if (msg->data == "ebs_test")     mission_mode_ = State::MISSION_EBS_TEST;
  else {
    RCLCPP_WARN(get_logger(), "Unknown mission mode: %s", msg->data.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Mission mode set to: %s", msg->data.c_str());
  publishState();

  // 车检：选择 inspection 任务即进入车检演示（台架测试）
  if (mission_mode_ == State::MISSION_INSPECTION &&
      (current_state_ == State::IDLE || current_state_ == State::READY)) {
    RCLCPP_INFO(get_logger(), "Inspection triggered by mission mode.");
    transitionTo(State::INSPECTION);
  }
}

void MissionManager::onStartCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  start_requested_ = true;
  advanceWhenReady();
}

void MissionManager::onMissionComplete(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  // 车检动作演示完成 → FINISH（与正常项目一致，由 controller 回报）
  if (current_state_ == State::INSPECTION) {
    RCLCPP_INFO(get_logger(), "Inspection complete, FINISH.");
    transitionTo(State::FINISH);
    return;
  }
  if (mission_mode_ == State::MISSION_TRACKDRIVE) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring external Trackdrive completion; /system/lap_count owns the finish.");
    return;
  }
  if (current_state_ == State::EXPLORE || current_state_ == State::MAPPING_DONE ||
      current_state_ == State::RACE) {
    transitionTo(State::FINISH);
  }
}

void MissionManager::selfCheckTick()
{
  if (sensor_fault_) return;  // 已故障，保持 EMERGENCY
  const double up = (now() - startup_time_).seconds();

  // 宽限期内允许陆续上线；过期后：从未上线 或 中途断开 均判故障
  const auto offline = [&](const rclcpp::Time & last, bool enabled) {
    if (!enabled) return false;
    if (last.nanoseconds() == 0) return up > selfcheck_grace_sec_;
    return (now() - last).seconds() > sensor_timeout_sec_;
  };
  const bool lidar_off = offline(lidar_last_seen_, check_lidar_);
  const bool imu_off   = offline(imu_last_seen_,   check_imu_);
  const bool cam_off   = offline(camera_last_seen_, check_camera_);

  if (lidar_off || imu_off || cam_off) {
    sensor_fault_ = true;
    RCLCPP_ERROR(get_logger(), "Sensor self-check FAILED. EMERGENCY.");
    transitionTo(State::EMERGENCY);
    std::vector<std::string> failures;
    if (lidar_off) failures.push_back("lidar");
    if (imu_off)   failures.push_back("imu");
    if (cam_off)   failures.push_back("camera");
    publishDevicesInspection(false, failures);  // 通知 can_interface
  }
  // 通过：不干预现有流程，IDLE→READY 仍由 advanceWhenReady() 决定
}

void MissionManager::publishDevicesInspection(
  bool ok, const std::vector<std::string> & failures)
{
  wuta_msgs::msg::DevicesInspection msg;
  msg.ok = ok;
  msg.failures = failures;
  devices_inspection_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Devices inspection published: ok=%d", ok);
}

}  // namespace mission_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_manager::MissionManager>());
  rclcpp::shutdown();
  return 0;
}
