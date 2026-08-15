#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "wuta_msgs/msg/cone_map.hpp"
#include "wuta_msgs/msg/devices_inspection.hpp"
#include "wuta_msgs/msg/mission_state.hpp"

namespace mission_manager
{

class MissionManager : public rclcpp::Node
{
public:
  explicit MissionManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using State = wuta_msgs::msg::MissionState;

  // State machine
  uint8_t current_state_{State::IDLE};
  uint8_t mission_mode_{State::MISSION_TRACKDRIVE};
  uint8_t localization_mode_{State::LOC_KISS_ICP};

  void transitionTo(uint8_t new_state);
  void publishState();

  // Normal mission callbacks
  void onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg);
  void onEmergency(const std_msgs::msg::Bool::SharedPtr msg);
  void onMissionModeCmd(const std_msgs::msg::String::SharedPtr msg);
  void onStartCommand(const std_msgs::msg::Bool::SharedPtr msg);
  void onMissionComplete(const std_msgs::msg::Bool::SharedPtr msg);
  void onMapReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onGlobalCenterlineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onLocalizationConfidence(const std_msgs::msg::Float32::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void advanceWhenReady();
  void advanceRaceWhenReady();
  bool coneMapQualityPasses(const wuta_msgs::msg::ConeMap & map) const;
  void updateLapCounter(const geometry_msgs::msg::PoseStamped & pose);
  void publishLapCount();

  // ---------------------------------------------------------------------------
  // 开机传感器自检（心跳监控）
  // 各设备话题按参数开关启用，超时/未上线 → EMERGENCY + 发布 devices_inspection
  // ---------------------------------------------------------------------------
  void onLidarData(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onImuData(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onCameraData(const sensor_msgs::msg::Image::SharedPtr msg);  // 预留
  void selfCheckTick();
  void publishDevicesInspection(bool ok, const std::vector<std::string> & failures);

  // System readiness flags
  bool lidar_ready_{false};
  bool localization_ready_{false};
  bool map_closed_{false};
  bool map_quality_ok_{false};
  bool global_centerline_ready_{false};
  bool ndt_map_ready_{false};
  bool start_requested_{false};

  // Trackdrive lap counter. The first active localization pose defines the
  // finite start/finish line; no simulator truth is consumed here.
  geometry_msgs::msg::PoseStamped lap_reference_pose_;
  geometry_msgs::msg::PoseStamped previous_lap_pose_;
  bool lap_reference_ready_{false};
  bool previous_lap_pose_ready_{false};
  bool lap_line_armed_{false};
  uint32_t lap_count_{0};
  double lap_traveled_distance_{0.0};
  rclcpp::Time lap_started_at_;
  rclcpp::Time last_pose_received_at_;

  int min_blue_cones_{12};
  int min_yellow_cones_{12};
  double min_map_average_confidence_{0.40};
  double min_map_color_balance_{0.35};
  double min_localization_confidence_{0.45};
  double localization_timeout_sec_{0.50};
  double localization_confidence_{0.0};
  int trackdrive_finish_laps_{3};
  double lap_min_duration_sec_{10.0};
  double lap_min_distance_{30.0};
  double lap_arm_distance_{10.0};
  double lap_line_half_width_{4.0};
  double lap_heading_tolerance_deg_{75.0};
  bool use_ndt_race_localization_{false};

  // 开机传感器自检（心跳监控）参数与状态
  bool check_lidar_{false};   // 预留，型号/话题未定
  bool check_imu_{true};      // CGI-410
  bool check_camera_{false};  // 预留
  double sensor_timeout_sec_{2.0};
  double selfcheck_interval_sec_{1.0};
  double selfcheck_grace_sec_{4.0};   // 开机上线宽限期
  rclcpp::Time lidar_last_seen_;
  rclcpp::Time imu_last_seen_;
  rclcpp::Time camera_last_seen_;
  bool sensor_fault_{false};          // 故障锁，置位后保持 EMERGENCY
  rclcpp::Time startup_time_;

  // Publishers
  rclcpp::Publisher<wuta_msgs::msg::MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr lap_count_pub_;
  rclcpp::Publisher<wuta_msgs::msg::DevicesInspection>::SharedPtr devices_inspection_pub_;

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::ConeMap>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_centerline_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr localization_confidence_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  // 传感器数据订阅（心跳监控）
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_data_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_data_sub_;

  // Timer for periodic state broadcast
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr selfcheck_timer_;
};

}  // namespace mission_manager
