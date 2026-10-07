#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_msgs/msg/lane.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "wuta_msgs/msg/mission_state.hpp"

namespace path_generator
{

/**
 * Path Generator Node
 *
 * Subscribes to MissionState and routes to the correct path generation mode:
 *
 *  TRACKDRIVE  → forwards centerline from boundary_detector (Delaunay)
 *  SKIDPAD     → publishes a fixed four-lap figure-8 and exit path
 *  ACCELERATION → generates straight-line path to finish
 *
 * All modes output to /planning/final_waypoints (autoware_msgs::Lane),
 * which is directly consumed by the controller.
 */
class PathGeneratorNode : public rclcpp::Node
{
public:
  explicit PathGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onCenterline(const autoware_msgs::msg::Lane::SharedPtr msg);    // from boundary_detector
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onGlobalCenterlineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onPathConfidence(const std_msgs::msg::Float32::SharedPtr msg);
  void onLocalizationReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onLocalizationConfidence(const std_msgs::msg::Float32::SharedPtr msg);
  void onLapCount(const std_msgs::msg::UInt32::SharedPtr msg);

  // Mode-specific path generators
  autoware_msgs::msg::Lane generateSkidpadPath() const;
  autoware_msgs::msg::Lane generateStraightRun(
    double start_x, double start_y, double start_yaw,
    double timing_start_x, double length, double stopping_distance,
    double velocity) const;
  autoware_msgs::msg::Lane generateAccelerationPath() const;
  autoware_msgs::msg::Lane generateEbsTestPath() const;
  autoware_msgs::msg::Lane resampleTrackdriveLane(const autoware_msgs::msg::Lane & lane) const;
  autoware_msgs::msg::Lane extractGlobalTrackdriveHorizon();
  void applyTrackdriveSpeedProfile(autoware_msgs::msg::Lane & lane) const;
  void publishTrackdriveLane(
    const autoware_msgs::msg::Lane & source, bool short_source);
  void publishGlobalTrackdriveHorizon();
  bool trackdriveLaneHasForwardTarget(const autoware_msgs::msg::Lane & lane) const;
  bool trackdriveStateActive() const;
  double activeTrackdriveMaxVelocity() const;
  double activeTrackdriveMinVelocity() const;
  double activeTrackdriveLateralAccelLimit() const;
  double currentTrackdriveConfidence() const;

  struct SkidpadCsvRow
  {
    std::string phase;
    int lap{0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double velocity{0.0};
  };
  void exportSkidpadCsv(const std::vector<SkidpadCsvRow> & rows) const;

  // Visualization helpers
  void publishVisualization(const autoware_msgs::msg::Lane & lane,
                            float r, float g, float b);
  void publishTrajectory();

  // State
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  uint8_t system_state_{wuta_msgs::msg::MissionState::IDLE};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_ready_{false};
  autoware_msgs::msg::Lane skidpad_path_;
  bool skidpad_path_ready_{false};
  autoware_msgs::msg::Lane acceleration_path_;
  bool acceleration_path_ready_{false};
  autoware_msgs::msg::Lane ebs_path_;
  bool ebs_path_ready_{false};
  autoware_msgs::msg::Lane last_trackdrive_lane_;
  bool last_trackdrive_lane_ready_{false};
  autoware_msgs::msg::Lane global_trackdrive_lane_;
  bool global_centerline_ready_{false};
  bool global_trackdrive_lane_ready_{false};
  bool global_progress_ready_{false};
  std::size_t global_progress_index_{0};
  uint32_t lap_count_{0};
  double path_confidence_{0.0};
  bool localization_ready_{false};
  double localization_confidence_{0.0};
  rclcpp::Time last_pose_received_at_;
  rclcpp::Time last_global_publish_at_;

  // Trajectory history — accumulates driven positions for visualization
  std::vector<geometry_msgs::msg::Point> trajectory_;
  geometry_msgs::msg::Point last_trajectory_point_;
  geometry_msgs::msg::Point filtered_trajectory_point_;
  bool trajectory_filter_ready_{false};

  // Parameters
  // Trackdrive — EXPLORE（第1圈，建图/探索圈）速度配置
  double trackdrive_explore_max_velocity_{7.0};        // m/s
  double trackdrive_explore_min_velocity_{3.0};        // m/s
  double trackdrive_explore_lateral_accel_limit_{4.0}; // m/s^2
  // Trackdrive — RACE（比赛圈）速度配置
  double trackdrive_race_lap2_max_velocity_{9.0};      // m/s 第2圈
  double trackdrive_race_lap3_max_velocity_{10.0};     // m/s 第3圈起
  double trackdrive_race_min_velocity_{4.0};           // m/s
  double trackdrive_race_lateral_accel_limit_{6.0};    // m/s^2
  // Trackdrive — 通用处理（所有状态共用）
  double trackdrive_resample_spacing_{1.0};            // m
  double trackdrive_min_forward_target_{0.5};          // m
  double trackdrive_full_speed_forward_distance_{15.0}; // m
  // Trackdrive — 降级限速（短中心线 / 低置信度共用）
  double trackdrive_degraded_velocity_{3.0};           // m/s
  int trackdrive_short_centerline_points_{3};          // source centerline points
  // Trackdrive — 置信度 → 速度映射
  double trackdrive_confidence_slow_threshold_{0.45};
  double trackdrive_confidence_full_threshold_{0.75};
  double trackdrive_confidence_timeout_sec_{0.50};     // s
  // Trackdrive — 全局冻结中心线
  double trackdrive_global_horizon_distance_{40.0};    // m
  int trackdrive_global_search_points_{24};
  int trackdrive_global_min_points_{20};
  double trackdrive_global_publish_period_sec_{0.10};

  // Skidpad reference in map.  This matches tracks/skidpad.yaml by default.
  double skidpad_radius_{9.125};       // m
  double skidpad_velocity_{5.0};       // m/s
  int    skidpad_points_{72};          // waypoints per circle (every 5 deg)
  double skidpad_start_x_{0.0};        // m, crossing reference
  double skidpad_start_y_{0.0};        // m, crossing reference
  double skidpad_start_yaw_{0.0};      // rad, entry/exit direction
  double skidpad_entry_x_{-15.0};      // m, local to crossing reference
  double skidpad_entry_y_{0.0};        // m, local to crossing reference
  double skidpad_exit_length_{25.0};   // m, measured from the crossing
  double skidpad_braking_distance_{10.0};  // m
  // Relative paths are rooted at the detected WUTA-FSD directory.
  std::string skidpad_csv_path_{"ros2_ws/log/trajectory/skidpad_trajectory.csv"};

  // Driven-trajectory visualization only. These do not affect localization
  // or the controller; they prevent INS/EKF measurement noise from appearing
  // as a jagged vehicle path in RViz.
  double driven_trajectory_smoothing_alpha_{0.20};
  double driven_trajectory_min_distance_{0.10};
  double driven_trajectory_max_step_{2.0};

  // Acceleration reference in map.  These values match acceleration.yaml:
  // start at -0.30 m, timing starts at 0 m, finish is 75 m later, and the
  // marked exit/stopping lane extends another 100 m.
  double acceleration_start_x_{-0.30};      // m
  double acceleration_start_y_{0.0};        // m
  double acceleration_start_yaw_{0.0};      // rad
  double acceleration_timing_start_x_{0.0}; // m
  double acceleration_length_{75.0};        // timed distance, m
  double acceleration_stopping_distance_{100.0};  // after finish, m
  double acceleration_velocity_{15.0};      // m/s

  // EBS test reference — 满足赛规 7.5：起点后 0.3m，25m 测速点 ≥40km/h(11.11)，
  // RES 急停后 ≤10m 内停车。结构复用 generateStraightRun（同 acceleration）。
  double ebs_start_x_{0.3};         // m, start-position line
  double ebs_start_y_{0.0};         // m
  double ebs_start_yaw_{0.0};       // rad
  double ebs_timing_start_x_{25.0}; // m, 25m 测速/急停标记
  double ebs_length_{0.0};          // m, 无计时段
  double ebs_stopping_distance_{10.0}; // m, 制动段 ≤10m
  double ebs_velocity_{12.0};       // m/s, ≥40km/h(11.11)，留余量

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Lane>::SharedPtr centerline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_centerline_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr path_confidence_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr localization_confidence_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr lap_count_sub_;

  // Publishers
  rclcpp::Publisher<autoware_msgs::msg::Lane>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub_;
};

}  // namespace path_generator
