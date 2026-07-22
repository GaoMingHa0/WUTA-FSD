#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_msgs/msg/lane.hpp>
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

  // Mode-specific path generators
  autoware_msgs::msg::Lane generateSkidpadPath() const;
  autoware_msgs::msg::Lane generateAccelerationPath() const;
  autoware_msgs::msg::Lane resampleTrackdriveLane(const autoware_msgs::msg::Lane & lane) const;
  void applyTrackdriveSpeedProfile(autoware_msgs::msg::Lane & lane) const;

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

  // Trajectory history — accumulates driven positions for visualization
  std::vector<geometry_msgs::msg::Point> trajectory_;
  geometry_msgs::msg::Point last_trajectory_point_;
  geometry_msgs::msg::Point filtered_trajectory_point_;
  bool trajectory_filter_ready_{false};

  // Parameters
  // Trackdrive
  double trackdrive_velocity_{7.0};    // m/s
  double trackdrive_resample_spacing_{1.0};  // m
  double trackdrive_min_velocity_{3.0}; // m/s
  double trackdrive_lateral_accel_limit_{4.0}; // m/s^2

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

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Lane>::SharedPtr centerline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  rclcpp::Publisher<autoware_msgs::msg::Lane>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub_;
};

}  // namespace path_generator
