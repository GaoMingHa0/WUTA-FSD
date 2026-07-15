#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_msgs/msg/lane.hpp>

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

  // State
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  uint8_t system_state_{wuta_msgs::msg::MissionState::IDLE};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_ready_{false};
  autoware_msgs::msg::Lane skidpad_path_;
  bool skidpad_path_ready_{false};

  // Parameters
  // Trackdrive
  double trackdrive_velocity_{7.0};    // m/s

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

  // Acceleration (75m straight)
  double acceleration_length_{75.0};   // m
  double acceleration_velocity_{15.0}; // m/s

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Lane>::SharedPtr centerline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publisher
  rclcpp::Publisher<autoware_msgs::msg::Lane>::SharedPtr waypoints_pub_;
};

}  // namespace path_generator
