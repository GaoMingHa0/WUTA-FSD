#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_msgs/msg/lane.hpp>

#include "wuta_msgs/msg/mission_state.hpp"

namespace path_generator
{

/**
 * Path Generator Node
 *
 * Routes to the correct path-generation mode based on MissionState:
 *
 *  TRACKDRIVE   → forwards /planning/centerline from boundary_detector (Delaunay)
 *
 *  SKIDPAD      → figure-8 using circle centres from skidpad_detector.
 *                 Falls back to pose-based estimation if detector not yet ready.
 *
 *  ACCELERATION → straight waypoints from current pose to the endpoint detected
 *                 by line_detector (PCA on cone map).
 *                 Falls back to fixed acceleration_length_ if detector not ready.
 *
 * All modes output /planning/final_waypoints (autoware_msgs::Lane).
 */
class PathGeneratorNode : public rclcpp::Node
{
public:
  explicit PathGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onCenterline(const autoware_msgs::msg::Lane::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onSkidpadCircles(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onAccelerationLine(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Path generators
  autoware_msgs::msg::Lane generateSkidpadPath() const;
  autoware_msgs::msg::Lane generateAccelerationPath() const;

  // State
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  uint8_t system_state_{wuta_msgs::msg::MissionState::IDLE};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_ready_{false};

  // Detector outputs (optional — fall back to pose-based if absent)
  geometry_msgs::msg::PoseArray skidpad_circles_;   // [0]=right, [1]=left
  bool skidpad_circles_ready_{false};

  geometry_msgs::msg::PoseStamped acceleration_endpoint_;
  bool acceleration_line_ready_{false};

  // Parameters — Trackdrive
  double trackdrive_velocity_{7.0};

  // Parameters — Skidpad
  double skidpad_radius_{9.125};
  double skidpad_velocity_{5.0};
  int    skidpad_points_{72};

  // Parameters — Acceleration
  double acceleration_length_{75.0};   // fallback length [m]
  double acceleration_velocity_{15.0};

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr       mission_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Lane>::SharedPtr           centerline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr      skidpad_circles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    accel_line_sub_;

  // Publisher
  rclcpp::Publisher<autoware_msgs::msg::Lane>::SharedPtr waypoints_pub_;
};

}  // namespace path_generator
