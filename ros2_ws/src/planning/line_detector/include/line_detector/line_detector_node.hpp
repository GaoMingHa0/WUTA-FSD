#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "wuta_msgs/msg/cone_map.hpp"
#include "wuta_msgs/msg/mission_state.hpp"

namespace line_detector
{

/**
 * LineDetectorNode
 *
 * Analyses the ConeMap to find the acceleration track axis and endpoint.
 *
 * Algorithm (PCA on all cone positions):
 *   1. Compute centroid of all visible cones.
 *   2. PCA: find principal direction (max-variance axis) — this is the track heading.
 *   3. Project all cones onto the principal axis; the furthest projection + buffer
 *      defines the endpoint (finish line position).
 *
 * Publishes /planning/acceleration_line (PoseStamped):
 *   position    = endpoint (finish line midpoint estimate)
 *   orientation = track heading (quaternion from yaw)
 *
 * path_generator subscribes to this and replaces its fixed-length straight-line
 * generation with a geometry derived from actual cone positions.
 */
class LineDetectorNode : public rclcpp::Node
{
public:
  explicit LineDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);

  // State
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_ACCELERATION};
  uint8_t system_state_{wuta_msgs::msg::MissionState::IDLE};

  // Parameters
  int    min_cones_{4};          // minimum total cones to attempt fit
  double endpoint_buffer_{3.0};  // extra distance beyond last cone [m]

  // I/O
  rclcpp::Subscription<wuta_msgs::msg::ConeMap>::SharedPtr      cone_map_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr line_pub_;
};

}  // namespace line_detector
