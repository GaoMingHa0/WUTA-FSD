#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "wuta_msgs/msg/cone_map.hpp"
#include "wuta_msgs/msg/mission_state.hpp"

namespace skidpad_detector
{

/**
 * SkidpadDetectorNode
 *
 * Fits a circle to the blue-cone cluster and the yellow-cone cluster from the
 * ConeMap using the Kasa algebraic least-squares method.
 *
 * Publishes circle centres to /planning/skidpad_circles (PoseArray, 2 entries):
 *   pose[0] = right circle centre  (yellow cones, CCW lap)
 *   pose[1] = left  circle centre  (blue cones,   CW  lap)
 *
 * path_generator subscribes to this topic and uses it to build the figure-8
 * instead of estimating centres purely from vehicle pose.
 */
class SkidpadDetectorNode : public rclcpp::Node
{
public:
  explicit SkidpadDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);

  /**
   * Kasa circle fit.
   * Returns false if the fit fails (too few points or degenerate geometry).
   * cx, cy — fitted centre; r — fitted radius.
   */
  bool fitCircle(
    const std::vector<geometry_msgs::msg::Point> & pts,
    double & cx, double & cy, double & r) const;

  // State
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_SKIDPAD};
  uint8_t system_state_{wuta_msgs::msg::MissionState::IDLE};

  // Parameters
  int    min_cones_per_circle_{4};    // minimum cones needed to attempt a fit
  double expected_radius_{9.125};     // FSG standard radius [m]
  double radius_tolerance_{2.5};      // reject fit if |r - expected| > tolerance [m]

  // I/O
  rclcpp::Subscription<wuta_msgs::msg::ConeMap>::SharedPtr       cone_map_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr  mission_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    circles_pub_;
};

}  // namespace skidpad_detector
