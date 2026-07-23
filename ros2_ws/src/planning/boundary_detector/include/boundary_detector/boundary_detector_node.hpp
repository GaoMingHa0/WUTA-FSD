#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wuta_msgs/msg/cone_map.hpp"
#include "wuta_msgs/msg/mission_state.hpp"

// HRT-D pathplanning library (pure C++, no ROS)
#include "pathplanning/PathSearch.h"
#include "pathplanning/structs/Point2d.h"
#include "pathplanning/structs/MidPoint.h"

#include <autoware_msgs/msg/lane.hpp>
#include <autoware_msgs/msg/waypoint.hpp>

namespace boundary_detector
{

class BoundaryDetectorNode : public rclcpp::Node
{
public:
  explicit BoundaryDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);

  // Convert ConeMap to Point2d list for Delaunay
  std::vector<Point2d> coneMapToPoints(const wuta_msgs::msg::ConeMap & map) const;

  // Run Delaunay + path search, return centerline waypoints
  autoware_msgs::msg::Lane computeCenterline(const std::vector<Point2d> & points);

  // Online Trackdrive path: pair forward blue/yellow cones in the local driving direction.
  autoware_msgs::msg::Lane computePairedCenterline(const wuta_msgs::msg::ConeMap & map) const;
  autoware_msgs::msg::Lane computeLocalFrameCenterline(const wuta_msgs::msg::ConeMap & map) const;
  bool hasSevereColorImbalance(const wuta_msgs::msg::ConeMap & map) const;

  void publishVisualization(const autoware_msgs::msg::Lane & lane);

  // Algorithm
  PathSearch path_search_;
  std::vector<std::shared_ptr<MidPoint>> last_midps_;

  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_ready_{false};
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  int short_color_pair_streak_{0};

  // Parameters
  double lookahead_distance_{15.0};  // m — how far ahead to plan
  double desired_velocity_{7.0};     // m/s — default, overridden by path_generator
  int local_pairing_min_streak_{10}; // cycles before geometry-only pairing is allowed
  double local_pairing_color_imbalance_ratio_{0.20};

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::ConeMap>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;

  // Publishers
  rclcpp::Publisher<autoware_msgs::msg::Lane>::SharedPtr centerline_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace boundary_detector
