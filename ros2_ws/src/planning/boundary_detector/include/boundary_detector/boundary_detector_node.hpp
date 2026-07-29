#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
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
  autoware_msgs::msg::Lane computeGlobalCenterline(
    const wuta_msgs::msg::ConeMap & map, double & confidence) const;
  bool hasSevereColorImbalance(const wuta_msgs::msg::ConeMap & map) const;
  void publishPlanningStatus(bool global_ready, double confidence);

  void publishVisualization(const autoware_msgs::msg::Lane & lane);

  // Algorithm
  PathSearch path_search_;
  std::vector<std::shared_ptr<MidPoint>> last_midps_;

  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  bool pose_ready_{false};
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  uint8_t system_state_{wuta_msgs::msg::MissionState::IDLE};
  int short_color_pair_streak_{0};
  autoware_msgs::msg::Lane frozen_global_centerline_;
  bool global_centerline_ready_{false};
  double global_centerline_confidence_{0.0};

  // Parameters
  double lookahead_distance_{15.0};  // m — how far ahead to plan
  double desired_velocity_{7.0};     // m/s — default, overridden by path_generator
  int local_pairing_min_streak_{10}; // cycles before geometry-only pairing is allowed
  double local_pairing_color_imbalance_ratio_{0.20};
  int delaunay_min_waypoints_{5};     // reject short fallback paths that can reverse in tight turns
  double global_pairing_min_width_{1.5};
  double global_pairing_max_width_{7.5};
  double global_pairing_dedup_distance_{0.75};
  int global_geometry_neighbor_count_{8};
  double global_geometry_neighbor_distance_{5.0};
  double global_geometry_max_tangent_alignment_{0.45};
  double global_max_segment_length_{10.0};
  double global_max_closure_distance_{8.0};
  int global_min_waypoints_{20};
  double global_min_coverage_ratio_{0.60};

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::ConeMap>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;

  // Publishers
  rclcpp::Publisher<autoware_msgs::msg::Lane>::SharedPtr centerline_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;
};

}  // namespace boundary_detector
