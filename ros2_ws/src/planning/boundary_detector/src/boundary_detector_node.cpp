#include "boundary_detector/boundary_detector_node.hpp"
#include <cmath>

namespace boundary_detector
{

BoundaryDetectorNode::BoundaryDetectorNode(const rclcpp::NodeOptions & options)
: Node("boundary_detector_node", options)
{
  lookahead_distance_ = declare_parameter("lookahead_distance", lookahead_distance_);
  desired_velocity_   = declare_parameter("desired_velocity",   desired_velocity_);

  // Subscribers
  cone_map_sub_ = create_subscription<wuta_msgs::msg::ConeMap>(
    "/mapping/cone_map", 10,
    std::bind(&BoundaryDetectorNode::onConeMap, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&BoundaryDetectorNode::onPose, this, std::placeholders::_1));

  mission_sub_ = create_subscription<wuta_msgs::msg::MissionState>(
    "/system/mission_state", 10,
    std::bind(&BoundaryDetectorNode::onMissionState, this, std::placeholders::_1));

  // Publishers
  centerline_pub_ = create_publisher<autoware_msgs::msg::Lane>("/planning/centerline", 10);
  marker_pub_     = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/centerline_viz", 10);

  RCLCPP_INFO(get_logger(), "BoundaryDetectorNode ready.");
}

void BoundaryDetectorNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  pose_ready_ = true;
}

void BoundaryDetectorNode::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  mission_mode_ = msg->mission_mode;
}

void BoundaryDetectorNode::onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg)
{
  if (!pose_ready_) return;

  // Only TRACKDRIVE uses Delaunay boundary detection
  // SKIDPAD and ACCELERATION handle their own path in path_generator
  if (mission_mode_ != wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE) return;

  // Build merged cone list (blue + yellow + unknown)
  auto points = coneMapToPoints(*msg);
  if (points.size() < 4) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Not enough cones for Delaunay (%zu cones)", points.size());
    return;
  }

  auto lane = computeCenterline(points);
  if (lane.waypoints.empty()) return;

  lane.header.stamp    = now();
  lane.header.frame_id = "map";
  centerline_pub_->publish(lane);

  if (marker_pub_->get_subscription_count() > 0) {
    publishVisualization(lane);
  }
}

std::vector<Point2d> BoundaryDetectorNode::coneMapToPoints(
  const wuta_msgs::msg::ConeMap & map) const
{
  std::vector<Point2d> points;

  // Extract cones within lookahead range of current pose
  const double vx = current_pose_.pose.position.x;
  const double vy = current_pose_.pose.position.y;

  const auto addCones = [&](const auto & cones, int color) {
    for (const auto & cone : cones) {
      const double dx = cone.position.x - vx;
      const double dy = cone.position.y - vy;
      if (std::sqrt(dx * dx + dy * dy) < lookahead_distance_) {
        points.emplace_back(cone.position.x, cone.position.y, color);
      }
    }
  };

  addCones(map.blue_cones,    1);  // Point2d color=1 → blue (left)
  addCones(map.yellow_cones,  2);  // Point2d color=2 → yellow (right)
  addCones(map.unknown_cones, 0);

  return points;
}

autoware_msgs::msg::Lane BoundaryDetectorNode::computeCenterline(
  const std::vector<Point2d> & points)
{
  autoware_msgs::msg::Lane lane;

  // Set vehicle position as search start
  Point2d vehicle_pos(
    current_pose_.pose.position.x,
    current_pose_.pose.position.y);

  // Rebuild the local Delaunay graph as the visible cone map changes.
  path_search_.Clear();
  path_search_.SetPoints(points);

  // Compute vehicle heading as former direction
  const auto & q = current_pose_.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  path_search_.SetFormer(Vect(std::cos(yaw), std::sin(yaw)));

  auto vehicle_midpoint = std::make_shared<MidPoint>();
  vehicle_midpoint->mid = vehicle_pos;
  path_search_.SetStartPoint(vehicle_midpoint);
  path_search_.SetPathStartPoint(vehicle_midpoint);

  if (!last_midps_.empty()) {
    path_search_.SetLastMidps(last_midps_);
  }

  if (!path_search_.IsInit()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "PathSearch initialization failed");
    return lane;
  }

  // Get best path through Delaunay midpoints
  auto midps = path_search_.GetBestMidps();
  last_midps_ = midps;

  // Convert midpoints to autoware Lane waypoints
  for (const auto & mp : midps) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = mp->mid.x;
    wp.pose.pose.position.y = mp->mid.y;
    wp.pose.pose.position.z = current_pose_.pose.position.z;
    wp.pose.pose.orientation.w = 1.0;
    wp.twist.twist.linear.x = desired_velocity_;
    lane.waypoints.push_back(wp);
  }

  return lane;
}

void BoundaryDetectorNode::publishVisualization(const autoware_msgs::msg::Lane & lane)
{
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker del;
  del.header.frame_id = "map";
  del.header.stamp    = now();
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  // Line strip through centerline
  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp    = now();
  line.ns     = "centerline";
  line.id     = 0;
  line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.1;
  line.color.r = 0.0f; line.color.g = 1.0f; line.color.b = 0.0f; line.color.a = 1.0f;

  for (const auto & wp : lane.waypoints) {
    geometry_msgs::msg::Point p;
    p.x = wp.pose.pose.position.x;
    p.y = wp.pose.pose.position.y;
    p.z = wp.pose.pose.position.z;
    line.points.push_back(p);
  }
  arr.markers.push_back(line);
  marker_pub_->publish(arr);
}

}  // namespace boundary_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<boundary_detector::BoundaryDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
