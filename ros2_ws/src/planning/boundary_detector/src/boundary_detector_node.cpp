#include "boundary_detector/boundary_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace boundary_detector
{
namespace
{
double yawFromPose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double forwardProjection(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  const Point2d & point)
{
  const double dx = point.x - pose.pose.position.x;
  const double dy = point.y - pose.pose.position.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}

double lateralProjection(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  const Point2d & point)
{
  const double dx = point.x - pose.pose.position.x;
  const double dy = point.y - pose.pose.position.y;
  return -std::sin(yaw) * dx + std::cos(yaw) * dy;
}

std::vector<std::shared_ptr<MidPoint>> orientAndFilterForwardMidpoints(
  const std::vector<std::shared_ptr<MidPoint>> & midpoints,
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw)
{
  std::vector<std::shared_ptr<MidPoint>> forward_midpoints;
  forward_midpoints.reserve(midpoints.size());

  for (const auto & midpoint : midpoints) {
    if (forwardProjection(pose, yaw, midpoint->mid) >= -1.0) {
      forward_midpoints.push_back(midpoint);
    }
  }

  if (forward_midpoints.size() >= 2 &&
      forwardProjection(pose, yaw, forward_midpoints.back()->mid) + 0.5 <
      forwardProjection(pose, yaw, forward_midpoints.front()->mid)) {
    std::reverse(forward_midpoints.begin(), forward_midpoints.end());
  }

  return forward_midpoints;
}

double planeDistance(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

BoundaryDetectorNode::BoundaryDetectorNode(const rclcpp::NodeOptions & options)
: Node("boundary_detector_node", options)
{
  lookahead_distance_ = declare_parameter("lookahead_distance", lookahead_distance_);
  desired_velocity_   = declare_parameter("desired_velocity",   desired_velocity_);
  local_pairing_min_streak_ = declare_parameter(
    "local_pairing_min_streak", local_pairing_min_streak_);
  local_pairing_color_imbalance_ratio_ = declare_parameter(
    "local_pairing_color_imbalance_ratio", local_pairing_color_imbalance_ratio_);

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

  // Only TRACKDRIVE uses online boundary detection
  // SKIDPAD and ACCELERATION handle their own path in path_generator
  if (mission_mode_ != wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE) return;

  auto lane = computePairedCenterline(*msg);
  bool using_pair_lane = lane.waypoints.size() >= 3;
  if (using_pair_lane) {
    short_color_pair_streak_ = 0;
  } else {
    ++short_color_pair_streak_;
  }

  const bool color_imbalanced = hasSevereColorImbalance(*msg);

  if (!using_pair_lane &&
      (color_imbalanced || short_color_pair_streak_ >= local_pairing_min_streak_)) {
    auto local_lane = computeLocalFrameCenterline(*msg);
    if (local_lane.waypoints.size() > lane.waypoints.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Using local-frame cone pairing (%zu waypoints, color_imbalanced=%s)",
        local_lane.waypoints.size(), color_imbalanced ? "true" : "false");
      lane = local_lane;
      using_pair_lane = true;
    }
  }

  // Fall back to Delaunay when color-separated cone pairs are not available.
  auto points = coneMapToPoints(*msg);
  if (lane.waypoints.size() < 3 && points.size() < 4) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Not enough cones for Trackdrive centerline (blue=%zu, yellow=%zu, total=%zu)",
      msg->blue_cones.size(), msg->yellow_cones.size(), points.size());
    return;
  }

  if (lane.waypoints.size() < 3) {
    auto fallback_lane = computeCenterline(points);
    if (!fallback_lane.waypoints.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Using Delaunay centerline fallback (%zu waypoints)",
        fallback_lane.waypoints.size());
      lane = fallback_lane;
      using_pair_lane = false;
    }
  }
  if (lane.waypoints.empty()) return;
  if (using_pair_lane && lane.waypoints.size() >= 3) {
    last_midps_.clear();
  }

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
  const double yaw = yawFromPose(current_pose_);
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
  midps = orientAndFilterForwardMidpoints(midps, current_pose_, yaw);
  if (midps.size() < 2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Rejected centerline with fewer than 2 forward midpoints");
    return lane;
  }
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

autoware_msgs::msg::Lane BoundaryDetectorNode::computeLocalFrameCenterline(
  const wuta_msgs::msg::ConeMap & map) const
{
  autoware_msgs::msg::Lane lane;
  if (!pose_ready_) {
    return lane;
  }

  struct SideCone
  {
    double x;
    double y;
    double forward;
    double lateral;
  };

  struct Candidate
  {
    double forward;
    double lateral;
    double cost;
    std::size_t left_index;
    std::size_t right_index;
    double x;
    double y;
    double tangent_x;
    double tangent_y;
  };

  const double yaw = yawFromPose(current_pose_);
  std::vector<SideCone> left_cones;
  std::vector<SideCone> right_cones;

  const auto add_cones = [&](const auto & cones) {
    for (const auto & cone : cones) {
      const Point2d point(cone.position.x, cone.position.y);
      const double forward = forwardProjection(current_pose_, yaw, point);
      const double lateral = lateralProjection(current_pose_, yaw, point);
      const double distance = planeDistance(
        cone.position.x, cone.position.y,
        current_pose_.pose.position.x, current_pose_.pose.position.y);

      if (distance > lookahead_distance_ || forward < -2.0) continue;
      if (std::abs(lateral) < 0.7 || std::abs(lateral) > 8.0) continue;

      SideCone side_cone{cone.position.x, cone.position.y, forward, lateral};
      if (lateral > 0.0) {
        left_cones.push_back(side_cone);
      } else {
        right_cones.push_back(side_cone);
      }
    }
  };

  add_cones(map.blue_cones);
  add_cones(map.yellow_cones);
  add_cones(map.unknown_cones);

  if (left_cones.empty() || right_cones.empty()) {
    return lane;
  }

  std::vector<Candidate> pair_candidates;
  for (std::size_t left_index = 0; left_index < left_cones.size(); ++left_index) {
    const auto & left = left_cones[left_index];
    for (std::size_t right_index = 0; right_index < right_cones.size(); ++right_index) {
      const auto & right = right_cones[right_index];
      const double lateral_span = left.lateral - right.lateral;
      if (lateral_span < 1.5 || lateral_span > 7.5) continue;

      const double width = planeDistance(left.x, left.y, right.x, right.y);
      if (width < 2.0 || width > 7.5) continue;

      const double forward_gap = std::abs(left.forward - right.forward);
      if (forward_gap > 7.0) continue;

      const double cx = 0.5 * (left.x + right.x);
      const double cy = 0.5 * (left.y + right.y);
      const Point2d center(cx, cy);
      const double center_forward = forwardProjection(current_pose_, yaw, center);
      if (center_forward < -1.0) continue;

      const double center_lateral = lateralProjection(current_pose_, yaw, center);
      const double left_x = (left.x - right.x) / width;
      const double left_y = (left.y - right.y) / width;
      double tangent_x = left_y;
      double tangent_y = -left_x;
      if (tangent_x * std::cos(yaw) + tangent_y * std::sin(yaw) < 0.0) {
        tangent_x = -tangent_x;
        tangent_y = -tangent_y;
      }

      const double heading_alignment = tangent_x * std::cos(yaw) + tangent_y * std::sin(yaw);
      const double cost =
        2.0 * forward_gap + std::abs(center_lateral) + std::abs(width - 4.0) +
        2.0 * (1.0 - heading_alignment) + 0.05 * (left.forward + right.forward);
      pair_candidates.push_back({
        center_forward, center_lateral, cost, left_index, right_index, cx, cy,
        tangent_x, tangent_y});
    }
  }

  std::sort(pair_candidates.begin(), pair_candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.cost < rhs.cost;
    });

  std::vector<bool> used_left(left_cones.size(), false);
  std::vector<bool> used_right(right_cones.size(), false);
  std::vector<Candidate> candidates;
  candidates.reserve(pair_candidates.size());
  for (const auto & candidate : pair_candidates) {
    if (used_left[candidate.left_index] || used_right[candidate.right_index]) continue;
    used_left[candidate.left_index] = true;
    used_right[candidate.right_index] = true;
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.forward < rhs.forward;
    });

  std::vector<Candidate> unique_candidates;
  unique_candidates.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    const bool duplicate = std::any_of(
      unique_candidates.begin(), unique_candidates.end(),
      [&candidate](const Candidate & existing) {
        return planeDistance(candidate.x, candidate.y, existing.x, existing.y) < 0.75;
      });
    if (!duplicate) {
      unique_candidates.push_back(candidate);
    }
  }

  if (unique_candidates.size() < 3) {
    return lane;
  }

  std::vector<Candidate> ordered_candidates;
  ordered_candidates.reserve(unique_candidates.size());
  std::vector<bool> used(unique_candidates.size(), false);
  double cursor_x = current_pose_.pose.position.x;
  double cursor_y = current_pose_.pose.position.y;
  double heading_x = std::cos(yaw);
  double heading_y = std::sin(yaw);

  for (std::size_t step = 0; step < unique_candidates.size(); ++step) {
    int best_index = -1;
    double best_score = std::numeric_limits<double>::max();

    for (std::size_t index = 0; index < unique_candidates.size(); ++index) {
      if (used[index]) continue;
      const auto & candidate = unique_candidates[index];
      const double dx = candidate.x - cursor_x;
      const double dy = candidate.y - cursor_y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < 0.35) continue;
      if (ordered_candidates.empty()) {
        if (candidate.forward < -1.0 || distance > lookahead_distance_) continue;
      } else if (distance > 8.0) {
        continue;
      }

      const double segment_alignment = (dx * heading_x + dy * heading_y) / distance;
      if (segment_alignment < -0.15) continue;
      const double tangent_alignment =
        candidate.tangent_x * heading_x + candidate.tangent_y * heading_y;
      if (tangent_alignment < -0.35) continue;

      const double jump_penalty = ordered_candidates.empty()
        ? 0.5 * std::abs(candidate.lateral)
        : std::max(0.0, distance - 4.0);
      const double score =
        distance +
        5.0 * (1.0 - segment_alignment) +
        2.0 * (1.0 - tangent_alignment) +
        0.2 * candidate.cost +
        jump_penalty;
      if (score < best_score) {
        best_score = score;
        best_index = static_cast<int>(index);
      }
    }

    if (best_index < 0) break;
    used[best_index] = true;
    const auto & selected = unique_candidates[best_index];
    const double dx = selected.x - cursor_x;
    const double dy = selected.y - cursor_y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double segment_x = dx / distance;
    const double segment_y = dy / distance;
    heading_x = segment_x + selected.tangent_x;
    heading_y = segment_y + selected.tangent_y;
    const double heading_norm = std::sqrt(heading_x * heading_x + heading_y * heading_y);
    if (heading_norm > 1e-6) {
      heading_x /= heading_norm;
      heading_y /= heading_norm;
    } else {
      heading_x = segment_x;
      heading_y = segment_y;
    }
    cursor_x = selected.x;
    cursor_y = selected.y;
    ordered_candidates.push_back(selected);
  }

  if (ordered_candidates.size() < 3) {
    return lane;
  }

  lane.header.stamp = now();
  lane.header.frame_id = "map";
  for (const auto & candidate : ordered_candidates) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = candidate.x;
    wp.pose.pose.position.y = candidate.y;
    wp.pose.pose.position.z = current_pose_.pose.position.z;
    wp.pose.pose.orientation.w = 1.0;
    wp.twist.twist.linear.x = desired_velocity_;
    lane.waypoints.push_back(wp);
  }

  return lane;
}

bool BoundaryDetectorNode::hasSevereColorImbalance(const wuta_msgs::msg::ConeMap & map) const
{
  const std::size_t blue_count = map.blue_cones.size();
  const std::size_t yellow_count = map.yellow_cones.size();
  const std::size_t colored_count = blue_count + yellow_count;
  if (colored_count < 6) return false;

  const double min_fraction = static_cast<double>(std::min(blue_count, yellow_count)) /
    static_cast<double>(colored_count);
  return min_fraction < local_pairing_color_imbalance_ratio_;
}

autoware_msgs::msg::Lane BoundaryDetectorNode::computePairedCenterline(
  const wuta_msgs::msg::ConeMap & map) const
{
  autoware_msgs::msg::Lane lane;
  if (!pose_ready_ || map.blue_cones.empty() || map.yellow_cones.empty()) {
    return lane;
  }

  struct Candidate
  {
    double forward;
    double lateral;
    double cost;
    std::size_t blue_index;
    std::size_t yellow_index;
    double x;
    double y;
    double tangent_x;
    double tangent_y;
  };

  const double yaw = yawFromPose(current_pose_);
  const auto in_window = [this, yaw](const auto & cone) {
    const Point2d point(cone.position.x, cone.position.y);
    const double forward = forwardProjection(current_pose_, yaw, point);
    const double distance = planeDistance(
      cone.position.x, cone.position.y,
      current_pose_.pose.position.x, current_pose_.pose.position.y);
    return distance <= lookahead_distance_ && forward >= -2.0;
  };

  std::vector<Candidate> pair_candidates;

  for (std::size_t blue_index = 0; blue_index < map.blue_cones.size(); ++blue_index) {
    const auto & blue = map.blue_cones[blue_index];
    if (!in_window(blue)) continue;

    const auto blue_point = Point2d(blue.position.x, blue.position.y);
    const double blue_forward = forwardProjection(current_pose_, yaw, blue_point);
    const double blue_lateral = lateralProjection(current_pose_, yaw, blue_point);

    for (std::size_t yellow_index = 0; yellow_index < map.yellow_cones.size(); ++yellow_index) {
      const auto & yellow = map.yellow_cones[yellow_index];
      if (!in_window(yellow)) continue;

      const auto yellow_point = Point2d(yellow.position.x, yellow.position.y);
      const double yellow_forward = forwardProjection(current_pose_, yaw, yellow_point);
      const double yellow_lateral = lateralProjection(current_pose_, yaw, yellow_point);
      const double lateral_span = blue_lateral - yellow_lateral;
      if (std::abs(lateral_span) > 7.0) continue;

      const double width = planeDistance(
        blue.position.x, blue.position.y,
        yellow.position.x, yellow.position.y);
      if (width < 2.0 || width > 7.0) continue;

      const double forward_gap = std::abs(yellow_forward - blue_forward);
      if (forward_gap > 6.0) continue;

      const double cx = 0.5 * (blue.position.x + yellow.position.x);
      const double cy = 0.5 * (blue.position.y + yellow.position.y);
      const Point2d center(cx, cy);
      const double center_forward = forwardProjection(current_pose_, yaw, center);
      if (center_forward < -1.0) continue;

      const double center_lateral = lateralProjection(current_pose_, yaw, center);

      // Blue is the left boundary and yellow is the right boundary. Their
      // cross-track vector gives a local tangent without using any simulator
      // reference line. Choose the tangent direction closest to the car's
      // current heading.
      const double left_x = (blue.position.x - yellow.position.x) / width;
      const double left_y = (blue.position.y - yellow.position.y) / width;
      double tangent_x = left_y;
      double tangent_y = -left_x;
      if (tangent_x * std::cos(yaw) + tangent_y * std::sin(yaw) < 0.0) {
        tangent_x = -tangent_x;
        tangent_y = -tangent_y;
      }

      const double side_penalty = lateral_span >= 1.0 ? 0.0 : 3.0 + std::abs(lateral_span);
      const double heading_alignment = tangent_x * std::cos(yaw) + tangent_y * std::sin(yaw);
      const double cost =
        2.0 * forward_gap + std::abs(center_lateral) + std::abs(width - 4.0) +
        side_penalty + 2.0 * (1.0 - heading_alignment);
      pair_candidates.push_back({
        center_forward, center_lateral, cost, blue_index, yellow_index, cx, cy,
        tangent_x, tangent_y});
    }
  }

  std::sort(pair_candidates.begin(), pair_candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.cost < rhs.cost;
    });

  std::vector<bool> used_blue(map.blue_cones.size(), false);
  std::vector<bool> used_yellow(map.yellow_cones.size(), false);
  std::vector<Candidate> candidates;
  for (const auto & candidate : pair_candidates) {
    if (used_blue[candidate.blue_index] || used_yellow[candidate.yellow_index]) continue;
    used_blue[candidate.blue_index] = true;
    used_yellow[candidate.yellow_index] = true;
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.forward < rhs.forward;
    });

  std::vector<Candidate> unique_candidates;
  unique_candidates.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    const bool duplicate = std::any_of(
      unique_candidates.begin(), unique_candidates.end(),
      [&candidate](const Candidate & existing) {
        return planeDistance(candidate.x, candidate.y, existing.x, existing.y) < 0.75;
      });
    if (!duplicate) {
      unique_candidates.push_back(candidate);
    }
  }

  if (unique_candidates.size() < 2) {
    return lane;
  }

  // Order local center points as a continuous path. A pure forward-projection
  // sort can jump to a nearby but different branch on tight autocross layouts.
  // This greedy chain still uses only the online cone map, but prefers smooth
  // motion from the vehicle pose through locally consistent cone pairs.
  std::vector<Candidate> ordered_candidates;
  ordered_candidates.reserve(unique_candidates.size());
  std::vector<bool> used(unique_candidates.size(), false);
  double cursor_x = current_pose_.pose.position.x;
  double cursor_y = current_pose_.pose.position.y;
  double heading_x = std::cos(yaw);
  double heading_y = std::sin(yaw);

  for (std::size_t step = 0; step < unique_candidates.size(); ++step) {
    int best_index = -1;
    double best_score = std::numeric_limits<double>::max();

    for (std::size_t index = 0; index < unique_candidates.size(); ++index) {
      if (used[index]) continue;
      const auto & candidate = unique_candidates[index];
      const double dx = candidate.x - cursor_x;
      const double dy = candidate.y - cursor_y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < 0.35) continue;
      if (ordered_candidates.empty()) {
        if (candidate.forward < -1.0 || distance > lookahead_distance_) continue;
      } else if (distance > 8.0) {
        continue;
      }

      const double segment_alignment = (dx * heading_x + dy * heading_y) / distance;
      if (segment_alignment < -0.15) continue;
      const double tangent_alignment =
        candidate.tangent_x * heading_x + candidate.tangent_y * heading_y;
      if (tangent_alignment < -0.35) continue;

      const double jump_penalty = ordered_candidates.empty()
        ? 0.5 * std::abs(candidate.lateral)
        : std::max(0.0, distance - 4.0);
      const double score =
        distance +
        5.0 * (1.0 - segment_alignment) +
        2.0 * (1.0 - tangent_alignment) +
        0.2 * candidate.cost +
        jump_penalty;
      if (score < best_score) {
        best_score = score;
        best_index = static_cast<int>(index);
      }
    }

    if (best_index < 0) break;
    used[best_index] = true;
    const auto & selected = unique_candidates[best_index];
    const double dx = selected.x - cursor_x;
    const double dy = selected.y - cursor_y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double segment_x = dx / distance;
    const double segment_y = dy / distance;
    heading_x = segment_x + selected.tangent_x;
    heading_y = segment_y + selected.tangent_y;
    const double heading_norm = std::sqrt(heading_x * heading_x + heading_y * heading_y);
    if (heading_norm > 1e-6) {
      heading_x /= heading_norm;
      heading_y /= heading_norm;
    } else {
      heading_x = segment_x;
      heading_y = segment_y;
    }
    cursor_x = selected.x;
    cursor_y = selected.y;
    ordered_candidates.push_back(selected);
  }

  for (const auto & candidate : ordered_candidates) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = candidate.x;
    wp.pose.pose.position.y = candidate.y;
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
