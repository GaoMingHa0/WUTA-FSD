#include "path_generator/path_generator_node.hpp"
#include <cmath>

namespace path_generator
{

using State = wuta_msgs::msg::MissionState;

PathGeneratorNode::PathGeneratorNode(const rclcpp::NodeOptions & options)
: Node("path_generator_node", options)
{
  trackdrive_velocity_    = declare_parameter("trackdrive_velocity",    trackdrive_velocity_);
  skidpad_radius_         = declare_parameter("skidpad_radius",         skidpad_radius_);
  skidpad_velocity_       = declare_parameter("skidpad_velocity",       skidpad_velocity_);
  skidpad_points_         = declare_parameter("skidpad_points",         skidpad_points_);
  acceleration_length_    = declare_parameter("acceleration_length",    acceleration_length_);
  acceleration_velocity_  = declare_parameter("acceleration_velocity",  acceleration_velocity_);

  // Core subscriptions
  mission_sub_ = create_subscription<State>(
    "/system/mission_state", 10,
    std::bind(&PathGeneratorNode::onMissionState, this, std::placeholders::_1));

  centerline_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/centerline", 10,
    std::bind(&PathGeneratorNode::onCenterline, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&PathGeneratorNode::onPose, this, std::placeholders::_1));

  // Detector subscriptions (optional, improve accuracy for skidpad/acceleration)
  skidpad_circles_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "/planning/skidpad_circles", 10,
    std::bind(&PathGeneratorNode::onSkidpadCircles, this, std::placeholders::_1));

  accel_line_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/acceleration_line", 10,
    std::bind(&PathGeneratorNode::onAccelerationLine, this, std::placeholders::_1));

  waypoints_pub_ = create_publisher<autoware_msgs::msg::Lane>("/planning/final_waypoints", 10);

  RCLCPP_INFO(get_logger(), "PathGeneratorNode ready.");
}

void PathGeneratorNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  pose_ready_ = true;
}

void PathGeneratorNode::onSkidpadCircles(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.size() >= 2) {
    skidpad_circles_ = *msg;
    skidpad_circles_ready_ = true;
  }
}

void PathGeneratorNode::onAccelerationLine(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  acceleration_endpoint_ = *msg;
  acceleration_line_ready_ = true;
}

void PathGeneratorNode::onMissionState(const State::SharedPtr msg)
{
  mission_mode_ = msg->mission_mode;
  system_state_ = msg->state;

  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;
  if (!pose_ready_) return;

  if (mission_mode_ == State::MISSION_SKIDPAD) {
    auto lane = generateSkidpadPath();
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
  } else if (mission_mode_ == State::MISSION_ACCELERATION) {
    auto lane = generateAccelerationPath();
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
  }
  // TRACKDRIVE: forwarded by onCenterline
}

void PathGeneratorNode::onCenterline(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  if (mission_mode_ != State::MISSION_TRACKDRIVE) return;
  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;

  auto lane = *msg;
  for (auto & wp : lane.waypoints) {
    wp.twist.twist.linear.x = trackdrive_velocity_;
  }
  waypoints_pub_->publish(lane);
}

autoware_msgs::msg::Lane PathGeneratorNode::generateSkidpadPath() const
{
  autoware_msgs::msg::Lane lane;

  const double z   = current_pose_.pose.position.z;
  const double d_theta = 2.0 * M_PI / skidpad_points_;

  double right_cx, right_cy, left_cx, left_cy;

  if (skidpad_circles_ready_) {
    // Use circle centres from skidpad_detector (cone-fitted)
    right_cx = skidpad_circles_.poses[0].position.x;
    right_cy = skidpad_circles_.poses[0].position.y;
    left_cx  = skidpad_circles_.poses[1].position.x;
    left_cy  = skidpad_circles_.poses[1].position.y;
    RCLCPP_INFO(get_logger(), "Skidpad: using cone-fitted circle centres.");
  } else {
    // Fallback: estimate from vehicle pose + heading
    const double cx = current_pose_.pose.position.x;
    const double cy = current_pose_.pose.position.y;
    const auto & q  = current_pose_.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    right_cx = cx + skidpad_radius_ * std::sin(yaw);
    right_cy = cy - skidpad_radius_ * std::cos(yaw);
    left_cx  = cx - skidpad_radius_ * std::sin(yaw);
    left_cy  = cy + skidpad_radius_ * std::cos(yaw);
    RCLCPP_WARN(get_logger(), "Skidpad: detector not ready, using pose-based circle centres.");
  }

  // Two laps right circle (FSG: enter from south, CW), then two laps left circle (CCW)
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 0; i <= skidpad_points_; ++i) {
      const double theta = -M_PI_2 + i * d_theta;
      autoware_msgs::msg::Waypoint wp;
      wp.pose.pose.position.x = right_cx + skidpad_radius_ * std::cos(theta);
      wp.pose.pose.position.y = right_cy + skidpad_radius_ * std::sin(theta);
      wp.pose.pose.position.z = z;
      wp.pose.pose.orientation.w = 1.0;
      wp.twist.twist.linear.x = skidpad_velocity_;
      lane.waypoints.push_back(wp);
    }
  }
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 0; i <= skidpad_points_; ++i) {
      const double theta = -M_PI_2 - i * d_theta;
      autoware_msgs::msg::Waypoint wp;
      wp.pose.pose.position.x = left_cx + skidpad_radius_ * std::cos(theta);
      wp.pose.pose.position.y = left_cy + skidpad_radius_ * std::sin(theta);
      wp.pose.pose.position.z = z;
      wp.pose.pose.orientation.w = 1.0;
      wp.twist.twist.linear.x = skidpad_velocity_;
      lane.waypoints.push_back(wp);
    }
  }

  RCLCPP_INFO(get_logger(), "Skidpad path generated: %zu waypoints.", lane.waypoints.size());
  return lane;
}

autoware_msgs::msg::Lane PathGeneratorNode::generateAccelerationPath() const
{
  autoware_msgs::msg::Lane lane;

  const double sx = current_pose_.pose.position.x;
  const double sy = current_pose_.pose.position.y;
  const double z  = current_pose_.pose.position.z;

  double dx, dy, total_length;

  if (acceleration_line_ready_) {
    // Use endpoint from line_detector (PCA on cone map)
    const double ex = acceleration_endpoint_.pose.position.x;
    const double ey = acceleration_endpoint_.pose.position.y;
    total_length = std::sqrt((ex-sx)*(ex-sx) + (ey-sy)*(ey-sy));
    dx = (ex - sx) / total_length;
    dy = (ey - sy) / total_length;
    RCLCPP_INFO(get_logger(), "Acceleration: cone-detected endpoint, length=%.1fm.", total_length);
  } else {
    // Fallback: fixed length along vehicle heading
    const auto & q  = current_pose_.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    dx = std::cos(yaw);
    dy = std::sin(yaw);
    total_length = acceleration_length_;
    RCLCPP_WARN(get_logger(), "Acceleration: detector not ready, using fixed %.0fm.", total_length);
  }

  const int num_points = static_cast<int>(total_length);
  for (int i = 0; i <= num_points; ++i) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = sx + i * dx;
    wp.pose.pose.position.y = sy + i * dy;
    wp.pose.pose.position.z = z;
    wp.pose.pose.orientation.w = 1.0;
    const double remaining = total_length - i;
    wp.twist.twist.linear.x = (remaining < 10.0)
      ? acceleration_velocity_ * (remaining / 10.0)
      : acceleration_velocity_;
    lane.waypoints.push_back(wp);
  }

  RCLCPP_INFO(get_logger(), "Acceleration path generated: %zu waypoints.", lane.waypoints.size());
  return lane;
}

}  // namespace path_generator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_generator::PathGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
