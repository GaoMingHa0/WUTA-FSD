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

  // Subscribers
  mission_sub_ = create_subscription<State>(
    "/system/mission_state", 10,
    std::bind(&PathGeneratorNode::onMissionState, this, std::placeholders::_1));

  centerline_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/centerline", 10,
    std::bind(&PathGeneratorNode::onCenterline, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&PathGeneratorNode::onPose, this, std::placeholders::_1));

  // Publisher — final_waypoints consumed by controller
  waypoints_pub_ = create_publisher<autoware_msgs::msg::Lane>("/planning/final_waypoints", 10);

  RCLCPP_INFO(get_logger(), "PathGeneratorNode ready.");
}

void PathGeneratorNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  pose_ready_ = true;
}

void PathGeneratorNode::onMissionState(const State::SharedPtr msg)
{
  mission_mode_  = msg->mission_mode;
  system_state_  = msg->state;

  // Trigger non-trackdrive paths when system is active
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
  // TRACKDRIVE: forwarded by onCenterline callback
}

void PathGeneratorNode::onCenterline(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  // Only forward trackdrive centerline
  if (mission_mode_ != State::MISSION_TRACKDRIVE) return;
  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;

  // Update velocity for trackdrive
  auto lane = *msg;
  for (auto & wp : lane.waypoints) {
    wp.twist.twist.linear.x = trackdrive_velocity_;
  }
  waypoints_pub_->publish(lane);
}

autoware_msgs::msg::Lane PathGeneratorNode::generateSkidpadPath() const
{
  autoware_msgs::msg::Lane lane;

  // FSG Skidpad: two circles of radius 9.125m
  // Right circle first (standard FSG direction), then left circle
  // Start at vehicle position
  const double cx = current_pose_.pose.position.x;
  const double cy = current_pose_.pose.position.y;
  const double z  = current_pose_.pose.position.z;

  // Vehicle heading
  const auto & q = current_pose_.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Circle centers: perpendicular to heading, offset by radius
  const double right_cx = cx + skidpad_radius_ * std::sin(yaw);
  const double right_cy = cy - skidpad_radius_ * std::cos(yaw);
  const double left_cx  = cx - skidpad_radius_ * std::sin(yaw);
  const double left_cy  = cy + skidpad_radius_ * std::cos(yaw);

  const double d_theta = 2.0 * M_PI / skidpad_points_;

  // Two laps right circle, two laps left circle (FSG rules)
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 0; i <= skidpad_points_; ++i) {
      const double theta = -M_PI_2 + i * d_theta;  // Start from bottom of circle
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
      const double theta = -M_PI_2 - i * d_theta;  // Opposite direction
      autoware_msgs::msg::Waypoint wp;
      wp.pose.pose.position.x = left_cx + skidpad_radius_ * std::cos(theta);
      wp.pose.pose.position.y = left_cy + skidpad_radius_ * std::sin(theta);
      wp.pose.pose.position.z = z;
      wp.pose.pose.orientation.w = 1.0;
      wp.twist.twist.linear.x = skidpad_velocity_;
      lane.waypoints.push_back(wp);
    }
  }

  RCLCPP_INFO(get_logger(), "Skidpad path generated: %zu waypoints", lane.waypoints.size());
  return lane;
}

autoware_msgs::msg::Lane PathGeneratorNode::generateAccelerationPath() const
{
  autoware_msgs::msg::Lane lane;

  const double cx  = current_pose_.pose.position.x;
  const double cy  = current_pose_.pose.position.y;
  const double z   = current_pose_.pose.position.z;

  // Vehicle heading direction
  const auto & q = current_pose_.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double dx = std::cos(yaw);
  const double dy = std::sin(yaw);

  // Waypoints every 1m along straight line
  const int num_points = static_cast<int>(acceleration_length_);
  for (int i = 0; i <= num_points; ++i) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = cx + i * dx;
    wp.pose.pose.position.y = cy + i * dy;
    wp.pose.pose.position.z = z;
    wp.pose.pose.orientation.w = 1.0;
    // Ramp down velocity in last 10m
    const double remaining = acceleration_length_ - i;
    wp.twist.twist.linear.x = (remaining < 10.0)
      ? acceleration_velocity_ * (remaining / 10.0)
      : acceleration_velocity_;
    lane.waypoints.push_back(wp);
  }

  RCLCPP_INFO(get_logger(), "Acceleration path generated: %zu waypoints", lane.waypoints.size());
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
