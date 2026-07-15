#include "controller/controller_node.hpp"
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>

namespace controller
{

using MissionState = wuta_msgs::msg::MissionState;

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  // --- Vehicle parameters ---
  VehicleParams vp;
  vp.wheel_base       = declare_parameter("wheel_base",       vp.wheel_base);
  vp.lf               = declare_parameter("lf",               vp.lf);
  vp.max_steer_angle  = declare_parameter("max_steer_angle",  vp.max_steer_angle);

  // --- Pure Pursuit config ---
  PurePursuit::Config pp_cfg;
  pp_cfg.ld_ratio       = declare_parameter("ld_ratio",       pp_cfg.ld_ratio);
  pp_cfg.min_lookahead  = declare_parameter("min_lookahead",  pp_cfg.min_lookahead);
  pp_cfg.max_lookahead  = declare_parameter("max_lookahead",  pp_cfg.max_lookahead);
  pp_cfg.max_progress_advance = declare_parameter(
    "max_progress_advance", pp_cfg.max_progress_advance);
  skidpad_lookahead_ = declare_parameter("skidpad_lookahead", skidpad_lookahead_);

  pure_pursuit_ = std::make_unique<PurePursuit>(vp, pp_cfg);
  twist_filter_ = std::make_unique<TwistFilter>(vp);

  // --- Control loop rate ---
  const int rate_hz = declare_parameter("control_rate_hz", 50);
  finish_position_tolerance_ = declare_parameter(
    "finish_position_tolerance", finish_position_tolerance_);
  finish_speed_threshold_ = declare_parameter(
    "finish_speed_threshold", finish_speed_threshold_);

  // --- Subscribers ---
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&ControllerNode::onPose, this, std::placeholders::_1));

  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/localization/velocity", 10,
    std::bind(&ControllerNode::onVelocity, this, std::placeholders::_1));

  waypoints_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/final_waypoints", 10,
    std::bind(&ControllerNode::onWaypoints, this, std::placeholders::_1));

  mission_sub_ = create_subscription<MissionState>(
    "/system/mission_state", 10,
    std::bind(&ControllerNode::onMissionState, this, std::placeholders::_1));

  // --- Publishers ---
  cmd_pub_ = create_publisher<autoware_msgs::msg::Command>("/control/command", 10);
  mission_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/mission_complete", 10);
  target_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/control/target_viz", 10);

  // --- Control loop timer ---
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / rate_hz),
    std::bind(&ControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "ControllerNode ready. rate=%dHz, LD_ratio=%.1f",
    rate_hz, pp_cfg.ld_ratio);
}

void ControllerNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  vehicle_state_.x = msg->pose.position.x;
  vehicle_state_.y = msg->pose.position.y;

  const auto & q = msg->pose.orientation;
  vehicle_state_.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  pose_ready_ = true;
}

void ControllerNode::onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Magnitude of velocity vector
  const double vx = msg->twist.linear.x;
  const double vy = msg->twist.linear.y;
  vehicle_state_.velocity = std::sqrt(vx * vx + vy * vy);
}

void ControllerNode::onWaypoints(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  const bool changed = !isSamePath(msg->waypoints);
  waypoints_ = msg->waypoints;
  waypoints_ready_ = !waypoints_.empty();
  if (changed) {
    pure_pursuit_->reset();
    mission_complete_ = false;
  }
}

void ControllerNode::onMissionState(const MissionState::SharedPtr msg)
{
  mission_mode_ = msg->mission_mode;
  enabled_ = (msg->state == MissionState::EXPLORE || msg->state == MissionState::RACE);

  if (!enabled_) {
    twist_filter_->reset();
    // Publish stop command
    autoware_msgs::msg::Command stop;
    stop.speed = 0.0;
    stop.angle = 0.0;
    stop.dv_state = 4;
    cmd_pub_->publish(stop);
  }
}

void ControllerNode::controlLoop()
{
  if (!enabled_ || !pose_ready_ || !waypoints_ready_) return;

  if (mission_complete_) return;

  // 1. Pure Pursuit
  // At 5 m/s the generic LD=v*2 would preview 10 m, almost one skidpad
  // radius.  At the entry, circle transition, and exit this selects a point
  // from the following path segment and makes the bicycle model cut inward or
  // unload steering before the crossing.  Keep that behaviour for other
  // missions, but use the calibrated local preview for skidpad.
  const double lookahead_override =
    mission_mode_ == MissionState::MISSION_SKIDPAD ? skidpad_lookahead_ : 0.0;
  auto raw_cmd = pure_pursuit_->compute(
    vehicle_state_, waypoints_, lookahead_override);

  if (mission_mode_ == MissionState::MISSION_SKIDPAD &&
      pure_pursuit_->progressIndex() == static_cast<int>(waypoints_.size()) - 1 &&
      std::hypot(
        waypoints_.back().pose.pose.position.x - vehicle_state_.x,
        waypoints_.back().pose.pose.position.y - vehicle_state_.y) <= finish_position_tolerance_ &&
      vehicle_state_.velocity <= finish_speed_threshold_)
  {
    publishMissionComplete();
    return;
  }

  if (!raw_cmd.valid) return;

  // 2. Safety filter
  auto filtered = twist_filter_->filter(raw_cmd.steering_angle, raw_cmd.velocity);

  // 3. Publish command
  autoware_msgs::msg::Command cmd;
  cmd.speed    = filtered.velocity;
  cmd.angle    = filtered.steering_angle;
  cmd.dv_state = filtered.emergency ? 6 : 4;  // 4=normal, 6=emergency
  cmd_pub_->publish(cmd);

  // 4. Visualization (target waypoint marker)
  if (target_viz_pub_->get_subscription_count() > 0 &&
      pure_pursuit_->targetIndex() < static_cast<int>(waypoints_.size()))
  {
    const auto & wp = waypoints_[pure_pursuit_->targetIndex()];
    publishVisualization(
      wp.pose.pose.position.x,
      wp.pose.pose.position.y);
  }
}

bool ControllerNode::isSamePath(
  const std::vector<autoware_msgs::msg::Waypoint> & candidate) const
{
  if (candidate.size() != waypoints_.size()) return false;
  for (size_t i = 0; i < candidate.size(); ++i) {
    const auto & lhs = candidate[i].pose.pose.position;
    const auto & rhs = waypoints_[i].pose.pose.position;
    if (std::abs(lhs.x - rhs.x) > 1e-6 || std::abs(lhs.y - rhs.y) > 1e-6 ||
        std::abs(lhs.z - rhs.z) > 1e-6) {
      return false;
    }
  }
  return true;
}

void ControllerNode::publishMissionComplete()
{
  mission_complete_ = true;
  enabled_ = false;
  twist_filter_->reset();

  autoware_msgs::msg::Command stop;
  stop.speed = 0.0;
  stop.angle = 0.0;
  stop.dv_state = 4;
  cmd_pub_->publish(stop);

  std_msgs::msg::Bool complete;
  complete.data = true;
  mission_complete_pub_->publish(complete);
  RCLCPP_INFO(
    get_logger(),
    "Skidpad complete: progress=%d/%zu pose=(%.3f, %.3f) speed=%.3f m/s",
    pure_pursuit_->progressIndex(), waypoints_.size() - 1,
    vehicle_state_.x, vehicle_state_.y, vehicle_state_.velocity);
}

void ControllerNode::publishVisualization(double target_x, double target_y)
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp    = now();
  m.ns     = "pp_target";
  m.id     = 0;
  m.type   = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x  = target_x;
  m.pose.position.y  = target_y;
  m.pose.position.z  = 0.5;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
  m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 1.0f;
  arr.markers.push_back(m);

  // Lookahead circle
  visualization_msgs::msg::Marker circle;
  circle.header = m.header;
  circle.ns   = "pp_lookahead";
  circle.id   = 1;
  circle.type = visualization_msgs::msg::Marker::CYLINDER;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.pose.position.x = vehicle_state_.x;
  circle.pose.position.y = vehicle_state_.y;
  circle.pose.position.z = 0.0;
  circle.pose.orientation.w = 1.0;
  const double ld = pure_pursuit_->lookaheadDistance();
  circle.scale.x = ld * 2; circle.scale.y = ld * 2; circle.scale.z = 0.05;
  circle.color.r = 0.0f; circle.color.g = 0.6f; circle.color.b = 1.0f; circle.color.a = 0.3f;
  arr.markers.push_back(circle);

  target_viz_pub_->publish(arr);
}

}  // namespace controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
