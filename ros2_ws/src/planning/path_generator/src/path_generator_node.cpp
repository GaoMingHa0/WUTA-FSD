#include "path_generator/path_generator_node.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

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
  skidpad_start_x_        = declare_parameter("skidpad_start_x",        skidpad_start_x_);
  skidpad_start_y_        = declare_parameter("skidpad_start_y",        skidpad_start_y_);
  skidpad_start_yaw_      = declare_parameter("skidpad_start_yaw",      skidpad_start_yaw_);
  skidpad_entry_x_        = declare_parameter("skidpad_entry_x",        skidpad_entry_x_);
  skidpad_entry_y_        = declare_parameter("skidpad_entry_y",        skidpad_entry_y_);
  skidpad_exit_length_    = declare_parameter("skidpad_exit_length",    skidpad_exit_length_);
  skidpad_braking_distance_ = declare_parameter(
    "skidpad_braking_distance", skidpad_braking_distance_);
  skidpad_csv_path_       = declare_parameter("skidpad_csv_path",       skidpad_csv_path_);
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

  if (mission_mode_ == State::MISSION_SKIDPAD) {
    if (!skidpad_path_ready_) {
      skidpad_path_ = generateSkidpadPath();
      skidpad_path_ready_ = true;
    }
    auto lane = skidpad_path_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
  } else if (mission_mode_ == State::MISSION_ACCELERATION) {
    if (!pose_ready_) return;
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

void PathGeneratorNode::exportSkidpadCsv(const std::vector<SkidpadCsvRow> & rows) const
{
  if (skidpad_csv_path_.empty()) return;

  std::ofstream stream(skidpad_csv_path_);
  if (!stream.is_open()) {
    RCLCPP_ERROR(get_logger(), "Unable to write skidpad CSV: %s", skidpad_csv_path_.c_str());
    return;
  }

  stream << "index,phase,lap,x_m,y_m,yaw_rad,target_speed_mps\n";
  stream << std::fixed << std::setprecision(6);
  for (std::size_t index = 0; index < rows.size(); ++index) {
    const auto & row = rows[index];
    stream << index << ',' << row.phase << ',' << row.lap << ','
           << row.x << ',' << row.y << ',' << row.yaw << ',' << row.velocity << '\n';
  }
  RCLCPP_INFO(get_logger(), "Skidpad trajectory CSV: %s (%zu rows)",
    skidpad_csv_path_.c_str(), rows.size());
}

autoware_msgs::msg::Lane PathGeneratorNode::generateSkidpadPath() const
{
  autoware_msgs::msg::Lane lane;
  std::vector<SkidpadCsvRow> csv_rows;

  // The track is fixed in map, not regenerated from the moving vehicle pose.
  // At yaw=0 the crossing is (0, 0), the right circle is below it and the
  // left circle above it, matching perception_simulation/tracks/skidpad.yaml.
  const double c = std::cos(skidpad_start_yaw_);
  const double s = std::sin(skidpad_start_yaw_);
  const auto to_map = [this, c, s](double local_x, double local_y, double local_yaw,
                                    autoware_msgs::msg::Waypoint & wp) {
    wp.pose.pose.position.x = skidpad_start_x_ + local_x * c - local_y * s;
    wp.pose.pose.position.y = skidpad_start_y_ + local_x * s + local_y * c;
    wp.pose.pose.position.z = 0.0;
    const double yaw = skidpad_start_yaw_ + local_yaw;
    wp.pose.pose.orientation.z = std::sin(yaw * 0.5);
    wp.pose.pose.orientation.w = std::cos(yaw * 0.5);
  };

  const auto append_waypoint = [&lane, &csv_rows, &to_map, this](
    double local_x, double local_y, double local_yaw, double velocity,
    const std::string & phase, int lap) {
      autoware_msgs::msg::Waypoint wp;
      to_map(local_x, local_y, local_yaw, wp);
      wp.twist.twist.linear.x = velocity;
      lane.waypoints.push_back(wp);
      csv_rows.push_back({phase, lap, wp.pose.pose.position.x, wp.pose.pose.position.y,
        skidpad_start_yaw_ + local_yaw, velocity});
    };

  const int circle_points = std::max(8, skidpad_points_);
  const double d_theta = 2.0 * M_PI / circle_points;

  // FSAC: the vehicle starts 15 m before the timing line and enters in the
  // same direction as the eventual exit.  Include the straight explicitly so
  // the controller never shortcuts from the staging point to a circle.
  const double entry_length = std::hypot(skidpad_entry_x_, skidpad_entry_y_);
  const int entry_segments = std::max(1, static_cast<int>(std::ceil(entry_length)));
  for (int i = 0; i <= entry_segments; ++i) {
    const double ratio = static_cast<double>(i) / entry_segments;
    append_waypoint(skidpad_entry_x_ * (1.0 - ratio),
      skidpad_entry_y_ * (1.0 - ratio), skidpad_entry_y_ == 0.0 ? 0.0 :
      std::atan2(-skidpad_entry_y_, -skidpad_entry_x_), skidpad_velocity_, "entry", 0);
  }

  // The first right lap establishes steering, the second is timed.  Start at
  // i=1 because the entry already contributes the crossing waypoint; each
  // subsequent phase similarly reuses only the preceding phase's endpoint.
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 1; i <= circle_points; ++i) {
      const double theta = M_PI_2 - i * d_theta;  // clockwise, starts at crossing
      append_waypoint(skidpad_radius_ * std::cos(theta),
        -skidpad_radius_ + skidpad_radius_ * std::sin(theta),
        std::atan2(-std::cos(theta), std::sin(theta)), skidpad_velocity_,
        "right_circle", lap + 1);
    }
  }

  // Third lap enters the left circle; the fourth is timed.  Counter-clockwise
  // travel preserves the +x crossing direction.
  for (int lap = 0; lap < 2; ++lap) {
    for (int i = 1; i <= circle_points; ++i) {
      const double theta = -M_PI_2 + i * d_theta;  // counter-clockwise
      append_waypoint(skidpad_radius_ * std::cos(theta),
        skidpad_radius_ + skidpad_radius_ * std::sin(theta),
        std::atan2(std::cos(theta), -std::sin(theta)), skidpad_velocity_,
        "left_circle", lap + 3);
    }
  }

  // Leave the crossing in the same direction as entry and stop at 25 m.
  // The final braking segment gives the controller a decreasing speed target.
  for (int i = 1; i <= static_cast<int>(std::ceil(skidpad_exit_length_)); ++i) {
    const double distance = std::min(static_cast<double>(i), skidpad_exit_length_);
    const double remaining = skidpad_exit_length_ - distance;
    const double velocity = remaining < skidpad_braking_distance_
      ? skidpad_velocity_ * remaining / skidpad_braking_distance_
      : skidpad_velocity_;
    append_waypoint(distance, 0.0, 0.0, velocity, "exit", 0);
  }

  exportSkidpadCsv(csv_rows);

  RCLCPP_INFO(get_logger(),
    "Fixed skidpad path generated: %.1f m entry, right lap 1/2, left lap 3/4, %.1f m exit (%zu waypoints)",
    entry_length, skidpad_exit_length_, lane.waypoints.size());
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
