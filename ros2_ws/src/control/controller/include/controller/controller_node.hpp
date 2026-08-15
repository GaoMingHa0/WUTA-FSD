#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_msgs/msg/lane.hpp>
#include <autoware_msgs/msg/command.hpp>

#include "wuta_msgs/msg/mission_state.hpp"
#include "controller/vehicle_state.hpp"
#include "controller/pure_pursuit.hpp"
#include "controller/twist_filter.hpp"

namespace controller
{

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onWaypoints(const autoware_msgs::msg::Lane::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onEmergency(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();
  void runInspection();
  void finishInspection();
  bool isSamePath(const std::vector<autoware_msgs::msg::Waypoint> & candidate) const;
  double trackdriveLookahead(const rclcpp::Time & loop_time);
  void publishMissionComplete();
  void publishZeroCommand();

  void publishVisualization(double target_x, double target_y);

  // Algorithm objects
  std::unique_ptr<PurePursuit>  pure_pursuit_;
  std::unique_ptr<TwistFilter>  twist_filter_;

  // A figure-eight has tangent-continuous but curvature-discontinuous joins at
  // the timing-line crossing.  It needs a shorter preview than open tracks.
  double skidpad_lookahead_{2.5};
  // Trackdrive derives a bounded preview from upcoming centerline curvature,
  // independent of path_generator's race-speed target.
  bool trackdrive_dynamic_lookahead_{true};
  double trackdrive_lookahead_{5.0};
  double trackdrive_min_lookahead_{3.0};
  double trackdrive_curvature_preview_distance_{12.0};
  double trackdrive_straight_curvature_{0.03};
  double trackdrive_corner_curvature_{0.16};
  double trackdrive_lookahead_rate_limit_{3.0};
  double filtered_trackdrive_lookahead_{5.0};
  rclcpp::Time last_trackdrive_lookahead_time_;
  double trackdrive_target_loss_hold_time_{0.5};
  double trackdrive_target_loss_hold_speed_{2.0};
  // Keep the Trackdrive launch transient slow while the first local map and
  // centreline stabilize. The timer starts at the first valid forward target.
  double trackdrive_start_speed_{3.0};
  double trackdrive_start_speed_duration_{4.0};
  rclcpp::Time trackdrive_start_speed_time_;
  bool trackdrive_start_speed_started_{false};

  // State
  VehicleState vehicle_state_;
  std::vector<autoware_msgs::msg::Waypoint> waypoints_;
  bool pose_ready_{false};
  bool waypoints_ready_{false};
  bool enabled_{false};  // Run while Trackdrive can still make forward progress
  bool emergency_{false};  // 急停命令，置位时持续输出全零命令
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  uint8_t state_{wuta_msgs::msg::MissionState::IDLE};  // 最近一次任务状态
  bool mission_complete_{false};
  double finish_position_tolerance_{0.75};
  double finish_speed_threshold_{0.2};
  ControlCommand last_valid_trackdrive_cmd_;
  rclcpp::Time last_valid_trackdrive_cmd_time_;
  bool last_valid_trackdrive_cmd_ready_{false};

  // 车检模式（INSPECTION）参数与状态
  double inspection_speed_{1.0};      // 慢速驱动速度 m/s
  double inspection_steer_amp_{15.0}; // 正弦波转向幅值 deg
  double inspection_steer_freq_{0.4}; // 正弦波频率 Hz
  double inspection_duration_{10.0};  // 演示时长 s
  rclcpp::Time inspection_start_time_;
  bool inspection_done_published_{false};

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Lane>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;

  // Publishers
  rclcpp::Publisher<autoware_msgs::msg::Command>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_viz_pub_;

  // Control loop timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace controller
