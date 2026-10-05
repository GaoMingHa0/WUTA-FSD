#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
  void publishZeroCommand(const char * tag = "");
  // 统一控制日志格式：正常路径与零指令路径共用同一格式串，保证逐行对齐；
  // 节流放在各调用点，避免两路共用节流互相顶掉。
  std::string controlLine(double raw_angle, double raw_vel,
                          double cmd_angle, double cmd_vel, double cmd_thr,
                          const char * tag);

  // 速度 PID（目标速度 → 油门/刹车开度）
  double computeSpeedPid(double target_speed);
  double computeThrottleBrake(double target_speed);

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
  double inspection_speed_{1.0};      // 车检名义车速 m/s：仅用于 TwistFilter 与日志，不决定开度
  double inspection_throttle_{0.16};  // 车检恒定纵向开度 [0,1]：唯一转速旋钮，不走 PID
  double inspection_steer_amp_{5.7692}; // 正弦波转向幅值 deg（前轮；= 方向盘 ±30° ÷ 转向比 5.2）
  double inspection_steer_period_{9.0}; // 正弦波周期 s（>0 时优先，覆盖下面的 freq）
  double inspection_steer_freq_{0.25};  // 正弦波频率 Hz（兼容旧参数；仅 period<=0 时生效）
  double inspection_duration_{27.0};  // 车检时长 s（= 6.0s × 4.5 个周期，半整数同样过零）
  rclcpp::Time inspection_start_time_;
  bool inspection_done_published_{false};

  // 速度 PID（纵向开度 → /control/command.throttle_brake）
  bool speed_feedback_available_{false};
  double pid_speed_kp_{1.0};
  double pid_speed_ki_{0.05};
  double pid_speed_kd_{0.1};
  // 停车消积分判据阈值：TwistFilter 输出的目标速度渐近趋 0 但永不为 0，
  // 用严格 `target <= 0` 会导致停车后残留积分开度（溜车），故按阈值判定
  double pid_stop_clear_eps_{0.05};
  double pid_integral_{0.0};
  double pid_prev_err_{0.0};
  rclcpp::Time pid_last_time_;

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
