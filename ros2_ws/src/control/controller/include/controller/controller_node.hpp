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
  // 单个赛项的横向（Pure Pursuit）+ 纵向（PID）+ 终点调参
  struct MissionConfig
  {
    PurePursuit::Config pp;
    double pid_kp{1.0};
    double pid_ki{0.05};
    double pid_kd{0.1};
    double pid_stop_clear_eps{0.05};
    double pid_stop_clear_speed{0.5};
    double pid_output_limit{1.0};
    double pid_integral_limit{1.0};
    bool   has_finish{false};
    double finish_position_tolerance{0.75};
    double finish_speed_threshold{0.2};
  };

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
  std::string controlLine(double raw_angle, double raw_vel,
                          double cmd_angle, double cmd_vel, double cmd_thr,
                          const char * tag);

  // 参数加载与赛项切换
  void loadMissionConfig();
  void applyMissionConfig(uint8_t mission_mode);
  const MissionConfig & configFor(uint8_t mission_mode) const;

  // 控制状态复位
  void resetSpeedPid();
  void resetControlPipeline();

  // 速度 PID（目标速度 → 油门/刹车开度）
  double computeSpeedPid(double target_speed);
  double computeThrottleBrake(double target_speed);

  void publishVisualization(double target_x, double target_y);

  // 算法对象
  std::unique_ptr<PurePursuit>  pure_pursuit_;
  std::unique_ptr<TwistFilter>  twist_filter_;

  // 各赛项独立调参
  MissionConfig accel_cfg_;
  MissionConfig ebs_cfg_;
  MissionConfig skidpad_cfg_;
  MissionConfig trackdrive_cfg_;
  double skidpad_lookahead_{2.5};  // 八字固定前视 m

  // 高速循迹曲率自适应前视
  bool   trackdrive_dynamic_lookahead_{true};
  double trackdrive_lookahead_{5.0};
  double trackdrive_min_lookahead_{3.0};
  double trackdrive_curvature_preview_distance_{12.0};
  double trackdrive_straight_curvature_{0.03};
  double trackdrive_corner_curvature_{0.16};
  double trackdrive_lookahead_rate_limit_{3.0};
  double curvature_percentile_{0.75};
  double curvature_peak_factor_{0.6};
  double filtered_trackdrive_lookahead_{5.0};
  rclcpp::Time last_trackdrive_lookahead_time_;
  double trackdrive_target_loss_hold_time_{0.5};
  double trackdrive_target_loss_hold_speed_{2.0};
  double trackdrive_start_speed_{3.0};
  double trackdrive_start_speed_duration_{4.0};
  rclcpp::Time trackdrive_start_speed_time_;
  bool trackdrive_start_speed_started_{false};

  // 状态
  VehicleState vehicle_state_;
  std::vector<autoware_msgs::msg::Waypoint> waypoints_;
  bool pose_ready_{false};
  bool waypoints_ready_{false};
  bool enabled_{false};
  bool emergency_{false};
  uint8_t mission_mode_{wuta_msgs::msg::MissionState::MISSION_TRACKDRIVE};
  uint8_t state_{wuta_msgs::msg::MissionState::IDLE};
  bool mission_complete_{false};
  ControlCommand last_valid_trackdrive_cmd_;
  rclcpp::Time last_valid_trackdrive_cmd_time_;
  bool last_valid_trackdrive_cmd_ready_{false};

  // 车检参数与状态
  double inspection_throttle_{0.05};   // 车检恒定纵向开度[0,1]
  double inspection_steer_amp_{6.0};     // deg，前轮正弦转向幅值
  double inspection_steer_period_{9.0};  // s，正弦转向周期
  double inspection_steer_freq_{0.25};   // Hz，由周期与时长推出
  double inspection_duration_{27.0};     // s，车检时长
  rclcpp::Time inspection_start_time_;
  bool inspection_done_published_{false};

  // 速度 PID（当前赛项生效的增益）
  bool speed_feedback_available_{false};
  double pid_speed_kp_{1.0};
  double pid_speed_ki_{0.05};
  double pid_speed_kd_{0.1};
  double pid_stop_clear_eps_{0.05};
  double pid_stop_clear_speed_{0.5};
  double pid_output_limit_{1.0};
  double pid_integral_limit_{1.0};
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
