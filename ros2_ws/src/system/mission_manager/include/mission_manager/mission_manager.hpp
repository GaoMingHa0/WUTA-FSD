#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "wuta_msgs/msg/mission_state.hpp"
#include "wuta_msgs/msg/cone_map.hpp"

namespace mission_manager
{

class MissionManager : public rclcpp::Node
{
public:
  explicit MissionManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using State = wuta_msgs::msg::MissionState;

  // State machine
  uint8_t current_state_{State::IDLE};
  uint8_t mission_mode_{State::MISSION_TRACKDRIVE};
  uint8_t localization_mode_{State::LOC_KISS_ICP};

  void transitionTo(uint8_t new_state);
  void publishState();

  // Normal mission callbacks
  void onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg);
  void onEmergency(const std_msgs::msg::Bool::SharedPtr msg);
  void onMissionModeCmd(const std_msgs::msg::String::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // INSPECTION interface (预留，暂不接其他模块)
  // 触发：向 /system/inspection_trigger 发布 true
  // 功能：传感器自检 + 向 VCU 发送 CAN 测试报文
  // ---------------------------------------------------------------------------
  void onInspectionTrigger(const std_msgs::msg::Bool::SharedPtr msg);
  void runInspection();   // TODO: 实现传感器检查逻辑
  void sendInspectionCAN(); // TODO: 实现 VCU CAN 报文发送

  // System readiness flags
  bool lidar_ready_{false};
  bool localization_ready_{false};
  bool map_closed_{false};

  // Publishers
  rclcpp::Publisher<wuta_msgs::msg::MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspection_result_pub_;  // 预留

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::ConeMap>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inspection_trigger_sub_; // 预留

  // Timer for periodic state broadcast
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace mission_manager
