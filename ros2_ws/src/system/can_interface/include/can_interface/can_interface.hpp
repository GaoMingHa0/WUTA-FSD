#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <autoware_msgs/msg/command.hpp>
#include <wuta_msgs/msg/devices_inspection.hpp>
#include <wuta_msgs/msg/mission_state.hpp>

#include "can_interface/can_frame.hpp"
#include "can_interface/can_socket.hpp"

namespace can_interface
{

// 节点层：编排收发、在 ROS 回调里执行发送、定时器轮询接收
// 发送路径：ROS 话题 → pack 编码 → CanSocket → CAN 总线
// 接收路径：CAN 总线 → CanSocket（非阻塞轮询）→ parse 解析 → ROS 话题
class CANInterfaceNode : public rclcpp::Node
{
public:
  CANInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // 发送路径入口（ROS 订阅回调）
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onDevicesInspection(const wuta_msgs::msg::DevicesInspection::SharedPtr msg);
  void onControlCommand(const autoware_msgs::msg::Command::SharedPtr msg);
  // 接收路径入口（定时器轮询）
  void pollReceiver();

  // 发送 0x210（工控机→VCU 单帧）：Signal1 纵向(Byte1-2)、Signal2 横向(Byte3-4)、
  // Signal3 上线(Byte5)、Signal4 完成(Byte6)、Signal5 空(Byte7-8)
  void sendControlFrame();
  CanFrame packControlFrame(double throttle_brake, double steer_deg,
    bool online, bool finished) const;

  // ---- 报文解析（VCU→工控机单帧 0x501：Byte1=VCU状态、Byte2=测试模式） ----
  void parseVcuFrame(const CanFrame & frame);  // 状态→start/emergency，模式→mission_mode_cmd
  // 保活：VCU 处于驾驶态/EMERGENCY 期间周期性重复发布，防启动乱序丢信号
  void repeatVcuSignals();

  // 配置
  std::string can_device_;
  double poll_interval_sec_{0.02};  // 接收轮询周期，默认 50Hz
  double max_steer_deg_{25.0};      // Signal2 满量程转向角（deg），与 controller 一致

  // 0x210 帧缓存（Signal3/4 由状态回调更新，随下帧一起发出）
  double throttle_brake_{0.0};   // 纵向开度 [-1,1]（controller 速度 PID 输出）
  double cmd_angle_{0.0};        // 横向转向角（deg）
  bool can_online_{false};       // Signal3：设备自检通过
  bool can_finished_{false};     // Signal4：任务 FINISH

  // 0x501 帧缓存（仅状态/模式变化时发布，去重）
  uint8_t last_vcu_state_{0xFF};   // 最近一次 VCU 状态（Byte1）
  uint8_t last_test_mode_{0xFF};   // 最近一次测试模式（Byte2）

  // 设备层
  CanSocket can_;

  // 订阅 / 发布
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<wuta_msgs::msg::DevicesInspection>::SharedPtr devices_inspection_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Command>::SharedPtr control_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr keepalive_timer_;  // 无控制指令时的保活帧
  rclcpp::TimerBase::SharedPtr go_heartbeat_timer_;  // 1Hz 信号保活（GO/EMERGENCY 防丢）
};

}  // namespace can_interface
