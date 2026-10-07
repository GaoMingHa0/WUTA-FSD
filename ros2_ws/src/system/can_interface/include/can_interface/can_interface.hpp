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

  // 发送 0x301（工控机→VCU 上线心跳）：标准帧 / DLC=1，
  // Data[0]=0x01 心跳正常（在线）/ 0x00 异常（离线）。
  void sendHeartbeatFrame();
  CanFrame packHeartbeatFrame() const;

  // ---- 报文解析（VCU→工控机 0x501：Byte1=测试模式） ----
  void parseVcuFrame(const CanFrame & frame);  // 测试模式 → mission_mode_cmd
  // 模式保活：周期重发当前档位，防晚启动/重启的 mission_manager 错过单发模式
  void repeatMissionMode();

  // ---- 报文解析（RES→工控机 0x1E4：Byte1=遥控器状态） ----
  // 0x11 遥控器上线 / 0x13 发车按钮被按下 / 0x10 按下急停
  // 发车严格边沿：0x13 变化沿发布一次 /system/start_command，不做窗口重放
  void parseResFrame(const CanFrame & frame);  // 状态 → start_command / emergency
  void publishStartCommand(bool start);
  void publishEmergency(bool emergency);

  // ---- 通信接口 ----
  std::string can_device_{"can0"};      // SocketCAN 接口名
  double poll_interval_sec_{0.02};      // 接收轮询周期

  // ---- 报文 ID ----
  int control_frame_id_{0x210};         // 工控机→VCU 控制帧
  int heartbeat_frame_id_{0x301};       // 工控机→VCU 上线心跳帧
  int vcu_frame_id_{0x501};             // VCU→工控机 模式帧（仅接收）
  int res_frame_id_{0x1E4};             // RES→工控机 遥控器帧（仅接收）

  // ---- 发送周期 ----
  double control_period_sec_{0.1};      // 控制帧周期发送间隔，<=0 关闭周期发送
  double heartbeat_period_sec_{0.05};   // 心跳帧周期发送间隔，<=0 关闭

  // ---- 横向定标 ----
  // Signal2 满量程前轮转角，须与 controller vehicle.max_steer_angle 一致
  double max_steer_angle_{28.0};

  // ---- 模式保活重发 ----
  double mode_repeat_period_sec_{1.0};  // mission_mode_cmd 保活重发周期，<=0 关闭

  // 0x210 帧缓存（Signal3/4 由状态回调更新，随下帧一起发出）
  double throttle_brake_{0.0};   // 纵向开度 [-1,1]（controller 速度 PID 输出）
  double cmd_angle_{0.0};        // 横向转向角（deg）
  bool can_online_{false};       // Signal3：设备自检通过
  bool can_finished_{false};     // Signal4：任务 FINISH
  // 发送门控：收到首份自检结论（devices_inspection）前控制帧都不发，
  // 避免开机默认 Signal3=0 被 VCU 当作自检故障。
  bool tx_armed_{false};

  // 0x501 帧缓存（仅模式变化时发布，去重）
  uint8_t last_vcu_mission_mode_{0xFF};   // 最近一次 VCU 任务模式（Byte1）

  // 0x1E4 帧缓存（电平信号：按当前值处理，仅变化沿触发动作/日志）
  uint8_t last_res_state_{0xFF};          // 最近一次 RES 状态（Byte1）

  // 设备层
  CanSocket can_;

  // 订阅 / 发布
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<wuta_msgs::msg::DevicesInspection>::SharedPtr devices_inspection_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Command>::SharedPtr control_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_command_pub_;  // RES GO 放行
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;      // RES 急停（锁存）
  rclcpp::TimerBase::SharedPtr receive_timer_;      // 接收轮询
  rclcpp::TimerBase::SharedPtr keepalive_timer_;    // 控制帧周期发送
  rclcpp::TimerBase::SharedPtr mode_repeat_timer_;  // 模式保活重发
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;    // 上线心跳周期发送
};

}  // namespace can_interface
