#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <wuta_msgs/msg/devices_inspection.hpp>
#include <wuta_msgs/msg/mission_state.hpp>

#include "can_interface/can_frame.hpp"
#include "can_interface/can_receiver.hpp"
#include "can_interface/can_sender.hpp"

namespace can_interface
{

// 节点层：编排收发、在 ROS 回调里执行发送、定时器轮询接收
// 发送路径：ROS 话题 → pack 编码（TODO）→ CanSender → CAN 总线
// 接收路径：CAN 总线 → CanReceiver（非阻塞轮询）→ parse 解析（TODO）→ ROS 话题
class CANInterfaceNode : public rclcpp::Node
{
public:
  CANInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // 发送路径入口（ROS 订阅回调）
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onDevicesInspection(const wuta_msgs::msg::DevicesInspection::SharedPtr msg);
  // 接收路径入口（定时器轮询）
  void pollReceiver();

  // ---- 报文编码/解析（各字节格式未确定，先预留） ----
  CanFrame packMissionState(const wuta_msgs::msg::MissionState & msg);          // TODO
  CanFrame packDevicesInspection(const wuta_msgs::msg::DevicesInspection & msg);  // TODO
  void parseVcuFrame(const CanFrame & frame);  // TODO

  // 配置
  std::string can_device_;
  double poll_interval_sec_{0.02};  // 接收轮询周期，默认 50Hz

  // 设备层
  CanSender sender_;
  CanReceiver receiver_;

  // 订阅 / 发布
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<wuta_msgs::msg::DevicesInspection>::SharedPtr devices_inspection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
};

}  // namespace can_interface
