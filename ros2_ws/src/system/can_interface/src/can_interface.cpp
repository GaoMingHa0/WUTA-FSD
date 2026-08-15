#include "can_interface/can_interface.hpp"

namespace can_interface
{

CANInterfaceNode::CANInterfaceNode(const rclcpp::NodeOptions & options)
: Node("can_interface", options)
{
  can_device_ = declare_parameter<std::string>("can_device", "can0");
  // can_baud_rate 由系统侧 ip link 设置，此处仅预留声明
  declare_parameter<int>("can_baud_rate", 500000);
  declare_parameter<bool>("can_loopback", false);
  poll_interval_sec_ = declare_parameter<double>("poll_interval_sec", 0.02);

  // 打开 CAN 接口；失败时静默降级，节点继续运行（收不到/发不出由上层兜底）
  const bool opened = sender_.open(can_device_) && receiver_.open(can_device_);
  if (!opened) {
    RCLCPP_WARN(get_logger(), "CAN device '%s' unavailable, run in degraded mode.",
      can_device_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "CAN interface opened on '%s'.", can_device_.c_str());
  }

  // 发送路径：ROS 订阅回调驱动
  mission_state_sub_ = create_subscription<wuta_msgs::msg::MissionState>(
    "/system/mission_state", 10,
    std::bind(&CANInterfaceNode::onMissionState, this, std::placeholders::_1));

  devices_inspection_sub_ = create_subscription<wuta_msgs::msg::DevicesInspection>(
    "/system/devices_inspection", 10,
    std::bind(&CANInterfaceNode::onDevicesInspection, this, std::placeholders::_1));

  // 接收路径：定时器轮询（非阻塞）
  receive_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(poll_interval_sec_ * 1000.0)),
    std::bind(&CANInterfaceNode::pollReceiver, this));

  // 接收方向解析结果发布（VCU→工控机帧，解析 TODO 后使用）
  mission_mode_cmd_pub_ = create_publisher<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10);
  start_command_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/start_command", 10);
  emergency_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/emergency", 10);
  inspection_trigger_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/inspection_trigger", 10);
  velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/localization/velocity", 50);

  RCLCPP_INFO(get_logger(), "CAN Interface initialized (tx/rx separated).");
  RCLCPP_INFO(get_logger(), "Note: CAN frame byte layout not defined yet, encode/parse TODO.");
}

void CANInterfaceNode::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Received mission_state: state=%d mode=%d",
    msg->state, msg->mission_mode);
  // 工控机→VCU 单帧：编码未实现，暂不发送
  // sender_.send(packMissionState(*msg));
}

void CANInterfaceNode::onDevicesInspection(
  const wuta_msgs::msg::DevicesInspection::SharedPtr msg)
{
  RCLCPP_WARN(get_logger(), "Received devices_inspection: ok=%d failures=%zu",
    msg->ok, msg->failures.size());
  // TODO: 编码为 CAN 报文（Signal3=0）通知 VCU，字节格式未定
  // sender_.send(packDevicesInspection(*msg));
}

void CANInterfaceNode::pollReceiver()
{
  CanFrame frame;
  while (receiver_.receive(frame)) {
    parseVcuFrame(frame);  // TODO: 解析 VCU→工控机单帧
  }
}

CanFrame CANInterfaceNode::packMissionState(const wuta_msgs::msg::MissionState & msg)
{
  (void)msg;
  RCLCPP_WARN_ONCE(get_logger(), "packMissionState() not implemented (CAN layout TBD).");
  return CanFrame{};
}

CanFrame CANInterfaceNode::packDevicesInspection(
  const wuta_msgs::msg::DevicesInspection & msg)
{
  (void)msg;
  RCLCPP_WARN_ONCE(get_logger(), "packDevicesInspection() not implemented (CAN layout TBD).");
  return CanFrame{};
}

void CANInterfaceNode::parseVcuFrame(const CanFrame & frame)
{
  (void)frame;
  RCLCPP_WARN_ONCE(get_logger(), "parseVcuFrame() not implemented (CAN layout TBD).");
}

}  // namespace can_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
