#include "can_interface/can_interface.hpp"
#include <algorithm>

namespace can_interface
{

namespace
{
// 16bit 定标：控制量 x∈[-1,1] → 10~65525，32767 为中心（0 控制）
// 驱动/右：32767 + x*32758；制动/左：32767 + x*32757；钳位 [10, 65525]
uint16_t scaleControl(double x)
{
  const double value = x >= 0.0
    ? 32767.0 + x * 32758.0
    : 32767.0 + x * 32757.0;
  return static_cast<uint16_t>(std::clamp(value, 10.0, 65525.0));
}
}  // namespace

CANInterfaceNode::CANInterfaceNode(const rclcpp::NodeOptions & options)
: Node("can_interface", options)
{
  can_device_ = declare_parameter<std::string>("can_device", "can0");
  // can_baud_rate 由系统侧 ip link 设置，此处仅预留声明
  declare_parameter<int>("can_baud_rate", 500000);
  declare_parameter<bool>("can_loopback", false);
  poll_interval_sec_ = declare_parameter<double>("poll_interval_sec", 0.02);
  max_steer_deg_ = declare_parameter<double>("max_steer_deg", max_steer_deg_);

  // 打开 CAN 接口；失败时静默降级，节点继续运行（收不到/发不出由上层兜底）
  const bool opened = can_.open(can_device_);
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

  // 0x210 数据源：横向 angle 与纵向 throttle_brake 均来自 /control/command
  control_command_sub_ = create_subscription<autoware_msgs::msg::Command>(
    "/control/command", 10,
    std::bind(&CANInterfaceNode::onControlCommand, this, std::placeholders::_1));

  // 保活发送：无控制指令时也以 10Hz 持续上报（Signal3/4 状态随帧携带）
  keepalive_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CANInterfaceNode::sendControlFrame, this));

  // 接收路径：定时器轮询（非阻塞）
  receive_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(poll_interval_sec_ * 1000.0)),
    std::bind(&CANInterfaceNode::pollReceiver, this));

  // 接收方向解析结果发布（VCU→工控机帧）
  mission_mode_cmd_pub_ = create_publisher<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10);
  start_command_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/start_command", 10);
  emergency_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/emergency", 10);
  // 预留
  // velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
  //   "/chcnav/velocity", 50);

  RCLCPP_INFO(get_logger(), "CAN Interface initialized (tx/rx separated).");
  RCLCPP_INFO(get_logger(), "Tx frame 0x210 active; Rx 0x501 parse TODO.");
}

void CANInterfaceNode::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  // Signal4：任务 FINISH 后上报 VCU
  can_finished_ = (msg->state == wuta_msgs::msg::MissionState::FINISH);
  sendControlFrame();
}

void CANInterfaceNode::onDevicesInspection(
  const wuta_msgs::msg::DevicesInspection::SharedPtr msg)
{
  // Signal3：设备自检通过 → 上线=1；失败 → 0（mission_manager 已切 EMERGENCY）
  can_online_ = msg->ok;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "Devices inspection: ok=%d online=%d", msg->ok, can_online_);
  sendControlFrame();
}

void CANInterfaceNode::onControlCommand(const autoware_msgs::msg::Command::SharedPtr msg)
{
  throttle_brake_ = msg->throttle_brake;  // 纵向开度 [-1,1]，Signal1 数据源
  cmd_angle_ = msg->angle;                // 转向角（deg），Signal2 数据源
  sendControlFrame();
}

void CANInterfaceNode::pollReceiver()
{
  CanFrame frame;
  while (can_.receive(frame)) {
    parseVcuFrame(frame);  // TODO: 解析 VCU→工控机单帧（0x501）
  }
}

void CANInterfaceNode::sendControlFrame()
{
  const CanFrame frame = packControlFrame(
    throttle_brake_, cmd_angle_, can_online_, can_finished_);
  if (!can_.send(frame)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to send control frame to CAN bus.");
  }
}

CanFrame CANInterfaceNode::packControlFrame(
  double throttle_brake, double steer_deg, bool online, bool finished) const
{
  CanFrame frame;
  frame.can_id = 0x210;  // 工控机→VCU 单帧
  frame.dlc = 8;

  const uint16_t s1 = scaleControl(throttle_brake);          // 纵向：驱动/制动
  // 横向：angle 正=左（autoware 约定），协议 Signal2 小值=左，故取反映射
  const uint16_t s2 = scaleControl(-steer_deg / max_steer_deg_);
  frame.data[0] = static_cast<uint8_t>(s1 & 0xFF);
  frame.data[1] = static_cast<uint8_t>((s1 >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>(s2 & 0xFF);
  frame.data[3] = static_cast<uint8_t>((s2 >> 8) & 0xFF);
  frame.data[4] = online ? 0x01 : 0x00;      // Signal3 工控机上线
  frame.data[5] = finished ? 0x01 : 0x00;    // Signal4 任务已完成
  frame.data[6] = 0x00;                      // Signal5 空
  frame.data[7] = 0x00;
  return frame;
}

void CANInterfaceNode::parseVcuFrame(const CanFrame & frame)
{
  (void)frame;
  RCLCPP_WARN_ONCE(get_logger(), "parseVcuFrame() not implemented (0x501 layout parse TODO).");
}

}  // namespace can_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
