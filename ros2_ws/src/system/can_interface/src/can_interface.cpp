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

// 0x501 Byte1 测试模式 → mission_mode_cmd 字符串；空串表示不转发。
//   1 = 操控性测试（有人驾驶，与无人算法无关）→ 不转发
//   0 / 7+ = 未定义 → 不转发（调用方按需告警）
std::string missionModeName(uint8_t vcu_mission_mode)
{
  switch (vcu_mission_mode) {
    case 2:  return "acceleration";
    case 3:  return "trackdrive";
    case 4:  return "skidpad";
    case 5:  return "ebs_test";
    case 6:  return "inspection";
    default: return {};
  }
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

  // 接收方向解析结果发布（VCU→工控机帧）：新协议只解析测试模式
  mission_mode_cmd_pub_ = create_publisher<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10);
  // 预留，目前暂时不由VCU发轮边速度
  // velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
  //   "/chcnav/velocity", 50);

  // 模式保活：模式是「电平」信号（VCU 持续 10Hz 重发同一档位），而上面只在字节
  // 「变化」时转发一次。mission_manager 晚启动/重启时那一次发布早已过去，会一直
  // 停在 IDLE（L4 每个用例重启 mission_manager 必然踩到）。故按上游同款语义周期性
  // 重发当前档位；离开任务档（Byte1=1 有人驾驶 / 未知）自动停发，不重复失效档位。
  // mode_repeat_period_sec <= 0 关闭保活。
  mode_repeat_period_sec_ = declare_parameter<double>(
    "mode_repeat_period_sec", mode_repeat_period_sec_);
  if (mode_repeat_period_sec_ > 0.0) {
    mode_repeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int64_t>(mode_repeat_period_sec_ * 1000.0)),
      std::bind(&CANInterfaceNode::repeatMissionMode, this));
  }

  RCLCPP_INFO(get_logger(), "CAN Interface initialized (tx/rx separated).");
  RCLCPP_INFO(get_logger(), "Tx frame 0x210 active; Rx frame 0x501 active.");
}

void CANInterfaceNode::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  // Signal4：任务 FINISH 后上报 VCU（急停归零由 controller 负责，本节点不参与判定）
  can_finished_ = (msg->state == wuta_msgs::msg::MissionState::FINISH);
  sendControlFrame();
}

void CANInterfaceNode::onDevicesInspection(
  const wuta_msgs::msg::DevicesInspection::SharedPtr msg)
{
  // Signal3：设备自检通过 → 上线=1；失败 → 0（mission_manager 已切 EMERGENCY）
  can_online_ = msg->ok;
  if (msg->ok) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Devices inspection: ok=%d online=%d", msg->ok, can_online_);
  } else {
    RCLCPP_ERROR(get_logger(), "Devices inspection: ok=%d online=%d",
      msg->ok, can_online_);
  }
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
    parseVcuFrame(frame);  // 解析 VCU→工控机单帧（0x501）
  }
}

void CANInterfaceNode::sendControlFrame()
{
  const CanFrame frame = packControlFrame(
    throttle_brake_, cmd_angle_, can_online_, can_finished_);
  if (!can_.send(frame)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to send control frame to the VCU on CAN bus.");
  }
}

CanFrame CANInterfaceNode::packControlFrame(
  double throttle_brake, double steer_deg, bool online, bool finished) const
{
  CanFrame frame;
  frame.can_id = 0x210;  // 工控机→VCU 单帧
  frame.dlc = 8;

  // 纯转发：急停归零由 controller 完成，本节点不做任何判定
  const uint16_t s1 = scaleControl(throttle_brake);    // 纵向：驱动/制动
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
  // 只处理 VCU→工控机 0x501 帧
  if (frame.can_id != 0x501) return;

  const uint8_t vcu_mission_mode = frame.data[0];  // Byte1：VCU派发的任务模式

  // ---- Byte1 测试模式 → mission_mode_cmd（变化时立即发布；同值重发由保活定时器兜底） ----
  if (vcu_mission_mode != last_vcu_mission_mode_) {
    const std::string mode = missionModeName(vcu_mission_mode);
    if (!mode.empty()) {
      std_msgs::msg::String msg;
      msg.data = mode;
      mission_mode_cmd_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "VCU mission mode %u -> mission mode '%s'.",
        vcu_mission_mode, mode.c_str());
    } else if (vcu_mission_mode == 1) {
      // 操控性测试：有人驾驶，与无人算法无关，不做处理
      RCLCPP_INFO(get_logger(), "Driving by human,FSD is ignored.");
    } else {
      RCLCPP_WARN(get_logger(), "Unknown VCU mission mode %u.", vcu_mission_mode);
    }
    last_vcu_mission_mode_ = vcu_mission_mode;
  }
}

void CANInterfaceNode::repeatMissionMode()
{
  // 周期重发当前档位，供晚启动/重启的 mission_manager 追上（模式是电平信号）。
  // 当前档位不是任务档（有人驾驶/未知/尚未收到）时停发，避免重复已失效的档位。
  const std::string mode = missionModeName(last_vcu_mission_mode_);
  if (mode.empty()) return;
  std_msgs::msg::String msg;
  msg.data = mode;
  mission_mode_cmd_pub_->publish(msg);
}

}  // namespace can_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
