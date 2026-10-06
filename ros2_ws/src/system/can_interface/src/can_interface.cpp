#include "can_interface/can_interface.hpp"
#include <algorithm>

namespace can_interface
{

namespace
{
// Signal1 纵向定标（26 赛季正式协议：0~65535，32767 为中心 = 0 控制）
//   0~32767    ：越靠近 0 制动力越大 → 0 = 满制动
//   32767~65535：越大驱动力越大     → 65535 = 满驱动
//   两侧跨度不同：制动半段 32767（32767→0），驱动半段 32768（32767→65535）
constexpr double kLongitudinalCenter = 32767.0;
constexpr double kLongitudinalBrakeSpan = kLongitudinalCenter;            // 32767 → 0
constexpr double kLongitudinalDriveSpan = 65535.0 - kLongitudinalCenter;  // 32767 → 65535

uint16_t scaleLongitudinal(double x)
{
  const double span = x >= 0.0 ? kLongitudinalDriveSpan : kLongitudinalBrakeSpan;
  const double value = kLongitudinalCenter + x * span;
  return static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
}

// Signal2 横向定标（26 赛季正式协议：0~65535，32767 为中心 = 回正）
//   0~32767   ：越靠近 0 越向左 → 0 = 满左
//   32767~65535：越大越向右   → 65535 = 满右
//   steer_deg 正 = 左（autoware 约定），满量程 ±max_steer_deg
//   两侧跨度不同：左半段 32767（32767→0），右半段 32768（32767→65535）
constexpr double kLateralCenter = 32767.0;
constexpr double kLateralLeftSpan = kLateralCenter;                  // 32767 → 0
constexpr double kLateralRightSpan = 65535.0 - kLateralCenter;       // 32767 → 65535

uint16_t scaleLateral(double steer_deg, double max_steer_deg)
{
  const double x = max_steer_deg > 0.0 ? steer_deg / max_steer_deg : 0.0;
  const double span = x >= 0.0 ? kLateralLeftSpan : kLateralRightSpan;
  const double value = kLateralCenter - x * span;
  return static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
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

// 0x1E4 Byte1 RES（遥控器）状态定义
constexpr uint8_t kResEmergency   = 0x10;  // 按下急停
constexpr uint8_t kResRemoteOnline = 0x11;  // 遥控器上线
constexpr uint8_t kResStartButton = 0x13;  // 发车按钮被按下

// 0x301 上线心跳定义（工控机→VCU，标准帧 / DLC=1）
constexpr uint8_t kHeartbeatOnline  = 0x01;  // 心跳正常/在线
constexpr uint8_t kHeartbeatOffline = 0x00;  // 异常/离线
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
  res_frame_id_ = declare_parameter<int>("res_frame_id", res_frame_id_);
  heartbeat_frame_id_ = declare_parameter<int>("heartbeat_frame_id", heartbeat_frame_id_);
  heartbeat_period_sec_ = declare_parameter<double>(
    "heartbeat_period_sec", heartbeat_period_sec_);

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
  // tx_armed_ 未置位（首份自检结论未到）时 sendControlFrame() 直接返回：本定时器同样静默
  keepalive_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CANInterfaceNode::sendControlFrame, this));

  // 上线心跳：0x301 / DLC=1 / 20Hz（50ms）。**不并入 0x210 的 tx_armed_ 门控**：
  // 本帧从节点启动即发，首份 /system/devices_inspection 到达前 Data[0]=0x00，
  // 之后随自检结论翻转（0x01=在线 / 0x00=异常）。period<=0 关闭。
  if (heartbeat_period_sec_ > 0.0) {
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(
        std::max(0.001, heartbeat_period_sec_) * 1000.0)),
      std::bind(&CANInterfaceNode::sendHeartbeatFrame, this));
  }

  // 接收路径：定时器轮询（非阻塞）
  receive_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(poll_interval_sec_ * 1000.0)),
    std::bind(&CANInterfaceNode::pollReceiver, this));

  // 接收方向解析结果发布（VCU→工控机帧）：新协议只解析测试模式
  mission_mode_cmd_pub_ = create_publisher<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10);
  // RES 状态解析结果：GO 放行（严格边沿：仅 0x13 变化沿发布一次，不重放不锁存）
  start_command_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/start_command", 10);
  // 急停总线：与 mission_manager 自检失败同一话题。锁存（transient_local），
  // 晚启动且请求 transient_local 的订阅者能立即拿到已置位的急停态。
  emergency_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/system/emergency", rclcpp::QoS(1).reliable().transient_local());
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

  // GO **不重放/不锁存**（无 go_repeat_timer_）。曾按 res_go_hold_sec 窗口重放：
  // 一次按键 + 之后任意一次选模式即发车，还能授权给窗口内新起的 mission_manager
  // 实例（vcan0 实测确认会让未按 GO 的用例发车），故移除。重启实例需重按 GO。

  RCLCPP_INFO(get_logger(), "CAN Interface initialized (tx/rx separated).");
  RCLCPP_INFO(get_logger(),
    "Tx frame 0x210 gated until first /system/devices_inspection; "
    "Rx frame 0x501 (mode) + 0x%X (RES).",
    static_cast<unsigned>(res_frame_id_));
  RCLCPP_INFO(get_logger(),
    "Tx heartbeat 0x%X: dlc=1, %.0f ms (20Hz), ungated from startup.",
    static_cast<unsigned>(heartbeat_frame_id_), heartbeat_period_sec_ * 1000.0);
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
  if (!tx_armed_) {
    // 首份自检结论到达才开闸（ok=true/false 都开）：真故障同样经此送达 VCU
    tx_armed_ = true;
    RCLCPP_INFO(get_logger(),
      "First devices inspection verdict received (ok=%d); 0x210 transmission enabled.",
      msg->ok);
  }
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
    // 接收方向按 ID 分发：0x501 = VCU 任务模式；0x1E4 = RES 遥控器状态
    if (frame.can_id == 0x501) {
      parseVcuFrame(frame);
    } else if (frame.can_id == static_cast<uint32_t>(res_frame_id_)) {
      parseResFrame(frame);
    }
  }
}

void CANInterfaceNode::sendControlFrame()
{
  // 门控：首份自检结论到达前不上总线——开机默认 Signal3=0 会被 VCU 当作自检故障
  if (!tx_armed_) return;
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
  const uint16_t s1 = scaleLongitudinal(throttle_brake);   // 纵向：驱动/制动
  // 横向：angle 正=左（autoware 约定）→ Signal2 小值（0 = 满左），中心 32767
  const uint16_t s2 = scaleLateral(steer_deg, max_steer_deg_);
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

void CANInterfaceNode::sendHeartbeatFrame()
{
  const CanFrame frame = packHeartbeatFrame();
  if (!can_.send(frame)) {
    // 降级模式（can_.open 失败）或总线异常下会持续失败：节流告警，不刷屏
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to send heartbeat frame 0x301 on CAN bus.");
  }
}

CanFrame CANInterfaceNode::packHeartbeatFrame() const
{
  CanFrame frame;
  frame.can_id = static_cast<uint32_t>(heartbeat_frame_id_);  // 0x301，标准帧
  frame.dlc = 1;                                              // DLC=1，只发 1 字节
  // 与 0x210 Signal3 同源：/system/devices_inspection.ok
  // （首份结论未到达时 can_online_ 保持 false → 0x00）
  frame.data[0] = can_online_ ? kHeartbeatOnline : kHeartbeatOffline;
  return frame;  // 其余字节不参与 DLC，保持 0
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

void CANInterfaceNode::parseResFrame(const CanFrame & frame)
{
  // RES→工控机单帧 0x1E4（标准帧 / 500k / 协议文档 DLC=3）。
  // 实测（2026-10-03 zlgcan_bridge 抓包，log/20261003_203934/vcu2fsd/0x1E4.log）：
  //   33.3Hz 周期电平广播（中位 30ms，P10/P90 = 26/34ms），桥侧 DLC 记为 8；
  //   Byte1=0x11 遥控器上线（长电平）、0x13 发车按钮（0.15~0.51s 瞬时脉冲）、
  //   0x10 急停（持续电平）、0x00 遥控器未上线。
  // 故只读 Byte1 且不校验 DLC（>=1 即可），协议文档与实际抓包都能兼容。
  if (frame.dlc < 1) return;
  const uint8_t res_state = frame.data[0];
  const bool changed = (res_state != last_res_state_);

  switch (res_state) {
    case kResStartButton:
      // 发车：**只在变化沿发布一次**（严格边沿，不重放不锁存）。
      // 0x13 是 0.15~0.51s 的短脉冲，但远端 mission_manager 是常驻订阅者，
      // 一定收得到；漏收只可能是它当时没在跑——那种情况要求重按 GO，不能自动放行。
      if (changed) {
        publishStartCommand(true);
        RCLCPP_INFO(get_logger(), "RES start button pressed -> start command.");
      }
      break;
    case kResEmergency:
      // 急停：锁存发布（controller 收到后不可恢复，持续输出全零）。
      if (changed) {
        publishEmergency(true);
        RCLCPP_ERROR(get_logger(), "RES EMERGENCY pressed!");
      }
      break;
    case kResRemoteOnline:
      // 遥控器上线：纯状态，只记日志（实测 33Hz 持续电平）
      if (changed) {
        RCLCPP_INFO(get_logger(), "RES remote online.");
      }
      break;
    default:
      // 0x00（遥控器未上线）等未定义值：不触发任何动作
      if (changed) {
        RCLCPP_WARN(get_logger(), "Unknown RES state 0x%02X on 0x%X.",
          res_state, static_cast<unsigned>(frame.can_id));
      }
      break;
  }

  last_res_state_ = res_state;
}

void CANInterfaceNode::publishStartCommand(bool start)
{
  std_msgs::msg::Bool msg;
  msg.data = start;
  start_command_pub_->publish(msg);
}

void CANInterfaceNode::publishEmergency(bool emergency)
{
  std_msgs::msg::Bool msg;
  msg.data = emergency;
  emergency_pub_->publish(msg);
}

}  // namespace can_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
