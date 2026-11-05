#include "can_interface/can_interface.hpp"
#include <algorithm>
#include <cmath>

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
constexpr double kBenchPreDriveLimit = 0.05;  // Target 3 only: at most 5% drive.

uint16_t scaleLongitudinal(double x)
{
  const double span = x >= 0.0 ? kLongitudinalDriveSpan : kLongitudinalBrakeSpan;
  const double value = kLongitudinalCenter + x * span;
  return static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
}

// Signal2 横向定标（26 赛季正式协议：0~65535，32767 为中心 = 回正）
//   0~32767   ：越靠近 0 越向左 → 0 = 满左
//   32767~65535：越大越向右   → 65535 = 满右
//   steer_deg 正 = 左（autoware 约定），满量程 ±max_steer_angle
//   两侧跨度不同：左半段 32767（32767→0），右半段 32768（32767→65535）
constexpr double kLateralCenter = 32767.0;
constexpr double kLateralLeftSpan = kLateralCenter;                  // 32767 → 0
constexpr double kLateralRightSpan = 65535.0 - kLateralCenter;       // 32767 → 65535

uint16_t scaleLateral(double steer_deg, double max_steer_angle)
{
  const double x = max_steer_angle > 0.0 ? steer_deg / max_steer_angle : 0.0;
  const double span = x >= 0.0 ? kLateralLeftSpan : kLateralRightSpan;
  const double value = kLateralCenter - x * span;
  return static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
}

// VCU 模式帧 Byte1 测试模式 → mission_mode_cmd 字符串；空串表示不转发。
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

// RES 遥控器帧 Byte1 状态定义
constexpr uint8_t kResEmergency   = 0x10;  // 按下急停
constexpr uint8_t kResRemoteOnline = 0x11;  // 遥控器上线
constexpr uint8_t kResStartButton = 0x13;  // 发车按钮被按下

// 上线心跳 Data[0] 状态定义（工控机→VCU，标准帧 / DLC=1）
constexpr uint8_t kHeartbeatOnline  = 0x01;  // 心跳正常/在线
constexpr uint8_t kHeartbeatOffline = 0x00;  // 异常/离线
}  // namespace

CANInterfaceNode::CANInterfaceNode(const rclcpp::NodeOptions & options)
: Node("can_interface", options)
{
  // 通信接口
  brake_bench_mode_ = declare_parameter("brake_bench_mode", false);
  receive_only_ = declare_parameter("receive_only", false);
  if (receive_only_ && brake_bench_mode_) throw std::invalid_argument("receive_only conflicts with brake_bench_mode");
  can_device_ = declare_parameter<std::string>("can.device", can_device_);
  poll_interval_sec_ = declare_parameter<double>("can.poll_interval_sec", poll_interval_sec_);
  // WUTA_DIRECT_BRAKE_PAUSED: if (brake_bench_mode_) poll_interval_sec_ = std::min(poll_interval_sec_, 0.002);

  // 报文 ID
  control_frame_id_ = declare_parameter<int>("frames.control.id", control_frame_id_);
  heartbeat_frame_id_ = declare_parameter<int>("frames.heartbeat.id", heartbeat_frame_id_);
  vcu_frame_id_ = declare_parameter<int>("frames.vcu.id", vcu_frame_id_);
  res_frame_id_ = declare_parameter<int>("frames.res.id", res_frame_id_);

  // 发送周期
  control_period_sec_ = declare_parameter<double>(
    "frames.control.period_sec", control_period_sec_);
  if (brake_bench_mode_) {
    // 台架模式始终维持至多100ms的发送周期，使500ms命令超时可执行。
    control_period_sec_ = 0.1;
  }
  heartbeat_period_sec_ = declare_parameter<double>(
    "frames.heartbeat.period_sec", heartbeat_period_sec_);

  // 横向定标
  max_steer_angle_ = declare_parameter<double>("steer.max_steer_angle", max_steer_angle_);

  // 模式保活重发周期
  mode_repeat_period_sec_ = declare_parameter<double>(
    "mode_repeat_period_sec", mode_repeat_period_sec_);

  mode_cache_enabled_ = declare_parameter<bool>("mode_cache.enabled", true);
  mode_cache_directory_ = declare_parameter<std::string>(
    "mode_cache.directory", "/run/zlgcan_bridge");
  mode_cache_wait_sec_ = declare_parameter<double>("mode_cache.bootstrap_wait_sec", 2.0);
  mode_cache_max_age_sec_ = declare_parameter<double>("mode_cache.max_age_sec", 0.0);
  if (!std::isfinite(mode_cache_wait_sec_) || mode_cache_wait_sec_ < 0 ||
      !std::isfinite(mode_cache_max_age_sec_) || mode_cache_max_age_sec_ < 0) {
    throw std::invalid_argument("mode_cache timing parameters must be finite and nonnegative");
  }
  mode_cache_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(
      std::clamp(mode_cache_wait_sec_, 0.0, 60.0) * 1000));

  // 打开 CAN 接口；失败时静默降级，节点继续运行（收不到/发不出由上层兜底）
  const bool opened = can_.open(can_device_);
  if (!opened) {
    RCLCPP_WARN(get_logger(), "CAN device '%s' unavailable, run in degraded mode.",
      can_device_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "CAN interface opened on '%s'.", can_device_.c_str());
  }

  mode_cache_done_ = !mode_cache_enabled_ || !opened;

  // 发送路径：ROS 订阅回调驱动
  mission_state_sub_ = create_subscription<wuta_msgs::msg::MissionState>(
    "/system/mission_state", 10,
    std::bind(&CANInterfaceNode::onMissionState, this, std::placeholders::_1));

  devices_inspection_sub_ = create_subscription<wuta_msgs::msg::DevicesInspection>(
    "/system/devices_inspection", 10,
    std::bind(&CANInterfaceNode::onDevicesInspection, this, std::placeholders::_1));

  // 控制帧数据源：横向 angle 与纵向 throttle_brake 均来自 /control/command
  control_command_sub_ = create_subscription<autoware_msgs::msg::Command>(
    "/control/command", 10,
    std::bind(&CANInterfaceNode::onControlCommand, this, std::placeholders::_1));

  if (brake_bench_mode_) {
    bench_target_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/control/brake_bench/target", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        // 1=VCU brake, 3=VCU pre-drive; disabled direct target 2/unknown remain neutral.
        const uint8_t target = (msg->data == 1 || msg->data == 3) ? msg->data : 0;
        if (target != bench_target_) {
          // 切换出口先丢弃旧Command，收到新Command前保持中性。
          throttle_brake_ = 0.0;
          bench_command_seen_ = false;
        }
        bench_target_ = target;
        bench_target_seen_ = true;
        bench_target_at_ = std::chrono::steady_clock::now();
        sendControlFrame();
      });
#if 0  // WUTA_DIRECT_BRAKE_PAUSED: 按用户要求停用工控机直发，保留源码供恢复。
    bench_initialized_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/can/brake_bench/initialized", 10);
    bench_initialize_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/control/brake_bench/initialize", 10,
      std::bind(&CANInterfaceNode::startBrakeBenchInitialization, this, std::placeholders::_1));
#endif

  }

  // 控制帧周期发送：无控制指令时也按周期持续上报（Signal3/4 状态随帧携带）。
  // tx_armed_ 未置位（首份自检结论未到）时 sendControlFrame() 直接返回：本定时器同样静默。
  // control_period_sec <= 0 关闭周期发送（控制帧仍随 /control/command 回调即时发出）。
  if (!receive_only_ && control_period_sec_ > 0.0) {
    keepalive_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int64_t>(std::max(0.001, control_period_sec_) * 1000.0)),
      std::bind(&CANInterfaceNode::sendControlFrame, this));
  }

  // 上线心跳：标准帧 / DLC=1。**不并入控制帧的 tx_armed_ 门控**：
  // 本帧从节点启动即发，首份 /system/devices_inspection 到达前 Data[0]=0x00，
  // 之后随自检结论翻转（0x01=在线 / 0x00=异常）。period<=0 关闭。
  if (!receive_only_ && heartbeat_period_sec_ > 0.0) {
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

  // 模式保活：模式是「电平」信号（VCU 持续重发同一档位），而上面只在字节
  // 「变化」时转发一次。mission_manager 晚启动/重启时那一次发布早已过去，会一直
  // 停在 IDLE。故周期性重发当前档位；离开任务档（有人驾驶 / 未知）自动停发。
  // mode_repeat_period_sec <= 0 关闭保活。
  if (mode_repeat_period_sec_ > 0.0) {
    mode_repeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int64_t>(mode_repeat_period_sec_ * 1000.0)),
      std::bind(&CANInterfaceNode::repeatMissionMode, this));
  }

  // GO **不重放/不锁存**。重启实例需重按 GO。

  if (receive_only_) {
    RCLCPP_WARN(get_logger(), "RECEIVE ONLY: control, heartbeat and shutdown transmission disabled.");
    return;
  }
  RCLCPP_INFO(get_logger(), "CAN Interface initialized (tx/rx separated).");
  RCLCPP_INFO(get_logger(),
    "Tx control frame 0x%X gated until first /system/devices_inspection; "
    "Rx mode frame 0x%X + RES frame 0x%X.",
    static_cast<unsigned>(control_frame_id_),
    static_cast<unsigned>(vcu_frame_id_),
    static_cast<unsigned>(res_frame_id_));
  RCLCPP_INFO(get_logger(),
    "Tx heartbeat 0x%X: dlc=1, %.0f ms, ungated from startup.",
    static_cast<unsigned>(heartbeat_frame_id_), heartbeat_period_sec_ * 1000.0);
}

void CANInterfaceNode::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  bench_mission_active_ = msg->brake_bench_mode &&
    msg->mission_mode == wuta_msgs::msg::MissionState::MISSION_ACCELERATION &&
    msg->state == wuta_msgs::msg::MissionState::EXPLORE;
  bench_mission_at_ = std::chrono::steady_clock::now();
  // Signal4：任务 FINISH 后上报 VCU（急停归零由 controller 负责，本节点不参与判定）
  can_finished_ = (msg->state == wuta_msgs::msg::MissionState::FINISH);
  sendControlFrame();
}

void CANInterfaceNode::onDevicesInspection(
  const wuta_msgs::msg::DevicesInspection::SharedPtr msg)
{
  if (receive_only_) return;
  // Signal3：设备自检通过 → 上线=1；失败 → 0（mission_manager 已切 EMERGENCY）
  can_online_ = msg->ok;
  if (!tx_armed_) {
    // 首份自检结论到达才开闸（ok=true/false 都开）：真故障同样经此送达 VCU
    tx_armed_ = true;
    RCLCPP_INFO(get_logger(),
      "First devices inspection verdict received (ok=%d); 0x%X transmission enabled.",
      msg->ok, static_cast<unsigned>(control_frame_id_));
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
  bench_command_seen_ = true;
  bench_command_at_ = std::chrono::steady_clock::now();
  throttle_brake_ = msg->throttle_brake;  // 纵向开度 [-1,1]，Signal1 数据源
  cmd_angle_ = msg->angle;                // 转向角（deg），Signal2 数据源
  sendControlFrame();
}

void CANInterfaceNode::pollReceiver()
{
  CanFrame frame;
  while (can_.receive(frame)) {
    // 接收方向按 ID 分发：VCU 任务模式帧 / RES 遥控器帧
    if (frame.can_id == static_cast<uint32_t>(vcu_frame_id_)) {
      parseVcuFrame(frame);
    } else if (frame.can_id == static_cast<uint32_t>(res_frame_id_)) {
      parseResFrame(frame);
    }
#if 0  // WUTA_DIRECT_BRAKE_PAUSED: 停用SDO应答解析。
    else if (brake_bench_mode_ && frame.can_id == 0x583) {
      parseBrakeBenchSdo(frame);
    }
#endif
  }
  // WUTA_DIRECT_BRAKE_PAUSED: tickBrakeBenchInitialization();
  restoreModeCache();
}

void CANInterfaceNode::sendControlFrame()
{
  if (receive_only_) return;
  // 门控：首份自检结论到达前不上总线——开机默认 Signal3=0 会被 VCU 当作自检故障
  if (!tx_armed_) return;
  if (brake_bench_mode_) {
    const auto t = std::chrono::steady_clock::now();
    if (!bench_target_seen_ || bench_target_ == 0 ||
      // WUTA_DIRECT_BRAKE_PAUSED: (bench_target_ == 1 && bench_init_attempted_) ||
      std::chrono::duration<double>(t - bench_target_at_).count() > 0.5 ||
      // WUTA_DIRECT_BRAKE_PAUSED: (bench_target_ == 2 && !bench_initialized_) ||
      last_res_state_ == kResEmergency || !can_online_ || !bench_command_seen_ || !bench_mission_active_ ||
      std::chrono::duration<double>(t - bench_command_at_).count() > 0.5 ||
      std::chrono::duration<double>(t - bench_mission_at_).count() > 0.5 ||
      !std::isfinite(throttle_brake_)) throttle_brake_ = 0.0;
    // Only target 3 admits a small positive request; target 1 remains brake-only.
    throttle_brake_ = bench_target_ == 3 ?
      std::clamp(throttle_brake_, 0.0, kBenchPreDriveLimit) :
      std::clamp(throttle_brake_, -1.0, 0.0);
    cmd_angle_ = 0.0;
    // WUTA_DIRECT_BRAKE_PAUSED: updateBrakeBenchMotor(bench_target_ == 2 ? -throttle_brake_ : 0.0);
  }
  const CanFrame frame = packControlFrame(
    brake_bench_mode_ && bench_target_ != 1 && bench_target_ != 3 ? 0.0 : throttle_brake_,
    cmd_angle_, can_online_, can_finished_);
  if (!can_.send(frame)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to send control frame to the VCU on CAN bus.");
  }
}

CANInterfaceNode::~CANInterfaceNode()
{
  if (!receive_only_ && brake_bench_mode_ && tx_armed_) {
    // WUTA_DIRECT_BRAKE_PAUSED: stopBrakeBenchMotorBestEffort();
    can_.send(packControlFrame(0.0, 0.0, can_online_, can_finished_));
  }
}

CanFrame CANInterfaceNode::packControlFrame(
  double throttle_brake, double steer_deg, bool online, bool finished) const
{
  CanFrame frame;
  frame.can_id = static_cast<uint32_t>(control_frame_id_);  // 工控机→VCU 控制帧
  frame.dlc = 8;

  // 纯转发：急停归零由 controller 完成，本节点不做任何判定
  const uint16_t s1 = scaleLongitudinal(throttle_brake);   // 纵向：驱动/制动
  // 横向：angle 正=左（autoware 约定）→ Signal2 小值（0 = 满左），中心 32767
  const uint16_t s2 = scaleLateral(steer_deg, max_steer_angle_);
  // Signal1 / Signal2 字节序：**大端（Motorola，高字节在前）**
  //   [0]=S1 高字节 [1]=S1 低字节 [2]=S2 高字节 [3]=S2 低字节
  //   例：纵向 +16% 驱动 38009=0x9479 → 94 79；横向中位 32767=0x7FFF → 7F FF
  //   改回小端时须同步改 hil_test/config/protocol.yaml 的 signals.*.little_endian
  //   与 plot_motor_image.py（否则 L1 post 的 0x210 透传用例会失败）
  frame.data[0] = static_cast<uint8_t>((s1 >> 8) & 0xFF);
  frame.data[1] = static_cast<uint8_t>(s1 & 0xFF);
  frame.data[2] = static_cast<uint8_t>((s2 >> 8) & 0xFF);
  frame.data[3] = static_cast<uint8_t>(s2 & 0xFF);
  frame.data[4] = online ? 0x01 : 0x00;      // Signal3 工控机上线
  frame.data[5] = finished ? 0x01 : 0x00;    // Signal4 任务已完成
  frame.data[6] = 0x00;                      // Signal5 空
  frame.data[7] = 0x00;
  return frame;
}

void CANInterfaceNode::sendHeartbeatFrame()
{
  if (receive_only_) return;
  const CanFrame frame = packHeartbeatFrame();
  if (!can_.send(frame)) {
    // 降级模式（can_.open 失败）或总线异常下会持续失败：节流告警，不刷屏
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to send heartbeat frame 0x%X on CAN bus.",
      static_cast<unsigned>(heartbeat_frame_id_));
  }
}

CanFrame CANInterfaceNode::packHeartbeatFrame() const
{
  CanFrame frame;
  frame.can_id = static_cast<uint32_t>(heartbeat_frame_id_);  // 标准帧
  frame.dlc = 1;                                              // DLC=1，只发 1 字节
  // 与控制器控制帧 Signal3 同源：/system/devices_inspection.ok
  // （首份结论未到达时 can_online_ 保持 false → 0x00）
  frame.data[0] = can_online_ ? kHeartbeatOnline : kHeartbeatOffline;
  return frame;  // 其余字节不参与 DLC，保持 0
}

void CANInterfaceNode::parseVcuFrame(const CanFrame & frame)
{
  // 只处理 VCU→工控机 模式帧
  if (frame.can_id != static_cast<uint32_t>(vcu_frame_id_)) return;

  if (frame.dlc < 1 || frame.dlc > 8) return;
  mode_cache_done_ = true;  // 启动以来已收到实时模式，禁止历史缓存覆盖。
  applyMissionMode(frame.data[0]);
}

void CANInterfaceNode::applyMissionMode(uint8_t vcu_mission_mode)
{

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

void CANInterfaceNode::restoreModeCache()
{
  if (mode_cache_done_) return;
  ModeSnapshot snapshot;
  if (readModeCache(mode_cache_directory_, can_device_,
      static_cast<uint32_t>(vcu_frame_id_), mode_cache_max_age_sec_, snapshot,
      mode_cache_reason_)) {
    // Close the read/receive race: drain frames queued during file validation.
    CanFrame frame;
    while (can_.receive(frame)) {
      if (frame.can_id == static_cast<uint32_t>(vcu_frame_id_)) parseVcuFrame(frame);
      else if (frame.can_id == static_cast<uint32_t>(res_frame_id_)) parseResFrame(frame);
    }
    if (mode_cache_done_) return;
    mode_cache_done_ = true;
    RCLCPP_INFO(get_logger(),
      "Restored VCU mode %u from bridge cache (session=%s sequence=%llu age=%.3fs).",
      static_cast<unsigned>(snapshot.mode), snapshot.session.c_str(),
      static_cast<unsigned long long>(snapshot.sequence), snapshot.age_sec);
    applyMissionMode(snapshot.mode);
  } else if (std::chrono::steady_clock::now() >= mode_cache_deadline_) {
    mode_cache_done_ = true;
    RCLCPP_INFO(get_logger(), "Mode cache bootstrap skipped: %s; waiting for live CAN.",
      mode_cache_reason_.c_str());
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
  // RES→工控机单帧（标准帧 / 500k）：Byte1 遥控器状态。
  // 只读 Byte1 且不校验 DLC（>=1 即可），兼容协议文档与实际抓包。
  if (frame.dlc < 1) return;
  const uint8_t res_state = frame.data[0];
  const bool changed = (res_state != last_res_state_);

  switch (res_state) {
    case kResStartButton:
      // 发车：只在变化沿发布一次（严格边沿，不重放不锁存）。
      if (changed) {
        // 当前仅VCU台架：GO后微驱动5秒、VCU制动10秒，随后中性/FINISH。
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
      // 遥控器上线：纯状态，只记日志
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

#if 0  // WUTA_DIRECT_BRAKE_PAUSED: 按用户要求停用工控机直发，保留源码供恢复。
void CANInterfaceNode::publishBrakeBenchInitialized(bool success)
{
  if (!bench_initialized_pub_) return;
  std_msgs::msg::Bool result;
  result.data = success;
  bench_initialized_pub_->publish(result);
}

void CANInterfaceNode::failBrakeBenchInitialization(const std::string & reason)
{
  bench_init_running_ = false;
  bench_init_waiting_ = false;
  bench_initialized_ = false;
  stopBrakeBenchMotorBestEffort();
  throttle_brake_ = 0.0;
  publishBrakeBenchInitialized(false);
  sendControlFrame();
  publishEmergency(true);
  RCLCPP_ERROR(get_logger(), "Brake motor initialization failed: %s; neutral and EMERGENCY.",
    reason.c_str());
}

void CANInterfaceNode::startBrakeBenchInitialization(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!brake_bench_mode_ || receive_only_ || !msg->data) return;
  if (bench_init_attempted_) {
    if (!bench_init_running_) publishBrakeBenchInitialized(bench_initialized_);
    return;
  }
  bench_init_attempted_ = true;
  const auto t = std::chrono::steady_clock::now();
  if (!bench_mission_active_ || !can_online_ || !tx_armed_ ||
    !bench_command_seen_ || std::chrono::duration<double>(t - bench_mission_at_).count() > 0.5 ||
    std::chrono::duration<double>(t - bench_command_at_).count() > 0.5 ||
    last_res_state_ == kResEmergency) {
    failBrakeBenchInitialization("missing fresh active bench state/command or inspection");
    return;
  }
  throttle_brake_ = 0.0;
  sendControlFrame();
  CanFrame nmt;
  nmt.can_id = 0x000;
  nmt.dlc = 2;
  nmt.data[0] = 0x01;
  nmt.data[1] = 0x00;  // 严格保留最小代码的01 00广播。
  if (!can_.send(nmt)) {
    failBrakeBenchInitialization("NMT 01 00 write failed");
    return;
  }
  bench_device_touched_ = true;
  bench_init_step_ = 0;
  bench_init_running_ = true;
  bench_init_waiting_ = false;
  bench_init_send_at_ = t + std::chrono::milliseconds(20);
  RCLCPP_INFO(get_logger(), "Brake motor setup started: exact minimal-code sequence; final EF deferred to controller request.");
}

CanFrame CANInterfaceNode::brakeBenchInitFrame() const
{
  // 保留最小代码的顺序及内容，按用户修正：目标转矩+1000（E8 03），最后启动使用EF。
  // NMT 01 00由入口发送；最后EF由controller在配置完成后等待3秒触发。
  static const std::array<uint8_t, 8> steps[] = {
    {0x2B, 0x40, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00},
    {0x2B, 0x40, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00},
    {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00},
    {0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00},
    {0x23, 0x81, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00},
    {0x2B, 0x71, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00},
    {0x2B, 0x87, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00},
    {0x2B, 0x40, 0x60, 0x00, 0xEF, 0x00, 0x00, 0x00},
    // 以下仅为测试结束/异常释放，不插入初始化序列。
    {0x2B, 0x71, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x01, 0x00, 0x00}};
  CanFrame frame;
  frame.can_id = 0x603;
  frame.dlc = 8;
  frame.data = steps[bench_init_step_];
  return frame;
}

void CANInterfaceNode::tickBrakeBenchInitialization()
{
  if (!bench_init_running_) return;
  const auto t = std::chrono::steady_clock::now();
  // 释放事务即使任务已FINISH/急停/输入失联也须继续，不能被活动状态门控挡住。
  if (bench_init_step_ < 8 && (!bench_target_seen_ ||
    std::chrono::duration<double>(t - bench_target_at_).count() > 0.5 ||
    !bench_mission_active_ || !can_online_ || last_res_state_ == kResEmergency ||
    std::chrono::duration<double>(t - bench_mission_at_).count() > 0.5 ||
    std::chrono::duration<double>(t - bench_command_at_).count() > 0.5)) {
    failBrakeBenchInitialization("bench state/input lost or emergency during initialization");
    return;
  }
  if (bench_init_waiting_) {
    if (t > bench_init_deadline_) failBrakeBenchInitialization("SDO response timeout");
    return;
  }
  if (t < bench_init_send_at_) return;
  const auto frame = brakeBenchInitFrame();
  if (!can_.send(frame)) {
    failBrakeBenchInitialization("SDO request write failed");
    return;
  }
  // 原代码模式帧后2ms，其余20ms；以发送时刻计算最小间隔，SDO应答也必须到达。
  bench_init_send_at_ = t + std::chrono::milliseconds(bench_init_step_ == 3 ? 2 : 20);
  bench_init_waiting_ = true;
  bench_init_deadline_ = t + std::chrono::milliseconds(500);
  RCLCPP_INFO(get_logger(), "Brake motor SDO step %zu: index=0x%02X%02X cmd=0x%02X",
    bench_init_step_ + 1, frame.data[2], frame.data[1], frame.data[0]);
}

void CANInterfaceNode::parseBrakeBenchSdo(const CanFrame & frame)
{
  if (!bench_init_running_ || !bench_init_waiting_ || frame.dlc < 4) return;
  const auto request = brakeBenchInitFrame();
  if (frame.data[1] != request.data[1] || frame.data[2] != request.data[2] ||
    frame.data[3] != request.data[3]) return;
  if (frame.dlc != 8 || frame.data[0] == 0x80) {
    failBrakeBenchInitialization("SDO abort or invalid response length");
    return;
  }
  if (frame.data[0] != 0x60) {
    failBrakeBenchInitialization("unexpected SDO write response");
    return;
  }
  bench_init_waiting_ = false;
  ++bench_init_step_;
  if (bench_init_step_ == 7) {
    bench_init_running_ = false;
    bench_initialized_ = true;
    publishBrakeBenchInitialized(true);
    RCLCPP_INFO(get_logger(), "Brake motor setup acknowledged: waiting for controller delay before final EF start.");
  } else if (bench_init_step_ == 8 || bench_init_step_ == 10) {
    bench_init_running_ = false;
    RCLCPP_INFO(get_logger(), "Brake motor %s acknowledged.",
      bench_init_step_ == 8 ? "EF start" : "zero torque/pause");
  }
}

void CANInterfaceNode::updateBrakeBenchMotor(double opening)
{
  if (!brake_bench_mode_ || receive_only_ || !bench_initialized_) return;
  if (last_res_state_ == kResEmergency) opening = 0.0;
  // 目标转矩已按最小代码在配置阶段写入；负Command只触发最后EF启动帧。
  if (opening <= 0.0) {
    if (!bench_motor_active_) return;
    bench_motor_active_ = false;
    bench_init_step_ = 8;  // 立即覆盖未完成的制动事务，清零后暂停。
  } else {
    if (bench_init_running_ || bench_motor_active_ || bench_motor_started_once_) return;
    bench_motor_started_once_ = true;
    bench_motor_active_ = true;
    bench_init_step_ = 7;
  }
  bench_init_running_ = true;
  bench_init_waiting_ = false;
  bench_init_send_at_ = std::chrono::steady_clock::now();
}

void CANInterfaceNode::stopBrakeBenchMotorBestEffort()
{
  if (receive_only_ || !brake_bench_mode_ || !bench_device_touched_) return;
  CanFrame zero;
  zero.can_id = 0x603; zero.dlc = 8;
  zero.data = {0x2B, 0x71, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  const bool zero_ok = can_.send(zero);
  zero.data = {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x01, 0x00, 0x00};
  const bool pause_ok = can_.send(zero);
  bench_motor_active_ = false;
  if (!zero_ok || !pause_ok) RCLCPP_ERROR(get_logger(), "Brake motor stop write failed.");
}

#endif
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

#ifndef CAN_INTERFACE_NO_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
#endif
