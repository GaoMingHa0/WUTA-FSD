#include "mission_manager/mission_manager.hpp"

namespace mission_manager
{

using State = wuta_msgs::msg::MissionState;

MissionManager::MissionManager(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  // Default mission mode from parameter
  const std::string mode_str = declare_parameter<std::string>("mission_mode", "trackdrive");
  if (mode_str == "skidpad")       mission_mode_ = State::MISSION_SKIDPAD;
  else if (mode_str == "acceleration") mission_mode_ = State::MISSION_ACCELERATION;
  else                             mission_mode_ = State::MISSION_TRACKDRIVE;

  // Publishers
  state_pub_ = create_publisher<State>("/system/mission_state", 10);
  inspection_result_pub_ = create_publisher<std_msgs::msg::String>(
    "/system/inspection_result", 10);  // 预留，车检结果输出

  // Subscribers — normal mission
  cone_map_sub_ = create_subscription<wuta_msgs::msg::ConeMap>(
    "/mapping/cone_map", 10,
    std::bind(&MissionManager::onConeMap, this, std::placeholders::_1));

  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency", 10,
    std::bind(&MissionManager::onEmergency, this, std::placeholders::_1));

  mission_mode_sub_ = create_subscription<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10,
    std::bind(&MissionManager::onMissionModeCmd, this, std::placeholders::_1));

  lidar_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/lidar_ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      lidar_ready_ = msg->data;
      if (lidar_ready_ && localization_ready_ && current_state_ == State::IDLE) {
        transitionTo(State::READY);
      }
    });

  localization_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/localization_ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      localization_ready_ = msg->data;
      if (lidar_ready_ && localization_ready_ && current_state_ == State::IDLE) {
        transitionTo(State::READY);
      }
    });

  // ---------------------------------------------------------------------------
  // INSPECTION interface — 预留，暂不接其他模块
  // 发布 true 到此 topic 触发车检流程
  // ---------------------------------------------------------------------------
  inspection_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/inspection_trigger", 10,
    std::bind(&MissionManager::onInspectionTrigger, this, std::placeholders::_1));

  // Periodic state broadcast at 10 Hz
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionManager::publishState, this));

  RCLCPP_INFO(get_logger(), "Mission Manager initialized. mode=%s state=IDLE",
    mode_str.c_str());
}

void MissionManager::transitionTo(uint8_t new_state)
{
  const auto state_name = [](uint8_t s) -> std::string {
    switch (s) {
      case State::IDLE:         return "IDLE";
      case State::READY:        return "READY";
      case State::INSPECTION:   return "INSPECTION";
      case State::EXPLORE:      return "EXPLORE";
      case State::MAPPING_DONE: return "MAPPING_DONE";
      case State::RACE:         return "RACE";
      case State::FINISH:       return "FINISH";
      case State::EMERGENCY:    return "EMERGENCY";
      default:                  return "UNKNOWN";
    }
  };

  RCLCPP_INFO(get_logger(), "State: %s → %s",
    state_name(current_state_).c_str(), state_name(new_state).c_str());

  current_state_ = new_state;

  if (new_state == State::RACE) {
    localization_mode_ = State::LOC_NDT;
    RCLCPP_INFO(get_logger(), "Localization: NDT map matching");
  } else if (new_state == State::EXPLORE) {
    localization_mode_ = State::LOC_KISS_ICP;
    RCLCPP_INFO(get_logger(), "Localization: KISS-ICP + EKF");
  }

  publishState();
}

void MissionManager::publishState()
{
  State msg;
  msg.header.stamp    = now();
  msg.state           = current_state_;
  msg.mission_mode    = mission_mode_;
  msg.localization_mode = localization_mode_;
  state_pub_->publish(msg);
}

void MissionManager::onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg)
{
  if (msg->is_closed && !map_closed_ && current_state_ == State::EXPLORE) {
    map_closed_ = true;
    RCLCPP_INFO(get_logger(), "Cone map closed. %zu blue + %zu yellow cones.",
      msg->blue_cones.size(), msg->yellow_cones.size());
    transitionTo(State::MAPPING_DONE);
    // TODO: wait for NDT map build completion, then transitionTo(State::RACE)
  }
}

void MissionManager::onEmergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY triggered!");
    transitionTo(State::EMERGENCY);
  }
}

void MissionManager::onMissionModeCmd(const std_msgs::msg::String::SharedPtr msg)
{
  if (current_state_ != State::IDLE && current_state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Cannot change mission mode in state %d", current_state_);
    return;
  }
  if (msg->data == "trackdrive")    mission_mode_ = State::MISSION_TRACKDRIVE;
  else if (msg->data == "skidpad")  mission_mode_ = State::MISSION_SKIDPAD;
  else if (msg->data == "acceleration") mission_mode_ = State::MISSION_ACCELERATION;
  else {
    RCLCPP_WARN(get_logger(), "Unknown mission mode: %s", msg->data.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Mission mode set to: %s", msg->data.c_str());
  publishState();
}

// ---------------------------------------------------------------------------
// INSPECTION — 预留接口，暂未接入其他模块
// ---------------------------------------------------------------------------

void MissionManager::onInspectionTrigger(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;

  if (current_state_ != State::IDLE && current_state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Inspection only available in IDLE/READY state.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Inspection triggered.");
  transitionTo(State::INSPECTION);
  runInspection();
  sendInspectionCAN();

  // Return to READY after inspection
  transitionTo(State::READY);
}

void MissionManager::runInspection()
{
  // TODO: 检查各传感器 topic 是否在线（LiDAR、相机、CG-410）
  // TODO: 检查 TF tree 是否完整
  // TODO: 发布检查结果到 /system/inspection_result

  RCLCPP_INFO(get_logger(), "[INSPECTION] Sensor check — not yet implemented.");

  std_msgs::msg::String result;
  result.data = "INSPECTION_NOT_IMPLEMENTED";
  inspection_result_pub_->publish(result);
}

void MissionManager::sendInspectionCAN()
{
  // TODO: 通过 CAN 接口向 VCU 发送车检测试报文
  // 建议通过 can_msgs::msg::Frame 发布到 /can/tx topic
  // 具体报文格式需根据 VCU 协议文档确定

  RCLCPP_INFO(get_logger(), "[INSPECTION] VCU CAN test — not yet implemented.");
}

}  // namespace mission_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_manager::MissionManager>());
  rclcpp::shutdown();
  return 0;
}
