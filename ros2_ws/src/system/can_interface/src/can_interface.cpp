#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "wuta_msgs/msg/mission_state.hpp"

namespace can_interface
{

class CANInterfaceNode : public rclcpp::Node
{
public:
  CANInterfaceNode(const rclcpp::NodeOptions & options)
  : Node("can_interface", options)
  {
    declare_parameter<std::string>("can_device", "can0");
    declare_parameter<int>("can_baud_rate", 500000);

    mission_state_sub_ = create_subscription<wuta_msgs::msg::MissionState>(
      "/system/mission_state", 10,
      std::bind(&CANInterfaceNode::onMissionState, this, std::placeholders::_1));

    inspection_result_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/inspection_result", 10,
      std::bind(&CANInterfaceNode::onInspectionResult, this, std::placeholders::_1));

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

    RCLCPP_INFO(get_logger(), "CAN Interface stub initialized.");
    RCLCPP_INFO(get_logger(), "Note: This is a placeholder for hardware CAN implementation.");
    RCLCPP_INFO(get_logger(), "Implement CAN bus read/write logic before using on real vehicle.");
  }

private:
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received mission_state: state=%d mode=%d",
      msg->state, msg->mission_mode);
    // TODO: 转发到 CAN 总线
  }

  void onInspectionResult(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received inspection_result: %s", msg->data.c_str());
    // TODO: 转发到 CAN 总线
  }

  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inspection_result_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inspection_trigger_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
};

}  // namespace can_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<can_interface::CANInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}