// Exercise real callbacks and receive decoding with an in-memory CAN device.
#include "can_interface/can_interface.hpp"
#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
static std::vector<can_interface::CanFrame> sent;
static std::deque<can_interface::CanFrame> incoming;
namespace can_interface {
CanSocket::~CanSocket() = default;
bool CanSocket::open(const std::string &) {return true;}
void CanSocket::close() {}
bool CanSocket::receive(CanFrame & frame) {
  if (incoming.empty()) return false;
  frame = incoming.front(); incoming.pop_front(); return true;
}
bool CanSocket::send(const CanFrame & frame) {sent.push_back(frame); return true;}
}
int main(int argc, char ** argv) {
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) == "0" ||
      !std::getenv("ROS_LOCALHOST_ONLY") || std::string(std::getenv("ROS_LOCALHOST_ONLY")) != "1") return 2;
  rclcpp::init(argc, argv);
  auto device = std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("receive_only", true), rclcpp::Parameter("mode_cache.enabled", false)}));
  auto probe = std::make_shared<rclcpp::Node>("receive_only_probe");
  auto commands = probe->create_publisher<autoware_msgs::msg::Command>("/control/command", 10);
  auto missions = probe->create_publisher<wuta_msgs::msg::MissionState>("/system/mission_state", 10);
  auto inspections = probe->create_publisher<wuta_msgs::msg::DevicesInspection>("/system/devices_inspection", 10);
  bool mode_received = false;
  auto modes = probe->create_subscription<std_msgs::msg::String>("/system/mission_mode_cmd", 10,
    [&](std_msgs::msg::String::SharedPtr msg) {mode_received |= msg->data == "acceleration";});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(device); executor.add_node(probe);
  auto spin = [&](int ms) {
    auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < until) {
      executor.spin_some(); std::this_thread::sleep_for(10ms);
    }
  };
  spin(700);
  for (int i = 0; i < 20; ++i) {
    autoware_msgs::msg::Command cmd; cmd.throttle_brake = 1; cmd.angle = 28;
    wuta_msgs::msg::DevicesInspection check; check.ok = true;
    wuta_msgs::msg::MissionState state; state.state = state.EXPLORE;
    commands->publish(cmd); missions->publish(state); inspections->publish(check);
    spin(30);
  }
  can_interface::CanFrame frame; frame.can_id = 0x501; frame.dlc = 1; frame.data[0] = 2;
  incoming.push_back(frame); spin(300);
  executor.remove_node(device); device.reset();
  const bool ok = sent.empty() && mode_received;
  if (!ok) std::cerr << "receive-only: sent=" << sent.size() << " mode_received=" << mode_received << '\n';
  rclcpp::shutdown(); return ok ? 0 : 1;
}
