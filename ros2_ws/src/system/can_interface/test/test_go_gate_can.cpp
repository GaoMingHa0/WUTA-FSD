// Validate the RES GO interlock: 0x211 Byte1 7->8 plus wait time, otherwise GO is dropped.
#include "can_interface/can_interface.hpp"
#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
static std::deque<can_interface::CanFrame> incoming;
namespace can_interface {
CanSocket::~CanSocket() = default;
bool CanSocket::open(const std::string &) {return true;}
void CanSocket::close() {}
bool CanSocket::receive(CanFrame & frame) {
  if (incoming.empty()) return false;
  frame = incoming.front(); incoming.pop_front(); return true;
}
bool CanSocket::send(const CanFrame &) {return true;}
}

static void push(uint32_t id, uint8_t value)
{
  can_interface::CanFrame frame;
  frame.can_id = id;
  frame.dlc = 8;
  frame.data[0] = value;
  incoming.push_back(frame);
}

int main(int argc, char ** argv)
{
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) == "0" ||
      !std::getenv("ROS_LOCALHOST_ONLY") || std::string(std::getenv("ROS_LOCALHOST_ONLY")) != "1") return 2;
  rclcpp::init(argc, argv);
  auto probe = std::make_shared<rclcpp::Node>("go_gate_probe");
  int go_count = 0;
  auto go = probe->create_subscription<std_msgs::msg::Bool>("/system/start_command", 10,
    [&](std_msgs::msg::Bool::SharedPtr msg) {if (msg->data) ++go_count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);

  auto spin = [&](int ms) {
    auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < until) {
      executor.spin_some(); std::this_thread::sleep_for(5ms);
    }
  };
  auto press_go = [&]() {
    push(0x1E4, 0x11); push(0x1E4, 0x13); spin(80);
  };

  auto device = std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("receive_only", true),
    rclcpp::Parameter("mode_cache.enabled", false),
    rclcpp::Parameter("go_gate.wait_sec", 0.3),
    rclcpp::Parameter("go_gate.stale_sec", 1.0)}));
  executor.add_node(device);
  spin(300);

  int failures = 0;
  auto check = [&](bool ok, const char * what) {
    if (!ok) {std::cerr << "FAIL: " << what << " (go_count=" << go_count << ")\n"; ++failures;}
  };

  // 1. No 0x211 at all -> GO dropped.
  press_go();
  check(go_count == 0, "GO accepted without 0x211");

  // 2. 7->8 seen but wait not elapsed -> GO dropped.
  push(0x211, 7); spin(40);
  push(0x211, 8); spin(80);
  press_go();
  check(go_count == 0, "GO accepted before 5s window");

  // 3. After the wait elapses -> GO accepted once.
  spin(300);
  push(0x211, 8); spin(20);  // refresh freshness
  press_go();
  check(go_count == 1, "GO not accepted after self-check window");

  // 4. Leaving state 8 closes the gate again.
  push(0x211, 7); spin(40);
  press_go();
  check(go_count == 1, "GO accepted after leaving state 8");

  // 5. A new 7->8 window reopens the gate.
  push(0x211, 8); spin(40);
  push(0x211, 8);
  spin(320);
  push(0x211, 8); spin(20);
  press_go();
  check(go_count == 2, "GO not accepted after second self-check window");

  // 6. Stale 0x211 (no frames for > stale_sec) closes the gate.
  spin(1200);
  press_go();
  check(go_count == 2, "GO accepted with stale VCU state");

  // 7. Explicitly disabled gate restores the old behaviour.
  executor.remove_node(device); device.reset();
  auto bypass = std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("receive_only", true),
    rclcpp::Parameter("mode_cache.enabled", false),
    rclcpp::Parameter("go_gate.enabled", false)}));
  executor.add_node(bypass);
  spin(200);
  press_go();
  check(go_count == 3, "GO not accepted when gate disabled");

  executor.remove_node(bypass); bypass.reset();
  executor.remove_node(probe);
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
