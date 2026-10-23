// Test real receive decoding/ROS GO ordering, with an in-memory SocketCAN substitute.
#include "can_interface/can_interface.hpp"
#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
static std::vector<can_interface::CanFrame> sent;
static std::deque<can_interface::CanFrame> incoming;
static int preparation_attempts = 0;
static int fail_on = 0;
namespace can_interface {
CanSocket::~CanSocket() = default;
bool CanSocket::open(const std::string &) {return true;}
void CanSocket::close() {}
bool CanSocket::receive(CanFrame & frame) {
  if (incoming.empty()) return false;
  frame = incoming.front(); incoming.pop_front(); return true;
}
bool CanSocket::send(const CanFrame & frame) {
  if (frame.can_id == 0x603 && ++preparation_attempts == fail_on) return false;
  sent.push_back(frame); return true;
}
}
int main(int argc, char ** argv) {
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) == "0" ||
      !std::getenv("ROS_LOCALHOST_ONLY") || std::string(std::getenv("ROS_LOCALHOST_ONLY")) != "1") return 2;
  rclcpp::init(argc, argv);
  int failures = 0;
  auto check = [&](bool ok, const char * message) {
    if (!ok) {++failures; std::cerr << message << '\n';}
  };
  for (bool bench : {true, false}) {
    for (bool fail : {false, true}) {
      if (!bench && fail) continue;
      sent.clear(); incoming.clear(); preparation_attempts = 0; fail_on = fail ? 2 : 0;
      auto device = std::make_shared<can_interface::CANInterfaceNode>(rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("brake_bench_mode", bench), rclcpp::Parameter("mode_cache.enabled", false),
        rclcpp::Parameter("frames.heartbeat.period_sec", 0.0)}));
      auto probe = std::make_shared<rclcpp::Node>("bench_prepare_probe");
      auto missions = probe->create_publisher<wuta_msgs::msg::MissionState>("/system/mission_state", 10);
      auto inspections = probe->create_publisher<wuta_msgs::msg::DevicesInspection>("/system/devices_inspection", 10);
      int go_count = 0;
      bool emergency_seen = false;
      auto preparation_count = [&]() {
        int count = 0;
        for (const auto & frame : sent) if (frame.can_id == 0x603) ++count;
        return count;
      };
      auto go = probe->create_subscription<std_msgs::msg::Bool>("/system/start_command", 10,
        [&](std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            ++go_count;
            check(preparation_count() == (bench ? 6 : 0), "GO published before six alternating preparation frames / normal mode sent 0x603");
          }
        });
      auto emergency = probe->create_subscription<std_msgs::msg::Bool>("/system/emergency",
        rclcpp::QoS(1).reliable().transient_local(),
        [&](std_msgs::msg::Bool::SharedPtr msg) {emergency_seen |= msg->data;});
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(device); executor.add_node(probe);
      auto spin = [&](int ms) {
        auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end) {
          executor.spin_some(); std::this_thread::sleep_for(5ms);
        }
      };
      auto res = [&](uint8_t value) {
        can_interface::CanFrame frame; frame.can_id = 0x1E4; frame.dlc = 8; frame.data[0] = value;
        incoming.push_back(frame);
      };
      spin(500);
      wuta_msgs::msg::MissionState state;
      state.brake_bench_mode = bench; state.state = state.READY; state.mission_mode = state.MISSION_SKIDPAD;
      missions->publish(state); spin(60);
      if (bench) {
        res(0x13); spin(60);
        check(preparation_count() == 0 && go_count == 0, "Wrong mission triggered preparation/GO");
        state.mission_mode = state.MISSION_ACCELERATION;
        missions->publish(state); spin(60);
        res(0x11); res(0x13); spin(60);
        check(preparation_count() == 0 && go_count == 0, "No inspection triggered preparation/GO");
      }
      state.mission_mode = state.MISSION_ACCELERATION;
      missions->publish(state);
      wuta_msgs::msg::DevicesInspection inspection; inspection.ok = true; inspections->publish(inspection);
      spin(60);
      res(0x11); res(0x13); spin(80);
      if (fail) {
        check(preparation_attempts == 2 && preparation_count() == 1, "Send failure did not stop the preparation sequence");
        check(go_count == 0 && emergency_seen, "Failed preparation forwarded GO or failed to publish emergency");
      } else {
        check(go_count == 1, "Valid GO was not forwarded exactly once");
        check(preparation_count() == (bench ? 6 : 0), "Incorrect preparation frame count");
        check(!emergency_seen, "Successful preparation caused emergency");
      }
      const std::array<uint8_t, 8> expected[2] = {
        {0x2B, 0x71, 0x60, 0x00, 0x18, 0xFC, 0xFF, 0xFF},
        {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00}};
      int index = 0;
      for (const auto & frame : sent) if (frame.can_id == 0x603) {
        check(frame.dlc == 8 && frame.data == expected[index % 2], "0x603 payload/DLC/order differs from requested A/B sequence");
        ++index;
      }
      if (bench) {
        res(0x13); res(0x11); res(0x13); spin(80);
        check(go_count == (fail ? 0 : 1), "Repeated GO restarted bench preparation");
        check(preparation_attempts == (fail ? 2 : 6), "Preparation was retransmitted after success/failure");
      }
      executor.remove_node(device); device.reset();
    }
  }
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
