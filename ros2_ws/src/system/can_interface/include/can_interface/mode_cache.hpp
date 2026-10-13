#pragma once
#include <cstdint>
#include <string>
namespace can_interface {
struct ModeSnapshot {
  uint8_t mode{};
  uint64_t sequence{};
  double age_sec{};
  std::string session;
};
bool readModeCache(const std::string & directory, const std::string & iface,
  uint32_t id, double max_age_sec, ModeSnapshot & snapshot, std::string & reason);
}
