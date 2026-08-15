#pragma once

#include <array>
#include <cstdint>

namespace can_interface
{

// CAN 数据帧（标准帧 11bit ID，最多 8 字节数据）
// VCU-工控机 与 工控机-VCU 各一帧；
// 报文 ID 与字节布局未确定，编码/解析在节点层 TODO 实现
struct CanFrame
{
  uint32_t can_id{0};                    // 报文 ID（预留）
  uint8_t dlc{8};                        // 数据长度
  std::array<uint8_t, 8> data{};         // 数据字节（预留）
};

}  // namespace can_interface
