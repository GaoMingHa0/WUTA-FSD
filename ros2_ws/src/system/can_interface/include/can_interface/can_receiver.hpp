#pragma once

#include <string>

#include "can_interface/can_frame.hpp"

namespace can_interface
{

// 接收设备层：封装 SocketCAN 接收套接字（非阻塞）
// 非阻塞保证轮询节拍稳定，无数据时立即返回
class CanReceiver
{
public:
  CanReceiver() = default;
  ~CanReceiver();

  // 打开指定 CAN 设备（如 can0），失败返回 false
  bool open(const std::string & device);
  void close();

  // 非阻塞读一帧；无数据或错误返回 false
  bool receive(CanFrame & frame);

  bool isOpen() const { return sock_ >= 0; }

private:
  int sock_{-1};
};

}  // namespace can_interface
