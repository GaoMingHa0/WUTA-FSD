#pragma once

#include <string>

#include "can_interface/can_frame.hpp"

namespace can_interface
{

// 发送设备层：封装 SocketCAN 发送套接字
// 只负责把 CanFrame 写回总线，不关心业务逻辑
class CanSender
{
public:
  CanSender() = default;
  ~CanSender();

  // 打开指定 CAN 设备（如 can0），失败返回 false
  bool open(const std::string & device);
  void close();

  // 发送一帧；套接字未打开或写入失败返回 false
  bool send(const CanFrame & frame);

  bool isOpen() const { return sock_ >= 0; }

private:
  int sock_{-1};
};

}  // namespace can_interface
