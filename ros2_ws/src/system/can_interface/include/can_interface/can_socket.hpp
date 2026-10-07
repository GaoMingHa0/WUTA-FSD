#pragma once

#include <string>

#include "can_interface/can_frame.hpp"

namespace can_interface
{

// 设备层：单个 SocketCAN 套接字，双向收发
// 非阻塞读保证轮询节拍稳定；发送失败由上层兜底
class CanSocket
{
public:
  CanSocket() = default;
  ~CanSocket();

  // 打开指定 CAN 设备（如 can0），失败返回 false
  bool open(const std::string & device);
  void close();

  // 发送一帧；套接字未打开或写入失败返回 false
  bool send(const CanFrame & frame);

  // 非阻塞读一帧；无数据或错误返回 false
  bool receive(CanFrame & frame);

  bool isOpen() const { return sock_ >= 0; }

private:
  int sock_{-1};
};

}  // namespace can_interface
