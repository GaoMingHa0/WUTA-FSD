#include "can_interface/can_socket.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace can_interface
{

CanSocket::~CanSocket()
{
  close();
}

bool CanSocket::open(const std::string & device)
{
  close();

  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  // 非阻塞：无数据时 read 立即返回，保证轮询节拍稳定
  const int flags = fcntl(sock_, F_GETFL, 0);
  if (flags < 0 || fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
    close();
    return false;
  }
  return true;
}

void CanSocket::close()
{
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

bool CanSocket::send(const CanFrame & frame)
{
  if (sock_ < 0) {
    return false;
  }

  struct can_frame cf {};
  cf.can_id = frame.can_id;
  cf.can_dlc = frame.dlc;
  std::memcpy(cf.data, frame.data.data(), frame.dlc);

  const ssize_t n = ::write(sock_, &cf, sizeof(cf));
  return n == static_cast<ssize_t>(sizeof(cf));
}

bool CanSocket::receive(CanFrame & frame)
{
  if (sock_ < 0) {
    return false;
  }

  struct can_frame cf {};
  const ssize_t n = ::read(sock_, &cf, sizeof(cf));
  if (n <= 0) {
    return false;  // EAGAIN 无数据，或读取错误
  }

  frame.can_id = cf.can_id & CAN_EFF_MASK;
  frame.dlc = cf.can_dlc;
  std::memcpy(frame.data.data(), cf.data, cf.can_dlc);
  return true;
}

}  // namespace can_interface
