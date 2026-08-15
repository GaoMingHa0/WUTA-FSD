#include "can_interface/can_sender.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace can_interface
{

CanSender::~CanSender()
{
  close();
}

bool CanSender::open(const std::string & device)
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
  return true;
}

void CanSender::close()
{
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

bool CanSender::send(const CanFrame & frame)
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

}  // namespace can_interface
