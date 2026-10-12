#include "controller/speed_pid.hpp"
#include <cmath>
#include <iostream>

int main()
{
  using namespace controller;
  int failures = 0;
  const auto check = [&](bool ok, const char * message) {
      if (!ok) {std::cerr << message << '\n'; ++failures;}
    };
  SpeedPidConfig c{0.3, 0.05, 0.1, 1.0, 0.2, 1.0};
  double integral = 0.0;
  // 起步微分冲击与持续零速反馈均不能突破油门上限。
  check(speedPidEffort(1, 0.02, 0, c, integral) == 0.2, "起步油门超过20%");
  for (int i = 0; i < 10000; ++i) {
    check(speedPidEffort(1, 0.02, 1, c, integral) == 0.2, "持续误差突破油门上限");
  }
  check(integral == 0.0, "油门饱和时积分继续累积");
  check(speedPidEffort(0, 0.02, 0, c, integral) == 0.0, "误差消失后残余积分驱动");
  check(speedPidEffort(-10, 0.02, -10, c, integral) == -1.0, "油门上限削弱了满制动");
  c.max_throttle = 0;
  check(speedPidEffort(1, 0.02, 1, c, integral) == 0.0, "零油门上限未生效");
  check(speedPidEffort(-1, 0.02, -1, c, integral) < -0.2, "零油门上限阻止制动");
  c.max_throttle = 1;
  integral = 0;
  check(speedPidEffort(10, 0.02, 10, c, integral) == 1.0, "其他赛项默认满量程被限制");
  c.max_throttle = 0.2;
  integral = 0;
  const double effort = speedPidEffort(0.1, 0.1, 0.1, c, integral);
  check(std::abs(effort - 0.0305) < 1e-12, "未饱和时正常积分失效");
  // 制动上限独立于油门，负向饱和同样阻止积分积累。
  c.max_throttle = 0.8;
  c.max_brake = 0.1;
  integral = 0;
  for (int i = 0; i < 10000; ++i) {
    check(speedPidEffort(-1, 0.02, -1, c, integral) == -0.1, "制动超过独立上限");
  }
  check(integral == 0.0, "制动饱和时积分继续累积");
  check(speedPidEffort(10, 0.02, 10, c, integral) == 0.8, "制动限幅影响了油门上限");
  if (!failures) {std::cout << "PID油门限幅、积分防饱和、完整制动测试通过\n";}
  return failures ? 1 : 0;
}
