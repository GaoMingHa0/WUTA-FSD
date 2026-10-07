#pragma once

#include <autoware_msgs/msg/lane.hpp>
#include <autoware_msgs/msg/waypoint.hpp>
#include "controller/vehicle_state.hpp"
#include <cmath>
#include <vector>

namespace controller
{

struct ControlCommand
{
  double steering_angle{0.0};  // deg，左正
  double velocity{0.0};        // m/s
  bool   valid{false};
};

// Pure Pursuit 横向控制：按前视距离选目标点，算前轮转角。
class PurePursuit
{
public:
  struct Config
  {
    double ld_ratio{2.0};                     // 前视 = 车速 × 该系数
    double min_lookahead{2.0};                // m，低速前视下限
    double max_lookahead{20.0};               // m，高速前视上限
    int    max_progress_advance{4};           // 单周期最多推进的路径点数
    double terminal_progress_distance{0.75};  // m，终点进度点触发距离
    double course_speed_threshold{0.5};       // m/s，低于此速用 yaw 作航向
    double forward_margin{0.5};               // m，目标点前向接纳门限
  };

  PurePursuit(const VehicleParams & params, const Config & cfg);

  // 切换赛项时更新参数
  void setConfig(const Config & cfg) { cfg_ = cfg; }

  ControlCommand compute(const VehicleState & state,
                         const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
                         double lookahead_override = 0.0);

  double lookaheadDistance() const { return lookahead_dist_; }
  int    targetIndex()       const { return target_idx_; }
  int    progressIndex()     const { return progress_idx_; }
  void   reset();

private:
  int findTargetIndex(const VehicleState & state,
                      const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
                      double ld, double course) const;

  int findNearestForwardIndex(
    const VehicleState & state,
    const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
    double course) const;

  // 目标点相对车体的横向偏移（左正）
  static double lateralOffset(double target_x, double target_y,
                               double car_x, double car_y, double course_angle);

  // 目标点相对车体的前向偏移（前方为正）
  static double longitudinalOffset(double target_x, double target_y,
                                   double car_x, double car_y, double course_angle);

  static double planeDist(double ax, double ay, double bx, double by);

  VehicleParams params_;
  Config cfg_;

  double lookahead_dist_{0.0};
  int    target_idx_{0};
  int    progress_idx_{0};
};

}  // namespace controller
