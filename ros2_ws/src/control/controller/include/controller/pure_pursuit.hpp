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
  double steering_angle{0.0};  // degrees, positive = left
  double velocity{0.0};        // m/s
  bool   valid{false};
};

/**
 * Pure Pursuit lateral + velocity-from-waypoint longitudinal controller.
 * Pure C++ — no ROS dependencies.
 *
 * Algorithm (from HRT-D, stripped of HPL):
 *   1. Compute lookahead distance: LD = velocity × ld_ratio
 *   2. Find target waypoint: first waypoint farther than LD
 *   3. Transform target to vehicle body frame
 *   4. Compute curvature: kappa = 2·x_body / dist²
 *   5. Steering angle: δ = atan(wheelBase × kappa) [deg]
 *   6. Velocity: from target waypoint twist
 */
class PurePursuit
{
public:
  struct Config
  {
    double ld_ratio{2.0};         // lookahead = velocity × ratio
    double min_lookahead{2.0};    // m — clamp at low speed
    double max_lookahead{20.0};   // m — clamp at high speed
  };

  explicit PurePursuit(const VehicleParams & params, const Config & cfg);

  /**
   * Compute control command for one cycle.
   * @param state    Current vehicle state (pose + velocity)
   * @param waypoints  Reference path (autoware_msgs Lane waypoints)
   */
  ControlCommand compute(const VehicleState & state,
                         const std::vector<autoware_msgs::msg::Waypoint> & waypoints);

  // Accessors for diagnostics
  double lookaheadDistance() const { return lookahead_dist_; }
  int    targetIndex()       const { return target_idx_; }

private:
  int findTargetIndex(const VehicleState & state,
                      const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
                      double ld) const;

  // Transform global point to vehicle body frame, return lateral offset x
  static double lateralOffset(double target_x, double target_y,
                               double car_x, double car_y, double car_yaw);

  static double planeDist(double ax, double ay, double bx, double by);

  VehicleParams params_;
  Config cfg_;

  double lookahead_dist_{0.0};
  int    target_idx_{0};
};

}  // namespace controller
