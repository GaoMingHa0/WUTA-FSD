#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>

namespace boundary_detector
{
struct LinePoint {double x, y;};
struct AccelerationLineConfig
{
  int min_cones{3};
  double inlier_distance{0.25}, min_inlier_ratio{0.65}, min_span{4.0};
  double min_width{1.5}, max_width{7.5};
  double max_heading_angle{0.60}, max_parallel_angle{0.14};
  double forward_range{40.0}, lateral_range{8.0};
};
struct AccelerationLines
{
  bool valid{false};
  double yaw{0}, left{0}, right{0}, confidence{0};
};

inline double lineAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline LinePoint centroid(const std::vector<LinePoint> & points)
{
  LinePoint c{0, 0};
  for (const auto & p : points) {c.x += p.x; c.y += p.y;}
  c.x /= points.size(); c.y /= points.size();
  return c;
}

// Deterministic two-point RANSAC. Inputs are bounded, deduplicated map cones.
inline std::vector<LinePoint> accelerationInliers(
  const std::vector<LinePoint> & points, double heading,
  const AccelerationLineConfig & cfg)
{
  std::vector<LinePoint> best;
  double best_error = 1e100;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      const double dx = points[j].x - points[i].x, dy = points[j].y - points[i].y;
      const double length = std::hypot(dx, dy);
      if (length < cfg.min_span * 0.5) continue;
      double yaw = std::atan2(dy, dx);
      if (std::cos(yaw - heading) < 0) yaw = lineAngle(yaw + std::acos(-1.0));
      if (std::abs(lineAngle(yaw - heading)) > cfg.max_heading_angle) continue;
      const double nx = -std::sin(yaw), ny = std::cos(yaw);
      std::vector<LinePoint> inliers;
      double error = 0;
      for (const auto & p : points) {
        const double residual = std::abs(nx * (p.x - points[i].x) + ny * (p.y - points[i].y));
        if (residual <= cfg.inlier_distance) {
          inliers.push_back(p); error += residual * residual;
        }
      }
      if (inliers.size() > best.size() || (inliers.size() == best.size() && error < best_error)) {
        best = std::move(inliers); best_error = error;
      }
    }
  }
  if (best.size() < static_cast<std::size_t>(cfg.min_cones) ||
    best.size() < cfg.min_inlier_ratio * points.size()) return {};
  return best;
}

inline double scatterYaw(const std::vector<LinePoint> & points, double heading)
{
  const auto c = centroid(points);
  double xx = 0, xy = 0, yy = 0;
  for (const auto & p : points) {
    xx += (p.x-c.x)*(p.x-c.x); xy += (p.x-c.x)*(p.y-c.y); yy += (p.y-c.y)*(p.y-c.y);
  }
  double yaw = 0.5 * std::atan2(2*xy, xx-yy);
  if (std::cos(yaw-heading) < 0) yaw = lineAngle(yaw + std::acos(-1.0));
  return yaw;
}

// Joint orthogonal least squares with a shared tangent and separate offsets.
inline AccelerationLines fitAccelerationLines(
  const std::vector<LinePoint> & red, const std::vector<LinePoint> & blue,
  double heading, const AccelerationLineConfig & cfg)
{
  AccelerationLines result;
  const auto left = accelerationInliers(red, heading, cfg);
  const auto right = accelerationInliers(blue, heading, cfg);
  if (left.empty() || right.empty()) return result;
  if (std::abs(lineAngle(scatterYaw(left, heading)-scatterYaw(right, heading))) >
    cfg.max_parallel_angle) return result;
  double xx = 0, xy = 0, yy = 0;
  for (const auto * side : {&left, &right}) {
    const auto c = centroid(*side);
    for (const auto & p : *side) {
      xx += (p.x-c.x)*(p.x-c.x); xy += (p.x-c.x)*(p.y-c.y); yy += (p.y-c.y)*(p.y-c.y);
    }
  }
  double yaw = 0.5 * std::atan2(2*xy, xx-yy);
  if (std::cos(yaw-heading) < 0) yaw = lineAngle(yaw + std::acos(-1.0));
  if (std::abs(lineAngle(yaw-heading)) > cfg.max_heading_angle) return result;
  const double tx = std::cos(yaw), ty = std::sin(yaw), nx = -ty, ny = tx;
  const auto a = centroid(left), b = centroid(right);
  result.left = nx*a.x + ny*a.y; result.right = nx*b.x + ny*b.y;
  const double width = result.left-result.right;
  if (width < cfg.min_width || width > cfg.max_width) return {};
  double rms = 0;
  std::vector<double> starts, ends;
  for (const auto * side : {&left, &right}) {
    const double offset = side == &left ? result.left : result.right;
    double first = 1e100, last = -1e100;
    for (const auto & p : *side) {
      const double s = tx*p.x+ty*p.y, e = nx*p.x+ny*p.y-offset;
      first = std::min(first, s); last = std::max(last, s); rms += e*e;
    }
    starts.push_back(first); ends.push_back(last);
  }
  if (std::min(ends[0], ends[1])-std::max(starts[0], starts[1]) < cfg.min_span) return {};
  rms = std::sqrt(rms/(left.size()+right.size()));
  if (rms > cfg.inlier_distance) return {};
  result.valid = true; result.yaw = yaw;
  result.confidence = std::min(double(left.size())/red.size(), double(right.size())/blue.size()) *
    (1.0-0.5*std::min(1.0, rms/cfg.inlier_distance));
  return result;
}
}  // namespace boundary_detector
