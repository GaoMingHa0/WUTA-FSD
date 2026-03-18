#include "line_detector/line_detector_node.hpp"
#include <cmath>
#include <vector>

namespace line_detector
{

using State = wuta_msgs::msg::MissionState;

LineDetectorNode::LineDetectorNode(const rclcpp::NodeOptions & options)
: Node("line_detector_node", options)
{
  min_cones_       = declare_parameter("min_cones",       min_cones_);
  endpoint_buffer_ = declare_parameter("endpoint_buffer", endpoint_buffer_);

  cone_map_sub_ = create_subscription<wuta_msgs::msg::ConeMap>(
    "/mapping/cone_map", 10,
    std::bind(&LineDetectorNode::onConeMap, this, std::placeholders::_1));

  mission_sub_ = create_subscription<State>(
    "/system/mission_state", 10,
    std::bind(&LineDetectorNode::onMissionState, this, std::placeholders::_1));

  line_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/planning/acceleration_line", 10);

  RCLCPP_INFO(get_logger(), "LineDetectorNode ready (min_cones=%d, buffer=%.1fm).",
    min_cones_, endpoint_buffer_);
}

void LineDetectorNode::onMissionState(const State::SharedPtr msg)
{
  mission_mode_ = msg->mission_mode;
  system_state_ = msg->state;
}

void LineDetectorNode::onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg)
{
  if (mission_mode_ != State::MISSION_ACCELERATION) return;
  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;

  // Collect all cone positions (blue + yellow + orange; ignore unknown)
  std::vector<double> xs, ys;
  for (const auto & c : msg->blue_cones) {
    xs.push_back(c.position.x);
    ys.push_back(c.position.y);
  }
  for (const auto & c : msg->yellow_cones) {
    xs.push_back(c.position.x);
    ys.push_back(c.position.y);
  }
  for (const auto & c : msg->orange_cones) {
    xs.push_back(c.position.x);
    ys.push_back(c.position.y);
  }

  const int n = static_cast<int>(xs.size());
  if (n < min_cones_) {
    RCLCPP_DEBUG(get_logger(), "Too few cones (%d/%d), skipping.", n, min_cones_);
    return;
  }

  // 1. Centroid
  double mx = 0, my = 0;
  for (int i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; }
  mx /= n;  my /= n;

  // 2. 2×2 covariance matrix
  double Sxx = 0, Sxy = 0, Syy = 0;
  for (int i = 0; i < n; ++i) {
    double dx = xs[i] - mx, dy = ys[i] - my;
    Sxx += dx*dx;
    Sxy += dx*dy;
    Syy += dy*dy;
  }

  // 3. Principal eigenvector of [[Sxx,Sxy],[Sxy,Syy]] (analytic formula for 2×2)
  //    eigenvalues: λ = (Sxx+Syy)/2 ± sqrt(((Sxx-Syy)/2)^2 + Sxy^2)
  //    principal eigenvector for λ_max:
  double trace_half = (Sxx + Syy) * 0.5;
  double disc = std::sqrt(std::pow((Sxx - Syy) * 0.5, 2.0) + Sxy * Sxy);
  double lambda_max = trace_half + disc;

  // Eigenvector corresponding to lambda_max
  double evx, evy;
  if (std::abs(Sxy) > 1e-10) {
    evx = lambda_max - Syy;
    evy = Sxy;
  } else {
    // Already diagonal — largest diagonal gives principal axis
    evx = (Sxx >= Syy) ? 1.0 : 0.0;
    evy = (Sxx >= Syy) ? 0.0 : 1.0;
  }
  double ev_norm = std::sqrt(evx*evx + evy*evy);
  evx /= ev_norm;
  evy /= ev_norm;

  // 4. Project all cones onto principal axis; find max projection
  double max_proj = -1e9;
  for (int i = 0; i < n; ++i) {
    double proj = (xs[i] - mx) * evx + (ys[i] - my) * evy;
    if (proj > max_proj) max_proj = proj;
  }

  // 5. Endpoint = centroid + (max_proj + buffer) * direction
  double endpoint_x = mx + (max_proj + endpoint_buffer_) * evx;
  double endpoint_y = my + (max_proj + endpoint_buffer_) * evy;

  // 6. Build heading quaternion from yaw
  double yaw = std::atan2(evy, evx);
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp    = msg->header.stamp;
  out.header.frame_id = msg->header.frame_id;
  out.pose.position.x = endpoint_x;
  out.pose.position.y = endpoint_y;
  out.pose.position.z = 0.0;
  // quaternion from yaw: q = [0, 0, sin(yaw/2), cos(yaw/2)]
  out.pose.orientation.z = std::sin(yaw * 0.5);
  out.pose.orientation.w = std::cos(yaw * 0.5);

  line_pub_->publish(out);

  RCLCPP_INFO(get_logger(),
    "Acceleration line: heading=%.1f°, endpoint=(%.2f, %.2f), track_length~=%.1fm",
    yaw * 180.0 / M_PI, endpoint_x, endpoint_y, max_proj + endpoint_buffer_);
}

}  // namespace line_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<line_detector::LineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
