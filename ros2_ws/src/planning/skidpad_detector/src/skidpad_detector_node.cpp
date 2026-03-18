#include "skidpad_detector/skidpad_detector_node.hpp"
#include <cmath>
#include <vector>

namespace skidpad_detector
{

using Cone  = wuta_msgs::msg::Cone;
using State = wuta_msgs::msg::MissionState;

SkidpadDetectorNode::SkidpadDetectorNode(const rclcpp::NodeOptions & options)
: Node("skidpad_detector_node", options)
{
  min_cones_per_circle_ = declare_parameter("min_cones_per_circle", min_cones_per_circle_);
  expected_radius_      = declare_parameter("expected_radius",       expected_radius_);
  radius_tolerance_     = declare_parameter("radius_tolerance",      radius_tolerance_);

  cone_map_sub_ = create_subscription<wuta_msgs::msg::ConeMap>(
    "/mapping/cone_map", 10,
    std::bind(&SkidpadDetectorNode::onConeMap, this, std::placeholders::_1));

  mission_sub_ = create_subscription<State>(
    "/system/mission_state", 10,
    std::bind(&SkidpadDetectorNode::onMissionState, this, std::placeholders::_1));

  circles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/planning/skidpad_circles", 10);

  RCLCPP_INFO(get_logger(), "SkidpadDetectorNode ready (min_cones=%d, r=%.3fm ±%.1fm).",
    min_cones_per_circle_, expected_radius_, radius_tolerance_);
}

void SkidpadDetectorNode::onMissionState(const State::SharedPtr msg)
{
  mission_mode_ = msg->mission_mode;
  system_state_ = msg->state;
}

void SkidpadDetectorNode::onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg)
{
  // Only run in skidpad mode while the car is active
  if (mission_mode_ != State::MISSION_SKIDPAD) return;
  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;

  // Separate blue and yellow cones
  std::vector<geometry_msgs::msg::Point> blue_pts, yellow_pts;
  for (const auto & c : msg->blue_cones) {
    blue_pts.push_back(c.position);
  }
  for (const auto & c : msg->yellow_cones) {
    yellow_pts.push_back(c.position);
  }

  const int nb = static_cast<int>(blue_pts.size());
  const int ny = static_cast<int>(yellow_pts.size());

  if (nb < min_cones_per_circle_ && ny < min_cones_per_circle_) {
    RCLCPP_DEBUG(get_logger(), "Not enough cones yet (blue=%d, yellow=%d).", nb, ny);
    return;
  }

  // Fit circles — blue → left circle, yellow → right circle (FSG convention)
  double lcx = 0, lcy = 0, lr = 0;
  double rcx = 0, rcy = 0, rr = 0;
  bool left_ok  = (nb >= min_cones_per_circle_) && fitCircle(blue_pts,   lcx, lcy, lr);
  bool right_ok = (ny >= min_cones_per_circle_) && fitCircle(yellow_pts, rcx, rcy, rr);

  if (!left_ok && !right_ok) {
    RCLCPP_WARN(get_logger(), "Circle fit failed for both clusters.");
    return;
  }

  geometry_msgs::msg::PoseArray pa;
  pa.header.stamp    = msg->header.stamp;
  pa.header.frame_id = msg->header.frame_id;

  // pose[0] = right circle centre (yellow)
  geometry_msgs::msg::Pose right_pose;
  if (right_ok) {
    right_pose.position.x  = rcx;
    right_pose.position.y  = rcy;
    right_pose.orientation.w = 1.0;
    RCLCPP_INFO(get_logger(), "Right circle: (%.2f, %.2f), r=%.2fm", rcx, rcy, rr);
  } else {
    RCLCPP_WARN(get_logger(), "Right circle fit failed — placeholder zero.");
    right_pose.orientation.w = 1.0;
  }
  pa.poses.push_back(right_pose);

  // pose[1] = left circle centre (blue)
  geometry_msgs::msg::Pose left_pose;
  if (left_ok) {
    left_pose.position.x  = lcx;
    left_pose.position.y  = lcy;
    left_pose.orientation.w = 1.0;
    RCLCPP_INFO(get_logger(), "Left  circle: (%.2f, %.2f), r=%.2fm", lcx, lcy, lr);
  } else {
    RCLCPP_WARN(get_logger(), "Left  circle fit failed — placeholder zero.");
    left_pose.orientation.w = 1.0;
  }
  pa.poses.push_back(left_pose);

  circles_pub_->publish(pa);
}

bool SkidpadDetectorNode::fitCircle(
  const std::vector<geometry_msgs::msg::Point> & pts,
  double & cx, double & cy, double & r) const
{
  const int n = static_cast<int>(pts.size());
  if (n < 3) return false;

  // Kasa method: solve [2x 2y 1] * [a; b; c] = x²+y²
  // where circle is (x-a)²+(y-b)²=r², c = r²-a²-b²
  // Normal equations: A^T A [a;b;c] = A^T rhs
  //
  // Let S = Σ, accumulate sums directly (avoids heap allocation for 3×3 solve)
  double Sx=0, Sy=0, Sxx=0, Sxy=0, Syy=0, Sxr=0, Syr=0, Sr=0;
  for (const auto & p : pts) {
    double x = p.x, y = p.y;
    double z = x*x + y*y;
    Sx  += x;  Sy  += y;
    Sxx += x*x; Sxy += x*y; Syy += y*y;
    Sxr += x*z; Syr += y*z; Sr  += z;
  }
  double sn = static_cast<double>(n);

  // A^T A (symmetric 3×3):
  // [ 4*Sxx   4*Sxy   2*Sx  ] [a]   [2*Sxr]
  // [ 4*Sxy   4*Syy   2*Sy  ] [b] = [2*Syr]
  // [ 2*Sx    2*Sy    sn    ] [c]   [Sr   ]
  //
  // Divide first two rows/rhs by 2 for numerical cleanliness:
  double A[3][3] = {
    { 2*Sxx,  2*Sxy,  Sx  },
    { 2*Sxy,  2*Syy,  Sy  },
    { 2*Sx,   2*Sy,   sn  }
  };
  double b[3] = { Sxr, Syr, Sr };

  // Gaussian elimination with partial pivoting (3×3)
  for (int col = 0; col < 3; ++col) {
    // Find pivot
    int pivot = col;
    for (int row = col+1; row < 3; ++row) {
      if (std::abs(A[row][col]) > std::abs(A[pivot][col])) pivot = row;
    }
    if (std::abs(A[pivot][col]) < 1e-10) return false;

    // Swap rows
    if (pivot != col) {
      for (int k = 0; k < 3; ++k) std::swap(A[col][k], A[pivot][k]);
      std::swap(b[col], b[pivot]);
    }

    // Eliminate
    for (int row = col+1; row < 3; ++row) {
      double factor = A[row][col] / A[col][col];
      for (int k = col; k < 3; ++k) A[row][k] -= factor * A[col][k];
      b[row] -= factor * b[col];
    }
  }

  // Back substitution
  double x[3] = {0, 0, 0};
  for (int i = 2; i >= 0; --i) {
    x[i] = b[i];
    for (int j = i+1; j < 3; ++j) x[i] -= A[i][j] * x[j];
    x[i] /= A[i][i];
  }

  cx = x[0];
  cy = x[1];
  r  = std::sqrt(x[2] + cx*cx + cy*cy);

  // Sanity check against expected radius
  if (std::abs(r - expected_radius_) > radius_tolerance_) {
    RCLCPP_WARN(get_logger(),
      "Circle fit radius %.2fm outside expected range (%.2f ± %.1f), rejecting.",
      r, expected_radius_, radius_tolerance_);
    return false;
  }

  return true;
}

}  // namespace skidpad_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<skidpad_detector::SkidpadDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
