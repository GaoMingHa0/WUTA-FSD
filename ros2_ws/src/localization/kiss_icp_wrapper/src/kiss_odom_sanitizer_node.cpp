#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace kiss_icp_wrapper
{

class KissOdomSanitizerNode : public rclcpp::Node
{
public:
  KissOdomSanitizerNode()
  : Node("kiss_odom_sanitizer_node")
  {
    input_topic_ = declare_parameter("input_topic", "/kiss/odometry");
    output_topic_ = declare_parameter("output_topic", "/kiss/odometry_sanitized");
    ins_topic_ = declare_parameter("ins_topic", "/chcnav/odometry");
    ins_timeout_sec_ = declare_parameter("ins_timeout_sec", 0.25);
    max_linear_speed_ = declare_parameter("max_linear_speed", 20.0);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 3.0);
    linear_step_margin_ = declare_parameter("linear_step_margin", 0.75);
    yaw_step_margin_ = declare_parameter("yaw_step_margin", 0.35);
    max_ins_speed_disagreement_ = declare_parameter("max_ins_speed_disagreement", 5.0);
    max_ins_yaw_rate_disagreement_ = declare_parameter(
      "max_ins_yaw_rate_disagreement", 1.0);

    output_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    ins_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ins_topic_, 20,
      std::bind(&KissOdomSanitizerNode::onIns, this, std::placeholders::_1));
    kiss_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 10,
      std::bind(&KissOdomSanitizerNode::onKiss, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "KISS odometry sanitizer: %s -> %s, reference=%s.",
      input_topic_.c_str(), output_topic_.c_str(), ins_topic_.c_str());
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static bool poseIsFinite(const geometry_msgs::msg::Pose & pose)
  {
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  static void writeYaw(geometry_msgs::msg::Quaternion & q, double yaw)
  {
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
  }

  void onIns(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!poseIsFinite(msg->pose.pose)) return;
    latest_ins_stamp_ = rclcpp::Time(msg->header.stamp);
    latest_ins_speed_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    latest_ins_yaw_rate_ = msg->twist.twist.angular.z;
    ins_ready_ = std::isfinite(latest_ins_speed_) &&
      std::isfinite(latest_ins_yaw_rate_);
  }

  void onKiss(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!poseIsFinite(msg->pose.pose)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting non-finite KISS odometry.");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = yawFromQuaternion(msg->pose.pose.orientation);
    if (!std::isfinite(yaw)) return;

    if (!baseline_ready_) {
      gated_x_ = x;
      gated_y_ = y;
      gated_yaw_ = yaw;
      updateBaseline(stamp, x, y, yaw);
      publishSanitized(*msg, 0.0, 0.0, 0.0);
      return;
    }

    const double dt = (stamp - last_kiss_stamp_).seconds();
    const double dx = x - last_kiss_x_;
    const double dy = y - last_kiss_y_;
    const double dyaw = normalizeAngle(yaw - last_kiss_yaw_);
    const double reference_yaw = last_kiss_yaw_;
    const double step = std::hypot(dx, dy);
    const double bounded_dt = std::clamp(dt, 0.0, 1.0);
    bool accepted = dt > 0.0 && dt <= 1.0 &&
      step <= linear_step_margin_ + max_linear_speed_ * bounded_dt &&
      std::abs(dyaw) <= yaw_step_margin_ + max_yaw_rate_ * bounded_dt;

    double ins_speed_error = 0.0;
    double ins_yaw_rate_error = 0.0;
    const bool ins_fresh = ins_ready_ &&
      std::abs((stamp - latest_ins_stamp_).seconds()) <= ins_timeout_sec_;
    if (accepted && ins_fresh) {
      // Speed and yaw rate are frame-invariant and do not require callback
      // timestamps from the two sensors to line up exactly.
      ins_speed_error = std::abs(step / dt - latest_ins_speed_);
      ins_yaw_rate_error = std::abs(dyaw / dt - latest_ins_yaw_rate_);
      accepted = ins_speed_error <= max_ins_speed_disagreement_ &&
        ins_yaw_rate_error <= max_ins_yaw_rate_disagreement_;
    }

    updateBaseline(stamp, x, y, yaw);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected KISS increment: dt=%.3f s step=%.3f m yaw=%.3f rad "
        "INS errors=(%.3f m/s, %.3f rad/s).",
        dt, step, std::abs(dyaw), ins_speed_error, ins_yaw_rate_error);
      return;
    }

    gated_x_ += dx;
    gated_y_ += dy;
    gated_yaw_ = normalizeAngle(gated_yaw_ + dyaw);
    const double inv_dt = 1.0 / dt;
    const double velocity_x =
      (std::cos(reference_yaw) * dx + std::sin(reference_yaw) * dy) * inv_dt;
    const double velocity_y =
      (-std::sin(reference_yaw) * dx + std::cos(reference_yaw) * dy) * inv_dt;
    publishSanitized(*msg, velocity_x, velocity_y, dyaw * inv_dt);
  }

  void updateBaseline(
    const rclcpp::Time & stamp, double x, double y, double yaw)
  {
    last_kiss_stamp_ = stamp;
    last_kiss_x_ = x;
    last_kiss_y_ = y;
    last_kiss_yaw_ = yaw;
    baseline_ready_ = true;
  }

  void publishSanitized(
    const nav_msgs::msg::Odometry & source,
    double velocity_x, double velocity_y, double yaw_rate)
  {
    auto output = source;
    output.pose.pose.position.x = gated_x_;
    output.pose.pose.position.y = gated_y_;
    writeYaw(output.pose.pose.orientation, gated_yaw_);
    output.twist.twist.linear.x = velocity_x;
    output.twist.twist.linear.y = velocity_y;
    output.twist.twist.linear.z = 0.0;
    output.twist.twist.angular.x = 0.0;
    output.twist.twist.angular.y = 0.0;
    output.twist.twist.angular.z = yaw_rate;
    output.twist.covariance.fill(0.0);
    output.twist.covariance[0] = 0.25;
    output.twist.covariance[7] = 0.25;
    output.twist.covariance[35] = 0.04;
    output_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string ins_topic_;
  double ins_timeout_sec_{0.25};
  double max_linear_speed_{20.0};
  double max_yaw_rate_{3.0};
  double linear_step_margin_{0.75};
  double yaw_step_margin_{0.35};
  double max_ins_speed_disagreement_{5.0};
  double max_ins_yaw_rate_disagreement_{1.0};

  bool baseline_ready_{false};
  bool ins_ready_{false};
  double gated_x_{0.0};
  double gated_y_{0.0};
  double gated_yaw_{0.0};
  double last_kiss_x_{0.0};
  double last_kiss_y_{0.0};
  double last_kiss_yaw_{0.0};
  double latest_ins_speed_{0.0};
  double latest_ins_yaw_rate_{0.0};
  rclcpp::Time last_kiss_stamp_;
  rclcpp::Time latest_ins_stamp_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kiss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub_;
};

}  // namespace kiss_icp_wrapper

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiss_icp_wrapper::KissOdomSanitizerNode>());
  rclcpp::shutdown();
  return 0;
}
