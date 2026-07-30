#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace kiss_icp_wrapper
{

class KissOdomGateNode : public rclcpp::Node
{
public:
  KissOdomGateNode()
  : Node("kiss_odom_gate_node")
  {
    kiss_input_topic_ = declare_parameter("kiss_input_topic", "/kiss/odometry");
    gated_output_topic_ = declare_parameter("gated_output_topic", "/kiss/odometry_gated");
    ins_topic_ = declare_parameter("ins_topic", "/cg410/odometry");
    max_abs_yaw_rate_ = declare_parameter("max_abs_yaw_rate", 0.20);
    ins_timeout_sec_ = declare_parameter("ins_timeout_sec", 0.20);
    recovery_hold_time_sec_ = declare_parameter("recovery_hold_time_sec", 1.0);
    max_motion_disagreement_ = declare_parameter("max_motion_disagreement", 0.35);
    max_yaw_disagreement_ = declare_parameter("max_yaw_disagreement", 0.15);

    gated_pub_ = create_publisher<nav_msgs::msg::Odometry>(gated_output_topic_, 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/localization/kiss_gate_active", 10);
    ins_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ins_topic_, 20, std::bind(&KissOdomGateNode::onIns, this, std::placeholders::_1));
    kiss_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      kiss_input_topic_, 10, std::bind(&KissOdomGateNode::onKiss, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "KISS gate: %s -> %s, INS=%s, |yaw_rate| <= %.3f rad/s.",
      kiss_input_topic_.c_str(), gated_output_topic_.c_str(), ins_topic_.c_str(), max_abs_yaw_rate_);
  }

private:
  void onIns(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_ins_stamp_ = rclcpp::Time(msg->header.stamp);
    latest_ins_x_ = msg->pose.pose.position.x;
    latest_ins_y_ = msg->pose.pose.position.y;
    latest_ins_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    latest_yaw_rate_ = msg->twist.twist.angular.z;
    ins_received_ = true;
  }

  void onKiss(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time kiss_stamp(msg->header.stamp);
    const bool ins_fresh = ins_received_ &&
      std::abs((kiss_stamp - latest_ins_stamp_).seconds()) <= ins_timeout_sec_;
    const bool low_yaw_rate = ins_fresh && std::isfinite(latest_yaw_rate_) &&
      std::abs(latest_yaw_rate_) <= max_abs_yaw_rate_;

    if (!low_yaw_rate) {
      stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      kiss_baseline_ready_ = false;
      publishActive(false);
      return;
    }

    if (stable_since_.nanoseconds() == 0) {
      stable_since_ = kiss_stamp;
    }
    const bool recovered = (kiss_stamp - stable_since_).seconds() >= recovery_hold_time_sec_;
    if (!recovered) {
      publishActive(false);
      return;
    }

    // Do not forward KISS's raw absolute pose after a blocked interval. EKF
    // differential mode would interpret the skipped curved segment as one
    // enormous velocity update. Establish a new raw baseline, then publish
    // only verified frame-to-frame increments on a continuous gate-owned
    // odometry trajectory.
    const double kiss_x = msg->pose.pose.position.x;
    const double kiss_y = msg->pose.pose.position.y;
    const double kiss_yaw = yawFromQuaternion(msg->pose.pose.orientation);
    if (!kiss_baseline_ready_) {
      setBaseline(kiss_x, kiss_y, kiss_yaw);
      publishActive(false);
      return;
    }

    const double kiss_distance = std::hypot(kiss_x - last_kiss_x_, kiss_y - last_kiss_y_);
    const double ins_distance = std::hypot(
      latest_ins_x_ - last_ins_x_, latest_ins_y_ - last_ins_y_);
    const double kiss_yaw_delta = normalizeAngle(kiss_yaw - last_kiss_yaw_);
    const double ins_yaw_delta = normalizeAngle(latest_ins_yaw_ - last_ins_yaw_);
    const bool motion_consistent =
      std::abs(kiss_distance - ins_distance) <= max_motion_disagreement_ &&
      std::abs(normalizeAngle(kiss_yaw_delta - ins_yaw_delta)) <= max_yaw_disagreement_;
    if (!motion_consistent) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejecting KISS increment: distance disagreement %.3f m, yaw disagreement %.3f rad.",
        std::abs(kiss_distance - ins_distance),
        std::abs(normalizeAngle(kiss_yaw_delta - ins_yaw_delta)));
      setBaseline(kiss_x, kiss_y, kiss_yaw);
      publishActive(false);
      return;
    }

    gated_x_ += kiss_x - last_kiss_x_;
    gated_y_ += kiss_y - last_kiss_y_;
    gated_yaw_ = normalizeAngle(gated_yaw_ + kiss_yaw_delta);
    setBaseline(kiss_x, kiss_y, kiss_yaw);

    auto gated = *msg;
    gated.pose.pose.position.x = gated_x_;
    gated.pose.pose.position.y = gated_y_;
    gated.pose.pose.position.z = 0.0;
    writeYawQuaternion(gated.pose.pose.orientation, gated_yaw_);
    gated_pub_->publish(gated);
    publishActive(true);
  }

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

  static void writeYawQuaternion(geometry_msgs::msg::Quaternion & q, double yaw)
  {
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
  }

  void setBaseline(double kiss_x, double kiss_y, double kiss_yaw)
  {
    last_kiss_x_ = kiss_x;
    last_kiss_y_ = kiss_y;
    last_kiss_yaw_ = kiss_yaw;
    last_ins_x_ = latest_ins_x_;
    last_ins_y_ = latest_ins_y_;
    last_ins_yaw_ = latest_ins_yaw_;
    kiss_baseline_ready_ = true;
  }

  void publishActive(bool active)
  {
    if (active == gate_active_) return;
    gate_active_ = active;
    std_msgs::msg::Bool status;
    status.data = active;
    active_pub_->publish(status);
    RCLCPP_INFO(
      get_logger(), "KISS EKF input %s (INS yaw rate %.3f rad/s).",
      active ? "enabled" : "blocked", latest_yaw_rate_);
  }

  std::string kiss_input_topic_;
  std::string gated_output_topic_;
  std::string ins_topic_;
  double max_abs_yaw_rate_{0.20};
  double ins_timeout_sec_{0.20};
  double recovery_hold_time_sec_{1.0};
  double max_motion_disagreement_{0.35};
  double max_yaw_disagreement_{0.15};
  double latest_yaw_rate_{0.0};
  double latest_ins_x_{0.0};
  double latest_ins_y_{0.0};
  double latest_ins_yaw_{0.0};
  double last_ins_x_{0.0};
  double last_ins_y_{0.0};
  double last_ins_yaw_{0.0};
  double last_kiss_x_{0.0};
  double last_kiss_y_{0.0};
  double last_kiss_yaw_{0.0};
  double gated_x_{0.0};
  double gated_y_{0.0};
  double gated_yaw_{0.0};
  bool ins_received_{false};
  bool gate_active_{false};
  bool kiss_baseline_ready_{false};
  rclcpp::Time latest_ins_stamp_;
  rclcpp::Time stable_since_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kiss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gated_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
};

}  // namespace kiss_icp_wrapper

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiss_icp_wrapper::KissOdomGateNode>());
  rclcpp::shutdown();
  return 0;
}
