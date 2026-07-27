#include "localization_manager/localization_manager.hpp"

#include <algorithm>
#include <cmath>

namespace localization_manager
{

using MissionState = wuta_msgs::msg::MissionState;

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions & options)
: Node("localization_manager", options)
{
  // Subscriptions
  mission_sub_ = create_subscription<MissionState>(
    "/system/mission_state", 10,
    std::bind(&LocalizationManager::onMissionState, this, std::placeholders::_1));

  // EKF output (robot_localization) — active in EXPLORE mode
  ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&LocalizationManager::onEkfOdom, this, std::placeholders::_1));

  // NDT output — active in RACE mode
  ndt_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ndt/pose", 10,
    std::bind(&LocalizationManager::onNdtPose, this, std::placeholders::_1));

  // Publishers
  pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/localization_ready", 10);
  confidence_pub_ = create_publisher<std_msgs::msg::Float32>(
    "/system/localization_confidence", 10);

  RCLCPP_INFO(get_logger(), "LocalizationManager ready. Default mode: KISS-ICP + EKF");
}

void LocalizationManager::onMissionState(const MissionState::SharedPtr msg)
{
  if (msg->localization_mode == active_mode_) return;

  active_mode_ = msg->localization_mode;

  if (active_mode_ == MissionState::LOC_KISS_ICP) {
    RCLCPP_INFO(get_logger(), "Localization mode: KISS-ICP + EKF (EXPLORE)");
  } else {
    RCLCPP_INFO(get_logger(), "Localization mode: NDT map matching (RACE)");
  }
}

void LocalizationManager::onEkfOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (active_mode_ != MissionState::LOC_KISS_ICP) return;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose = msg->pose.pose;
  pose_pub_->publish(pose);

  const double position_variance = std::max(
    0.0, msg->pose.covariance[0] + msg->pose.covariance[7]);
  const double yaw_variance = std::max(0.0, msg->pose.covariance[35]);
  const double confidence = std::clamp(
    std::exp(-0.5 * position_variance - 2.0 * yaw_variance), 0.0, 1.0);
  publishLocalizationStatus(true, confidence);
}

void LocalizationManager::onNdtPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (active_mode_ != MissionState::LOC_NDT) return;

  pose_pub_->publish(*msg);
  // PoseStamped carries no covariance. NDT convergence gating remains inside
  // ndt_localization; a published pose is treated as accepted here.
  publishLocalizationStatus(true, 1.0);
}

void LocalizationManager::publishLocalizationStatus(bool ready, double confidence)
{
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = ready;
  ready_pub_->publish(ready_msg);

  std_msgs::msg::Float32 confidence_msg;
  confidence_msg.data = static_cast<float>(
    ready ? std::clamp(confidence, 0.0, 1.0) : 0.0);
  confidence_pub_->publish(confidence_msg);
}

}  // namespace localization_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization_manager::LocalizationManager>());
  rclcpp::shutdown();
  return 0;
}
