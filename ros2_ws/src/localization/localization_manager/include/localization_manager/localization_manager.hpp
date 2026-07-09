#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "wuta_msgs/msg/mission_state.hpp"

namespace localization_manager
{

/**
 * Localization Manager
 *
 * Subscribes to two localization sources:
 *   - EXPLORE mode: EKF output (KISS-ICP + CG-410 fusion)
 *   - RACE mode:    NDT map matching output
 *
 * Publishes a single unified topic: /localization/pose
 * Downstream nodes (planning, control) only subscribe to /localization/pose.
 *
 * Mode switching is driven by /system/mission_state from MissionManager.
 */
class LocalizationManager : public rclcpp::Node
{
public:
  explicit LocalizationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onEkfOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onNdtPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void publishLocalizationReady(bool ready);

  uint8_t active_mode_{wuta_msgs::msg::MissionState::LOC_KISS_ICP};

  // Subscriptions
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
};

}  // namespace localization_manager
