#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include "wuta_msgs/msg/mission_state.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_localization
{

/**
 * MapSaver
 *
 * During EXPLORE mode, accumulates KISS-ICP keyframe clouds into a global map.
 * When MissionState transitions to MAPPING_DONE, saves the map to a PCD file.
 * The saved map is then loaded by NdtLocalization for RACE mode.
 *
 * Triggered by: /system/mission_state (MAPPING_DONE)
 * Saves to: map_save_path parameter (e.g. /tmp/wuta_lidar_map.pcd)
 */
class MapSaver : public rclcpp::Node
{
public:
  explicit MapSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);

  void accumulateCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void saveMap();

  // State
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  PointCloud::Ptr accumulated_map_;
  nav_msgs::msg::Odometry latest_odom_;
  bool mapping_done_{false};
  bool map_saved_{false};

  // Parameters
  std::string map_save_path_{"/tmp/wuta_lidar_map.pcd"};
  double voxel_size_{0.2};        // m — downsample accumulated map
  double accumulate_dist_{0.5};   // m — only add new cloud if moved this far

  double last_accumulate_x_{0.0};
  double last_accumulate_y_{0.0};

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_ready_pub_;
};

}  // namespace ndt_localization
