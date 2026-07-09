#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wuta_msgs/msg/mission_state.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace ndt_localization
{

/**
 * NdtLocalization
 *
 * Loads a prior point cloud map (built by MapSaver during EXPLORE) and
 * runs NDT scan matching on each incoming LiDAR frame to estimate pose.
 *
 * Only active when MissionState == RACE (LOC_NDT mode).
 * Output: /ndt/pose → consumed by LocalizationManager.
 *
 * Initial pose must be provided via /initialpose (e.g. from RViz2 or
 * a fixed startup config aligned to the saved map).
 *
 * TODO (requires real vehicle data):
 *   - Tune ndt_resolution, step_size, max_iterations for Hesai 128-line
 *   - Fill in LiDAR→base_link extrinsic calibration
 *   - Validate map→odom→base_link TF chain
 */
class NdtLocalization : public rclcpp::Node
{
public:
  explicit NdtLocalization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadMap();
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);

  void runNDT(const sensor_msgs::msg::PointCloud2::SharedPtr scan);
  void publishPose(const Eigen::Matrix4f & transform, const rclcpp::Time & stamp);
  void publishPath(const geometry_msgs::msg::PoseStamped & pose);

  // NDT
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  PointCloud::Ptr map_cloud_;

  // State
  bool map_loaded_{false};
  bool initial_pose_set_{false};
  bool active_{false};   // true only when LOC_NDT mode
  Eigen::Matrix4f current_transform_{Eigen::Matrix4f::Identity()};
  nav_msgs::msg::Path path_history_;

  // Parameters
  std::string map_path_{"/tmp/wuta_lidar_map.pcd"};

  // NDT tuning — TODO: calibrate on real vehicle
  double ndt_resolution_{1.0};       // m — voxel size of NDT map
  double step_size_{0.1};            // m — Newton step size
  double transform_epsilon_{0.01};   // convergence criterion
  int    max_iterations_{30};

  // Downsampling input scan before NDT (speeds up matching)
  double scan_voxel_size_{0.5};      // m

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;  // debug
};

}  // namespace ndt_localization
