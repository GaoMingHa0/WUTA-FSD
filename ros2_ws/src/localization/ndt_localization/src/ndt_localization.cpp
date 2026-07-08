#include "ndt_localization/ndt_localization.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ndt_localization
{

NdtLocalization::NdtLocalization(const rclcpp::NodeOptions & options)
: Node("ndt_localization", options)
{
  map_path_           = declare_parameter("map_path",           map_path_);
  ndt_resolution_     = declare_parameter("ndt_resolution",     ndt_resolution_);
  step_size_          = declare_parameter("step_size",          step_size_);
  transform_epsilon_  = declare_parameter("transform_epsilon",  transform_epsilon_);
  max_iterations_     = declare_parameter("max_iterations",     max_iterations_);
  scan_voxel_size_    = declare_parameter("scan_voxel_size",    scan_voxel_size_);

  // Configure NDT
  ndt_.setResolution(ndt_resolution_);
  ndt_.setStepSize(step_size_);
  ndt_.setTransformationEpsilon(transform_epsilon_);
  ndt_.setMaximumIterations(max_iterations_);

  map_cloud_ = std::make_shared<PointCloud>();

  // Subscribers
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/hesai/pandar", rclcpp::SensorDataQoS(),
    std::bind(&NdtLocalization::onPointCloud, this, std::placeholders::_1));

  init_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&NdtLocalization::onInitialPose, this, std::placeholders::_1));

  mission_sub_ = create_subscription<wuta_msgs::msg::MissionState>(
    "/system/mission_state", 10,
    std::bind(&NdtLocalization::onMissionState, this, std::placeholders::_1));

  // Publishers
  pose_pub_         = create_publisher<geometry_msgs::msg::PoseStamped>("/ndt/pose", 10);
  path_pub_         = create_publisher<nav_msgs::msg::Path>("/ndt/path", 10);
  aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ndt/aligned_cloud", 10);

  path_history_.header.frame_id = "map";

  RCLCPP_INFO(get_logger(), "NdtLocalization initialized. Map path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(), "Waiting for RACE mode to activate...");
}

void NdtLocalization::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  using State = wuta_msgs::msg::MissionState;
  const bool should_be_active = (msg->localization_mode == State::LOC_NDT);

  if (should_be_active && !active_) {
    RCLCPP_INFO(get_logger(), "NDT mode activated. Loading map...");
    loadMap();
    active_ = true;
  } else if (!should_be_active && active_) {
    active_ = false;
    RCLCPP_INFO(get_logger(), "NDT mode deactivated.");
  }
}

void NdtLocalization::loadMap()
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *map_cloud_) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to load map: %s", map_path_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Map loaded: %zu points from %s",
    map_cloud_->size(), map_path_.c_str());

  ndt_.setInputTarget(map_cloud_);
  map_loaded_ = true;
}

void NdtLocalization::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;

  // Build initial transform from pose
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  Eigen::AngleAxisf rot(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
  Eigen::Translation3f trans(
    static_cast<float>(p.x),
    static_cast<float>(p.y),
    static_cast<float>(p.z));

  current_transform_ = (trans * rot).matrix();
  initial_pose_set_  = true;

  RCLCPP_INFO(get_logger(), "Initial pose set: (%.2f, %.2f, yaw=%.2f°)",
    p.x, p.y, yaw * 180.0 / M_PI);
}

void NdtLocalization::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!active_ || !map_loaded_ || !initial_pose_set_) return;
  runNDT(msg);
}

void NdtLocalization::runNDT(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg)
{
  // 1. Convert to PCL
  PointCloud::Ptr scan(new PointCloud);
  pcl::fromROSMsg(*scan_msg, *scan);

  // 2. Downsample scan
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(scan);
  vg.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
  PointCloud::Ptr scan_filtered(new PointCloud);
  vg.filter(*scan_filtered);

  if (scan_filtered->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty scan after downsampling");
    return;
  }

  // 3. Run NDT matching
  ndt_.setInputSource(scan_filtered);

  PointCloud::Ptr aligned(new PointCloud);
  ndt_.align(*aligned, current_transform_);

  if (!ndt_.hasConverged()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "NDT did not converge (score=%.3f)", ndt_.getFitnessScore());
    return;
  }

  const double score = ndt_.getFitnessScore();
  if (score > 1.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "NDT fitness score high: %.3f (localization may be unreliable)", score);
  }

  // 4. Update current transform
  current_transform_ = ndt_.getFinalTransformation();

  // 5. Publish
  publishPose(current_transform_, scan_msg->header.stamp);

  // 6. Publish aligned cloud for debug
  if (aligned_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned, aligned_msg);
    aligned_msg.header.frame_id = "map";
    aligned_msg.header.stamp    = scan_msg->header.stamp;
    aligned_cloud_pub_->publish(aligned_msg);
  }
}

void NdtLocalization::publishPose(
  const Eigen::Matrix4f & transform, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp    = stamp;
  pose_msg.header.frame_id = "map";

  // Extract translation
  pose_msg.pose.position.x = transform(0, 3);
  pose_msg.pose.position.y = transform(1, 3);
  pose_msg.pose.position.z = transform(2, 3);

  // Extract rotation → quaternion
  Eigen::Matrix3f rot = transform.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub_->publish(pose_msg);
  publishPath(pose_msg);
}

void NdtLocalization::publishPath(const geometry_msgs::msg::PoseStamped & pose)
{
  path_history_.header.stamp = pose.header.stamp;
  path_history_.poses.push_back(pose);

  // Keep last 500 poses
  if (path_history_.poses.size() > 500) {
    path_history_.poses.erase(path_history_.poses.begin());
  }

  path_pub_->publish(path_history_);
}

}  // namespace ndt_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ndt_localization::NdtLocalization>());
  rclcpp::shutdown();
  return 0;
}
