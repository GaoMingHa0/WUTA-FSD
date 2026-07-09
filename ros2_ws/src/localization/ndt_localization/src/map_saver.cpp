#include "ndt_localization/map_saver.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

namespace ndt_localization
{

MapSaver::MapSaver(const rclcpp::NodeOptions & options)
: Node("map_saver", options)
{
  map_save_path_    = declare_parameter("map_save_path",    map_save_path_);
  voxel_size_       = declare_parameter("voxel_size",       voxel_size_);
  accumulate_dist_  = declare_parameter("accumulate_dist",  accumulate_dist_);

  accumulated_map_ = std::make_shared<PointCloud>();

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/hesai/pandar", rclcpp::SensorDataQoS(),
    std::bind(&MapSaver::onPointCloud, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/kiss/odometry", 10,
    std::bind(&MapSaver::onOdometry, this, std::placeholders::_1));

  mission_sub_ = create_subscription<wuta_msgs::msg::MissionState>(
    "/system/mission_state", 10,
    std::bind(&MapSaver::onMissionState, this, std::placeholders::_1));

  map_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/ndt/map_ready", 10);

  RCLCPP_INFO(get_logger(), "MapSaver ready. Will save to: %s", map_save_path_.c_str());
}

void MapSaver::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
}

void MapSaver::onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg)
{
  using State = wuta_msgs::msg::MissionState;

  // Trigger save when exploration is done
  if (msg->state == State::MAPPING_DONE && !map_saved_) {
    RCLCPP_INFO(get_logger(), "MAPPING_DONE received. Saving map...");
    saveMap();
  }
}

void MapSaver::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Only accumulate during EXPLORE (before map_saved)
  if (map_saved_) return;

  const double cx = latest_odom_.pose.pose.position.x;
  const double cy = latest_odom_.pose.pose.position.y;
  const double dx = cx - last_accumulate_x_;
  const double dy = cy - last_accumulate_y_;

  // Only add cloud if vehicle has moved far enough (avoid redundant overlap)
  if (std::sqrt(dx * dx + dy * dy) < accumulate_dist_ && accumulated_map_->size() > 0) {
    return;
  }

  accumulateCloud(msg);
  last_accumulate_x_ = cx;
  last_accumulate_y_ = cy;
}

void MapSaver::accumulateCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);

  // TODO: transform cloud from sensor frame to map frame using TF2
  // For now, assumes cloud is already in map frame (requires KISS-ICP TF)

  *accumulated_map_ += *cloud;

  // Periodic downsampling to keep memory bounded
  if (accumulated_map_->size() > 500000) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(accumulated_map_);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    PointCloud::Ptr filtered(new PointCloud);
    vg.filter(*filtered);
    accumulated_map_ = filtered;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Map downsampled to %zu points", accumulated_map_->size());
  }
}

void MapSaver::saveMap()
{
  if (accumulated_map_->empty()) {
    RCLCPP_ERROR(get_logger(), "No points accumulated, cannot save map.");
    return;
  }

  // Final downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(accumulated_map_);
  vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  PointCloud::Ptr final_map(new PointCloud);
  vg.filter(*final_map);

  if (pcl::io::savePCDFileBinary(map_save_path_, *final_map) == 0) {
    map_saved_ = true;
    RCLCPP_INFO(get_logger(), "Map saved: %s (%zu points)",
      map_save_path_.c_str(), final_map->size());

    std_msgs::msg::Bool ready;
    ready.data = true;
    map_ready_pub_->publish(ready);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to save map to %s", map_save_path_.c_str());
  }
}

}  // namespace ndt_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ndt_localization::MapSaver>());
  rclcpp::shutdown();
  return 0;
}
