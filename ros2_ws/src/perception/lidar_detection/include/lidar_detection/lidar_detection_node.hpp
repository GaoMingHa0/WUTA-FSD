#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wuta_msgs/msg/cone_array.hpp"
#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

class LidarDetectionNode : public rclcpp::Node
{
public:
  explicit LidarDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishVisualization(const wuta_msgs::msg::ConeArray & cones,
                            const std_msgs::msg::Header & header);

  std::unique_ptr<IDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<wuta_msgs::msg::ConeArray>::SharedPtr cone_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace lidar_detection
