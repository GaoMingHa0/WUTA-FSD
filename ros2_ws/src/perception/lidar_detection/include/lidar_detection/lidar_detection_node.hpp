#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  bool transformConesForVisualization(
    const wuta_msgs::msg::ConeArray & cones,
    wuta_msgs::msg::ConeArray & cones_in_map);

  std::unique_ptr<IDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<wuta_msgs::msg::ConeArray>::SharedPtr cone_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double visualization_tf_timeout_sec_{0.1};
};

}  // namespace lidar_detection
