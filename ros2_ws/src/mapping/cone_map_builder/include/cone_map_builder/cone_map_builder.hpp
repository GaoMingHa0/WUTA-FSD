#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "wuta_msgs/msg/cone.hpp"
#include "wuta_msgs/msg/cone_array.hpp"
#include "wuta_msgs/msg/cone_map.hpp"

#include <vector>
#include <string>

namespace cone_map_builder
{

struct TrackedCone
{
  double x, y, z;
  uint8_t color;
  int hit_count{1};       // Number of times detected (confidence proxy)
};

class ConeMapBuilder : public rclcpp::Node
{
public:
  explicit ConeMapBuilder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void onCones(const wuta_msgs::msg::ConeArray::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Core logic
  void integrateDetections(const wuta_msgs::msg::ConeArray & cones_in_sensor_frame);
  uint8_t assignColor(double cone_x_map, double cone_y_map) const;
  bool checkLoopClosure();
  void publishMap();
  void publishVisualization();
  void saveMapToYaml() const;

  // Map state
  std::vector<TrackedCone> cone_map_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped start_pose_;
  bool pose_initialized_{false};
  bool loop_closed_{false};
  bool start_pose_set_{false};

  // Parameters
  double merge_distance_{0.5};         // m — cones closer than this are merged
  int min_hit_count_{2};               // Minimum detections before cone is added to published map
  double loop_closure_distance_{3.0};  // m — distance to start to trigger loop closure
  int min_cones_for_closure_{10};      // Minimum cones before loop closure is considered
  bool assign_colors_{true};           // Assign blue/yellow based on left/right heuristic
  double start_skip_distance_{5.0};    // m — skip loop closure check until we've moved this far
  std::string map_save_path_{"/tmp/cone_map.yaml"};

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers
  rclcpp::Publisher<wuta_msgs::msg::ConeMap>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Subscribers
  rclcpp::Subscription<wuta_msgs::msg::ConeArray>::SharedPtr cones_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Callback groups (separate so pose is never blocked by cone processing)
  rclcpp::CallbackGroup::SharedPtr pose_cbg_;
  rclcpp::CallbackGroup::SharedPtr cones_cbg_;

  // Publish timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace cone_map_builder
