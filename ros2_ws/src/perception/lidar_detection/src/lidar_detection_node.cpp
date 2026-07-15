#include "lidar_detection/lidar_detection_node.hpp"
#include "lidar_detection/traditional_detector.hpp"
#include "lidar_detection/dl_detector.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace lidar_detection
{

LidarDetectionNode::LidarDetectionNode(const rclcpp::NodeOptions & options)
: Node("lidar_detection_node", options)
{
  // --- Parameters ---
  const std::string detector_type = declare_parameter<std::string>("detector_type", "traditional");

  // Traditional detector config from parameters
  TraditionalDetectorConfig cfg;
  cfg.ground_z_threshold      = declare_parameter("ground_z_threshold",      cfg.ground_z_threshold);
  cfg.ransac_distance_threshold = declare_parameter("ransac_distance_threshold", cfg.ransac_distance_threshold);
  cfg.use_ransac              = declare_parameter("use_ransac",               cfg.use_ransac);
  cfg.voxel_leaf_size         = declare_parameter("voxel_leaf_size",          cfg.voxel_leaf_size);
  cfg.cluster_tolerance       = declare_parameter("cluster_tolerance",        cfg.cluster_tolerance);
  cfg.min_cluster_size        = declare_parameter("min_cluster_size",         cfg.min_cluster_size);
  cfg.max_cluster_size        = declare_parameter("max_cluster_size",         cfg.max_cluster_size);
  cfg.max_cone_width          = declare_parameter("max_cone_width",           cfg.max_cone_width);
  cfg.max_cone_height         = declare_parameter("max_cone_height",          cfg.max_cone_height);
  cfg.min_cone_height         = declare_parameter("min_cone_height",          cfg.min_cone_height);
  cfg.max_detection_range     = declare_parameter("max_detection_range",      cfg.max_detection_range);

  // --- Detector factory ---
  if (detector_type == "traditional") {
    detector_ = std::make_unique<TraditionalDetector>(cfg);
    RCLCPP_INFO(get_logger(), "Using traditional PCL detector");
  } else if (detector_type == "dl") {
    const std::string model_path = declare_parameter<std::string>("model_path", "");
    detector_ = std::make_unique<DLDetector>(model_path);
    RCLCPP_INFO(get_logger(), "Using DL detector, model: %s", model_path.c_str());
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown detector_type: %s", detector_type.c_str());
    throw std::runtime_error("Unknown detector_type: " + detector_type);
  }

  // --- Topics ---
  const std::string input_topic  = declare_parameter<std::string>("input_topic",  "/hesai/pandar");
  const std::string output_topic = declare_parameter<std::string>("output_topic", "/perception/lidar/cones");
  visualization_tf_timeout_sec_ = declare_parameter(
    "visualization_tf_timeout_sec", visualization_tf_timeout_sec_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&LidarDetectionNode::onPointCloud, this, std::placeholders::_1));

  cone_pub_   = create_publisher<wuta_msgs::msg::ConeArray>(output_topic, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/lidar/cones_viz", 10);

  RCLCPP_INFO(get_logger(), "LidarDetectionNode ready. Listening on %s", input_topic.c_str());
}

void LidarDetectionNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);

  auto cones = detector_->detect(cloud);
  cones.header = msg->header;

  cone_pub_->publish(cones);

  if (marker_pub_->get_subscription_count() > 0) {
    // Convert the visualization to map at the acquisition time.  This keeps
    // RViz independent of the TF buffer's historical lookup and avoids both
    // TF_ERROR and the spatial bias caused by applying latest TF to old data.
    wuta_msgs::msg::ConeArray cones_in_map;
    if (transformConesForVisualization(cones, cones_in_map)) {
      publishVisualization(cones_in_map, cones_in_map.header);
    }
  }
}

bool LidarDetectionNode::transformConesForVisualization(
  const wuta_msgs::msg::ConeArray & cones,
  wuta_msgs::msg::ConeArray & cones_in_map)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      "map", cones.header.frame_id, cones.header.stamp,
      rclcpp::Duration::from_seconds(visualization_tf_timeout_sec_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Perception visualization skipped: no exact map<-sensor TF: %s", ex.what());
    return false;
  }

  cones_in_map = cones;
  cones_in_map.header.frame_id = "map";
  for (auto & cone : cones_in_map.cones) {
    geometry_msgs::msg::PointStamped point_sensor;
    geometry_msgs::msg::PointStamped point_map;
    point_sensor.header = cones.header;
    point_sensor.point = cone.position;
    tf2::doTransform(point_sensor, point_map, transform);
    cone.position = point_map.point;
  }
  return true;
}

void LidarDetectionNode::publishVisualization(
  const wuta_msgs::msg::ConeArray & cones,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Delete old markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header = header;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  for (size_t i = 0; i < cones.cones.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "lidar_cones";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = cones.cones[i].position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.5;
    marker.color.a = 0.8f;
    marker.color.r = 1.0f;  // White: color unknown at this stage
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

}  // namespace lidar_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_detection::LidarDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
