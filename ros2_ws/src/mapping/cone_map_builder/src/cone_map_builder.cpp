#include "cone_map_builder/cone_map_builder.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>

namespace cone_map_builder
{

ConeMapBuilder::ConeMapBuilder(const rclcpp::NodeOptions & options)
: Node("cone_map_builder", options)
{
  // Parameters
  merge_distance_         = declare_parameter("merge_distance",         merge_distance_);
  min_hit_count_          = declare_parameter("min_hit_count",          min_hit_count_);
  loop_closure_distance_  = declare_parameter("loop_closure_distance",  loop_closure_distance_);
  min_cones_for_closure_  = declare_parameter("min_cones_for_closure",  min_cones_for_closure_);
  assign_colors_          = declare_parameter("assign_colors",          assign_colors_);
  start_skip_distance_    = declare_parameter("start_skip_distance",    start_skip_distance_);
  map_save_path_          = declare_parameter("map_save_path",          map_save_path_);

  // TF2
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Separate callback groups so pose (50Hz) is never blocked by slow cone processing
  pose_cbg_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cones_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions pose_opts, cones_opts;
  pose_opts.callback_group  = pose_cbg_;
  cones_opts.callback_group = cones_cbg_;

  // Subscribers
  cones_sub_ = create_subscription<wuta_msgs::msg::ConeArray>(
    "/perception/lidar/cones", 10,
    std::bind(&ConeMapBuilder::onCones, this, std::placeholders::_1), cones_opts);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&ConeMapBuilder::onPose, this, std::placeholders::_1), pose_opts);

  // Publishers
  map_pub_    = create_publisher<wuta_msgs::msg::ConeMap>("/mapping/cone_map", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mapping/cone_map_viz", 10);

  // Publish map at 5 Hz (map doesn't need high frequency)
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() {
      if (pose_initialized_) {
        publishMap();
        publishVisualization();
      }
    });

  RCLCPP_INFO(get_logger(), "ConeMapBuilder ready.");
}

void ConeMapBuilder::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;

  if (!pose_initialized_) {
    pose_initialized_ = true;
    RCLCPP_INFO(get_logger(), "First pose received.");
  }
}

void ConeMapBuilder::onCones(const wuta_msgs::msg::ConeArray::SharedPtr msg)
{
  if (!pose_initialized_) return;
  if (loop_closed_) return;  // Map is complete, stop updating

  integrateDetections(*msg);

  if (!loop_closed_ && checkLoopClosure()) {
    loop_closed_ = true;
    RCLCPP_INFO(get_logger(), "Loop closed! %zu cones in map. Saving map...", cone_map_.size());
    saveMapToYaml();
    publishMap();  // Publish immediately with is_closed = true
  }
}

void ConeMapBuilder::integrateDetections(const wuta_msgs::msg::ConeArray & cones_in_sensor_frame)
{
  // Transform each cone from sensor frame to map frame using TF2
  const std::string target_frame = "map";
  const std::string source_frame = cones_in_sensor_frame.header.frame_id;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame,
      cones_in_sensor_frame.header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF lookup failed: %s", ex.what());
    return;
  }

  for (const auto & cone : cones_in_sensor_frame.cones) {
    // Transform cone position to map frame
    geometry_msgs::msg::PointStamped pt_sensor, pt_map;
    pt_sensor.header = cones_in_sensor_frame.header;
    pt_sensor.point  = cone.position;
    tf2::doTransform(pt_sensor, pt_map, transform);

    const double cx = pt_map.point.x;
    const double cy = pt_map.point.y;
    const double cz = pt_map.point.z;

    // Search for existing cone within merge_distance
    bool merged = false;
    for (auto & tracked : cone_map_) {
      const double dx = cx - tracked.x;
      const double dy = cy - tracked.y;
      if (std::sqrt(dx * dx + dy * dy) < merge_distance_) {
        // Update position with running average
        tracked.x = (tracked.x * tracked.hit_count + cx) / (tracked.hit_count + 1);
        tracked.y = (tracked.y * tracked.hit_count + cy) / (tracked.hit_count + 1);
        tracked.z = (tracked.z * tracked.hit_count + cz) / (tracked.hit_count + 1);
        tracked.hit_count++;
        // Update color if not yet assigned and we have a better estimate
        if (tracked.color == wuta_msgs::msg::Cone::COLOR_UNKNOWN && assign_colors_) {
          tracked.color = assignColor(tracked.x, tracked.y);
        }
        merged = true;
        break;
      }
    }

    if (!merged) {
      TrackedCone new_cone;
      new_cone.x     = cx;
      new_cone.y     = cy;
      new_cone.z     = cz;
      new_cone.color = assign_colors_
        ? assignColor(cx, cy)
        : wuta_msgs::msg::Cone::COLOR_UNKNOWN;
      cone_map_.push_back(new_cone);

      // Record start pose on first cone detection
      if (!start_pose_set_ && cone_map_.size() == 1) {
        start_pose_ = current_pose_;
        start_pose_set_ = true;
      }
    }
  }
}

uint8_t ConeMapBuilder::assignColor(double cone_x_map, double cone_y_map) const
{
  // Determine left/right relative to current vehicle heading
  // Blue = left boundary, Yellow = right boundary (FSG rules)
  const double vx = current_pose_.pose.position.x;
  const double vy = current_pose_.pose.position.y;

  // Vehicle heading from quaternion (yaw)
  const auto & q = current_pose_.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Vector from vehicle to cone
  const double dcx = cone_x_map - vx;
  const double dcy = cone_y_map - vy;

  // Cross product: heading × d_cone → positive = left, negative = right
  const double cross = std::cos(yaw) * dcy - std::sin(yaw) * dcx;

  return cross > 0
    ? wuta_msgs::msg::Cone::COLOR_BLUE    // Left
    : wuta_msgs::msg::Cone::COLOR_YELLOW; // Right
}

bool ConeMapBuilder::checkLoopClosure()
{
  if (!start_pose_set_) return false;

  // Need minimum cones before considering closure
  const int confirmed_cones = std::count_if(cone_map_.begin(), cone_map_.end(),
    [this](const TrackedCone & c) { return c.hit_count >= min_hit_count_; });
  if (confirmed_cones < min_cones_for_closure_) return false;

  // Must have moved at least start_skip_distance_ before checking
  const double dx = current_pose_.pose.position.x - start_pose_.pose.position.x;
  const double dy = current_pose_.pose.position.y - start_pose_.pose.position.y;
  const double dist_from_start = std::sqrt(dx * dx + dy * dy);

  if (dist_from_start < start_skip_distance_) return false;

  // Check if we've returned close to start
  return dist_from_start < loop_closure_distance_;
}

void ConeMapBuilder::publishMap()
{
  wuta_msgs::msg::ConeMap map_msg;
  map_msg.header.stamp    = now();
  map_msg.header.frame_id = "map";
  map_msg.is_closed       = loop_closed_;

  for (const auto & tracked : cone_map_) {
    if (tracked.hit_count < min_hit_count_) continue;  // Filter noisy detections

    wuta_msgs::msg::Cone cone;
    cone.position.x = tracked.x;
    cone.position.y = tracked.y;
    cone.position.z = tracked.z;
    cone.color      = tracked.color;
    cone.confidence = std::min(1.0f, tracked.hit_count / 5.0f);

    switch (tracked.color) {
      case wuta_msgs::msg::Cone::COLOR_BLUE:    map_msg.blue_cones.push_back(cone);    break;
      case wuta_msgs::msg::Cone::COLOR_YELLOW:  map_msg.yellow_cones.push_back(cone);  break;
      case wuta_msgs::msg::Cone::COLOR_ORANGE:  map_msg.orange_cones.push_back(cone);  break;
      default:                                  map_msg.unknown_cones.push_back(cone); break;
    }
  }

  map_pub_->publish(map_msg);
}

void ConeMapBuilder::publishVisualization()
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Delete old markers
  visualization_msgs::msg::Marker del;
  del.header.frame_id = "map";
  del.header.stamp    = now();
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del);

  const auto color_rgba = [](uint8_t color, float & r, float & g, float & b) {
    switch (color) {
      case wuta_msgs::msg::Cone::COLOR_BLUE:   r=0; g=0.4; b=1;   break;
      case wuta_msgs::msg::Cone::COLOR_YELLOW: r=1; g=0.9; b=0;   break;
      case wuta_msgs::msg::Cone::COLOR_ORANGE: r=1; g=0.5; b=0;   break;
      default:                                 r=1; g=1;   b=1;   break;
    }
  };

  int id = 0;
  for (const auto & tracked : cone_map_) {
    if (tracked.hit_count < min_hit_count_) continue;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = now();
    m.ns     = "cone_map";
    m.id     = id++;
    m.type   = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x  = tracked.x;
    m.pose.position.y  = tracked.y;
    m.pose.position.z  = tracked.z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.5;
    m.color.a = 0.9f;
    color_rgba(tracked.color, m.color.r, m.color.g, m.color.b);
    marker_array.markers.push_back(m);
  }

  marker_pub_->publish(marker_array);
}

void ConeMapBuilder::saveMapToYaml() const
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "cone_map" << YAML::Value << YAML::BeginSeq;

  for (const auto & c : cone_map_) {
    if (c.hit_count < min_hit_count_) continue;
    out << YAML::BeginMap;
    out << YAML::Key << "x"         << YAML::Value << c.x;
    out << YAML::Key << "y"         << YAML::Value << c.y;
    out << YAML::Key << "z"         << YAML::Value << c.z;
    out << YAML::Key << "color"     << YAML::Value << static_cast<int>(c.color);
    out << YAML::Key << "hit_count" << YAML::Value << c.hit_count;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq << YAML::EndMap;

  std::ofstream file(map_save_path_);
  if (file.is_open()) {
    file << out.c_str();
    RCLCPP_INFO(get_logger(), "Map saved to %s (%zu cones)", map_save_path_.c_str(), cone_map_.size());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to save map to %s", map_save_path_.c_str());
  }
}

}  // namespace cone_map_builder

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cone_map_builder::ConeMapBuilder>();
  // MultiThreadedExecutor needed to run separate callback groups concurrently
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
