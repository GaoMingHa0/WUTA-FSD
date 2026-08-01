#include "cone_map_builder/cone_map_builder.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>

namespace cone_map_builder
{

ConeMapBuilder::ConeMapBuilder(const rclcpp::NodeOptions & options)
: Node("cone_map_builder", options)
{
  // Parameters
  merge_distance_         = declare_parameter("merge_distance",         merge_distance_);
  consolidation_distance_ = declare_parameter(
    "consolidation_distance", consolidation_distance_);
  min_hit_count_          = declare_parameter("min_hit_count",          min_hit_count_);
  loop_closure_distance_  = declare_parameter("loop_closure_distance",  loop_closure_distance_);
  min_cones_for_closure_  = declare_parameter("min_cones_for_closure",  min_cones_for_closure_);
  mapping_laps_           = declare_parameter("mapping_laps",           mapping_laps_);
  assign_colors_          = declare_parameter("assign_colors",          assign_colors_);
  tf_lookup_timeout_sec_  = declare_parameter("tf_lookup_timeout_sec",  tf_lookup_timeout_sec_);
  use_latest_tf_fallback_ = declare_parameter("use_latest_tf_fallback", use_latest_tf_fallback_);
  pending_detection_timeout_sec_ = declare_parameter(
    "pending_detection_timeout_sec", pending_detection_timeout_sec_);
  max_pending_detections_ = declare_parameter("max_pending_detections", max_pending_detections_);
  localization_jump_threshold_ = declare_parameter(
    "localization_jump_threshold", localization_jump_threshold_);
  localization_jump_max_speed_ = declare_parameter(
    "localization_jump_max_speed", localization_jump_max_speed_);
  localization_jump_cooldown_sec_ = declare_parameter(
    "localization_jump_cooldown_sec", localization_jump_cooldown_sec_);
  closure_low_support_ratio_ = declare_parameter(
    "closure_low_support_ratio", closure_low_support_ratio_);
  closure_low_support_separation_ratio_ = declare_parameter(
    "closure_low_support_separation_ratio", closure_low_support_separation_ratio_);
  closure_low_support_max_fraction_ = declare_parameter(
    "closure_low_support_max_fraction", closure_low_support_max_fraction_);
  closure_low_support_min_tracks_ = declare_parameter(
    "closure_low_support_min_tracks", closure_low_support_min_tracks_);
  start_skip_distance_    = declare_parameter("start_skip_distance",    start_skip_distance_);
  loop_closure_heading_tolerance_deg_ = declare_parameter(
    "loop_closure_heading_tolerance_deg", loop_closure_heading_tolerance_deg_);
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

  const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
  lap_count_sub_ = create_subscription<std_msgs::msg::UInt32>(
    "/system/lap_count", latched_qos,
    std::bind(&ConeMapBuilder::onLapCount, this, std::placeholders::_1), cones_opts);

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
    },
    cones_cbg_);

  // Retry scans whose exact-time TF was not available when they arrived.
  // This preserves temporal correctness without blocking the sensor callback.
  tf_retry_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&ConeMapBuilder::processPendingDetections, this),
    cones_cbg_);

  RCLCPP_INFO(get_logger(), "ConeMapBuilder ready.");
}

void ConeMapBuilder::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // The map is immutable after closure. Race-speed pose spacing and the
  // simulator's terminal reset must not trigger mapping recovery alarms.
  if (!loop_closed_ && mapping_pose_ready_) {
    const double mapping_step = std::hypot(
      msg->pose.position.x - last_mapping_pose_.pose.position.x,
      msg->pose.position.y - last_mapping_pose_.pose.position.y);
    const rclcpp::Time pose_stamp(msg->header.stamp);
    const rclcpp::Time previous_stamp(last_mapping_pose_.header.stamp);
    const double pose_dt = std::max(0.0, (pose_stamp - previous_stamp).seconds());
    const double allowed_step = localization_jump_threshold_ +
      std::max(0.0, localization_jump_max_speed_) * pose_dt;
    if (mapping_step > allowed_step) {
      mapping_pause_until_ns_.store((now() + rclcpp::Duration::from_seconds(
        std::max(0.0, localization_jump_cooldown_sec_))).nanoseconds());
      // Cone processing runs in another callback group. Let that group clear
      // its queue to avoid racing a deque mutation with TF retry processing.
      discard_pending_detections_.store(true);
      RCLCPP_ERROR(
        get_logger(),
        "Localization jump %.2f m exceeds %.2f m allowance over %.3f s; "
        "discarding pending detections and pausing map integration for %.2f s.",
        mapping_step, allowed_step, pose_dt, localization_jump_cooldown_sec_);
    }
  }
  last_mapping_pose_ = *msg;
  mapping_pose_ready_ = true;

  if (start_pose_set_ && travel_pose_ready_ && !loop_closed_) {
    const double step = std::hypot(
      msg->pose.position.x - last_travel_pose_.pose.position.x,
      msg->pose.position.y - last_travel_pose_.pose.position.y);
    // Ignore localization discontinuities while retaining normal high-rate motion.
    if (step <= 5.0) {
      traveled_distance_ += step;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring %.2f m localization jump in loop-closure distance.", step);
    }
  }

  current_pose_ = *msg;
  if (start_pose_set_) {
    last_travel_pose_ = *msg;
    travel_pose_ready_ = true;
  }

  if (!pose_initialized_) {
    pose_initialized_ = true;
    RCLCPP_INFO(get_logger(), "First pose received.");
  }
}

void ConeMapBuilder::onCones(const wuta_msgs::msg::ConeArray::SharedPtr msg)
{
  if (!pose_initialized_) return;
  if (loop_closed_) return;  // Map is complete, stop updating
  if (mappingPaused()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Skipping cone detections while localization recovers from a pose jump.");
    return;
  }

  if (max_pending_detections_ <= 0) {
    RCLCPP_WARN_ONCE(get_logger(), "max_pending_detections <= 0; dropping cone detections.");
    return;
  }

  if (pending_detections_.size() >= static_cast<size_t>(max_pending_detections_)) {
    pending_detections_.pop_front();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Pending cone queue is full; dropping the oldest detection.");
  }
  pending_detections_.push_back({msg, now()});
  processPendingDetections();
}

void ConeMapBuilder::onLapCount(const std_msgs::msg::UInt32::SharedPtr msg)
{
  if (msg->data < static_cast<uint32_t>(std::max(1, mapping_laps_))) {
    return;
  }

  formal_mapping_lap_completed_ = true;
  if (!loop_closed_ && hasMinimumConesForClosure()) {
    closeMap("formal lap count");
  }
}

void ConeMapBuilder::processPendingDetections()
{
  if (discard_pending_detections_.exchange(false)) {
    pending_detections_.clear();
  }
  if (mappingPaused()) return;
  while (!pending_detections_.empty() && !loop_closed_) {
    auto & pending = pending_detections_.front();
    if (!integrateDetections(*pending.message)) {
      const double age_sec = (now() - pending.queued_at).seconds();
      if (age_sec > pending_detection_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping cone detection after %.3f s without an exact-time TF.", age_sec);
        pending_detections_.pop_front();
        continue;
      }
      // Preserve ordering: a later scan must not be integrated ahead of an
      // earlier scan whose transform is still pending.
      break;
    }
    pending_detections_.pop_front();

    // A noisy track can be created just outside the merge radius and later
    // converge onto an existing track. Remove such duplicates as soon as they
    // become compatible instead of leaving them visible until loop closure.
    online_consolidated_count_ += consolidateMap();

    if (formal_mapping_lap_completed_ && hasMinimumConesForClosure()) {
      closeMap("formal lap count");
    } else if (checkLoopClosure()) {
      closeMap("geometric fallback");
    }
  }
}

bool ConeMapBuilder::mappingPaused() const
{
  const int64_t pause_until_ns = mapping_pause_until_ns_.load();
  return pause_until_ns != 0 && now().nanoseconds() < pause_until_ns;
}

bool ConeMapBuilder::integrateDetections(const wuta_msgs::msg::ConeArray & cones_in_sensor_frame)
{
  // Transform each cone from sensor frame to map frame using TF2
  const std::string target_frame = "map";
  const std::string source_frame = cones_in_sensor_frame.header.frame_id;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame,
      cones_in_sensor_frame.header.stamp,
      rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
  } catch (const tf2::TransformException & ex) {
    if (!use_latest_tf_fallback_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup at sensor stamp failed: %s", ex.what());
      return false;
    }

    try {
      // A delayed EKF may not retain/publish the exact ground-truth stamp
      // used by the simulated sensor.  Latest TF is a bounded-latency
      // fallback; detections are still retained instead of being dropped.
      transform = tf_buffer_->lookupTransform(
        target_frame, source_frame,
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF at sensor stamp unavailable; using latest map<-sensor transform.");
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed at sensor stamp and latest time: %s", latest_ex.what());
      return false;
    }
  }

  // A physical cone can only account for one cluster in one LiDAR frame.
  // Preventing a second observation in this frame from reusing the same track
  // preserves adjacent cones whose spacing is below the merge radius.
  std::unordered_set<size_t> tracks_used_in_scan;
  std::vector<size_t> tracks_observed_in_scan;
  for (const auto & cone : cones_in_sensor_frame.cones) {
    // Transform cone position to map frame
    geometry_msgs::msg::PointStamped pt_sensor, pt_map;
    pt_sensor.header = cones_in_sensor_frame.header;
    pt_sensor.point  = cone.position;
    tf2::doTransform(pt_sensor, pt_map, transform);

    const double cx = pt_map.point.x;
    const double cy = pt_map.point.y;
    const double cz = pt_map.point.z;
    // Associate to the nearest compatible existing track, rather than the
    // first one inside the radius. A semantic color conflict is never merged;
    // UNKNOWN keeps the backwards-compatible geometry-only association.
    size_t best_index = cone_map_.size();
    double best_distance = merge_distance_;
    for (size_t index = 0; index < cone_map_.size(); ++index) {
      if (tracks_used_in_scan.count(index) != 0U) continue;
      auto & tracked = cone_map_[index];
      if (cone.color != wuta_msgs::msg::Cone::COLOR_UNKNOWN &&
          tracked.has_semantic_color && tracked.color != cone.color) {
        continue;
      }
      const double dx = cx - tracked.x;
      const double dy = cy - tracked.y;
      const double distance = std::hypot(dx, dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }

    if (best_index != cone_map_.size()) {
      auto & tracked = cone_map_[best_index];
      tracked.x = (tracked.x * tracked.hit_count + cx) / (tracked.hit_count + 1);
      tracked.y = (tracked.y * tracked.hit_count + cy) / (tracked.hit_count + 1);
      tracked.z = (tracked.z * tracked.hit_count + cz) / (tracked.hit_count + 1);
      ++tracked.hit_count;
      updateColorEstimate(tracked, cone);
      tracks_used_in_scan.insert(best_index);
      tracks_observed_in_scan.push_back(best_index);
    } else {
      TrackedCone new_cone;
      new_cone.id    = next_track_id_++;
      new_cone.x     = cx;
      new_cone.y     = cy;
      new_cone.z     = cz;
      new_cone.color = wuta_msgs::msg::Cone::COLOR_UNKNOWN;
      updateColorEstimate(new_cone, cone);
      cone_map_.push_back(new_cone);
      const size_t new_index = cone_map_.size() - 1;
      tracks_used_in_scan.insert(new_index);
      tracks_observed_in_scan.push_back(new_index);

      // Record start pose on first cone detection
      if (!start_pose_set_ && cone_map_.size() == 1) {
        start_pose_ = current_pose_;
        last_travel_pose_ = current_pose_;
        start_pose_set_ = true;
        travel_pose_ready_ = true;
        traveled_distance_ = 0.0;
      }
    }
  }

  // Nearby cones seen together are distinct physical objects. Keep that
  // evidence across future map cleanup, so a broad duplicate-cleanup radius
  // only merges hypotheses that were never simultaneously observable.
  for (size_t i = 0; i < tracks_observed_in_scan.size(); ++i) {
    for (size_t j = i + 1; j < tracks_observed_in_scan.size(); ++j) {
      auto & first = cone_map_[tracks_observed_in_scan[i]];
      auto & second = cone_map_[tracks_observed_in_scan[j]];
      if (std::hypot(first.x - second.x, first.y - second.y) <
          consolidation_distance_) {
        first.coobserved_track_ids.insert(second.id);
        second.coobserved_track_ids.insert(first.id);
      }
    }
  }
  return true;
}

uint8_t ConeMapBuilder::classifyConeObservation(const wuta_msgs::msg::Cone & cone) const
{
  if (cone.color != wuta_msgs::msg::Cone::COLOR_UNKNOWN) {
    return cone.color;
  }

  if (!assign_colors_) {
    return cone.color;
  }

  // Traditional LiDAR detection has no color.  Use the LiDAR/body-aligned
  // lateral sign only as a fallback for those UNKNOWN observations.
  return cone.position.y >= 0.0
    ? wuta_msgs::msg::Cone::COLOR_BLUE
    : wuta_msgs::msg::Cone::COLOR_YELLOW;
}

void ConeMapBuilder::updateColorEstimate(
  TrackedCone & tracked, const wuta_msgs::msg::Cone & observation) const
{
  if (observation.color != wuta_msgs::msg::Cone::COLOR_UNKNOWN) {
    tracked.has_semantic_color = true;
    addColorVote(tracked, observation.color);
    tracked.color = majorityColor(tracked);
    return;
  }

  ++tracked.unknown_votes;
  if (!assign_colors_ || tracked.has_semantic_color) {
    return;
  }

  // A cone on a nearby section can be visible from the wrong side of the car
  // for many distant frames. The side observed at the closest pass is a much
  // stronger fallback signal for the section the vehicle is actually driving.
  const double observation_distance =
    std::hypot(observation.position.x, observation.position.y);
  if (observation_distance < tracked.closest_fallback_distance) {
    tracked.closest_fallback_distance = observation_distance;
    tracked.closest_fallback_color = classifyConeObservation(observation);
    tracked.color = tracked.closest_fallback_color;
  }
}

void ConeMapBuilder::addColorVote(TrackedCone & tracked, uint8_t color) const
{
  switch (color) {
    case wuta_msgs::msg::Cone::COLOR_BLUE:
      ++tracked.blue_votes;
      break;
    case wuta_msgs::msg::Cone::COLOR_YELLOW:
      ++tracked.yellow_votes;
      break;
    case wuta_msgs::msg::Cone::COLOR_ORANGE:
      ++tracked.orange_votes;
      break;
    default:
      ++tracked.unknown_votes;
      break;
  }
}

uint8_t ConeMapBuilder::majorityColor(const TrackedCone & tracked) const
{
  const int best_votes = std::max(
    {tracked.blue_votes, tracked.yellow_votes, tracked.orange_votes});
  if (best_votes <= 0) {
    return wuta_msgs::msg::Cone::COLOR_UNKNOWN;
  }

  const bool blue_best = tracked.blue_votes == best_votes;
  const bool yellow_best = tracked.yellow_votes == best_votes;
  const bool orange_best = tracked.orange_votes == best_votes;
  const int tied_best_count =
    static_cast<int>(blue_best) + static_cast<int>(yellow_best) + static_cast<int>(orange_best);
  if (tied_best_count > 1 && tracked.color != wuta_msgs::msg::Cone::COLOR_UNKNOWN) {
    return tracked.color;
  }

  if (blue_best) return wuta_msgs::msg::Cone::COLOR_BLUE;
  if (yellow_best) return wuta_msgs::msg::Cone::COLOR_YELLOW;
  return wuta_msgs::msg::Cone::COLOR_ORANGE;
}

bool ConeMapBuilder::hasMinimumConesForClosure() const
{
  const int confirmed_cones = std::count_if(
    cone_map_.begin(), cone_map_.end(),
    [this](const TrackedCone & cone) { return cone.hit_count >= min_hit_count_; });
  return confirmed_cones >= min_cones_for_closure_;
}

bool ConeMapBuilder::checkLoopClosure()
{
  if (!start_pose_set_) return false;

  if (!hasMinimumConesForClosure()) return false;

  // A Euclidean "moved away" check cannot also detect returning near the
  // start. Accumulated travel distinguishes a completed lap from startup.
  if (traveled_distance_ < start_skip_distance_) return false;

  const double dx = current_pose_.pose.position.x - start_pose_.pose.position.x;
  const double dy = current_pose_.pose.position.y - start_pose_.pose.position.y;
  const double dist_from_start = std::sqrt(dx * dx + dy * dy);
  if (dist_from_start >= loop_closure_distance_) return false;

  const auto yaw_from_pose = [](const geometry_msgs::msg::PoseStamped & pose) {
      const auto & q = pose.pose.orientation;
      return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };
  const double start_yaw = yaw_from_pose(start_pose_);
  const double current_yaw = yaw_from_pose(current_pose_);
  const double heading_error = std::abs(std::atan2(
      std::sin(current_yaw - start_yaw),
      std::cos(current_yaw - start_yaw)));
  const double heading_tolerance =
    std::clamp(loop_closure_heading_tolerance_deg_, 0.0, 180.0) * M_PI / 180.0;

  return heading_error <= heading_tolerance;
}

void ConeMapBuilder::closeMap(const char * reason)
{
  if (loop_closed_) return;

  const size_t consolidated_count = consolidateMap();
  const size_t pruned_count = pruneWeakTrackCluster();
  loop_closed_ = true;
  pending_detections_.clear();

  if (online_consolidated_count_ > 0) {
    RCLCPP_INFO(
      get_logger(), "Consolidated %zu duplicate cone tracks during online mapping.",
      online_consolidated_count_);
  }
  if (consolidated_count > 0) {
    RCLCPP_INFO(
      get_logger(), "Consolidated %zu duplicate cone tracks while freezing the map.",
      consolidated_count);
  }
  if (pruned_count > 0) {
    RCLCPP_INFO(
      get_logger(), "Pruned %zu statistically isolated low-support cone tracks.",
      pruned_count);
  }
  const auto confirmed_count = static_cast<std::size_t>(std::count_if(
      cone_map_.begin(), cone_map_.end(),
      [this](const TrackedCone & cone) { return cone.hit_count >= min_hit_count_; }));
  RCLCPP_INFO(
    get_logger(),
    "Loop closed! reason=%s, %zu confirmed cones (%zu internal tracks). Saving map...",
    reason, confirmed_count, cone_map_.size());
  saveMapToYaml();
  publishMap();
}

size_t ConeMapBuilder::consolidateMap()
{
  size_t consolidated_count = 0;
  bool merged = true;

  // Separate tracks can be created while their noisy centroids are farther
  // apart, then converge inside the normal merge radius over a full lap.
  while (merged) {
    merged = false;
    for (size_t i = 0; i < cone_map_.size() && !merged; ++i) {
      for (size_t j = i + 1; j < cone_map_.size(); ++j) {
        auto & first = cone_map_[i];
        const auto & second = cone_map_[j];
        const bool colors_compatible =
          first.color == second.color ||
          first.color == wuta_msgs::msg::Cone::COLOR_UNKNOWN ||
          second.color == wuta_msgs::msg::Cone::COLOR_UNKNOWN ||
          (!first.has_semantic_color && !second.has_semantic_color);
        const bool observed_together =
          first.coobserved_track_ids.count(second.id) != 0U ||
          second.coobserved_track_ids.count(first.id) != 0U;
        if (!colors_compatible || observed_together ||
          std::hypot(first.x - second.x, first.y - second.y) >= consolidation_distance_)
        {
          continue;
        }

        const int combined_hits = first.hit_count + second.hit_count;
        first.x =
          (first.x * first.hit_count + second.x * second.hit_count) / combined_hits;
        first.y =
          (first.y * first.hit_count + second.y * second.hit_count) / combined_hits;
        first.z =
          (first.z * first.hit_count + second.z * second.hit_count) / combined_hits;
        first.hit_count = combined_hits;
        first.blue_votes += second.blue_votes;
        first.yellow_votes += second.yellow_votes;
        first.orange_votes += second.orange_votes;
        first.unknown_votes += second.unknown_votes;
        first.has_semantic_color =
          first.has_semantic_color || second.has_semantic_color;
        if (second.closest_fallback_distance < first.closest_fallback_distance) {
          first.closest_fallback_distance = second.closest_fallback_distance;
          first.closest_fallback_color = second.closest_fallback_color;
        }
        first.coobserved_track_ids.insert(
          second.coobserved_track_ids.begin(), second.coobserved_track_ids.end());
        first.coobserved_track_ids.erase(second.id);
        first.coobserved_track_ids.erase(first.id);
        for (auto & other : cone_map_) {
          if (other.id == first.id || other.id == second.id) continue;
          if (other.coobserved_track_ids.erase(second.id) != 0U) {
            other.coobserved_track_ids.insert(first.id);
          }
        }
        first.color = first.has_semantic_color
          ? majorityColor(first)
          : first.closest_fallback_color;
        cone_map_.erase(cone_map_.begin() + static_cast<std::ptrdiff_t>(j));
        ++consolidated_count;
        merged = true;
        break;
      }
    }
  }

  return consolidated_count;
}

size_t ConeMapBuilder::pruneWeakTrackCluster()
{
  std::vector<int> confirmed_support;
  confirmed_support.reserve(cone_map_.size());
  for (const auto & cone : cone_map_) {
    if (cone.hit_count >= min_hit_count_) confirmed_support.push_back(cone.hit_count);
  }
  if (confirmed_support.size() < static_cast<size_t>(
      std::max(2, closure_low_support_min_tracks_ * 2)))
  {
    return 0;
  }

  std::sort(confirmed_support.begin(), confirmed_support.end());
  const int median_support = confirmed_support[confirmed_support.size() / 2];
  const int candidate_cutoff = std::min(
    min_hit_count_ * 2,
    static_cast<int>(std::floor(
      median_support * std::max(0.0, closure_low_support_ratio_))));
  if (candidate_cutoff < min_hit_count_) return 0;

  const auto first_strong = std::upper_bound(
    confirmed_support.begin(), confirmed_support.end(), candidate_cutoff);
  const size_t weak_count = static_cast<size_t>(
    std::distance(confirmed_support.begin(), first_strong));
  const size_t minimum_weak = static_cast<size_t>(
    std::max(1, closure_low_support_min_tracks_));
  const double weak_fraction = static_cast<double>(weak_count) /
    static_cast<double>(confirmed_support.size());
  if (weak_count < minimum_weak || first_strong == confirmed_support.end() ||
    weak_fraction > std::clamp(closure_low_support_max_fraction_, 0.0, 1.0))
  {
    return 0;
  }

  const int weak_max = *(first_strong - 1);
  const int strong_min = *first_strong;
  if (static_cast<double>(strong_min) < static_cast<double>(weak_max) *
    std::max(1.0, closure_low_support_separation_ratio_))
  {
    return 0;
  }

  const size_t original_size = cone_map_.size();
  cone_map_.erase(
    std::remove_if(
      cone_map_.begin(), cone_map_.end(),
      [weak_max](const TrackedCone & cone) { return cone.hit_count <= weak_max; }),
    cone_map_.end());
  return original_size - cone_map_.size();
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
    const auto confirmed_count = static_cast<std::size_t>(std::count_if(
        cone_map_.begin(), cone_map_.end(),
        [this](const TrackedCone & cone) { return cone.hit_count >= min_hit_count_; }));
    RCLCPP_INFO(
      get_logger(), "Map saved to %s (%zu cones)",
      map_save_path_.c_str(), confirmed_count);
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
