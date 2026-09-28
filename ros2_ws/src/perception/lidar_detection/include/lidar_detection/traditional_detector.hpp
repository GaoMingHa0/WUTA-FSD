#pragma once

#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

struct TraditionalDetectorConfig
{
  // Ground removal
  double ground_z_threshold{-0.8};     // Points below this height (m) relative to sensor are ground
  double ransac_distance_threshold{0.2}; // RANSAC inlier distance (m)
  double ground_max_tilt_deg{5.0};       // Ground normal must remain close to lidar Z
  bool use_ransac{true};               // true=RANSAC, false=simple height threshold
  int ransac_max_iterations{500};
  double ransac_probability{0.999};
  bool voxel_before_ground{false};

  // Voxel downsampling before clustering
  double voxel_leaf_size{0.1};         // m

  // Euclidean clustering
  double cluster_tolerance{0.4};       // m — max distance between points in same cluster
  int min_cluster_size{3};
  int max_cluster_size{200};

  // Cone shape filter (bounding box of cluster)
  double max_cone_width{0.5};          // m
  double max_cone_height{0.6};         // m
  double min_cone_height{0.1};         // m

  // Detection range
  double max_detection_range{20.0};    // m from sensor origin
};

struct DetectorTimings {
  double range_ms{0}, voxel_ms{0}, ground_ms{0}, cluster_ms{0}, shape_ms{0};
  std::size_t input_points{0}, range_points{0}, voxel_points{0}, nonground_points{0};
};

class TraditionalDetector : public IDetector
{
public:
  explicit TraditionalDetector(const TraditionalDetectorConfig & config);

  wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) override;
  const DetectorTimings & lastTimings() const { return last_timings_; }

private:
  TraditionalDetectorConfig cfg_;
  DetectorTimings last_timings_;

  PointCloud::Ptr removeGround(const PointCloud::ConstPtr & cloud) const;
  PointCloud::Ptr voxelDownsample(const PointCloud::ConstPtr & cloud) const;
  std::vector<PointCloud::Ptr> euclideanCluster(const PointCloud::ConstPtr & cloud) const;
  bool isConeShape(const PointCloud::Ptr & cluster) const;
};

}  // namespace lidar_detection
