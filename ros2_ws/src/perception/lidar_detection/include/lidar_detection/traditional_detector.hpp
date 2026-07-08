#pragma once

#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

struct TraditionalDetectorConfig
{
  // Ground removal
  double ground_z_threshold{-0.8};     // Points below this height (m) relative to sensor are ground
  double ransac_distance_threshold{0.2}; // RANSAC inlier distance (m)
  bool use_ransac{true};               // true=RANSAC, false=simple height threshold

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

class TraditionalDetector : public IDetector
{
public:
  explicit TraditionalDetector(const TraditionalDetectorConfig & config);

  wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) override;

private:
  TraditionalDetectorConfig cfg_;

  PointCloud::Ptr removeGround(const PointCloud::ConstPtr & cloud) const;
  PointCloud::Ptr voxelDownsample(const PointCloud::ConstPtr & cloud) const;
  std::vector<PointCloud::Ptr> euclideanCluster(const PointCloud::ConstPtr & cloud) const;
  bool isConeShape(const PointCloud::Ptr & cluster) const;
};

}  // namespace lidar_detection
