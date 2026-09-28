#include "lidar_detection/traditional_detector.hpp"

#include <chrono>
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace lidar_detection
{

TraditionalDetector::TraditionalDetector(const TraditionalDetectorConfig & config)
: cfg_(config) {}

wuta_msgs::msg::ConeArray TraditionalDetector::detect(const PointCloud::ConstPtr & cloud)
{
  wuta_msgs::msg::ConeArray result;

  using Clock = std::chrono::steady_clock;
  const auto elapsed = [](Clock::time_point before, Clock::time_point after) {
    return std::chrono::duration<double, std::milli>(after - before).count();
  };
  last_timings_ = {};
  last_timings_.input_points = cloud->size();
  const auto t0 = Clock::now();
  PointCloud::Ptr range_filtered(new PointCloud);
  range_filtered->reserve(cloud->size());
  const float max_range_sq = static_cast<float>(cfg_.max_detection_range * cfg_.max_detection_range);
  for (const auto & pt : cloud->points) {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) &&
        pt.x * pt.x + pt.y * pt.y < max_range_sq) range_filtered->push_back(pt);
  }
  const auto t1 = Clock::now();
  last_timings_.range_ms = elapsed(t0, t1);
  last_timings_.range_points = range_filtered->size();
  if (range_filtered->empty()) return result;

  PointCloud::Ptr no_ground;
  PointCloud::Ptr cluster_input;
  if (cfg_.voxel_before_ground) {
    const auto v0 = Clock::now();
    auto downsampled = voxelDownsample(range_filtered);
    const auto v1 = Clock::now();
    last_timings_.voxel_ms = elapsed(v0, v1);
    last_timings_.voxel_points = downsampled->size();
    if (downsampled->empty()) return result;
    no_ground = removeGround(downsampled);
    last_timings_.ground_ms = elapsed(v1, Clock::now());
    cluster_input = no_ground;
  } else {
    const auto g0 = Clock::now();
    no_ground = removeGround(range_filtered);
    const auto g1 = Clock::now();
    last_timings_.ground_ms = elapsed(g0, g1);
    if (no_ground->empty()) return result;
    const auto v0 = Clock::now();
    cluster_input = voxelDownsample(no_ground);
    last_timings_.voxel_ms = elapsed(v0, Clock::now());
    last_timings_.voxel_points = cluster_input->size();
  }
  last_timings_.nonground_points = no_ground->size();
  if (cluster_input->empty()) return result;
  const auto c0 = Clock::now();
  auto clusters = euclideanCluster(cluster_input);
  const auto c1 = Clock::now();
  last_timings_.cluster_ms = elapsed(c0, c1);

  // 5. Cone shape filter → centroid extraction
  for (const auto & cluster : clusters) {
    if (!isConeShape(cluster)) continue;

    // Compute centroid
    float cx = 0, cy = 0, cz = 0;
    for (const auto & pt : cluster->points) {
      cx += pt.x;
      cy += pt.y;
      cz += pt.z;
    }
    cx /= cluster->size();
    cy /= cluster->size();
    cz /= cluster->size();

    wuta_msgs::msg::Cone cone;
    cone.position.x = cx;
    cone.position.y = cy;
    cone.position.z = cz;
    cone.color = wuta_msgs::msg::Cone::COLOR_UNKNOWN;  // Color assigned downstream (fusion)
    cone.confidence = 1.0f;
    result.cones.push_back(cone);
  }

  last_timings_.shape_ms = elapsed(c1, Clock::now());
  return result;
}

PointCloud::Ptr TraditionalDetector::removeGround(const PointCloud::ConstPtr & cloud) const
{
  PointCloud::Ptr result(new PointCloud);

  if (!cfg_.use_ransac) {
    // Simple height threshold
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(cfg_.ground_z_threshold, 5.0);
    pass.filter(*result);
    return result;
  }

  // RANSAC plane segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  // An unconstrained plane frequently selects walls, doors, or vehicle panels
  // in indoor M1 scans. Only a plane whose normal follows the calibrated lidar
  // Z axis can be ground.
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(cfg_.ground_max_tilt_deg * 3.14159265358979323846 / 180.0);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(cfg_.ransac_max_iterations);
  seg.setProbability(cfg_.ransac_probability);
  seg.setDistanceThreshold(cfg_.ransac_distance_threshold);
  seg.setInputCloud(cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    // RANSAC failed, fall back to height threshold
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(cfg_.ground_z_threshold, 5.0);
    pass.filter(*result);
    return result;
  }

  // Extract non-ground points
  pcl::ExtractIndices<pcl::PointXYZ> extractor;
  extractor.setInputCloud(cloud);
  extractor.setIndices(inliers);
  extractor.setNegative(true);  // Keep non-ground
  extractor.filter(*result);

  return result;
}

PointCloud::Ptr TraditionalDetector::voxelDownsample(const PointCloud::ConstPtr & cloud) const
{
  PointCloud::Ptr result(new PointCloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(cfg_.voxel_leaf_size, cfg_.voxel_leaf_size, cfg_.voxel_leaf_size);
  voxel.filter(*result);
  return result;
}

std::vector<PointCloud::Ptr> TraditionalDetector::euclideanCluster(
  const PointCloud::ConstPtr & cloud) const
{
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cfg_.cluster_tolerance);
  ec.setMinClusterSize(cfg_.min_cluster_size);
  ec.setMaxClusterSize(cfg_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::vector<PointCloud::Ptr> clusters;
  clusters.reserve(cluster_indices.size());
  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    for (int idx : indices.indices) {
      cluster->points.push_back(cloud->points[idx]);
    }
    clusters.push_back(cluster);
  }
  return clusters;
}

bool TraditionalDetector::isConeShape(const PointCloud::Ptr & cluster) const
{
  float min_x = 1e9, max_x = -1e9;
  float min_y = 1e9, max_y = -1e9;
  float min_z = 1e9, max_z = -1e9;

  for (const auto & pt : cluster->points) {
    min_x = std::min(min_x, pt.x);  max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);  max_y = std::max(max_y, pt.y);
    min_z = std::min(min_z, pt.z);  max_z = std::max(max_z, pt.z);
  }

  float width_x = max_x - min_x;
  float width_y = max_y - min_y;
  float height  = max_z - min_z;

  return width_x < cfg_.max_cone_width &&
         width_y < cfg_.max_cone_width &&
         height  < cfg_.max_cone_height &&
         height  > cfg_.min_cone_height;
}

}  // namespace lidar_detection
