#include "lidar_detection/traditional_detector.hpp"

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

  // 1. Range filter: discard points beyond max detection range
  PointCloud::Ptr range_filtered(new PointCloud);
  for (const auto & pt : cloud->points) {
    if (pt.x * pt.x + pt.y * pt.y < cfg_.max_detection_range * cfg_.max_detection_range) {
      range_filtered->points.push_back(pt);
    }
  }

  // 2. Ground removal
  PointCloud::Ptr no_ground = removeGround(range_filtered);
  if (no_ground->empty()) return result;

  // 3. Voxel downsampling
  PointCloud::Ptr downsampled = voxelDownsample(no_ground);
  if (downsampled->empty()) return result;

  // 4. Euclidean clustering
  auto clusters = euclideanCluster(downsampled);

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
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
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
