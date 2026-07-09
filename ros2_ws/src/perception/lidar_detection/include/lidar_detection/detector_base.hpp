#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "wuta_msgs/msg/cone_array.hpp"

namespace lidar_detection
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * Abstract detector interface.
 * Swap backends (traditional PCL vs DL) by implementing this interface.
 */
class IDetector
{
public:
  virtual ~IDetector() = default;

  /**
   * Detect cones from a raw point cloud.
   * @param cloud  Input point cloud in sensor frame
   * @return       ConeArray (positions in sensor frame, color=UNKNOWN)
   */
  virtual wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) = 0;
};

}  // namespace lidar_detection
