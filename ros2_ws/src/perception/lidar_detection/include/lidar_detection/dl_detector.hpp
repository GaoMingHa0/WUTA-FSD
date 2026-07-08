#pragma once

#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

/**
 * Placeholder interface for deep learning-based cone detection.
 *
 * Future implementations:
 *   - PointPillars: pillars-based 3D object detection
 *   - CenterPoint:  center-based heatmap detection
 *
 * To integrate:
 *   1. Implement this class with model loading (ONNX / Horizon BPU .bin)
 *   2. Set detector_type: "dl" in lidar_detection.yaml
 *   3. The LidarDetectionNode will automatically use this backend
 */
class DLDetector : public IDetector
{
public:
  explicit DLDetector(const std::string & model_path);

  wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) override;

private:
  std::string model_path_;
  // TODO: model handle (e.g. horizon BPU context, TensorRT engine, ONNX session)
};

}  // namespace lidar_detection
