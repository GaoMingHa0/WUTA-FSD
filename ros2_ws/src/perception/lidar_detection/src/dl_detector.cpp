#include "lidar_detection/dl_detector.hpp"
#include <stdexcept>

namespace lidar_detection
{

DLDetector::DLDetector(const std::string & model_path)
: model_path_(model_path)
{
  // TODO: Load model
  // Options:
  //   - Horizon BPU:    use bpu_predict or libdnn API to load .bin model
  //   - TensorRT:       nvinfer1::IRuntime to load .engine
  //   - ONNX Runtime:   Ort::Session to load .onnx
  throw std::runtime_error("DLDetector not yet implemented. model_path: " + model_path_);
}

wuta_msgs::msg::ConeArray DLDetector::detect(const PointCloud::ConstPtr & /*cloud*/)
{
  // TODO: Implement inference pipeline
  // 1. Preprocess: voxelize point cloud into pillars (PointPillars)
  //                or into BEV feature map (CenterPoint)
  // 2. Run inference on BPU/GPU
  // 3. Post-process: decode heatmap / anchors → bounding boxes
  // 4. NMS
  // 5. Convert to ConeArray

  return wuta_msgs::msg::ConeArray{};
}

}  // namespace lidar_detection
