# lidar_detection

LiDAR 锥桶检测节点，支持传统 PCL 和深度学习两种 backend，通过配置文件切换。

## 架构

```
PointCloud2 (禾赛128线)
      │
      ▼
LidarDetectionNode
      │
      ├── detector_type: "traditional" ──→ TraditionalDetector
      │                                        ├── 距离裁剪
      │                                        ├── 地面去除 (RANSAC / 高度阈值)
      │                                        ├── Voxel 降采样
      │                                        ├── 欧式聚类 (PCL)
      │                                        └── 锥桶形状过滤
      │
      └── detector_type: "dl"          ──→ DLDetector (预留接口)
                                               └── TODO: PointPillars / CenterPoint
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/hesai/pandar` | `sensor_msgs/PointCloud2` | 原始点云（可通过参数修改） |
| 发布 | `/perception/lidar/cones` | `ConeArray` | 检测到的锥桶（sensor frame） |
| 发布 | `/perception/lidar/cones_viz` | `MarkerArray` | Foxglove/RViz 可视化 |

## 关键参数（lidar_detection.yaml）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `detector_type` | `"traditional"` | `"traditional"` 或 `"dl"` |
| `use_ransac` | `true` | 地面去除方式 |
| `cluster_tolerance` | `0.4m` | 聚类间距阈值 |
| `max_cone_width` | `0.5m` | 锥桶最大宽度（形状过滤） |
| `max_detection_range` | `20.0m` | 最大检测距离 |

## 接入 DL Backend

1. 实现 `DLDetector::DLDetector()` 加载模型（BPU/TensorRT/ONNX）
2. 实现 `DLDetector::detect()` 推理 + 后处理
3. 修改 config：`detector_type: "dl"`, `model_path: "/path/to/model.bin"`

## 线程模型

单线程。PCL 聚类在 128 线点云下耗时 < 20ms，10Hz 输入无压力。
