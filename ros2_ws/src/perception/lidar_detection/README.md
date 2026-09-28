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

单线程。硬件链路使用最新点云 QoS（深度 1），避免检测慢于扫描时堆积旧帧。

## RoboSense M1 硬件配置与实测

`fusion_hardware.launch.py` 对 M1 默认先做 8 cm PCL VoxelGrid 降采样，再做 RANSAC 地面去除和 PCL KD-tree 欧式聚类。它保留原有的 20 m 距离裁剪及锥桶形状筛选，没有增加高度 ROI。其他场景仍可用 `lidar_voxel_before_ground:=false` 保持原顺序。

硬件启动参数：`lidar_voxel_before_ground`、`lidar_voxel_leaf_size`、`lidar_ransac_max_iterations`、`lidar_ransac_probability`。`profile_lidar:=true` 会逐帧记录输入/各阶段点数和耗时，调试后应关闭。
实地常用雷达、相机和融合门限统一见 [perception/config/field_tuning.yaml](../config/field_tuning.yaml)，
参数含义与当日测量见同目录 [README](../config/README.md)。

2026-09-25，在 M1 每帧约 78,750 点的静态场景，原配置的雷达检测耗时中位数约 130 ms（聚类约 112 ms）；8 cm 预降采样后约 32 ms（聚类约 28 ms）。最终配置下，20 秒监测得到聚类和融合输出约 9.6 Hz，雷达时间戳到融合结果到达约 130 ms 中位数。画面中的近处橙色锥桶主要由融合节点的相机引导局部聚类检出，普通全局聚类仍可能漏检；这些数字不代表其他赛道上的召回率。
