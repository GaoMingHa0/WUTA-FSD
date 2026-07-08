# ndt_localization

NDT（Normal Distributions Transform）地图匹配定位包，包含两个节点：
- `map_saver_node`：探索圈期间建图，结束后保存点云地图
- `ndt_localization_node`：RACE 模式下加载地图，实时匹配定位

---

## 工作流程

```
EXPLORE 阶段:
  禾赛128线 ──→ map_saver_node
  KISS-ICP 里程计 ──→  (按行驶距离采样，累积点云)
  MissionState=MAPPING_DONE ──→ 保存 /tmp/wuta_lidar_map.pcd
                             ──→ 发布 /ndt/map_ready=true

RACE 阶段:
  MissionState=LOC_NDT ──→ ndt_localization_node 激活
                        ──→ 加载 wuta_lidar_map.pcd
                        ──→ 等待 /initialpose 设置起始位姿
  禾赛128线 ──→ 降采样 ──→ NDT匹配 ──→ /ndt/pose
                                    ──→ LocalizationManager 接收
```

---

## map_saver_node

### Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/hesai/pandar` | `PointCloud2` | 原始点云 |
| 订阅 | `/kiss/odometry` | `Odometry` | 里程计（控制采样间距） |
| 订阅 | `/system/mission_state` | `MissionState` | 触发保存 |
| 发布 | `/ndt/map_ready` | `std_msgs/Bool` | 地图保存完成通知 |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_save_path` | `/tmp/wuta_lidar_map.pcd` | 保存路径 |
| `voxel_size` | 0.2m | 最终地图降采样分辨率 |
| `accumulate_dist` | 0.5m | 每移动多远采一帧（避免冗余） |

> **TODO**：需要将点云从 sensor frame 变换到 map frame（依赖 TF2 + KISS-ICP TF 发布）

---

## ndt_localization_node

### Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/hesai/pandar` | `PointCloud2` | 实时扫描 |
| 订阅 | `/initialpose` | `PoseWithCovarianceStamped` | 初始位姿（RViz2 发布或固定配置） |
| 订阅 | `/system/mission_state` | `MissionState` | 激活/停用 NDT |
| 发布 | `/ndt/pose` | `PoseStamped` | NDT 定位结果 → LocalizationManager |
| 发布 | `/ndt/path` | `Path` | 历史轨迹（调试用） |
| 发布 | `/ndt/aligned_cloud` | `PointCloud2` | 对齐点云（调试用） |

### NDT 参数（需实车标定）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ndt_resolution` | 1.0m | NDT 体素大小，越小越精确但越慢 |
| `step_size` | 0.1 | Newton 步长 |
| `max_iterations` | 30 | 最大迭代次数 |
| `scan_voxel_size` | 0.5m | 输入扫描降采样（加速匹配） |

### 健康监控

- fitness score > 1.0 → 输出 WARN（定位可能不可靠）
- `hasConverged() == false` → 跳过本帧，不更新位姿

---

## 需要实车数据才能完成

- [ ] **LiDAR→base_link 外参标定**：禾赛128线安装位置和姿态，填入 TF static_transform
- [ ] **NDT 参数调优**：ndt_resolution / step_size 针对赛道环境调试
- [ ] **map_saver TF 变换**：将点云从 sensor frame 转到 map frame（依赖 KISS-ICP TF 树）
- [ ] **初始位姿方案**：比赛出发时如何自动设置 /initialpose（GPS 直接给？还是手动对齐？）

---

## 线程模型

两个节点均为单线程。NDT 匹配在 LiDAR 回调中同步执行，
如果 NDT 耗时 > LiDAR 帧间隔（100ms@10Hz），需改为异步线程。
