# WUTA-FSD → J6 重构计划

## 背景

原系统运行在工控机上，使用 ROS1 作为中间层。现适配**征程J6域控制器**，ROS1 不可用，迁移至 **ROS2**。

---

## 硬件平台

| 设备 | 型号 | 备注 |
|------|------|------|
| 域控制器 | 地平线征程 J6 | 128 TOPS BPU，CPU 137K DMIPS |
| 激光雷达 | 禾赛 128线 | 替换原 Velodyne |
| 组合导航 | 华测 CG-410 | GNSS + IMU 内部融合，原始IMU待确认 |
| 相机 | 待定 | 接口预留，暂不实现 |

---

## 技术选型

| 模块 | 原方案 | 新方案 | 原因 |
|------|--------|--------|------|
| 中间层 | ROS1 | ROS2 | J6 不支持 ROS1 |
| LiDAR SLAM | FAST-LIO2 + robot_localization | KISS-ICP + EKF（探索模式）/ NDT地图匹配（循迹模式） | 不确定能否拿到原始IMU；KISS-ICP无IMU依赖，更稳健 |
| 相机感知 | darknet_ros (YOLO) | YOLO with ROS2 (ultralytics 官方节点) | ROS2 原生支持 |
| 消息格式 | fsd_common_msgs (自定义) | autoware_msgs + 部分保留自定义 | 生态兼容性好，Control参考代码直接复用 |
| 控制框架 | 自实现 roscpp | 参考 HRT-D Control，去掉 HPL 换直接 rclcpp | HPL 为私有框架，不引入依赖 |
| 路径规划算法 | Voronoi（OpenCV） | Delaunay（参考 HRT-D Planning） | 算法更清晰，有多种实现可选 |
| 构建系统 | Catkin | ament_cmake / ament_cmake_auto | ROS2 标准 |

---

## 参考代码（HRT-D）

位置：`/Users/hb40338/Documents/WUTA-FSD/HRT-D/`

| 仓库 | 说明 | 参考价值 |
|------|------|---------|
| `Control-main` | ROS2 Pure Pursuit，模块化策略模式，autoware_msgs | 高，直接参考结构 |
| `Planning-main` | 纯C++ Delaunay算法库，无ROS耦合 | 高，需自己写ROS2 wrapper |
| `Lidar_Slam-master` | ROS2 NDT/GICP + OpenMP，GPU加速 | 参考双模式SLAM思路 |

---

## 新系统架构

### 分层设计

```
┌─────────────────────────────────────────────────────┐
│                   Mission Manager                    │
│   状态机: IDLE → READY → EXPLORE → RACE → FINISH    │
└────────────┬──────────────┬───────────────┬─────────┘
             │              │               │
┌────────────▼──┐  ┌────────▼──────┐  ┌────▼────────┐
│  Perception   │  │ Localization  │  │   Mapping   │
│               │  │               │  │             │
│ lidar_det     │  │ kiss_icp      │  │ cone_map    │
│ cam_det       │  │ ndt_match     │  │ _builder    │
│ fusion        │  │ gnss_ekf      │  │             │
└───────┬───────┘  └───────┬───────┘  └──────┬──────┘
        │                  │                  │
        └──────────────────▼──────────────────┘
                    ┌──────▼──────┐
                    │  Planning   │
                    │ boundary    │
                    │ path_gen    │
                    └──────┬──────┘
                    ┌──────▼──────┐
                    │   Control   │
                    │ pure_pursuit│
                    └─────────────┘
```

### 节点清单

| 节点 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `lidar_detection_node` | `sensor_msgs/PointCloud2` | `ConeArray` (3D) | LiDAR聚类backbone，独立 |
| `camera_detection_node` | `sensor_msgs/Image` | `ConeArray` (2D bbox) | 相机检测backbone，独立，接口预留 |
| `fusion_node` | LiDAR+Camera ConeArray | `ConeArray` (融合) | 相机缺失时直接pass through |
| `kiss_icp_node` | `PointCloud2` | 里程计pose | 探索模式定位 |
| `ndt_node` | `PointCloud2` + 先验地图 | pose | 循迹模式定位 |
| `gnss_ekf_node` | KISS-ICP/NDT + CG-410 | `/localization/pose` | robot_localization EKF融合 |
| `cone_map_builder` | ConeArray + pose | cone map | 探索圈建图，持久化保存 |
| `boundary_detector_node` | cone map + pose | 边界锥桶 | Delaunay三角剖分 |
| `path_generator_node` | 边界 + pose | `autoware_msgs/Lane` | 轨迹生成 |
| `control_node` | Lane + pose | `autoware_msgs/Command` | Pure Pursuit（参考HRT-D） |
| `mission_manager` | 全局状态 | 模式切换指令 | 新增，统一状态管理 |

### 定位双模式

```
EXPLORE模式（第一圈）:
  禾赛128线 → KISS-ICP → ┐
  CG-410 GNSS/INS    → ├→ EKF → /localization/pose
  CG-410 IMU (可选)  → ┘

RACE模式（高速循迹）:
  禾赛128线 → NDT地图匹配 → /localization/pose
  （先验地图来自探索圈的cone_map_builder）

切换由 Mission Manager 控制
```

---

## 消息标准

```
感知输出:  autoware_msgs::msg::DetectedObjectArray（锥桶检测）
定位输出:  geometry_msgs::msg::PoseStamped（/localization/pose）
规划输出:  autoware_msgs::msg::Lane
控制输出:  autoware_msgs::msg::Command
```

自定义消息（保留或重新定义）：
- `ConeArray.msg` — 锥桶列表（带颜色、3D位置）
- `ConeMap.msg` — 全局锥桶地图

---

## 设计原则

1. **感知解耦**：LiDAR backbone 和 Camera backbone 完全独立节点，互不依赖
2. **定位双模式**：探索/循迹分离，Mission Manager统一切换
3. **无HPL依赖**：Control层直接使用 rclcpp，不引入私有框架
4. **参数中心化**：所有 config YAML 统一管理，不分散在各包内
5. **时间戳统一**：所有节点使用同一时钟源，避免SLAM融合时序问题
6. **地图持久化**：探索圈结束后保存锥桶地图，供NDT模式和后续比赛复用

---

## 开发顺序

| 阶段 | 内容 | 状态 |
|------|------|------|
| 0 | 消息定义 + workspace结构 + Mission Manager框架 | ✅ 完成 |
| 1 | `lidar_detection_node`（LiDAR聚类backbone，预留DL接口） | ✅ 完成 |
| 2 | `kiss_icp_wrapper` + `localization_manager` + EKF配置 | ✅ 完成 |
| 3 | `cone_map_builder`（TF变换、去重合并、loop closure） | ✅ 完成 |
| 4 | `boundary_detector`（Delaunay）+ `path_generator`（三模式） | ✅ 完成 |
| 5 | `controller`（Pure Pursuit横向 + 速度跟踪纵向，去HPL） | ✅ 完成 |
| 6 | `ndt_localization` + `map_saver`（高速循迹框架） | ✅ 框架完成，待实车标定 |
| 7 | `camera_detection_node` + `fusion_node` | ⏳ 待相机型号确认 |

---

## 待确认事项

- [ ] CG-410 能否输出原始 IMU 数据（影响后期是否升级紧耦合SLAM）
- [ ] CG-410 驱动实际发布的 topic 名（ekf.yaml 中需更新）
- [ ] 是否有锥桶标注数据集（影响LiDAR聚类是否用DL方案）
- [ ] 相机型号确认后补充 camera_detection_node
- [ ] J6 ROS2 版本确认（选择了 Humble）
- [ ] LiDAR → base_link 外参标定（NDT 和 TF 树必需）
- [ ] NDT 参数实车调优（ndt_resolution / step_size）
- [ ] 比赛出发时 /initialpose 的设置方案（GPS 自动 or 手动）
- [ ] /control/command → VCU CAN 帧格式（需 VCU 协议文档）
- [ ] 车检 CAN 报文格式（mission_manager INSPECTION 接口）

---

## 文件结构（规划）

```
WUTA-FSD/
├── REFACTOR.md          # 本文件
├── ros2_ws/             # 新ROS2工作空间
│   └── src/
│       ├── common/
│       │   ├── wuta_msgs/        # 自定义消息
│       │   └── wuta_tools/       # 工具库（样条等）
│       ├── perception/
│       │   ├── lidar_detection/  # LiDAR backbone
│       │   ├── camera_detection/ # Camera backbone（预留）
│       │   └── detection_fusion/ # 融合节点
│       ├── localization/
│       │   ├── kiss_icp_wrapper/ # KISS-ICP ROS2封装
│       │   ├── ndt_localization/ # NDT地图匹配
│       │   └── localization_manager/ # EKF + 模式切换
│       ├── mapping/
│       │   └── cone_map_builder/
│       ├── planning/
│       │   ├── boundary_detector/
│       │   └── path_generator/
│       ├── control/
│       │   └── controller/       # Pure Pursuit（参考HRT-D）
│       └── system/
│           └── mission_manager/  # 状态机
├── ros/                 # 原ROS1代码（保留参考）
└── HRT-D/               # 参考代码
```
