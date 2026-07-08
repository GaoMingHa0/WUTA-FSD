# WUTA-FSD

武汉理工大学无人驾驶方程式赛车自动驾驶算法栈。

> **当前分支 `wuta0318`**：适配**地平线征程 J6 域控制器**的 ROS2 重构版本。
> 原 ROS1 版本保留在 `master` 分支。

---

## 硬件平台

| 设备 | 型号 |
|------|------|
| 域控制器 | 地平线征程 J6（128 TOPS BPU，CPU 137K DMIPS） |
| 激光雷达 | 禾赛 128线 |
| 组合导航 | 华测 CG-410（GNSS + IMU） |
| 相机 | 待定（接口已预留） |

---

## 系统架构

```
传感器层
  禾赛128线 ──→ lidar_detection  ──→ ConeArray
  相机(预留) ──→ camera_detection ──→ ConeArray  ──→ detection_fusion

定位层（双模式）
  EXPLORE: KISS-ICP + EKF(CG-410) ──┐
  RACE:    NDT 地图匹配            ──┴──→ /localization/pose

建图层
  ConeArray + pose ──→ cone_map_builder ──→ ConeMap（loop closure检测）

规划层
  ConeMap ──→ boundary_detector(Delaunay) ──→ path_generator ──→ Lane
  三模式：trackdrive / skidpad / acceleration

控制层
  Lane + pose ──→ controller(Pure Pursuit) ──→ Command → VCU

系统管理
  mission_manager：状态机，IDLE→EXPLORE→RACE→FINISH
```

---

## 目录结构

```
WUTA-FSD/
├── REFACTOR.md              # 详细重构计划和进度
├── ros2_ws/
│   └── src/
│       ├── common/
│       │   ├── wuta_msgs/           # 自定义消息定义
│       │   └── wuta_tools/          # 工具库
│       ├── perception/
│       │   ├── lidar_detection/     # LiDAR锥桶检测（PCL/DL双后端）
│       │   ├── camera_detection/    # 相机检测（预留）
│       │   └── detection_fusion/    # 多源融合（预留）
│       ├── localization/
│       │   ├── kiss-icp/            # [submodule] KISS-ICP
│       │   ├── robot_localization/  # [submodule] EKF/UKF融合
│       │   ├── kiss_icp_wrapper/    # 禾赛128线参数配置
│       │   ├── localization_manager/# 双模式切换，统一输出/localization/pose
│       │   └── ndt_localization/    # NDT地图匹配 + 地图保存
│       ├── mapping/
│       │   └── cone_map_builder/    # 锥桶地图构建，loop closure检测
│       ├── planning/
│       │   ├── boundary_detector/   # Delaunay三角剖分中心线提取
│       │   └── path_generator/      # 三模式路径生成
│       ├── control/
│       │   └── controller/          # Pure Pursuit横纵向控制
│       └── system/
│           └── mission_manager/     # 任务状态机（含车检接口预留）
└── ros/                     # 原ROS1代码（见master分支）
```

---

## 快速开始

### 依赖安装

```bash
# ROS2 Humble
sudo apt install ros-humble-pcl-ros ros-humble-tf2-ros \
  ros-humble-robot-localization ros-humble-autoware-msgs

# 初始化 submodules
git submodule update --init --recursive
```

### 编译

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 运行

```bash
# 启动定位
ros2 launch localization_manager localization.launch.py

# 启动感知
ros2 run lidar_detection lidar_detection_node \
  --ros-args --params-file src/perception/lidar_detection/config/lidar_detection.yaml

# 启动任务管理（设置比赛模式）
ros2 run mission_manager mission_manager_node \
  --ros-args -p mission_mode:=trackdrive

# 启动规划
ros2 run boundary_detector boundary_detector_node
ros2 run path_generator path_generator_node

# 启动控制
ros2 run controller controller_node \
  --ros-args --params-file src/control/controller/config/controller.yaml
```

### 切换任务模式

```bash
# 运行时切换（IDLE/READY状态下有效）
ros2 topic pub /system/mission_mode_cmd std_msgs/msg/String "data: 'skidpad'"
ros2 topic pub /system/mission_mode_cmd std_msgs/msg/String "data: 'acceleration'"
ros2 topic pub /system/mission_mode_cmd std_msgs/msg/String "data: 'trackdrive'"
```

---

## 开发进度

| 模块 | 状态 |
|------|------|
| 消息定义 + workspace结构 | ✅ |
| LiDAR锥桶检测（PCL，DL接口预留） | ✅ |
| KISS-ICP + EKF 定位 | ✅ |
| 锥桶地图构建 + loop closure | ✅ |
| Delaunay规划 + 三模式路径生成 | ✅ |
| Pure Pursuit 控制器 | ✅ |
| NDT 地图匹配（框架完成） | ⚙️ 待实车标定 |
| 相机感知 + 融合 | ⏳ 待相机型号确认 |

详细进度和待确认事项见 [REFACTOR.md](REFACTOR.md)。
