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
  ConeMap ──→ boundary_detector(局部路径/冻结全局中心线) ──→ path_generator ──→ Lane
  三模式：trackdrive / skidpad / acceleration

控制层
  Lane + pose ──→ controller(Pure Pursuit) ──→ Command → VCU

系统管理
  mission_manager：唯一状态机发布者，IDLE→READY→EXPLORE→…→FINISH
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

### 仿真闭环说明

由上层 `WUTA-SIM/simulator_bringup` 启动时，`mission_manager` 是
`/system/mission_state` 的唯一发布者：LiDAR 与定位 ready 后进入 `READY`，收到
`/system/start_command=true` 后进入 `EXPLORE`。Trackdrive 第一圈用局部中心线建图；闭环后，
`boundary_detector` 仅从冻结的 `ConeMap` 生成有序全局中心线。地图闭合、地图质量、定位质量、
全局中心线和首圈完成五项条件同时满足后进入 `RACE`。`mission_manager` 根据
`/localization/pose` 穿越有限起终线发布 `/system/lap_count`，第三圈后进入 `FINISH`。
Skidpad/Acceleration 仍由控制器停车后经 `/system/mission_complete=true` 完成。不要与
`simulation_bridge` 或外部节点同时发布 MissionState。

Trackdrive 默认分圈速度上限为第一圈 7 m/s、第二圈 9 m/s、第三圈 10 m/s，并同时受曲率、
前向路径长度、`/planning/path_confidence` 和 `/system/localization_confidence` 限制。
第一圈保留已经验证的速度，不因建图阶段无条件降速；低置信度或短路径会自动限制到保守速度。

控制侧使用连续的 Pure Pursuit 曲率，并以 `max_steering_rate_deg_s` 限制转向命令变化；该参数
是仿真初值，实车必须依转向执行器反馈与允许转向速率标定。

### 切换任务模式

```bash
# 运行时切换（IDLE/READY状态下有效）
ros2 topic pub /system/mission_mode_cmd std_msgs/msg/String "data: 'skidpad'"
ros2 topic pub /system/mission_mode_cmd std_msgs/msg/String "data: 'acceleration'"
ros2 topic pub /system/mission_mode_cmd std_msgs/msg/String "data: 'trackdrive'"
```

### 仿真 Skidpad 闭环

集成仿真由主仓库的启动脚本负责构建与编排；不要在本子仓库中单独拼接 INS、KISS-ICP 和
EKF 节点。于主仓库根目录运行：

```bash
./start_simulator.sh --rviz track_file:=skidpad mission_mode:=skidpad
```

该模式从 `(-15, 0)` 沿 `+X` 进入，依次完成下方右圆两圈、上方左圆两圈，并沿 `+X` 出口
在 25 m 内停车。默认使用 INS + KISS-ICP + EKF；若只需真值定位调试，使用：

```bash
./start_simulator.sh --skip-build \
  track_file:=skidpad mission_mode:=skidpad \
  use_ground_truth_localization:=true
```

Skidpad 固定轨迹的分析 CSV 写至
`ros2_ws/log/trajectory/skidpad_trajectory.csv`（相对于 `WUTA-FSD` 根目录）。

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
