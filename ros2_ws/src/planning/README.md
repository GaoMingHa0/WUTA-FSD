# planning

规划模块，包含两个节点，支持三种比赛模式。

```
planning/
├── boundary_detector/   ← Delaunay 三角剖分，输出赛道中心线
└── path_generator/      ← 三模式分发，输出 final_waypoints 给控制器
```

---

## 整体数据流

```
                    ┌──────────────────────────────────────┐
                    │           MissionState               │
                    │  mission_mode: TRACKDRIVE /          │
                    │              SKIDPAD /               │
                    │              ACCELERATION            │
                    └────────┬─────────────────────────────┘
                             │
              ┌──────────────▼──────────────┐
              │                             │
  TRACKDRIVE  │           SKIDPAD           │  ACCELERATION
              │                             │
              ▼                             ▼             ▼
  boundary_detector           path_generator (内部生成)
  (Delaunay算法)
              │
              ▼
  /planning/centerline
              │
              ▼
       path_generator
              │
              ▼
  /planning/final_waypoints  →  controller
```

---

## boundary_detector

### 算法：Delaunay 三角剖分（来自 HRT-D）

1. 从 `/mapping/cone_map` 提取当前 `lookahead_distance` 范围内的锥桶坐标
2. 对所有锥桶做 Delaunay 三角剖分（BowyerWatson 算法）
3. 取三角形各边中点作为候选路径点（`MidPoint`）
4. DFS 搜索最优路径（考虑方向连续性和路径代价）
5. 输出为 `autoware_msgs/Lane`

**只在 TRACKDRIVE 模式下运行**，SKIDPAD 和 ACCELERATION 直接在 path_generator 内生成。

算法库来源：`thirdparty/pathplanning/`（复制自 HRT-D/Planning，纯 C++，无 ROS 依赖）

### Topics

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/mapping/cone_map` | `ConeMap` |
| 订阅 | `/localization/pose` | `PoseStamped` |
| 订阅 | `/system/mission_state` | `MissionState` |
| 发布 | `/planning/centerline` | `autoware_msgs/Lane` |
| 发布 | `/planning/centerline_viz` | `MarkerArray` |

---

## path_generator

### 三种模式

#### TRACKDRIVE（高速循迹）
- 直接转发 `boundary_detector` 输出的中心线
- 更新各 waypoint 的速度为 `trackdrive_velocity`

#### SKIDPAD（八字绕桩）
- 使用 `skidpad_start_*` 固定 map 参考，与 `tracks/skidpad.yaml` 对齐，不随定位位姿重建
- 车辆参考点从计时线前 15 m 的 `(-15, 0)` 进入；生成下方右圆两圈（第一圈建立转向、第二圈计时）→ 上方左圈两圈（第三圈过渡、第四圈计时）→ 同向 25 m 出口停车
- 圆半径：9.125m（FSG 规定）
- 路径几何只生成一次；在有效任务状态下重复发布缓存路径，确保晚启动的控制器能够接收
- 路径生成时输出分析 CSV，默认位置为 `WUTA-FSD/ros2_ws/log/trajectory/skidpad_trajectory.csv`；路径的每行包含阶段、圈次、坐标、航向和目标速度

#### ACCELERATION（直线加速）
- 严格对齐 `WUTA-SIM/perception_simulation/tracks/acceleration.yaml`：车辆参考点从 `x=-0.30 m` 起步，计时起点为 `x=0 m`、计时终点为 `x=75 m`
- 在整个 75 m 计时段保持 `acceleration_velocity`；仅在终点线后进入 100 m 标记停止区时按恒减速度剖面制动，并在 `x=175 m` 停车
- 路径只按赛道 map 参考生成一次并缓存，绝不依据实时定位位姿重建，以免终点随车辆前移

### Topics

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/system/mission_state` | `MissionState` |
| 订阅 | `/planning/centerline` | `autoware_msgs/Lane` |
| 订阅 | `/localization/pose` | `PoseStamped` |
| 发布 | `/planning/final_waypoints` | `autoware_msgs/Lane` |
| 发布 | `/planning/final_waypoints_viz` | `MarkerArray` |
| 发布 | `/planning/driven_trajectory_viz` | `MarkerArray` |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `trackdrive_velocity` | 7.0 m/s | 循迹速度 |
| `skidpad_radius` | 9.125m | FSG 标准圆半径 |
| `skidpad_velocity` | 5.0 m/s | 八字速度 |
| `skidpad_entry_x/y` | -15.0 / 0.0 m | 相对交叉点的入口参考 |
| `skidpad_exit_length` | 25.0 m | 第四圈后的出口停车距离 |
| `skidpad_braking_distance` | 10.0 m | 出口末段线性降速距离 |
| `skidpad_csv_path` | `ros2_ws/log/trajectory/skidpad_trajectory.csv` | 分析轨迹输出；相对路径以 WUTA-FSD 根目录解析 |
| `driven_trajectory_smoothing_alpha` | 0.20 | 仅用于 RViz 实际轨迹的一阶平滑；不改变定位、建图或控制输入 |
| `driven_trajectory_min_distance` | 0.10 m | 平滑后轨迹点的最小空间间隔，抑制静止时的噪声折线 |
| `acceleration_start_x/y/yaw` | -0.30 m / 0 / 0 | 起跑位置线与朝向，来自赛道 YAML |
| `acceleration_timing_start_x` | 0.0 m | 计时起点线 |
| `acceleration_length` | 75.0 m | 计时距离；路径在此终点线前不减速 |
| `acceleration_stopping_distance` | 100.0 m | 终点线后的标记停止区；在其末端速度为零 |
| `acceleration_velocity` | 15.0 m/s | 加速直线速度 |

---

## 线程模型

两个节点均为单线程，回调轻量，无需多线程。

## 待完善

- [ ] Delaunay PathSearch 的起点初始化逻辑（`SetStartPoint`）
- [ ] Trackdrive 速度规划：根据曲率动态调整速度（曲率大→减速）
