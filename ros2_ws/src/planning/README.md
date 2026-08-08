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

### 算法：在线蓝黄锥配对 + 局部几何配对 + Delaunay 兜底

1. 从 `/mapping/cone_map` 和 `/localization/pose` 读取当前建图结果与车辆位姿；Trackdrive 不读取赛道 YAML 或完整参考中心线
2. 提取当前 `lookahead_distance` 范围内、位于车辆前方窗口的蓝/黄锥桶
3. 按车辆航向投影，过滤左右关系错误、赛道宽度异常、前向间隔过大的锥桶组合
4. 对可用蓝/黄锥桶做唯一配对，取两锥中点作为中心线候选点
5. 使用车辆当前航向、候选点间距离、蓝/黄锥横向向量推导出的局部赛道切向进行连续性排序，避免在相邻赛段较近时跳到错误分支
6. If color-based pairing is short for local_pairing_min_streak consecutive cycles, local-frame left/right geometric pairing may be used as a fallback. The default is 3 cycles, so fallback does not replace normal blue/yellow pairing too early.
7. If colors are severely imbalanced, local-frame pairing is allowed immediately; if that still fails, Delaunay fallback is used only when it produces at least `delaunay_min_waypoints` centerline points. The default is 3 because the current online cone map often exposes only a short local fallback; path_generator caps these short centerlines to low speed.
8. 兜底路径会按当前车辆航向过滤明显位于车后的中点，并在必要时翻转局部路径顺序，降低中心线反向导致掉头的概率
9. 输出为 `autoware_msgs/Lane`

> **后续演进：相机颜色融合。** 紧凑赛道中相邻赛段的几何距离可能小于 LiDAR-only
> Delaunay 兜底的可判别尺度，因而仍可能选择错误分支。实车应接入相机锥桶分类，将稳定的
> 蓝/黄语义颜色融合到现有 `ConeArray`/`ConeMap` 数据链路；规划即可优先进行显式左右边界配对。
> 该相机检测与融合节点尚未实现，Delaunay 继续仅作为颜色不足时的保守兜底。

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
- 使用 `boundary_detector` 基于在线锥桶地图输出的局部中心线
- 将稀疏局部中心线按 `trackdrive.resample_spacing` 重采样
- 根据重采样后的局部曲率限制 waypoint 速度：直道不超过 `trackdrive.explore.max_velocity`（第1圈），弯道不低于 `trackdrive.explore.min_velocity`，横向加速度上限由 `trackdrive.explore.lateral_accel_limit` 控制；RACE 状态使用 `trackdrive.race.*` 覆盖
- 当在线中心线源点数很少（默认不超过 3 点）时，将速度上限临时压到 `trackdrive.degraded_velocity`，避免短 Delaunay 兜底在紧凑弯道里被高速追踪成掉头
- 发布前检查 Trackdrive 局部中心线是否仍有车头前方目标点；若没有，则拒绝该帧反向/不可追踪路径并保持上一条有效路径，避免车辆被短局部路径诱导掉头

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
| `boundary_detector.lookahead_distance` | 15.0 m | 高速循迹在线蓝/黄锥配对和 Delaunay 兜底的局部取锥范围；大于控制器 14 m 高速前视，同时减少紧凑图上跨分支误配 |
| `boundary_detector.local_pairing_min_streak` | 3 | 颜色配对连续不足多少个周期后，允许车辆局部坐标系左右锥几何配对兜底；仿真中优先避免颜色误判后长时间断路 |
| `boundary_detector.local_pairing_color_imbalance_ratio` | 0.20 | 蓝/黄较少一侧低于该比例时，认为颜色严重失衡并立即启用局部左右配对兜底 |
| `boundary_detector.delaunay_min_waypoints` | 3 | Delaunay fallback must produce at least this many centerline points; 3-point fallback is allowed but path_generator caps short centerlines to low speed |
| `trackdrive.explore.max_velocity` | 7.0 m/s | 第1圈（探索圈）循迹速度 |
| `trackdrive.resample_spacing` | 1.0 m | 高速循迹局部中心线重采样间距，用于给 Pure Pursuit 提供连续前向目标 |
| `trackdrive.explore.min_velocity` | 3.0 m/s | 第1圈曲率限速的最低目标速度 |
| `trackdrive.explore.lateral_accel_limit` | 4.0 m/s^2 | 第1圈曲率限速使用的横向加速度上限 |
| `trackdrive.race.lap2_max_velocity` | 9.0 m/s | RACE 第2圈速度上限（lap_count≤1） |
| `trackdrive.race.lap3_max_velocity` | 10.0 m/s | RACE 第3圈起速度上限（lap_count>1） |
| `trackdrive.race.min_velocity` | 4.0 m/s | RACE 曲率限速最低目标速度 |
| `trackdrive.race.lateral_accel_limit` | 6.0 m/s^2 | RACE 曲率限速横向加速度上限 |
| `trackdrive.min_forward_target` | 0.5 m | Trackdrive 新局部路径至少需要包含一个车头前方目标点，否则保持上一条有效路径 |
| `trackdrive.degraded_velocity` | 3.0 m/s | 短中心线或低置信度时的降级速度帽，主要保护 2-3 点 Delaunay 兜底 |
| `trackdrive.short_centerline_points` | 3 | 源中心线点数小于等于该值时启用短中心线降速 |
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
- [ ] Trackdrive 局部中心线分支选择和平滑，重点降低紧凑外部图上的瞬时大偏差
