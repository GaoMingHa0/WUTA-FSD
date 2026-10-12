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

### 算法：在线红蓝锥配对 + 局部几何配对 + Delaunay 兜底

1. 从 `/mapping/cone_map` 和 `/localization/pose` 读取当前建图结果与车辆位姿；Trackdrive 不读取赛道 YAML 或完整参考中心线
2. 提取当前 `lookahead_distance` 范围内、位于车辆前方窗口的红/蓝锥桶
3. 按车辆航向投影，过滤左右关系错误、赛道宽度异常、前向间隔过大的锥桶组合
4. 对可用红/蓝锥桶做唯一配对，取两锥中点作为中心线候选点
5. 使用车辆当前航向、候选点间距离、红/蓝锥横向向量推导出的局部赛道切向进行连续性排序，避免在相邻赛段较近时跳到错误分支
6. If color-based pairing is short for local_pairing_min_streak consecutive cycles, local-frame left/right geometric pairing may be used as a fallback. The default is 3 cycles, so fallback does not replace normal red/blue pairing too early.
7. If colors are severely imbalanced, local-frame pairing is allowed immediately; if that still fails, Delaunay fallback is used only when it produces at least `delaunay_min_waypoints` centerline points. The default is 3 because the current online cone map often exposes only a short local fallback; path_generator caps these short centerlines to low speed.
8. 兜底路径会按当前车辆航向过滤明显位于车后的中点，并在必要时翻转局部路径顺序，降低中心线反向导致掉头的概率
9. 输出为 `autoware_msgs/Lane`

> **后续演进：相机颜色融合。** 紧凑赛道中相邻赛段的几何距离可能小于 LiDAR-only
> Delaunay 兜底的可判别尺度，因而仍可能选择错误分支。实车应接入相机锥桶分类，将稳定的
> 红/蓝语义颜色融合到现有 `ConeArray`/`ConeMap` 数据链路；规划即可优先进行显式左右边界配对。
> 检测级后融合 `detection_fusion_node` 和双目深度适配器现已实现；真实相机驱动与
> YOLOv8 推理仍预留。独立融合入口输出 ConeArray，再由 builder 建图；颜色不足时仍使用既有兜底。

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
- 将稀疏局部中心线按 `trackdrive.path.resample_spacing` 重采样
- 根据重采样后的局部曲率限制 waypoint 速度：直道不超过 `trackdrive.speed.explore.max_velocity`（第1圈），弯道不低于 `trackdrive.speed.explore.min_velocity`，横向加速度上限由 `trackdrive.speed.explore.lateral_accel_limit` 控制；RACE 状态使用 `trackdrive.speed.race.*` 覆盖
- 进入循迹后前 `trackdrive.speed.launch.distance` 米内，速度上限为 `trackdrive.speed.launch.velocity`（起步限速，替代原控制侧 start_speed）
- 当在线中心线源点数很少（默认不超过 3 点）时，将速度上限临时压到 `trackdrive.speed.degraded_velocity`，避免短 Delaunay 兜底在紧凑弯道里被高速追踪成掉头
- 发布前检查 Trackdrive 局部中心线是否仍有车头前方目标点；若没有，则拒绝该帧反向/不可追踪路径并保持上一条有效路径，避免车辆被短局部路径诱导掉头

#### SKIDPAD（八字绕桩）
- **临时无人 U 弯测试已启用**：`skidpad.u_turn_test.enabled: true`，沿用 SKIDPAD 任务、发布话题和控制参数。设置为 `false` 恢复下述原八字。
- 测试以 `skidpad.anchor.entry` 的绝对 map 位置/朝向为起点，直行 `straight_distance` → 左转半圆（`radius`）→ 反向直行同样距离，终点回到起点横截线，左侧偏移 `2*radius`。路径缓存后不随定位移动。
- 当前直行 15 m、半径 9.125 m、速度 1 m/s；返程最后 `brake_distance=5 m` 按速度平方插值减速到 0。默认起点 `(-15,0,0)`，转弯入口 `(0,0)`、出口 `(0,18.25)`，停车点 `(-15,18.25)`。测试不使用 `geometry.crossing` 或 `anchor.brake/stop`。

- 使用 `skidpad.geometry.crossing` 固定 map 参考（交叉点），与 `tracks/skidpad.yaml` 对齐，不随定位位姿重建
- 车辆从 `skidpad.anchor.entry`（默认 `(-15, 0)`，计时线前 15 m）发车，直行至交叉点后生成下方右圆两圈（第一圈建立转向、第二圈计时）→ 上方左圈两圈（第三圈过渡、第四圈计时）→ 出口 `brake`→`stop` 停车
- 出口减速段按匀减速剖面 `v²=2aΔs` 从 `skidpad.speed.velocity` 降到 0
- 圆半径：9.125m（FSG 规定）
- 路径几何只生成一次；在有效任务状态下重复发布缓存路径，确保晚启动的控制器能够接收
- 规划轨迹由通用记录器落盘（见 `record.*`），不再单独导出 skidpad CSV

#### ACCELERATION（直线加速）
- 默认 `acceleration.anchor_frame: start_pose`：YAML 锚点相对任务启动时冻结的车辆位姿，X 向前、Y 向左、yaw=0 沿发车车头；设为 `map` 可恢复绝对地图锚点。启动时转换到 map：`acceleration.anchor.entry`（发车，默认 `x=-0.30 m`）→ `brake`（赛道终点/开始制动，默认 `x=75 m`）→ `stop`（默认 `x=175 m`）
- 发车到终点巡航保持 `acceleration.speed.velocity`；终点后按匀减速剖面 `v²=2aΔs` 制动到 0
- 启动锚定需要有效、新鲜（接收不超过0.5s）的 map 定位及定位 ready；生成后缓存，不随车辆移动重建。同一赛项新任务重新锚定。

#### EBS TEST
- 用绝对 map 坐标锚点描述：`ebs.anchor.entry`（发车，默认 `x=0.3 m`）→ `brake`（25 m 测速/急停点）→ `stop`（默认 `x=35 m`，制动段 ≤10 m）
- 满足赛规 7.5：25 m 测速点 ≥40 km/h(11.11 m/s)，急停后 ≤10 m 内停车；结构与 acceleration 相同

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
| `boundary_detector.lookahead_distance` | 15.0 m | 高速循迹在线红/蓝锥配对和 Delaunay 兜底的局部取锥范围；大于控制器 14 m 高速前视，同时减少紧凑图上跨分支误配 |
| `boundary_detector.local_pairing_min_streak` | 3 | 颜色配对连续不足多少个周期后，允许车辆局部坐标系左右锥几何配对兜底；仿真中优先避免颜色误判后长时间断路 |
| `boundary_detector.local_pairing_color_imbalance_ratio` | 0.20 | 红/蓝较少一侧低于该比例时，认为颜色严重失衡并立即启用局部左右配对兜底 |
| `boundary_detector.delaunay_min_waypoints` | 3 | Delaunay fallback must produce at least this many centerline points; 3-point fallback is allowed but path_generator caps short centerlines to low speed |
| `trackdrive.speed.explore.max_velocity` | 7.0 m/s | 第1圈（探索圈）循迹速度 |
| `trackdrive.path.resample_spacing` | 1.0 m | 高速循迹局部中心线重采样间距，用于给 Pure Pursuit 提供连续前向目标 |
| `trackdrive.speed.explore.min_velocity` | 3.0 m/s | 第1圈曲率限速的最低目标速度 |
| `trackdrive.speed.explore.lateral_accel_limit` | 4.0 m/s^2 | 第1圈曲率限速使用的横向加速度上限 |
| `trackdrive.speed.race.lap2_max_velocity` | 9.0 m/s | RACE 第2圈速度上限（lap_count≤1） |
| `trackdrive.speed.race.lap3_max_velocity` | 10.0 m/s | RACE 第3圈起速度上限（lap_count>1） |
| `trackdrive.speed.race.min_velocity` | 4.0 m/s | RACE 曲率限速最低目标速度 |
| `trackdrive.speed.race.lateral_accel_limit` | 6.0 m/s^2 | RACE 曲率限速横向加速度上限 |
| `trackdrive.speed.launch.velocity` | 3.0 m/s | 起步阶段速度上限（进入循迹后前段距离） |
| `trackdrive.speed.launch.distance` | 12.0 m | 起步限速持续距离 |
| `trackdrive.path.min_forward_target` | 0.5 m | Trackdrive 新局部路径至少需要包含一个车头前方目标点，否则保持上一条有效路径 |
| `trackdrive.speed.degraded_velocity` | 3.0 m/s | 短中心线或低置信度时的降级速度帽，主要保护 2-3 点 Delaunay 兜底 |
| `trackdrive.path.short_centerline_points` | 3 | 源中心线点数小于等于该值时启用短中心线降速 |
| `map_origin.x/y/yaw` | 0.0 | 地图坐标系原点；路径锚点均用绝对 map 坐标，本项仅作声明/校验 |
| `skidpad.geometry.radius` | 9.125m | FSG 标准圆半径 |
| `skidpad.geometry.crossing.x/y/yaw` | 0.0 | 八字交叉点（几何锚点，进/出方向） |
| `skidpad.anchor.entry.x/y/yaw` | -15.0 / 0 / 0 | 发车坐标（计时线前 15 m） |
| `skidpad.anchor.brake.x/y/yaw` | 15.0 / 0 / 0 | 出口减速点 |
| `skidpad.anchor.stop.x/y/yaw` | 25.0 / 0 / 0 | 出口停止点 |
| `skidpad.speed.velocity` | 5.0 m/s | 八字速度 |
| `driven_trajectory.smoothing_alpha` | 0.20 | 仅用于 RViz 实际轨迹的一阶平滑；不改变定位、建图或控制输入 |
| `driven_trajectory.min_distance` | 0.10 m | 平滑后轨迹点的最小空间间隔，抑制静止时的噪声折线 |
| `driven_trajectory.max_step` | 2.0 m | 单帧跳变门限，超过视为定位跳变（显示与记录共用） |
| `driven_trajectory.display_window_m` | 200.0 m | RViz 显示只保留最近该长度的历史，避免内存无界增长 |
| `record.enabled` | true | 是否记录规划轨迹与实际行驶位姿 |
| `record.run_dir` | 空 | 启动入口提供的绝对运行目录；轨迹写入 jrajectory/，优先于 record.dir |
| `record.dir` | `logs/{mission}/{stamp}/jrajectory` | 独立启动目录模板；相对项目根目录解析；全栈由 record.run_dir 指定 |
| `record.planned_file` / `record.driven_file` | `planned.csv` / `driven.csv` | 规划轨迹 / 实际位姿文件名 |
| `record.flush_interval_sec` | 1.0 s | 落盘刷新周期 |
| `record.max_mb` | 50.0 | 单文件超过则切分 `_partNNN` |
| `record.attach_stamps` | true | driven.csv 是否附加车速/指令源时间戳 |
| `acceleration.anchor.entry.x/y/yaw` | -0.30 / 0 / 0 | 发车坐标（起点线） |
| `acceleration.anchor.brake.x/y/yaw` | 75.0 / 0 / 0 | 赛道终点 / 开始制动点 |
| `acceleration.anchor.stop.x/y/yaw` | 175.0 / 0 / 0 | 停止坐标（终点后 100 m） |
| `acceleration.speed.velocity` | 15.0 m/s | 加速直线速度 |
| `ebs.anchor.entry.x/y/yaw` | 0.3 / 0 / 0 | 发车坐标（起点线后 0.3 m） |
| `ebs.anchor.brake.x/y/yaw` | 25.0 / 0 / 0 | 25 m 测速/急停标记 |
| `ebs.anchor.stop.x/y/yaw` | 35.0 / 0 / 0 | 停止坐标（制动段 ≤10 m） |
| `ebs.speed.velocity` | 12.0 m/s | EBS 直线速度 |

---

## 线程模型

两个节点均为单线程，回调轻量，无需多线程。

## 待完善

- [ ] Delaunay PathSearch 的起点初始化逻辑（`SetStartPoint`）
- [ ] Trackdrive 局部中心线分支选择和平滑，重点降低紧凑外部图上的瞬时大偏差
