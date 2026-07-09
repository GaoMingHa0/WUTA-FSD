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
- 根据当前车辆位置和朝向，计算两个圆心
- 生成右圈×2 + 左圈×2 的完整八字路径（FSG 标准）
- 圆半径：9.125m（FSG 规定）
- 在 MissionState 变为 EXPLORE/RACE 时一次性生成并发布

#### ACCELERATION（直线加速）
- 沿车辆当前朝向生成 75m 直线
- 末尾 10m 线性减速到 0
- 在 MissionState 变为 EXPLORE/RACE 时一次性生成并发布

### Topics

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/system/mission_state` | `MissionState` |
| 订阅 | `/planning/centerline` | `autoware_msgs/Lane` |
| 订阅 | `/localization/pose` | `PoseStamped` |
| 发布 | `/planning/final_waypoints` | `autoware_msgs/Lane` |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `trackdrive_velocity` | 7.0 m/s | 循迹速度 |
| `skidpad_radius` | 9.125m | FSG 标准圆半径 |
| `skidpad_velocity` | 5.0 m/s | 八字速度 |
| `acceleration_velocity` | 15.0 m/s | 加速直线速度 |

---

## 线程模型

两个节点均为单线程，回调轻量，无需多线程。

## 待完善

- [ ] Delaunay PathSearch 的起点初始化逻辑（`SetStartPoint`）
- [ ] Skidpad 路径与实际赛道锥桶对齐（目前基于车辆位姿估算圆心）
- [ ] Trackdrive 速度规划：根据曲率动态调整速度（曲率大→减速）
