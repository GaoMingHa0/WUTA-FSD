# planning

规划模块，支持三种比赛模式。

```
planning/
├── boundary_detector/   ← Delaunay 三角剖分，输出赛道中心线（TRACKDRIVE）
├── skidpad_detector/    ← 圆拟合（Kasa 最小二乘），输出两个圆心（SKIDPAD）
├── line_detector/       ← PCA 直线检测，输出赛道终点（ACCELERATION）
└── path_generator/      ← 三模式分发，输出 final_waypoints 给控制器
```

---

## 整体数据流

```
ConeMap ──→ boundary_detector ──→ /planning/centerline ──┐
        ──→ skidpad_detector  ──→ /planning/skidpad_circles ──→ path_generator
        ──→ line_detector     ──→ /planning/acceleration_line ─┘
                                                               │
                                              /planning/final_waypoints
                                                               │
                                                          controller
```

| 比赛模式 | 用到的节点 |
|---------|-----------|
| TRACKDRIVE | boundary_detector → path_generator |
| SKIDPAD | skidpad_detector → path_generator |
| ACCELERATION | line_detector → path_generator |

---

## boundary_detector

### 算法：Delaunay 三角剖分（来自 HRT-D）

1. 从 `/mapping/cone_map` 提取当前 `lookahead_distance` 范围内的锥桶坐标
2. 对所有锥桶做 Delaunay 三角剖分（BowyerWatson 算法）
3. 取三角形各边中点作为候选路径点（`MidPoint`）
4. DFS 搜索最优路径（方向连续性 + 代价最低）
5. 输出为 `autoware_msgs/Lane`

**只在 TRACKDRIVE 模式下运行。**
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

## skidpad_detector

### 算法：Kasa 代数圆拟合

FSG 八字赛道由两个对称圆组成（标准内径 9.125m）。
蓝色锥桶围成左圆，黄色锥桶围成右圆。

1. 从 ConeMap 分别取蓝/黄锥桶坐标
2. 对每组锥桶用 Kasa 最小二乘法拟合圆（无需 Eigen，直接 3×3 线性方程）
3. 拒绝拟合半径与期望值（9.125m）偏差 >2.5m 的结果
4. 发布两个圆心坐标供 path_generator 使用

### Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/mapping/cone_map` | `ConeMap` | |
| 订阅 | `/system/mission_state` | `MissionState` | |
| 发布 | `/planning/skidpad_circles` | `PoseArray` | poses[0]=右圆心, poses[1]=左圆心 |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_cones_per_circle` | 4 | 最少锥桶数才尝试拟合 |
| `expected_radius` | 9.125m | FSG 标准圆半径 |
| `radius_tolerance` | 2.5m | 拟合结果合法区间 |

---

## line_detector

### 算法：PCA 主成分直线检测

加速直线赛道两侧各有一排锥桶，沿同一方向排列。

1. 取所有锥桶（蓝+黄+橙）坐标
2. PCA：计算协方差矩阵，解析求 2×2 最大特征向量 → 赛道方向
3. 将所有锥桶投影到主轴，取最大投影 + buffer 作为终点
4. 发布终点坐标及赛道方向四元数

### Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/mapping/cone_map` | `ConeMap` | |
| 订阅 | `/system/mission_state` | `MissionState` | |
| 发布 | `/planning/acceleration_line` | `PoseStamped` | position=终点, orientation=赛道方向 |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_cones` | 4 | 最少锥桶数 |
| `endpoint_buffer` | 3.0m | 最远锥桶投影后额外延伸距离 |

---

## path_generator

### 三种模式

#### TRACKDRIVE（高速循迹）
- 直接转发 `boundary_detector` 输出的中心线
- 更新各 waypoint 速度为 `trackdrive_velocity`

#### SKIDPAD（八字绕桩）
- **优先**使用 `skidpad_detector` 输出的圆心（锥桶拟合）
- 若检测器尚未就绪，回退到以车辆位姿估算圆心（带 WARN 日志）
- 生成右圈×2 + 左圈×2 完整八字路径（FSG 标准顺序）

#### ACCELERATION（直线加速）
- **优先**使用 `line_detector` 输出的终点和赛道方向（PCA 检测）
- 若检测器尚未就绪，回退到固定长度（`acceleration_length_`，默认 75m）+ 车辆朝向
- 末尾 10m 线性减速

### Topics

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/system/mission_state` | `MissionState` |
| 订阅 | `/planning/centerline` | `autoware_msgs/Lane` |
| 订阅 | `/localization/pose` | `PoseStamped` |
| 订阅 | `/planning/skidpad_circles` | `PoseArray` |
| 订阅 | `/planning/acceleration_line` | `PoseStamped` |
| 发布 | `/planning/final_waypoints` | `autoware_msgs/Lane` |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `trackdrive_velocity` | 7.0 m/s | 循迹速度 |
| `skidpad_radius` | 9.125m | 回退时用（FSG 标准圆半径） |
| `skidpad_velocity` | 5.0 m/s | 八字速度 |
| `acceleration_length` | 75.0m | 回退时用（固定直线长度） |
| `acceleration_velocity` | 15.0 m/s | 加速直线速度 |

---

## 线程模型

所有节点均为单线程，回调轻量，无需多线程。

---

## 待完善

- [ ] Delaunay PathSearch 起点初始化逻辑（`SetStartPoint`）
- [ ] Trackdrive 速度规划：根据曲率动态调整速度
- [ ] Skidpad 进出圆过渡段路径（当前两圆之间无平滑连接）
- [ ] Acceleration 中心线生成（当前路径为单条直线，未取锥桶左右均值）
