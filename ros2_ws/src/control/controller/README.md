# controller

Pure Pursuit 横向控制 + 速度跟踪纵向控制节点。
算法来自 HRT-D Control，去掉 HPL 抽象层，改用直接 rclcpp。

## 算法

### 横向控制：Pure Pursuit

```
输入: 当前位姿(x, y, yaw)，目标路径点(tx, ty)

1. 动态前视距离
   LD = velocity × ld_ratio，clamp 到 [min_lookahead, max_lookahead]

2. 寻找目标点
   从当前路径进度向前遍历，找车头前方且距离达到 LD 的点；若本帧没有车头前方目标则发布停车指令，避免追向车后路径点

3. 计算曲率（vehicle body frame）
   x_body = -dx·sin(yaw) + dy·cos(yaw)   ← 目标点在车体坐标系的横向偏移
   kappa = 2·x_body / dist²

4. 转向角（Ackermann 自行车模型）
   δ = atan(wheel_base × kappa)  [degrees]
```

### 赛项专用前视距离

Acceleration 保持通用动态前视：`LD = |velocity| × ld_ratio`，并限制在
`[min_lookahead, max_lookahead]`。`MISSION_TRACKDRIVE` 默认启用受限动态前视：在局部
中心线前方 12 m 内估计曲率，直线保持 `trackdrive_lookahead=5.0 m`，高曲率处平滑缩短至
`trackdrive_min_lookahead=3.0 m`。该计算不使用规划器的目标速度，因此不同正式圈的速度档位
不会直接改变横向控制。`MISSION_SKIDPAD` 使用固定 `skidpad_lookahead=3.0 m`。Trackdrive
仍保留 Pure Pursuit 的前向目标保护，局部在线中心线瞬时反向时会停车而不会掉头追向车后路径点。

Trackdrive 以曲率的稳健分位数和部分最大值兼顾路径噪声抑制与弯道入口预判，并用
`trackdrive_lookahead_rate_limit` 限制前视距离变化率，避免地图刷新造成目标点和转向突变。

Skidpad 目标速度为 5 m/s 时，过大的通用前视会接近 9.125 m 圆半径。
在入口、右/左圆切换和第四圈出口处，目标点会跨越交叉点的曲率突变，导致车辆切向圆内侧或在
出口过早卸载转向。3.0 m 前视只预览当前局部圆弧，保留转向直到实际切换点；它不改变其它赛项
的动态前视行为。

### 纵向控制：速度跟踪

- 转向目标取前视 waypoint。Trackdrive 的目标速度也取该前视点，保证每次在线局部中心线刷新后仍能采用即将进入弯道的曲率限速；Skidpad 与 Acceleration 保持取当前单调路径进度 waypoint 的速度，避免在停车出口提前一个前视距离减速
- Trackdrive 从首个有效前向目标开始，在 `trackdrive_start_speed_duration` 内将速度目标固定为 `trackdrive_start_speed`（默认 3 m/s、4 s）；该阶段让初始锥筒地图和在线中心线稳定，结束后自动恢复前视点的曲率速度剖面
- Skidpad/Acceleration 的零速终点只有在车辆进入 `finish_position_tolerance` 后才允许成为单调进度点；此前保持倒数正速度点，避免定位噪声让车辆在终点前数米停车
- 由 path_generator 在各模式下写入（trackdrive=7m/s, skidpad=5m/s, acceleration=15m/s）
- TwistFilter 做平滑处理，避免急加速/急减速

### TwistFilter 安全过滤

| 场景 | 速度滤波 | 说明 |
|------|----------|------|
| 加速 | `0.9×last + 0.1×input` | 缓慢加速，防轮滑 |
| 减速 | `0.3×last + 0.7×input` | 快速响应，保安全 |
| 转向 | hard clamp ±max_steer_angle | 超限直接截断 |
| 急停 | velocity = 0 | Emergency flag 触发 |

## 数据流

```
/localization/pose  ──→ 更新 (x, y, yaw)
/chcnav/velocity ──→ 更新 velocity（华测 INS 车速）
/planning/final_waypoints ──→ 更新路径
/system/mission_state ──→ enabled 标志（EXPLORE/RACE 时才运行）

50Hz 定时器:
  pure_pursuit.compute() → raw (angle, velocity)
  twist_filter.filter()  → filtered (angle, velocity)
  → /control/command (autoware_msgs/Command)
  → /control/target_viz (可视化：目标点 + 前视圆)
  → /system/mission_complete (Skidpad 或 Acceleration 在停车终点后一次发布 true)
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/localization/pose` | `PoseStamped` | 当前位姿 |
| 订阅 | `/chcnav/velocity` | `TwistStamped` | 当前速度（华测 INS，前视 + PID 反馈） |
| 订阅 | `/planning/final_waypoints` | `autoware_msgs/Lane` | 参考路径 |
| 订阅 | `/system/mission_state` | `MissionState` | 使能控制 |
| 发布 | `/control/command` | `autoware_msgs/Command` | 转向角 + 速度；发布前写入 `header.stamp`，供仿真统计 LiDAR→控制命令延迟 |
| 发布 | `/system/mission_complete` | `std_msgs/Bool` | Skidpad 在 25 m 出口或 Acceleration 在 100 m 停止区末端停车后发布 `true` |
| 发布 | `/control/target_viz` | `MarkerArray` | 目标点 + 前视圆 |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wheel_base` | 1.53m | 轴距 |
| `max_steer_angle` | 25° | 最大转向角 |
| `ld_ratio` | 2.0 | Acceleration 的动态前视距离系数 |
| `min_lookahead` | 2.0m | 动态前视距离下限（低速） |
| `max_lookahead` | 20.0m | 动态前视距离上限（高速） |
| `trackdrive_dynamic_lookahead` | true | 启用 Trackdrive 基于前方中心线曲率的受限动态前视；关闭时退回固定前视 |
| `trackdrive_lookahead` | 5.0m | Trackdrive 直线/低曲率时的前视上限；不随规划目标速度变化 |
| `trackdrive_min_lookahead` | 3.0m | Trackdrive 高曲率时的前视下限 |
| `trackdrive_curvature_preview_distance` | 12.0m | 提前检查的局部中心线长度，使进入弯道前已缩短前视 |
| `trackdrive_straight_curvature` | 0.03 1/m | 超过该曲率后开始从上限缩短前视 |
| `trackdrive_corner_curvature` | 0.16 1/m | 到达该曲率时采用最小前视 |
| `trackdrive_lookahead_rate_limit` | 3.0m/s | 前视距离的最大变化率，避免路径刷新导致突变 |
| `trackdrive_target_loss_hold_time` | 0.5s | Trackdrive 短暂没有前向目标时，保留上一有效命令的最长时间 |
| `trackdrive_target_loss_hold_speed` | 2.0m/s | 保留命令期间的速度上限；超时后控制器停车 |
| `trackdrive_start_speed` | 3.0m/s | 仅 Trackdrive 起步稳定阶段的固定速度目标 |
| `trackdrive_start_speed_duration` | 4.0s | 从第一个有效前向目标起算的固定速度时长；设为 `0` 可关闭 |
| `max_progress_advance` | 4 | 单次控制循环允许推进的最大路径点数；防止 Skidpad 跳至出口 |
| 前向目标保护 | 内置 | Pure Pursuit 只选择车体前方的目标点；Trackdrive 局部中心线瞬时反向时不会掉头追车后点 |
| `skidpad_lookahead` | 3.0m | 仅 `MISSION_SKIDPAD` 使用的固定前视距离；5 m/s 下替代通用 10 m 前视，避免跨越交叉点的曲率切换 |
| `trackdrive_target_loss_hold_time` | 0.5s | Trackdrive 局部中心线短暂不可追踪时，允许沿用上一条有效控制指令的时间窗口 |
| `trackdrive_target_loss_hold_speed` | 2.0m/s | 沿用上一条 Trackdrive 控制指令时的速度上限，避免一帧坏路径立即停车但仍限制盲开距离 |
| `control_rate_hz` | 50 | 控制频率 |
| `max_steering_rate_deg_s` | 180°/s | 每个控制周期限制转向变化量，抑制定位噪声和目标点离散化导致的指令抖动 |
| `finish_position_tolerance` | 0.75m | Skidpad/Acceleration 零速终点进度与任务完成的位置阈值 |
| `finish_speed_threshold` | 0.2m/s | Skidpad/Acceleration 终点完成速度阈值 |

## 线程模型

单线程。控制计算在 50Hz 定时器回调中执行，
ROS callbacks（pose/vel/waypoints）与 timer 在同一线程顺序执行。

## 待完善

- [ ] 速度 PID 闭环（目前仅前馈，无速度误差反馈）
- [x] Trackdrive 曲率速度规划：`path_generator` 为在线局部中心线生成速度剖面，控制器采用前视目标点速度以在路径刷新后保持弯道减速
- [ ] `/control/command` → VCU CAN 帧的转换（待 VCU 协议确认）
