# controller

Pure Pursuit 横向控制 + 速度跟踪纵向控制节点。算法来自 HRT-D Control，去掉 HPL 抽象层，
改用直接 rclcpp。

除正常赛项控制外，还包含 TwistFilter 安全滤波、速度 PID（目标速度 → 油门/刹车开度）
以及车检（INSPECTION）演示模式。

## 算法

### 横向控制：Pure Pursuit

```
输入: 当前位姿(x, y, yaw)，车速(vx, vy)，参考路径 waypoints

1. 速度航向角
   course = yaw                                  (|v| ≤ 0.5 m/s)
   course = yaw + atan2(vy, vx)                  (|v| > 0.5 m/s，含侧偏角 β)

2. 前视距离
   LD = override > 0 ? override : clamp(|v| × ld_ratio, min_lookahead, max_lookahead)

3. 推进进度点（单调）
   progress_idx = max(progress_idx, findNearestForwardIndex(...))
   只在 progress_idx 起 max_progress_advance 个点内搜索，防止自交路径跳圈

4. 选择目标点
   从 progress_idx 起找首个「在车体前方且距离 ≥ LD」的点；
   没有满足 LD 的点则取最远的前向点；连前向点都没有则回退到最后一点，
   再被前向校验判为无效 → 上层发布停车指令，避免掉头追向车后路径点

5. 计算曲率（vehicle body frame）
   x_body = -dx·sin(course) + dy·cos(course)   ← 目标点横向偏移，左正
   kappa  = 2·x_body / dist²

6. 转向角（Ackermann 自行车模型）
   δ = atan(wheel_base × kappa)  [degrees]

7. 速度：默认取进度点速度
   cmd.velocity = waypoints[progress_idx].twist.twist.linear.x
```

曲率公式对 `x_body = 0` 附近保持连续，不做小误差放大，避免阈值跳变引起转向抖动。

目标点与进度点的选择都带「前向保护」：只接受车体前方的点（`longitudinalOffset > 0`），
所以局部路径瞬时反向时不会追向车后的点。

### 赛项专用前视距离

Acceleration 保持通用动态前视：`LD = |velocity| × ld_ratio`，并限制在
`[min_lookahead, max_lookahead]`。

| 赛项 | 前视距离 |
|------|----------|
| `MISSION_ACCELERATION` 及其它 | `clamp(\|v\| × ld_ratio, 2.0, 20.0)` |
| `MISSION_TRACKDRIVE` | 曲率自适应，`[trackdrive_min_lookahead, trackdrive_lookahead]` = [3.0, 5.0] |
| `MISSION_SKIDPAD` | 固定 `skidpad_lookahead` = 2.5 m |

**Trackdrive**：在局部中心线前方 `trackdrive_curvature_preview_distance`（12 m）内估计曲率。
对每三点组合算离散曲率 `|Δheading| / 平均弧长`，取「75 分位数」与「0.6 × 最大值」的较大者：
前者抑制路径噪声，后者保留弯道入口预判。曲率从 `trackdrive_straight_curvature`（0.03 1/m）
到 `trackdrive_corner_curvature`（0.16 1/m）线性映射到前视距离从上限缩短到下限，并用
`trackdrive_lookahead_rate_limit`（3.0 m/s）限制变化率，避免地图刷新造成目标点和转向突变。
该计算不使用规划器的目标速度，因此不同正式圈的速度档位不会直接改变横向控制。
`trackdrive_dynamic_lookahead=false` 或路径点少于 3 个时退回固定 `trackdrive_lookahead`。

**Skidpad**：目标速度 5 m/s 时通用前视会接近 9.125 m 圆半径。在入口、右/左圆切换和第四圈
出口处，目标点会跨越交叉点的曲率突变，导致车辆切向圆内侧或在出口过早卸载转向。2.5 m 前视
只预览当前局部圆弧，保留转向直到实际切换点。

### 纵向控制：速度跟踪

- 默认速度取自当前单调进度点 waypoint 的 `twist.linear.x`，由 `path_generator` 在各模式写入
- Trackdrive 改用**前视目标点**的速度：每次在线局部中心线刷新都会把进度点重置到车辆原点
  附近的「曲率为 0、速度为最大」的点，用前视点速度才能在入弯前采用弯道曲率限速
- Trackdrive 从首个有效前向目标开始，在 `trackdrive_start_speed_duration`（4 s）内将速度目标
  固定为 `trackdrive_start_speed`（3 m/s）；该阶段让初始锥筒地图和在线中心线稳定，结束后
  自动恢复前视点的曲率速度剖面（`trackdrive_start_speed_duration=0` 可关闭）
- Trackdrive 短暂没有有效前向目标时，在 `trackdrive_target_loss_hold_time`（0.5 s）内沿用上一条
  有效命令，并把速度压到 `trackdrive_target_loss_hold_speed`（2 m/s）；超时后停车
- Skidpad/Acceleration 的零速终点只有在车辆进入 `finish_position_tolerance`（0.75 m）后才允许
  成为单调进度点；此前保持倒数正速度点，避免定位噪声让车辆在终点前数米停车
- TwistFilter 做速度平滑，避免急加速/急减速
- 平滑后的目标速度再经速度 PID 转成 `throttle_brake` 随命令下发

规划速度（`path_generator`）：trackdrive 探索圈 7 m/s、正式圈 9~10 m/s、曲率限速下限 3~4 m/s；
skidpad 5 m/s；acceleration 15 m/s；EBS 12 m/s。

### TwistFilter 安全过滤

[twist_filter.cpp](src/twist_filter.cpp)

| 场景 | 速度滤波 | 说明 |
|------|----------|------|
| 加速 | `0.9×last + 0.1×input` | 缓慢加速，防轮滑 |
| 减速 | `0.3×last + 0.7×input` | 快速响应，保安全 |
| 转向 | hard clamp ±max_steer_angle | 超限直接截断 |
| 转向变化率 | 每周期最大 `max_steering_rate_deg_s / control_rate_hz`（50 Hz 下 3.6°/周期） | 抑制定位噪声和目标点离散化导致的抖动 |

急停不经过滤波平滑：`emergency_` 置位时节点直接发布全零命令。

### 速度 PID（纵向开度）

[controller_node.cpp](src/controller_node.cpp) `computeSpeedPid()`

- 输入：平滑后的目标速度与 `/chcnav/velocity` 实测车速之差
- 输出：`throttle_brake ∈ [-1, 1]`，随 `/control/command` 一起发布
- 首拍只初始化时间，不输出
- 积分项与输出同量纲，钳位到 `[-1, 1]` 防饱和
- 目标速度 ≤ 0 且实测车速 < 0.5 m/s 时清零积分，防止停车后残留驱动开度溜车
- 急停或速度反馈未就绪时清 PID 状态并输出 0（保守，不驱动）

## 数据流

```
订阅输入（回调只更新成员变量，不做计算）
    /localization/pose        ──→ pose_ready_ = true;  x, y, yaw
    /chcnav/velocity          ──→ speed_feedback_available_ = true;  vx, vy, |v|
    /planning/final_waypoints ──→ waypoints_ready_ = true;  路径变化则 reset 进度
    /system/mission_state     ──→ enabled_ / mission_mode_ / state_
    /system/emergency         ──→ emergency_

50Hz 定时器 controlLoop()
    1. 早退守卫：emergency → INSPECTION → 未使能 → mission_complete_
    2. pure_pursuit_.compute()      → raw (angle, velocity)
    3. Trackdrive 速度改写 + 起步限速 → raw.velocity 调整
    4. twist_filter_.filter()       → filtered (平滑速度, 限速转向)
    5. computeThrottleBrake()       → throttle_brake（速度 PID）
    → /control/command
    → /control/target_viz
    → /system/mission_complete
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/localization/pose` | `PoseStamped` | 当前位姿 |
| 订阅 | `/chcnav/velocity` | `TwistStamped` | 当前速度（华测 INS，前视 + PID 反馈） |
| 订阅 | `/planning/final_waypoints` | `autoware_msgs/Lane` | 参考路径 |
| 订阅 | `/system/mission_state` | `MissionState` | 使能、赛项模式与任务状态 |
| 订阅 | `/system/emergency` | `std_msgs/Bool` | 急停，置位期间持续输出全零命令 |
| 发布 | `/control/command` | `autoware_msgs/Command` | `angle` + `speed` + `throttle_brake`；发布前写入 `header.stamp`（`frame_id=base_link`），供仿真统计 LiDAR→控制命令延迟 |
| 发布 | `/system/mission_complete` | `std_msgs/Bool` | Skidpad 在 25 m 出口、Acceleration 在 100 m 停止区末端、EBS 测试或车检演示结束后发布 `true` |
| 发布 | `/control/target_viz` | `MarkerArray` | 目标点 + 前视圆（仅当有订阅者时发布） |

`/control/command` 由 `system/can_interface` 消费，封装为 0x210 下发 VCU：
`throttle_brake` → Signal1，`angle` → Signal2。

## 状态与赛项

- 使能状态：`EXPLORE` / `MAPPING_DONE` / `RACE` 时运行控制；其它状态下复位滤波器并发布零命令
- 急停：`emergency_` 为真时持续发布全零命令；该标志由 `/system/emergency` 置位后
  **不可解除**（急停不可恢复，需重启）。该话题由 can_interface（0x501 Byte1=12，VCU 侧急停）
  与 mission_manager（传感器自检失败）共同发布——controller 是唯一的归零执行者
- 路径变化：`onWaypoints` 逐点比较路径，变化时复位 Pure Pursuit 进度与 `mission_complete_`
- 完成判定：`SKIDPAD` / `ACCELERATION` / `EBS_TEST` 在「进度到最后一个点 + 距终点 ≤
  `finish_position_tolerance` + 车速 ≤ `finish_speed_threshold`」时发布 `mission_complete`
- 车检：进入 `INSPECTION` 后以**恒定开度** `inspection_throttle` 驱动（**不走 PID**：
  车举升/拆胎时唯一的反馈——华测车速——恒为 0，速度环不可观测），并叠加
  `inspection_steer_amp` @ `inspection_steer_period` 的正弦转向，到达 `inspection_duration`
  后发零命令并回报完成。`inspection_duration` 应与转向周期成 **0.5 的整数倍**关系
  （半整数周期同样过零 → 收尾回中无跳变），否则启动时会打印告警。

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wheel_base` | 1.53 m | 轴距 |
| `lf` | 0.8 m | 质心到前轴距离（当前算法未参与解算） |
| `max_steer_angle` | 25° | 最大转向角 |
| `ld_ratio` | 2.0 | Acceleration 的动态前视距离系数 |
| `min_lookahead` | 2.0 m | 动态前视距离下限（低速） |
| `max_lookahead` | 20.0 m | 动态前视距离上限（高速） |
| `max_progress_advance` | 4 | 单次控制循环允许推进的最大路径点数；防止 Skidpad 跳至出口 |
| `skidpad_lookahead` | 2.5 m | 仅 `MISSION_SKIDPAD` 使用的固定前视距离 |
| `trackdrive_dynamic_lookahead` | true | 启用 Trackdrive 基于前方中心线曲率的受限动态前视；关闭时退回固定前视 |
| `trackdrive_lookahead` | 5.0 m | Trackdrive 直线/低曲率时的前视上限；不随规划目标速度变化 |
| `trackdrive_min_lookahead` | 3.0 m | Trackdrive 高曲率时的前视下限 |
| `trackdrive_curvature_preview_distance` | 12.0 m | 提前检查的局部中心线长度，使进入弯道前已缩短前视 |
| `trackdrive_straight_curvature` | 0.03 1/m | 超过该曲率后开始从上限缩短前视 |
| `trackdrive_corner_curvature` | 0.16 1/m | 到达该曲率时采用最小前视 |
| `trackdrive_lookahead_rate_limit` | 3.0 m/s | 前视距离的最大变化率，避免路径刷新导致突变 |
| `trackdrive_target_loss_hold_time` | 0.5 s | Trackdrive 短暂没有前向目标时，保留上一有效命令的最长时间 |
| `trackdrive_target_loss_hold_speed` | 2.0 m/s | 保留命令期间的速度上限；超时后控制器停车 |
| `trackdrive_start_speed` | 3.0 m/s | 仅 Trackdrive 起步稳定阶段的固定速度目标 |
| `trackdrive_start_speed_duration` | 4.0 s | 从第一个有效前向目标起算的固定速度时长；设为 `0` 可关闭 |
| `control_rate_hz` | 50 Hz | 控制频率 |
| `max_steering_rate_deg_s` | 180°/s | 每个控制周期限制转向变化量，抑制定位噪声和目标点离散化导致的指令抖动 |
| `finish_position_tolerance` | 0.75 m | Skidpad/Acceleration 零速终点进度与任务完成的位置阈值（同时作为 Pure Pursuit 的终点进度阈值） |
| `finish_speed_threshold` | 0.2 m/s | Skidpad/Acceleration 终点完成速度阈值 |
| `inspection_speed` | 1.0 m/s | 车检名义车速：仅用于 TwistFilter 与日志，**不决定纵向开度** |
| `inspection_throttle` | 0.16 | 车检**恒定**纵向开度 [0,1]：不走 PID，驱动系统转速的唯一旋钮（先低后调）。实测 0.15 不转、0.20 太快（5s 冲到 16847 且未稳） |
| `inspection_steer_amp` | 5.77° | 车检模式正弦转向幅值（**前轮** deg；= 方向盘 ±30° ÷ 转向比 5.2） |
| `inspection_steer_period` | 9.0 s | 车检模式正弦转向周期（**优先**；周期比频率直观） |
| `inspection_steer_freq` | 0.25 Hz | 兼容旧参数：仅当 `inspection_steer_period <= 0` 时生效 |
| `inspection_duration` | 27.0 s | 车检时长（= 9.0s × 3 个整周期；赛规 2.8.3 要求 25~30s），完成后发布 `mission_complete` |
| `pid_speed_kp` | 1.0 | 速度 PID 比例增益 |
| `pid_speed_ki` | 0.05 | 速度 PID 积分增益 |
| `pid_speed_kd` | 0.1 | 速度 PID 微分增益 |

## 线程模型

单线程。控制计算在 50 Hz 定时器回调 `controlLoop()` 中执行；订阅回调
（pose / velocity / waypoints / mission_state / emergency）与定时器在同一线程顺序执行，
因此无需加锁。订阅回调只刷新成员变量，不做计算。

## 实现状态

- [x] 速度 PID 闭环：`/chcnav/velocity` 反馈 → `/control/command.throttle_brake`
- [x] Trackdrive 曲率速度规划：`path_generator` 为在线局部中心线生成速度剖面，控制器采用前视目标点速度以在路径刷新后保持弯道减速
- [x] `/control/command` → VCU CAN 帧：由 `system/can_interface` 封装为 0x210（Signal1 `throttle_brake` / Signal2 `angle`）
