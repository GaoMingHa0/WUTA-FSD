# controller

Pure Pursuit 横向控制 + 速度跟踪纵向控制节点。算法来自 HRT-D Control，去掉 HPL 抽象层，
改用直接 rclcpp。

除正常赛项控制外，还包含 TwistFilter 安全滤波、速度 PID（目标速度 → 油门/刹车开度）
以及车检（INSPECTION）演示模式。

## 参数命名空间

参数按点分命名空间分类，直线、EBS、八字、高速循迹四个赛项各自拥有一套独立的
`pure_pursuit` / `pid` 调参，`inspection` 独立成流程，`vehicle` / `control` 全局共用。
调参只需修改 `config/controller.yaml`，无需改动源码。

```
vehicle.*        轴距、最大转向角
control.*        控制频率、转向速率限制、速度滤波系数
acceleration.*   直线加速：pure_pursuit + pid + finish
ebs.*            EBS 测试：pure_pursuit + pid + finish
skidpad.*        八字绕环：pure_pursuit + pid + finish
trackdrive.*     高速循迹：pure_pursuit + pid + 起步/丢目标
inspection.*     车检：恒定开度 + 正弦转向
```

## 算法

### 横向控制：Pure Pursuit

```
输入: 当前位姿(x, y, yaw)，车速(vx, vy)，参考路径 waypoints

1. 速度航向角
   course = yaw                                  (|v| ≤ course_speed_threshold)
   course = yaw + atan2(vy, vx)                  (|v| > course_speed_threshold，含侧偏角 β)

2. 前视距离
   LD = override > 0 ? override
      : clamp(|v| × lookahead_ratio, min_lookahead, max_lookahead)

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

`lookahead_ratio` / `min_lookahead` / `max_lookahead` / `max_progress_advance` /
`terminal_progress_distance` / `course_speed_threshold` / `forward_margin` 均取自当前赛项的
`pure_pursuit` 命名空间。曲率公式对 `x_body = 0` 附近保持连续，不做小误差放大。

目标点与进度点的选择都带「前向保护」：只接受车体前方的点（`longitudinalOffset > 0`），
所以局部路径瞬时反向时不会追向车后的点。

### 赛项专用前视距离

| 赛项 | 前视距离 |
|------|----------|
| `acceleration` / `ebs` | `clamp(\|v\| × lookahead_ratio, min_lookahead, max_lookahead)` |
| `trackdrive` | 曲率自适应，`[min_lookahead, max_lookahead]` = [3.0, 5.0] |
| `skidpad` | 固定 `lookahead` = 2.5 m |

**Trackdrive**：在局部中心线前方 `curvature_preview_distance`（12 m）内估计曲率。
对每三点组合算离散曲率 `|Δheading| / 平均弧长`，取「`curvature_percentile` 分位数」与
「`curvature_peak_factor` × 最大值」的较大者：前者抑制路径噪声，后者保留弯道入口预判。
曲率从 `straight_curvature`（0.03 1/m）到 `corner_curvature`（0.16 1/m）线性映射到前视距离
从上限缩短到下限，并用 `lookahead_rate_limit`（3.0 m/s）限制变化率，避免地图刷新造成目标点和
转向突变。该计算不使用规划器的目标速度，因此不同正式圈的速度档位不会直接改变横向控制。
`dynamic_lookahead=false` 或路径点少于 3 个时退回固定 `max_lookahead`。

**Skidpad**：目标速度 5 m/s 时通用前视会接近 9.125 m 圆半径。在入口、右/左圆切换和第四圈
出口处，目标点会跨越交叉点的曲率突变，导致车辆切向圆内侧或在出口过早卸载转向。2.5 m 前视
只预览当前局部圆弧，保留转向直到实际切换点。

### 纵向控制：速度跟踪

- 默认速度取自当前单调进度点 waypoint 的 `twist.linear.x`，由 `path_generator` 在各模式写入
- Trackdrive 改用**前视目标点**的速度：每次在线局部中心线刷新都会把进度点重置到车辆原点
  附近的「曲率为 0、速度为最大」的点，用前视点速度才能在入弯前采用弯道曲率限速
- 起步限速已划归 `path_generator`（`trackdrive.speed.launch.*`），控制侧不再覆盖起步速度
- Trackdrive 短暂没有有效前向目标时，在 `target_loss_hold_time`（0.5 s）内沿用上一条有效命令
  （其速度已由规划侧限速，控制侧不再二次封顶）；超时后停车
- Skidpad/Acceleration/EBS 的零速终点只有在车辆进入 `finish.position_tolerance`（0.75 m）后
  才允许成为单调进度点；此前保持倒数正速度点，避免定位噪声让车辆在终点前数米停车
- TwistFilter 做速度平滑，避免急加速/急减速
- 平滑后的目标速度再经速度 PID 转成 `throttle_brake` 随命令下发

规划速度（`path_generator`）：trackdrive 探索圈 7 m/s、正式圈 9~10 m/s、曲率限速下限 3~4 m/s；
skidpad 5 m/s；acceleration 15 m/s；EBS 12 m/s。

### TwistFilter 安全过滤

[twist_filter.cpp](src/twist_filter.cpp)

| 场景 | 速度滤波 | 说明 |
|------|----------|------|
| 加速 | `(1-accel_alpha)×last + accel_alpha×input` | 缓慢加速，防轮滑 |
| 减速 | `(1-decel_alpha)×last + decel_alpha×input` | 快速响应，保安全 |
| 转向 | hard clamp ±vehicle.max_steer_angle | 超限直接截断 |
| 转向变化率 | 每周期最大 `control.max_steering_rate_deg_s / control.rate_hz`（50 Hz 下 3.6°/周期） | 抑制定位噪声和目标点离散化导致的抖动 |

急停不经过滤波平滑：`emergency_` 置位时节点直接发布全零命令。

### 速度 PID（纵向开度）

[controller_node.cpp](src/controller_node.cpp) `computeSpeedPid()`

- 输入：平滑后的目标速度与 `/chcnav/velocity` 实测车速之差
- 输出：`throttle_brake ∈ [-output_limit, output_limit]`，随 `/control/command` 一起发布
- 每个赛项的 PID 增益独立，取自该赛项 `pid` 命名空间；切换赛项时生效
- 首拍只初始化时间，不输出
- 积分项与输出同量纲，分别钳位到 `integral_limit` / `output_limit`
- 目标速度 ≤ `pid.stop_clear_eps` 且实测车速 < `pid.stop_clear_speed` 时清零积分，防止溜车
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
  该赛项 `finish.position_tolerance` + 车速 ≤ 该赛项 `finish.speed_threshold`」时发布 `mission_complete`
- 车检：进入 `INSPECTION` 后以**恒定开度** `inspection.throttle` 驱动（**不走 PID**：
  车举升/拆胎时唯一的反馈——华测车速——恒为 0，速度环不可观测），并叠加
  `inspection.steer_amplitude` @ `inspection.steer_period` 的正弦转向，到达 `inspection.duration`
  后发零命令并回报完成。频率由周期与时长自动配成整数个半周期，保证收尾回中无跳变

## 关键参数

### 全局

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vehicle.wheel_base` | 1.614 m | 轴距 |
| `vehicle.max_steer_angle` | 28° | 前轮最大转角（外轮设计值） |
| `control.rate_hz` | 50 Hz | 控制频率 |
| `control.max_steering_rate_deg_s` | 180°/s | 每周期转向变化量上限，抑制指令抖动 |
| `control.accel_alpha` | 0.1 | 加速平滑系数 |
| `control.decel_alpha` | 0.7 | 减速响应系数 |

### 赛项：acceleration / ebs

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `*.pure_pursuit.lookahead_ratio` | 2.0 | 动态前视 = 车速 × 系数 |
| `*.pure_pursuit.min_lookahead` | 2.0 m | 前视下限 |
| `*.pure_pursuit.max_lookahead` | 20.0 m | 前视上限 |
| `*.pure_pursuit.max_progress_advance` | 4 | 单周期最大进度推进点数 |
| `*.pure_pursuit.terminal_progress_distance` | 0.75 m | 终点进度点触发距离 |
| `*.pure_pursuit.course_speed_threshold` | 0.5 m/s | 速度航向角切换门限 |
| `*.pure_pursuit.forward_margin` | 0.5 m | 目标点前向接纳门限 |
| `*.pid.kp / ki / kd` | 1.0 / 0.05 / 0.1 | 速度 PID 增益 |
| `*.pid.stop_clear_eps` | 0.05 m/s | 目标速度消积分阈值 |
| `*.pid.stop_clear_speed` | 0.5 m/s | 消积分车速门限 |
| `*.pid.output_limit` | 1.0 | 开度输出钳位 |
| `*.pid.integral_limit` | 1.0 | 积分项钳位 |
| `*.finish.position_tolerance` | 0.75 m | 终点位置容差 |
| `*.finish.speed_threshold` | 0.2 m/s | 终点速度阈值 |

### 赛项：skidpad

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `skidpad.pure_pursuit.lookahead` | 2.5 m | 固定前视距离 |
| `skidpad.pure_pursuit.*`（其余同上） | — | 进度、门限项同 acceleration |
| `skidpad.pid.*` | 同 acceleration | 独立 PID 增益 |
| `skidpad.finish.*` | 0.75 m / 0.2 m/s | 终点判定 |

### 赛项：trackdrive

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `trackdrive.pure_pursuit.min_lookahead` | 3.0 m | 高曲率前视下限 |
| `trackdrive.pure_pursuit.max_lookahead` | 5.0 m | 直线/低曲率前视上限 |
| `trackdrive.pure_pursuit.dynamic_lookahead` | true | 启用曲率自适应前视；关闭退回固定前视 |
| `trackdrive.pure_pursuit.curvature_preview_distance` | 12.0 m | 前方曲率检查长度 |
| `trackdrive.pure_pursuit.straight_curvature` | 0.03 1/m | 超过该曲率开始缩短前视 |
| `trackdrive.pure_pursuit.corner_curvature` | 0.16 1/m | 到达该曲率采用最小前视 |
| `trackdrive.pure_pursuit.lookahead_rate_limit` | 3.0 m/s | 前视距离最大变化率 |
| `trackdrive.pure_pursuit.curvature_percentile` | 0.75 | 曲率抗噪分位 |
| `trackdrive.pure_pursuit.curvature_peak_factor` | 0.6 | 峰值曲率权重 |
| `trackdrive.pure_pursuit.*`（进度、门限项） | — | 同 acceleration |
| `trackdrive.pid.*` | 同 acceleration | 独立 PID 增益 |
| `trackdrive.target_loss_hold_time` | 0.5 s | 无前向目标时保留上一命令的最长时间 |

### 赛项：inspection

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `inspection.throttle` | 0.05 | 车检**恒定**纵向开度 [0,1]：不走 PID，驱动转速的唯一旋钮 |
| `inspection.steer_amplitude` | 6.0° | 前轮正弦转向幅值（= 方向盘 ±30° ÷ 转向比 5） |
| `inspection.steer_period` | 9.0 s | 正弦转向周期 |
| `inspection.duration` | 27.0 s | 车检时长（= 9.0s × 3 个整周期；赛规 2.8.3 要求 25~30s） |

## 线程模型

单线程。控制计算在 50 Hz 定时器回调 `controlLoop()` 中执行；订阅回调
（pose / velocity / waypoints / mission_state / emergency）与定时器在同一线程顺序执行，
因此无需加锁。订阅回调只刷新成员变量，不做计算。

## 实现状态

- [x] 速度 PID 闭环：`/chcnav/velocity` 反馈 → `/control/command.throttle_brake`
- [x] Trackdrive 曲率速度规划：`path_generator` 为在线局部中心线生成速度剖面，控制器采用前视目标点速度以在路径刷新后保持弯道减速
- [x] `/control/command` → VCU CAN 帧：由 `system/can_interface` 封装为 0x210（Signal1 `throttle_brake` / Signal2 `angle`）
