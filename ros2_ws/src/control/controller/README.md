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
   从路径末尾往前遍历，找第一个距离 < LD 的点

3. 计算曲率（vehicle body frame）
   x_body = -dx·sin(yaw) + dy·cos(yaw)   ← 目标点在车体坐标系的横向偏移
   kappa = 2·x_body / dist²

4. 转向角（Ackermann 自行车模型）
   δ = atan(wheel_base × kappa)  [degrees]
```

### 纵向控制：速度跟踪

- 目标速度直接读取 waypoint 的 `twist.twist.linear.x`
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
/localization/velocity ──→ 更新 velocity
/planning/final_waypoints ──→ 更新路径
/system/mission_state ──→ enabled 标志（EXPLORE/RACE 时才运行）

50Hz 定时器:
  pure_pursuit.compute() → raw (angle, velocity)
  twist_filter.filter()  → filtered (angle, velocity)
  → /control/command (autoware_msgs/Command)
  → /control/target_viz (可视化：目标点 + 前视圆)
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/localization/pose` | `PoseStamped` | 当前位姿 |
| 订阅 | `/localization/velocity` | `TwistStamped` | 当前速度 |
| 订阅 | `/planning/final_waypoints` | `autoware_msgs/Lane` | 参考路径 |
| 订阅 | `/system/mission_state` | `MissionState` | 使能控制 |
| 发布 | `/control/command` | `autoware_msgs/Command` | 转向角 + 速度 |
| 发布 | `/control/target_viz` | `MarkerArray` | 目标点 + 前视圆 |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wheel_base` | 1.53m | 轴距 |
| `max_steer_angle` | 25° | 最大转向角 |
| `ld_ratio` | 2.0 | 前视距离系数 |
| `min_lookahead` | 2.0m | 前视距离下限（低速） |
| `max_lookahead` | 20.0m | 前视距离上限（高速） |
| `control_rate_hz` | 50 | 控制频率 |

## 线程模型

单线程。控制计算在 50Hz 定时器回调中执行，
ROS callbacks（pose/vel/waypoints）与 timer 在同一线程顺序执行。

## 待完善

- [ ] 速度 PID 闭环（目前仅前馈，无速度误差反馈）
- [ ] 曲率自适应速度规划（大曲率 → 自动降速）
- [ ] `/control/command` → VCU CAN 帧的转换（待 VCU 协议确认）
