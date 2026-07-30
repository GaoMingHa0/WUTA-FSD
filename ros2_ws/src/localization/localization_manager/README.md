# localization（定位模块）

定位模块包含三个组件：

```
localization/
├── kiss-icp/              # KISS-ICP 源码（外部，PRBonn/kiss-icp）
├── robot_localization/    # EKF/UKF 融合（外部，humble-devel 分支）
├── kiss_icp_wrapper/      # 禾赛128线参数配置
├── localization_manager/  # 模式切换节点（本文件所在包）
└── ndt_localization/      # NDT 地图匹配（阶段6实现）
```

---

## localization_manager

统一对外发布 `/localization/pose`，下游节点无感知定位模式切换。

### 定位双模式

| 模式 | 触发条件 | 数据源 | 特点 |
|------|----------|--------|------|
| `LOC_KISS_ICP` | EXPLORE 阶段 | KISS-ICP + EKF（融合 CG-410） | 无需先验地图，纯里程计，长时有漂移 |
| `LOC_NDT` | RACE 阶段 | NDT 地图匹配 | 需要先验地图，精度高，适合高速循迹 |

### 数据流

```
禾赛128线 ──→ kiss_icp_node ──→ /kiss/odometry ──→ ekf_node ──→ /odometry/filtered ──┐
                                                                                       ▼
CG-410 ────→ /cg410/odometry ──────────────────────────────────────────────────────→ localization_manager
                                                                                       │
ndt_node ──→ /ndt/pose ────────────────────────────────────────────────────────────→  │
                                                                                       ▼
                                                                              /localization/pose
```

实际 EKF 输入经过 `kiss_odom_gate_node`：`/kiss/odometry` 在 INS 角速度不超过
`max_abs_yaw_rate=0.20 rad/s` 且持续稳定 1 s 时才转发为 `/kiss/odometry_gated`。高角速度
连续弯时 INS 单独维持全局位姿；门控恢复后只发布连续、经 INS 帧间运动校验的 KISS 增量，
不会把阻断期间的局部 KISS 位姿差作为一次大速度更新注入 EKF。

### Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/system/mission_state` | `MissionState` | 接收模式切换指令 |
| 订阅 | `/odometry/filtered` | `nav_msgs/Odometry` | EKF 融合输出 |
| 订阅 | `/ndt/pose` | `geometry_msgs/PoseStamped` | NDT 输出 |
| 发布 | `/localization/pose` | `geometry_msgs/PoseStamped` | 统一定位输出 |
| 发布 | `/system/localization_ready` | `std_msgs/Bool` | 通知 MissionManager |

### EKF 配置（ekf.yaml）

融合两路输入：
- `odom0`：KISS-ICP（高频局部里程计，xy + yaw；KISS 不发布 Twist 估计）
- `odom1`：CG-410 INS（绝对位置，xyz + rpy，修正漂移）

KISS 设置 `odom0_differential=true`：robot_localization 从其 pose 派生相对运动，不能直接
定义 map 全局位置；INS 继续提供绝对 map 位置与航向。两路更新分别使用
`odom0_pose_rejection_threshold=3.0` / `odom0_twist_rejection_threshold=3.0` 与
`odom1_pose_rejection_threshold=5.0` 的 Mahalanobis 创新门限。尤其是 KISS 门限必须显式设置：
robot_localization 的默认值为无限大，在重复锥桶赛段可能接受错误重定位造成的整段位置跳变。

仿真默认由 `WUTA-SIM/wuta-ins-simulator` 发布 `/cg410/odometry`。真实车辆接入时可保持
该接口，或在 bringup 中重映射实际 CG-410 驱动话题。

---

## kiss_icp_wrapper

仅包含为禾赛128线调优的参数文件，不含代码。

关键参数（kiss_icp_hesai128.yaml）：

| 参数 | 值 | 说明 |
|------|----|------|
| `deskew` | true | 运动畸变补偿，高速时重要 |
| `max_range` | 50.0m | 赛道场景不需要200m |
| `min_range` | 1.5m | 避免车身自检测 |
| `voxel_size` | 0.5m | 适配锥桶尺度特征 |

## 线程模型

单线程。所有回调仅做数据转发，无重计算。
