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
| `LOC_KISS_ICP` | EXPLORE 阶段 | CG-410 + EKF（可选 KISS 速度） | 无需先验地图；绝对位姿由 INS 约束 |
| `LOC_NDT` | RACE 阶段 | NDT 地图匹配 | 需要先验地图，精度高，适合高速循迹 |

### 数据流

```
禾赛128线 ──→ kiss_icp_node ──→ sanitizer ──→ 可选平面速度 ──→ ekf_node ───────────┐
                                                                                       ▼
CG-410 ────→ /chcnav/odometry ──────────────────────────────────────────────────→ localization_manager
                                                                                       │
ndt_node ──→ /ndt/pose ────────────────────────────────────────────────────────────→  │
                                                                                       ▼
                                                                              /localization/pose
```

### Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/system/mission_state` | `MissionState` | 接收模式切换指令 |
| 订阅 | `/odometry/filtered` | `nav_msgs/Odometry` | EKF 融合输出 |
| 订阅 | `/ndt/pose` | `geometry_msgs/PoseStamped` | NDT 输出 |
| 发布 | `/localization/pose` | `geometry_msgs/PoseStamped` | 统一定位输出 |
| 发布 | `/system/localization_ready` | `std_msgs/Bool` | 通知 MissionManager |

### EKF 配置（ekf.yaml）

融合输入：
- `odom0`：可选的 KISS-ICP 平面速度与偏航角速度。净化器先拒绝非有限、超速或与 INS
  明显不一致的单帧增量；KISS 的全局 pose 不进入 EKF。
- `odom1`：CG-410 INS 的绝对位置、姿态、纵向速度与偏航角速度。

`fuse_kiss_odometry=false` 时净化器不启动，EKF 仅融合 INS。模拟器默认采用该模式，因为只有
锥桶特征的 ICP 可能匹配到相似赛段；需要在真实环境验证 KISS 冗余速度时才显式开启。

仿真默认由 `WUTA-SIM/wuta-ins-simulator` 发布 `/chcnav/odometry`，与实车华测驱动
（`humble-chcnav-cgi_ros2pkg` 的 `hc_cgi_protocol_process_node`）话题一致。

---

## kiss_icp_wrapper

包含禾赛128线参数文件和 `kiss_odom_sanitizer_node`。后者只派生并发布经过检查的局部速度，
不把 KISS 的漂移全局位姿交给 EKF。

关键参数（kiss_icp_hesai128.yaml）：

| 参数 | 值 | 说明 |
|------|----|------|
| `deskew` | true | 运动畸变补偿，高速时重要 |
| `max_range` | 50.0m | 赛道场景不需要200m |
| `min_range` | 1.5m | 避免车身自检测 |
| `voxel_size` | 0.5m | 适配锥桶尺度特征 |

## 线程模型

单线程。所有回调仅做数据转发，无重计算。
