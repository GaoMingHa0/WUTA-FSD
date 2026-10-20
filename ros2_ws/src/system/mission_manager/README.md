# mission_manager

系统状态机节点，管理整个比赛流程的模式切换。

## 职责

- 监控所有子系统就绪状态
- 驱动任务状态机：IDLE → READY → EXPLORE → MAPPING_DONE → RACE → FINISH
- 触发定位模式切换（KISS-ICP ↔ NDT）
- 根据定位位姿穿越起终线统计 Trackdrive 正式圈次
- 传感器自检失败时切 EMERGENCY 并发布急停

## 状态机

```
IDLE ──(传感器就绪)──→ READY ──(/system/start_command 且已选任务模式)──→ EXPLORE ──(地图闭合 is_closed=true)──→ MAPPING_DONE
                          │                                                    │
                          └(已选 inspection + start_command)─→ INSPECTION ──(演示完成)──→ FINISH
                                                                   (地图质量、定位、全局中心线、首圈均合格)
                                                                                           ▼
                                                                                         RACE ──(/system/lap_count 达到 3)──→ FINISH

传感器自检失败 / RES 急停 ──→ EMERGENCY（并发布 /system/emergency 通知 controller 归零）
```

> **启动门控（GO 回退）**：0x501 只剩「测试模式」字节，RES Go / 急停改由 RES 遥控器的
> **0x1E4** 承载——`can_interface` 解析 Byte1（`0x13` 发车按钮 / `0x10` 急停）后发布
> `/system/start_command` / `/system/emergency`。故**选模式不再启动**：
> 需 `mission_mode_cmd`（设置模式）**且** `/system/start_command=true`（RES 放行）才从
> READY 进 EXPLORE；车检（inspection）同样需要 start_command 才进 INSPECTION。
> 车检不设 GNSS 状态或新鲜度门槛；其他赛项启动及 RACE 仍要求新鲜 GNSS=4。
> 雷达、惯导、相机的设备心跳自检和急停锁存继续生效。
> GO 不锁存：档位真正变化时本节点会丢弃此前按下的 GO（`onMissionModeCmd` 清
> `start_requested_`；保活重发同值不触发），即 GO 必须晚于本次档位选择。

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/system/mission_state` | `MissionState` | **唯一发布者**；10Hz 周期广播 |
| 发布 | `/system/lap_count` | `std_msgs/UInt32` | Trackdrive 正式圈次；Transient Local |
| 订阅 | `/mapping/cone_map` | `ConeMap` | 监听 `is_closed` |
| 订阅 | `/planning/global_centerline_ready` | `std_msgs/Bool` | 冻结全局中心线已通过验收 |
| 订阅 | `/system/localization_confidence` | `std_msgs/Float32` | 定位质量门槛 |
| 订阅 | `/localization/pose` | `geometry_msgs/PoseStamped` | 定位新鲜度和正式过线计圈 |
| 订阅 | `/rslidar_points`（可配置） | `sensor_msgs/PointCloud2` | 真实点云心跳，收到且未超时才判定 LiDAR ready |
| 订阅 | `/system/localization_ready` | `std_msgs/Bool` | 定位就绪 |
| 订阅 | `/system/mission_mode_cmd` | `std_msgs/String` | 设置任务模式（trackdrive/skidpad/acceleration/inspection/ebs_test）；**仅设模式、不启动** |
| 订阅 | `/system/start_command` | `std_msgs/Bool` | RES 发车放行（来自 0x1E4 Byte1=0x13）；与已选模式共同满足才启动 |
| 订阅 | `/system/emergency` | `std_msgs/Bool` | RES 急停（来自 0x1E4 Byte1=0x10）或设备自检失败；**Transient Local 锁存**，收到即切 EMERGENCY |
| 订阅 | `/system/mission_complete` | `std_msgs/Bool` | 控制器完成停车后进入 FINISH（含车检演示完成 → FINISH） |
| 订阅 | `/ndt/map_ready` | `std_msgs/Bool` | 仅在 `use_ndt_race_localization=true` 时作为 RACE 门槛 |
| 发布 | `/system/devices_inspection` | `DevicesInspection` | 开机传感器自检结果（失败时发布，通知 can_interface） |

## 线程模型

单线程，所有回调轻量。

## Trackdrive 门槛与圈次

`MAPPING_DONE → RACE` 需要地图闭合、红蓝锥数量/置信度/颜色平衡合格、定位 ready 且新鲜、
定位置信度合格、冻结全局中心线 ready，以及第一圈已经完成。默认保留 KISS-ICP + EKF；
仅在 NDT 地图保存和初始化链路已集成时启用 `use_ndt_race_localization`。

起终线由首次有效定位位姿及其航向建立。车辆需先离线、达到最短距离和最短用时，再以允许的
航向穿越有限线段才计一圈，避免定位抖动重复计数。仿真真值圈次只用于对照，不控制状态机。

雷达 ready 由真实点云接收心跳计算，不再订阅 `/system/lidar_ready`。默认话题为
`/rslidar_points`，惯导为 `/chcnav/odometry`，相机为
`/zed/zed_node/rgb/image_rect_color`。点云和图像订阅使用 SensorDataQoS。
点云接收超过 `sensor_timeout_sec`（默认2秒）即清除雷达 ready；关闭 `check_lidar`
仅关闭雷达故障急停检查，不会绕过发车所需的真实点云心跳。READY 中收到 GO 时也会
重新检查点云新鲜度与定位 ready。启用雷达自检时，断流按原定时器周期进入锁存急停。

## 只读任务诊断

任务广播的 `description` 保留 `lap` 和门控字段，并增加 `diag_version=2`、
`wait=<逗号分隔原因或none>`。等待原因由任务机提供；显示端只格式化，不推导另一套规则。
雷达就绪通过只读 `lidarReady()` 计算，`publishState()` 不更新就绪标志。
RACE 判断和诊断复用 `raceGates()`，保留原有门槛、GO消费和急停行为。
只读监控退出不停止车辆模块，状态变化事件归入运行目录的 `manager.log`。
