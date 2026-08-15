# mission_manager

系统状态机节点，管理整个比赛流程的模式切换。

## 职责

- 监控所有子系统就绪状态
- 驱动任务状态机：IDLE → READY → EXPLORE → MAPPING_DONE → RACE → FINISH
- 触发定位模式切换（KISS-ICP ↔ NDT）
- 根据定位位姿穿越起终线统计 Trackdrive 正式圈次
- 响应急停信号

## 状态机

```
IDLE ──(传感器就绪)──→ READY ──(mode_cmd="inspection")──→ INSPECTION ──(演示完成)──→ FINISH
                          │
               (`/system/start_command=true`)
                          │
                          ▼
                       EXPLORE
                                                      │
                                              (地图闭合 is_closed=true)
                                                      │
                                               MAPPING_DONE
                                                      │
                           (地图质量、定位、全局中心线、首圈均合格)
                                                      │
                                                    RACE
                                                      │
                                         (/system/lap_count 达到 3)
                                                      │
                                                   FINISH

任意状态 ──(/system/emergency=true)──→ EMERGENCY
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/system/mission_state` | `MissionState` | **唯一发布者**；10Hz 周期广播 |
| 发布 | `/system/lap_count` | `std_msgs/UInt32` | Trackdrive 正式圈次；Transient Local |
| 订阅 | `/mapping/cone_map` | `ConeMap` | 监听 `is_closed` |
| 订阅 | `/planning/global_centerline_ready` | `std_msgs/Bool` | 冻结全局中心线已通过验收 |
| 订阅 | `/system/localization_confidence` | `std_msgs/Float32` | 定位质量门槛 |
| 订阅 | `/localization/pose` | `geometry_msgs/PoseStamped` | 定位新鲜度和正式过线计圈 |
| 订阅 | `/system/emergency` | `std_msgs/Bool` | 急停信号（仅切状态到 EMERGENCY；控制输出归零由 controller 负责） |
| 订阅 | `/system/lidar_ready` | `std_msgs/Bool` | LiDAR 就绪 |
| 订阅 | `/system/localization_ready` | `std_msgs/Bool` | 定位就绪 |
| 订阅 | `/system/mission_mode_cmd` | `std_msgs/String` | 设置任务模式（trackdrive/skidpad/acceleration/inspection/ebs_test） |
| 订阅 | `/system/start_command` | `std_msgs/Bool` | `true` 请求出发；在两项就绪后从 READY 进入 EXPLORE |
| 订阅 | `/system/mission_complete` | `std_msgs/Bool` | 控制器完成停车后进入 FINISH（含车检演示完成 → FINISH） |
| 订阅 | `/ndt/map_ready` | `std_msgs/Bool` | 仅在 `use_ndt_race_localization=true` 时作为 RACE 门槛 |
| 发布 | `/system/devices_inspection` | `DevicesInspection` | 开机传感器自检结果（失败时发布，通知 can_interface） |

## 线程模型

单线程，所有回调轻量。

## Trackdrive 门槛与圈次

`MAPPING_DONE → RACE` 需要地图闭合、蓝黄锥数量/置信度/颜色平衡合格、定位 ready 且新鲜、
定位置信度合格、冻结全局中心线 ready，以及第一圈已经完成。默认保留 KISS-ICP + EKF；
仅在 NDT 地图保存和初始化链路已集成时启用 `use_ndt_race_localization`。

起终线由首次有效定位位姿及其航向建立。车辆需先离线、达到最短距离和最短用时，再以允许的
航向穿越有限线段才计一圈，避免定位抖动重复计数。仿真真值圈次只用于对照，不控制状态机。
