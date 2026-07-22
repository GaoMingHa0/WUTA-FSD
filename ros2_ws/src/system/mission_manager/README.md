# mission_manager

系统状态机节点，管理整个比赛流程的模式切换。

## 职责

- 监控所有子系统就绪状态
- 驱动任务状态机：IDLE → READY → EXPLORE → MAPPING_DONE → RACE → FINISH
- 触发定位模式切换（KISS-ICP ↔ NDT）
- 响应急停信号

## 状态机

```
IDLE ──(传感器就绪)──→ READY ──(/system/inspection_trigger)──→ INSPECTION ──→ READY
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
                                              (NDT地图构建完成)
                                                      │
                                                    RACE
                                                      │
                                              (完成圈数/到达终点)
                                                      │
                                                   FINISH

任意状态 ──(/system/emergency=true)──→ EMERGENCY
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/system/mission_state` | `MissionState` | **唯一发布者**；10Hz 周期广播 |
| 订阅 | `/mapping/cone_map` | `ConeMap` | 监听 `is_closed` |
| 订阅 | `/system/emergency` | `std_msgs/Bool` | 急停信号 |
| 订阅 | `/system/lidar_ready` | `std_msgs/Bool` | LiDAR 就绪 |
| 订阅 | `/system/localization_ready` | `std_msgs/Bool` | 定位就绪 |
| 订阅 | `/system/mission_mode_cmd` | `std_msgs/String` | 设置任务模式（trackdrive/skidpad/acceleration） |
| 订阅 | `/system/start_command` | `std_msgs/Bool` | `true` 请求出发；在两项就绪后从 READY 进入 EXPLORE |
| 订阅 | `/system/mission_complete` | `std_msgs/Bool` | 控制器完成停车后进入 FINISH |
| 订阅 | `/ndt/map_ready` | `std_msgs/Bool` | map_saver 保存 NDT 地图后触发 `MAPPING_DONE` → `RACE` |
| 订阅 | `/system/inspection_trigger` | `std_msgs/Bool` | **[预留]** 触发车检流程 |
| 发布 | `/system/inspection_result` | `std_msgs/String` | **[预留]** 车检结果输出 |

## 线程模型

单线程，所有回调轻量。

## 待完善

- [x] NDT 地图保存完成后，由 `/ndt/map_ready=true` 触发 `MAPPING_DONE` → `RACE`
- [ ] 完成圈数计数（RACE → FINISH）
