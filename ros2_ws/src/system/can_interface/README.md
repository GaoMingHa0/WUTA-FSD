# CAN Interface (预留)

此目录为实车 CAN 接口预留，当前不编译不启动。

## 职责

实车链路下，此节点是 **VCU（整车控制器）与 ROS 系统之间的双向网关**：

**VCU → ROS（解析 CAN 报文）**
- 任务信息：ASR 通过 AMI（任务指示器）选择的任务，映射为任务模式
- 启动命令：ASR 通过 RES（遥控急停系统）给出的 "Go" 信号
- 急停信号：安全回路断开 / RES 急停触发
- 车检触发：进入车检流程
- 车速：整车车速反馈，发布到 `/localization/velocity`

**ROS → VCU（编码 CAN 报文）**
- 控制指令：目标速度与转向角，下发至驱动电机 / 转向执行器
- 任务状态：无人驾驶系统状态（AS OFF / READY / DRIVING / EMERGENCY / FINISHED）
- 车检结果：车检是否通过

## 规则依据

依据《2022 中国大学生无人驾驶方程式大赛规则》第四章：

- **2.4 / 2.6**：AS 状态机仅允许 5 个状态——AS OFF / AS READY / AS DRIVING / AS EMERGENCY / AS FINISHED，禁止其他状态或转换路径；ASSI 按此状态指示。
- **2.7.1**：AS 至少包含 6 项任务：直线加速、八字环绕、高速循迹、EBS 测试、车检测试、操控性测试（手动）。
- **2.7.3**：车检任务定义：慢速旋转驱动系统并以正弦波驱动转向系统，25~30 s 后任务完成并切换至 "AS Finish"。
- **第五章 11.2.2**：赛车处于 AS Ready 后，由 ASR 通过 RES 给出 "Go" 信号，赛车进入 AS Driving。

## 状态映射

规则 AS 状态 ↔ `MissionState.state`：

| 规则状态 | MissionState | 说明 |
|---------|--------------|------|
| AS OFF | `IDLE` | 系统初始化，等待激活 |
| AS READY | `READY` / `INSPECTION` | ASMS 闭合、EBS 装备，等待 Go；车检在此阶段进行 |
| AS DRIVING | `EXPLORE` / `MAPPING_DONE` / `RACE` | 收到 Go 后运行 |
| AS EMERGENCY | `EMERGENCY` | 安全回路断开 / RES 急停 |
| AS FINISHED | `FINISH` | 任务完成 |

## 接口约定

### 订阅 Topic（ROS → VCU，转发到 CAN 总线）
- `/control/command` (autoware_msgs/msg/Command) - 控制指令（目标速度 + 转向角），周期下发驱动/转向执行器
- `/system/mission_state` (wuta_msgs/msg/MissionState) - 任务状态，周期发送
- `/system/inspection_result` (std_msgs/msg/String) - 车检结果（通过/失败），车检完成后发送

### 发布 Topic（CAN 总线解析 → ROS）
- `/system/mission_mode_cmd` (std_msgs/String) - 任务模式（trackdrive/skidpad/acceleration/inspection）
- `/system/start_command` (std_msgs/Bool) - 出发命令（RES Go 信号）
- `/system/emergency` (std_msgs/Bool) - 急停命令（安全回路 / RES 急停）
- `/system/inspection_trigger` (std_msgs/Bool) - 车检触发
- `/localization/velocity` (geometry_msgs/TwistStamped) - 速度反馈

## 注意事项

- 急停信号优先级最高：收到 VCU 急停必须立即发布 `/system/emergency=true`，不得被其他信号延迟。
- Go 信号与任务选择来自 RES / AMI 等 VCU 输入，`start_command` 仅在 AS Ready 状态下有效。
- `Command.dv_state` 需映射为 VCU 使能/急停状态（控制器约定：`4`=正常，`6`=急停），急停时速度清零并置无效。
- 报文 ID / 信号位 / 字节序 / 周期须严格按照 VCU 协议文档（dbc 文件）配置。
- 所有 AS 信号要求为 SCS（规则第四章 2.1），CAN 通信需做超时与失效检测，超时按急停处理。

## 实车对接步骤

1. 添加 `package.xml` 和 `CMakeLists.txt`
2. 实现 `can_interface.cpp` 中的 CAN 收发逻辑
3. 根据 VCU 协议文档（dbc 文件）配置 CAN 报文格式
4. 在 launch 文件中启动此节点，替换仿真链路的 `can_simulator`
5. 在 `docs/ROS_INTERFACE.md` 中同步 `/system/*` 与 `/localization/velocity` 的实车发布者说明
