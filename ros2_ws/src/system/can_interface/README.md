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

所有消息均使用 **rclcpp 默认 QoS**（可靠 / 易失）或与发布方一致的 QoS，收发双方须保持一致，否则数据无法到达。CAN 周期（报文 ID / 信号位 / 字节序 / 周期）须严格按照 VCU 协议文档（dbc 文件）配置。

### 订阅 Topic（ROS → VCU，转发到 CAN 总线）

#### `/control/command` — 控制指令

- **类型**：`autoware_msgs/msg/Command`
- **发布方**：`controller` 节点（周期 10 Hz）
- **方向**：ROS → VCU，编码为 CAN 报文下发驱动电机 / 转向执行器

字段结构：

```text
std_msgs/Header header    # 时间戳与 frame_id（通常为 "base_link"）
float64 speed             # 目标速度（m/s，正值前进，负值后退）
float64 angle             # 目标转向角（rad，正值左转）
int32 dv_state            # 驱动状态码：4 = 正常，6 = 急停
```

字段说明：

| 字段 | 类型 | 说明 |
|------|------|------|
| `header.stamp` | `builtin_interfaces/Time` | 指令生成时间 |
| `speed` | `float64` | 目标纵向速度，单位 m/s，需按 VCU 协议换算为 rpm 或档位请求 |
| `angle` | `float64` | 目标前轮转角，单位 rad，需按 VCU 协议换算为转向执行器指令 |
| `dv_state` | `int32` | 驱动使能状态：`4`=正常（使能）、`6`=急停（失能，速度必须清零并置无效） |

注意：急停（`dv_state=6`）时本节点必须将速度置零后下发，禁止转发非零速度。

#### `/system/mission_state` — 任务状态

- **类型**：`wuta_msgs/msg/MissionState`
- **发布方**：`mission_manager` 节点
- **方向**：ROS → VCU，编码为 CAN 报文上报无人驾驶系统状态（对应规则 AS 状态机）

字段结构：

```text
std_msgs/Header header    # 时间戳与 frame_id
uint8 state               # AS 状态（见下表）
uint8 mission_mode        # 任务模式：0=trackdrive，1=skidpad，2=acceleration
uint8 localization_mode   # 定位模式：0=KISS-ICP，1=NDT
string description        # 人类可读的状态描述（调试用，不上 CAN）
```

`state` 枚举（`MissionState.state` ↔ 规则 AS 状态映射见上方"状态映射"表）：

| 值 | 常量 | 规则状态 | 说明 |
|----|------|---------|------|
| 0 | `IDLE` | AS OFF | 系统初始化，等待激活 |
| 1 | `READY` | AS READY | 传感器就绪，等待 Go 信号 |
| 2 | `INSPECTION` | AS READY（车检） | 车检流程：慢速转驱动 + 正弦波转转向 |
| 3 | `EXPLORE` | AS DRIVING | 第一圈建图（KISS-ICP + 锥桶地图） |
| 4 | `MAPPING_DONE` | AS DRIVING | 建图完成，切换竞速模式 |
| 5 | `RACE` | AS DRIVING | 高速循迹（NDT 匹配） |
| 6 | `FINISH` | AS FINISHED | 任务完成 |
| 7 | `EMERGENCY` | AS EMERGENCY | 急停 / 故障 |

`mission_mode` 枚举：

| 值 | 常量 | 对应赛道 |
|----|------|---------|
| 0 | `MISSION_TRACKDRIVE` | 高速循迹 |
| 1 | `MISSION_SKIDPAD` | 八字环绕 |
| 2 | `MISSION_ACCELERATION` | 直线加速 |

`localization_mode` 枚举：

| 值 | 常量 | 说明 |
|----|------|------|
| 0 | `LOC_KISS_ICP` | KISS-ICP 点云定位（建图阶段） |
| 1 | `LOC_NDT` | NDT 地图匹配（竞速阶段） |

#### `/system/inspection_result` — 车检结果

- **类型**：`std_msgs/msg/String`
- **发布方**：`mission_manager` 节点（车检流程完成后发送一次）
- **方向**：ROS → VCU，编码为 CAN 报文上报车检通过/失败

字段结构：

```text
string data    # 车检结果文本
```

`data` 取值约定（假设，目前待定）：

| 取值 | 说明 |
|------|------|
| `pass`（或 `"passed"`） | 车检通过，可进入 AS Ready |
| `fail`（或 `"failed"`） | 车检失败，禁止进入 AS Ready |

> 建议约定固定字符串（如 `pass` / `fail`），并体现在 VCU 协议文档中；本节点按协议映射为 CAN 信号位。

### 发布 Topic（CAN 总线解析 → ROS）

#### `/system/mission_mode_cmd` — 任务模式

- **类型**：`std_msgs/msg/String`
- **订阅方**：`mission_manager` 节点
- **方向**：VCU（AMI 任务指示器）→ ROS，ASR 选择的赛事任务

字段结构：

```text
string data    # 任务模式标识
```

`data` 取值约定（假设，目前待定）：

| 取值 | 对应任务 |
|------|---------|
| `trackdrive` | 高速循迹 |
| `skidpad` | 八字环绕 |
| `acceleration` | 直线加速 |
| `inspection` | 车检测试 |

#### `/system/start_command` — 出发命令

- **类型**：`std_msgs/msg/Bool`
- **订阅方**：`mission_manager` 节点
- **方向**：VCU（RES 遥控急停系统）→ ROS，Go 信号
- **触发条件**：仅在 AS Ready 状态下有效（规则第五章 11.2.2）

字段结构：

```text
bool data    # true = 收到 "Go" 出发信号；false = 无出发信号
```

#### `/system/emergency` — 急停命令

- **类型**：`std_msgs/msg/Bool`
- **订阅方**：`mission_manager` 节点
- **方向**：VCU（安全回路断开 / RES 急停）→ ROS
- **优先级**：最高，收到 `true` 必须立即发布，不得被其他信号延迟

字段结构：

```text
bool data    # true = 急停触发（安全回路断开 / RES 急停）；false = 正常
```

#### `/system/inspection_trigger` — 车检触发

- **类型**：`std_msgs/msg/Bool`
- **订阅方**：`mission_manager` 节点
- **方向**：VCU → ROS，进入车检流程的触发信号

字段结构：

```text
bool data    # true = 触发进入车检流程；false = 未触发
```

#### `/localization/velocity` — 速度反馈

- **类型**：`geometry_msgs/msg/TwistStamped`
- **订阅方**：`localization_manager` / `controller` 节点
- **方向**：VCU（整车车速反馈）→ ROS，发布到定位/控制链路

字段结构：

```text
std_msgs/Header header          # 时间戳与 frame_id（通常为 "base_link"）
geometry_msgs/Twist twist       # 线速度 + 角速度
  geometry_msgs/Vector3 linear  # linear.x = 纵向车速（m/s）；y / z = 0
  geometry_msgs/Vector3 angular # 角速度（rad/s），横摆角速度写入 angular.z
```

字段说明：

| 字段 | 类型 | 说明 |
|------|------|------|
| `header.stamp` | `builtin_interfaces/Time` | CAN 报文接收时刻 |
| `twist.linear.x` | `float64` | 整车纵向速度（m/s），由车速信号换算 |
| `twist.angular.z` | `float64` | 横摆角速度（rad/s），如协议提供则填充，否则为 0 |

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
