# CAN Interface (预留)

此目录为实车 CAN 接口预留，当前不编译不启动。

## 职责

实车链路下，此节点负责：
- 从 CAN 总线解析 VCU 指令，发布到 `/system/*` topic
- 发布 `/localization/velocity`（从 CAN 获取车速）
- 订阅 `/system/mission_state` 和 `/system/inspection_result`，转发到 CAN 总线

## 接口约定

### 订阅 Topic
- `/system/mission_state` (wuta_msgs/msg/MissionState) - 任务状态
- `/system/inspection_result` (std_msgs/msg/String) - 车检结果

### 发布 Topic
- `/system/mission_mode_cmd` (std_msgs/String) - 模式选择
- `/system/start_command` (std_msgs/Bool) - 出发命令
- `/system/emergency` (std_msgs/Bool) - 急停命令
- `/system/inspection_trigger` (std_msgs/Bool) - 车检触发
- `/localization/velocity` (geometry_msgs/TwistStamped) - 速度反馈

## 实车对接步骤

1. 添加 `package.xml` 和 `CMakeLists.txt`
2. 实现 `can_interface.cpp` 中的 CAN 收发逻辑
3. 根据 VCU 协议文档配置 CAN 报文格式
4. 在 launch 文件中启动此节点，替换仿真链路的 `can_simulator`