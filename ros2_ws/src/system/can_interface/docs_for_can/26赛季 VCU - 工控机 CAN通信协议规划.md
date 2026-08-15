# 26赛季 VCU \- 工控机 CAN通信协议规划

# VCU状态机流程

![image\.png](图片和附件/image%201.png)



# **接口约定**



### **工控机→ VCU，转发到 CAN 总线**

|无人工控机发至VCU的报文信息|||||
|---|---|---|---|---|
|**CAN报文ID**|0x210||||
|**报文长度DLC**|8 Bytes||||
|信号|含义|大小范围|位置|备注|
|Signal1|纵向控制|10\~65525<br>10\~32767：越靠近0制动力越大<br>32767\~65525：越大驱动力越大|Byte1\-Byte2|<br>|
|Signal2|横向控制|10\~65525<br>以32767为中心<br>10方向为左<br>65525方向为右|Byte3\-Byte4||
|Signal3|工控机上线|1：上线<br>0：未上线|Byte5|代表无人这边传感器自检没有问题|
|Signal4|无人任务已完成|1：无人任务已完成<br>0：无人任务未完成|Byte6|所有项目FINISH后发送|
|Signal5|空报文|空|Byte7\-Byte8||

![img\_v3\_0214f\_a289dee6\-83cb\-4b38\-9b09\-be842e9463dg\.jpg](图片和附件/img_v3_0214f_a289dee6-83cb-4b38-9b09-be842e9463dg.jpg)



---



#### **`/control/command`**** — 控制指令**



\- **类型**：`autoware_msgs/msg/Command`

\- **发布方**：\`controller\` 节点（周期 10 Hz）

\- **方向**：ROS → VCU，编码为 CAN 报文下发驱动电机 / 转向执行器



字段结构：

```Plain Text
std_msgs/Header header    # 时间戳与 frame_id（通常为 "base_link"）
float64 speed             # 目标速度（m/s）
float64 angle             # 目标转向角（rad，正值左转）
int32 dv_state            # 驱动状态码：4 = 正常，6 = 急停
```

字段说明：

|字段|类型|说明|
|---|---|---|
|`header.stamp`|`builtin_interfaces/Time`|指令生成时间|
|`speed`|`float64`|目标纵向速度，单位 m/s，需按 VCU 协议换算为 rpm 或档位请求|
|`angle`|`float64`|目标前轮转角，单位 rad，需按 VCU 协议换算为转向执行器指令|
|`dv_state`|`int32`|驱动使能状态：`4`=正常（使能）、`6`=急停（失能，速度必须清零并置无效）|



#### **`/system/mission_state`**** — 任务状态 ****不管这个**

\- **类型**：`wuta_msgs/msg/MissionState`

\- **发布方**：\`mission\_manager\` 节点

\- **方向**：ROS → VCU，编码为 CAN 报文上报无人驾驶系统状态（对应规则 AS 状态机）



字段结构：

```Plain Text
std_msgs/Header header    # 时间戳与 frame_id
uint8 state               # AS 状态（见下表）
uint8 mission_mode        # 任务模式：0=trackdrive，1=skidpad，2=acceleration
uint8 localization_mode   # 定位模式：0=KISS-ICP，1=NDT
string description        # 人类可读的状态描述（调试用，不上 CAN）
```



`state` 枚举（`MissionState.state` ↔ 规则 AS 状态映射见上方"状态映射"表）：

|值|常量|规则状态|说明|
|---|---|---|---|
|0|`IDLE`|AS OFF|系统初始化，等待激活|
|1|`READY`|AS READY|传感器就绪，等待 Go 信号|
|2|`INSPECTION`|AS READY（车检）|车检流程：慢速转驱动 \+ 正弦波转转向|
|3|`EXPLORE`|AS DRIVING|第一圈建图（KISS\-ICP \+ 锥桶地图）|
|4|`MAPPING_DONE`|AS DRIVING|建图完成，切换竞速模式|
|5|`RACE`|AS DRIVING|高速循迹（NDT 匹配）|
|6|`FINISH`|AS FINISHED|任务完成|
|7|`EMERGENCY`|AS EMERGENCY|急停 / 故障|



`mission_mode` 枚举：

|值|常量|对应赛道|
|---|---|---|
|0|`MISSION_TRACKDRIVE`|高速循迹|
|1|`MISSION_SKIDPAD`|八字环绕|
|2|`MISSION_ACCELERATION`|直线加速|



`localization_mode` 枚举：

|值|常量|说明|
|---|---|---|
|0|`LOC_KISS_ICP`|KISS\-ICP 点云定位（建图阶段）|
|1|`LOC_NDT`|NDT 地图匹配（竞速阶段）|



#### **`/system/devices_inspection`****— 开机传感器自检结果**

\- **类型**：\`wuta\_msgs/msg/DevicesInspection\`

\- **发布方**：\`mission\_manager\` 节点（开机自检发现设备故障时发布一次）

\- **订阅方**：\`can\_interface\` 节点

\- **方向**：ROS → VCU，编码为 CAN 报文上报设备自检失败（Signal3=0，暂未实现）



字段结构：

```Plain Text
bool ok              # 全部通过
string[] failures    # 失败设备名列表，如 ["lidar"] / ["lidar","imu"]
```



### **VCU → 工控机 ，CAN 总线解析**

|无人赛项任务AMI信号|||
|---|---|---|
|**CAN报文ID**|0x501||
|**报文长度DLC**|8 Byte||
|Byte1 <br>|值|含义|
||1|操控性测试|
||2|直线加速测试|
||3|高速循迹测试|
||4|八字绕环测试|
||5|EBS测试|
||6|车检测试|

![image\.png](图片和附件/image.png)



#### **`/system/mission_mode_cmd`**** — 任务模式**



\- **类型**：`std_msgs/msg/String` 

\- **订阅方**：\`mission\_manager\`节点

\- **方向**：VCU（AMI 任务指示器）→ ROS，ASR 选择的赛事任务



字段结构：

```Plain Text
string data    # 任务模式标识
```

`data` 取值约定：

|取值|对应任务|
|---|---|
|`trackdrive`|高速循迹|
|`skidpad`|八字环绕|
|`acceleration`|直线加速|
|`inspection`|车检测试|



#### **`/system/start_command`**** — 出发命令**

GO与急停，从VCU读trigger，切状态。



\- **类型**：\`std\_msgs/msg/Bool\`

\- **订阅方**：\`mission\_manager\` 节点

\- **方向**：VCU（RES 遥控急停系统）→ ROS，Go 信号

\- **触发条件**：仅在 AS Ready 状态下有效（规则第五章 11\.2\.2）



字段结构：



```Plain Text
bool data    # true = 收到 "Go" 出发信号；false = 无出发信号
```



#### **`/system/emergency`**** — 急停命令**

急停只需要监听切状态，停发控制命令，不需要做急停。



\- **类型**：\`std\_msgs/msg/Bool\`

\- **订阅方**：\`mission\_manager\` 节点

\- **方向**：VCU（安全回路断开 / RES 急停）→ ROS

\- **优先级**：最高，收到 \`true\` 必须立即发布，不得被其他信号延迟



字段结构：

```Plain Text
bool data    # true = 急停触发（安全回路断开 / RES 急停）；false = 正常
```



#### **`/system/inspection_trigger`**** — 车检触发**



\- **类型**：\`std\_msgs/msg/Bool\`

\- **订阅方**：\`mission\_manager\` 节点

\- **方向**：VCU → ROS，进入车检流程的触发信号



字段结构：

```Plain Text
bool data    # true = 触发进入车检流程；false = 未触发
```



#### **`/localization/velocity`**** — 速度**

目前解决办法：从惯导读取速度



\- **类型**：\`geometry\_msgs/msg/TwistStamped\`

\- **订阅方**：\`localization\_manager\` / \`controller\` 节点

\- **方向**：VCU（整车车速读取）→ ROS，发布到定位/控制链路



字段结构：

```Plain Text
std_msgs/Header header          # 时间戳与 frame_id（通常为 "base_link"）
geometry_msgs/Twist twist       # 线速度 + 角速度
  geometry_msgs/Vector3 linear  # linear.x = 纵向车速（m/s）；y / z = 0
  geometry_msgs/Vector3 angular # 角速度（rad/s），横摆角速度写入 angular.z
```



字段说明：

|字段|类型|说明|
|---|---|---|
|`header.stamp`|`builtin_interfaces/Time`|CAN 报文接收时刻|
|`twist.linear.x`|`float64`|整车纵向速度（m/s），由车速信号换算|
|`twist.angular.z`|`float64`|横摆角速度（rad/s），如协议提供则填充，否则为 0|



## 

## 

