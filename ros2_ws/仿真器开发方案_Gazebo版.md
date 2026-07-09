# WUTA-FSD 基于 Gazebo 的仿真器开发方案

> 日期: 2026-06-29

---

## 前提条件

| 要求 | 说明 |
|------|------|
| **GPU** | Gazebo 物理渲染 + LiDAR 光线追踪必须 |
| **ROS 2 Humble** | 已满足 |
| **Gazebo Fortress (Ignition)** | ROS 2 Humble 官方推荐版本 |
| **gazebo_ros_pkgs** | ROS 2 ↔ Gazebo 桥接 |

> 工控机 (Nuvo-7000, 无独显) 跑不了此方案，仅适用于有 NVIDIA 显卡的开发机。

---

## 整体架构

```
┌── Gazebo Simulator ──────────────────────────────────────────────┐
│                                                                    │
│  World SDF (.sdf)                                                  │
│  ├── Ground plane + lighting                                      │
│  ├── Cone models (blue/yellow/orange, 锥筒 3D 模型)               │
│  └── Track layout (锥筒阵列围成赛道)                               │
│                                                                    │
│  Vehicle URDF/SDF (.urdf/.sdf)                                     │
│  ├── chassis (车身刚体, 质量/惯量)                                 │
│  ├── 4 wheels (悬挂/摩擦参数)                                      │
│  ├── steering joints (前轮转向)                                    │
│  ├── LiDAR sensor  → /hesai/pandar                                │
│  ├── Camera sensor  → /camera/image_raw (为 camera_detection 铺路)│
│  ├── IMU sensor     → /imu/data                                   │
│  └── Ground Truth   → /sim/ground_truth                            │
│                                                                    │
│  Controller Plugin                                                 │
│  ├── Sub /control/command (speed, steering_angle)                  │
│  └── Actuate steering/throttle joints                              │
│                                                                    │
│  INS Bridge Node (外部, ROS 2)                                     │
│  ├── Sub /sim/ground_truth + /imu/data                             │
│  └── Pub /cg410/odometry (模拟 INS 融合输出)                       │
└────────────────────────────────────────────────────────────────────┘
         │              │              │
         ▼              ▼              ▼
    /hesai/pandar  /camera/image  /cg410/odometry
         │              │              │
         ▼              ▼              ▼
┌── 现有 FSD 管线 (不修改) ─────────────────────────────────────────┐
│                                                                     │
│  lidar_detection ──→ cone_map_builder ──→ boundary_detector        │
│  camera_detection (开发中) ──→ detection_fusion (开发中)            │
│                                            │                        │
│                              boundary_detector ──→ path_generator  │
│                                                       │            │
│                                          path_generator ──→ controller
│                                                                     │
│  kiss_icp → ekf → localization_manager → /localization/pose        │
│                                               │                    │
│                                               ▼                    │
│                            /localization/pose → controller          │
│                                                                     │
│  controller ──→ /control/command ──→ 回到 Gazebo (闭环)            │
└─────────────────────────────────────────────────────────────────────┘
```

闭环：Gazebo 发布传感器数据 → FSD 管线处理 → /control/command → Gazebo Controller Plugin 驱动车辆 → 车辆移动、传感器数据更新 → ...

---

## 文件结构

```
src/simulation/
├── wuta_gazebo/
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── launch/
│   │   ├── trackdrive.launch.py       # 赛道追逐仿真
│   │   ├── skidpad.launch.py          # 八字绕桩仿真
│   │   └── acceleration.launch.py     # 直线加速仿真
│   ├── config/
│   │   └── simulator.yaml             # 仿真全局参数
│   ├── urdf/
│   │   ├── wuta_car.urdf.xacro        # 车辆模型（主文件）
│   │   ├── chassis.xacro              # 车身
│   │   ├── wheels.xacro               # 车轮 + 悬挂
│   │   ├── sensors.xacro              # LiDAR + Camera + IMU
│   │   └── materials.xacro            # 颜色/材质定义
│   ├── worlds/
│   │   ├── trackdrive.sdf             # 赛道追逐世界
│   │   ├── skidpad.sdf                # 八字绕桩世界
│   │   └── acceleration.sdf           # 直线加速世界
│   ├── models/
│   │   ├── blue_cone/                 # 蓝色锥筒模型
│   │   │   ├── model.sdf
│   │   │   └── model.config
│   │   ├── yellow_cone/               # 黄色锥筒模型
│   │   │   ├── model.sdf
│   │   │   └── model.config
│   │   └── orange_cone/               # 橙色锥筒模型
│   │       ├── model.sdf
│   │       └── model.config
│   └── src/
│       └── ins_bridge.cpp             # INS 桥接节点（模拟 CG-410 输出）
```

---

## 各组件详细设计

### 1. 锥筒模型

FSG 标准锥筒：高 505mm，底面 285mm×285mm。

```xml
<!-- blue_cone/model.sdf -->
<model name="blue_cone">
  <static>true</static>
  <link name="body">
    <collision name="collision">
      <geometry>
        <box><size>0.285 0.285 0.505</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>0.285 0.285 0.505</size></box>
      </geometry>
      <material>
        <ambient>0 0 1 1</ambient>    <!-- 蓝色 -->
        <diffuse>0 0.4 1 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

黄锥和橙锥同理，只改颜色。更精细的可以替换为 mesh 文件（.dae /.stl），如果团队有锥筒 3D 扫描模型可以直接用。

### 2. 赛道世界

赛道由锥筒模型实例排列组成：

```xml
<!-- worlds/trackdrive.sdf -->
<world name="trackdrive">
  <!-- 场景基础 -->
  <include><uri>model://sun</uri></include>
  <include><uri>model://ground_plane</uri></include>

  <!-- 左侧锥筒（蓝色） -->
  <include><uri>model://blue_cone</uri><name>cone_L01</name>
    <pose>1.5 5.0 0 0 0 0</pose></include>
  <include><uri>model://blue_cone</uri><name>cone_L02</name>
    <pose>2.0 5.0 0 0 0 0</pose></include>
  <!-- ...更多... -->

  <!-- 右侧锥筒（黄色） -->
  <include><uri>model://yellow_cone</uri><name>cone_R01</name>
    <pose>1.5 -5.0 0 0 0 0</pose></include>
  <!-- ...更多... -->

  <!-- 起点锥筒（橙色） -->
  <include><uri>model://orange_cone</uri><name>cone_start_01</name>
    <pose>0.0 0.0 0 0 0 0</pose></include>
</world>
```

赛道布局按 FSG 规则：
- Trackdrive：两列锥筒间距 3-5m，混合直道 + 不同半径弯道
- Skidpad：两个半径 9.125m 的圆，圆心距 18.25m
- Acceleration：75m 直线，两侧锥筒

后期可用 Python 脚本自动生成锥筒 pose 并写入 .sdf。

### 3. 车辆模型（URDF/Xacro）

车辆参数取自 controller 配置 (`controller.yaml`)：

| 参数 | 值 | 来源 |
|------|------|------|
| 轴距 | 1.53m | controller.yaml |
| 前轴到质心 | 1.0m | controller.yaml |
| 最大转向角 | 25° | controller.yaml |
| 车重 | ~230kg | 实车测量 |
| LiDAR 安装高度 | ~0.8m | 实车测量 |

```
wuta_car.urdf.xacro 结构:

base_link (质心)
├── chassis (车身刚体)
│   ├── visual: 简化车身 box
│   └── collision: 简化车身 box
├── front_left_wheel
│   └── joint: revolute (steering + rotation)
├── front_right_wheel
│   └── joint: revolute (steering + rotation)
├── rear_left_wheel
│   └── joint: revolute (rotation only)
├── rear_right_wheel
│   └── joint: revolute (rotation only)
├── hesai_lidar_link
│   └── joint: fixed (安装位姿)
│       └── sensor: gpu_lidar (128 线)
├── camera_link
│   └── joint: fixed (安装位姿)
│       └── sensor: camera (color + depth)
└── imu_link
    └── joint: fixed
        └── sensor: imu
```

### 4. 传感器配置

#### LiDAR

```xml
<sensor name="hesai_lidar" type="gpu_lidar">
  <topic>/hesai/pandar</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>         <!-- 128线 × 约14点/线 = 1800点/scan -->
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>    <!-- 360° -->
        <max_angle>3.14</max_angle>
      </horizontal>
      <vertical>
        <samples>128</samples>          <!-- 128 线 -->
        <min_angle>-0.4</min_angle>     <!-- -23° -->
        <max_angle>0.26</max_angle>     <!-- +15° (Hesai128 FOV) -->
      </vertical>
    </scan>
    <range>
      <min>1.5</min>                    <!-- 滤车体 -->
      <max>50.0</max>                   <!-- FSD 赛道范围 -->
    </range>
  </ray>
</sensor>
```

#### Camera

```xml
<sensor name="stereo_camera" type="camera">
  <topic>/camera/image_raw</topic>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
    <image>
      <width>1280</width>
      <height>720</height>
    </image>
  </camera>
</sensor>
```

#### IMU

```xml
<sensor name="imu" type="imu">
  <topic>/imu/data</topic>
  <update_rate>100</update_rate>
</sensor>
```

### 5. Controller Plugin（Gazebo 侧）

Gazebo 自带的 `DiffDrive` 或 `SkidSteer` 不适用于 Ackermann 转向。使用 `libgazebo_ros_ackermann_drive.so` 插件：

```xml
<plugin name="ackermann_drive" filename="libgazebo_ros_ackermann_drive.so">
  <ros>
    <namespace>/</namespace>
    <remapping>cmd_vel:=/control/command</remapping>
  </ros>
  <wheelbase>1.53</wheelbase>
  <max_steer>0.436</max_steer>          <!-- 25° in rad -->
  <wheel_separation>1.2</wheel_separation>
  <wheel_radius>0.23</wheel_radius>
  <max_speed>20.0</max_speed>
</plugin>
```

需要适配：FSD 的 `/control/command` 是自定义的 `autoware_msgs::Command`（speed/angle/dv_state），而 ackermann_drive 插件期望的是 `AckermannDrive` 消息。需要写一个薄转换节点：

```python
# 转换节点：/control/command (Command) → /gazebo/cmd (AckermannDrive)
class CommandToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_converter')
        self.sub = create_subscription(Command, '/control/command', self.callback, 10)
        self.pub = create_publisher(AckermannDrive, '/gazebo/cmd', 10)

    def callback(self, cmd):
        ack = AckermannDrive()
        ack.speed = cmd.speed
        ack.steering_angle = math.radians(cmd.angle)
        self.pub.publish(ack)
```

### 6. INS 桥接节点

Gazebo 发布的是分离的 `/sim/ground_truth` + `/imu/data`，CG-410 INS 发布的是融合后的 `/cg410/odometry`。需要一个节点做组合：

```cpp
// ins_bridge.cpp
// Sub: /sim/ground_truth (Odometry) + /imu/data (Imu)
// Pub: /cg410/odometry (Odometry)
//
// 逻辑：用 ground truth 做位置，IMU 做姿态，加噪音模拟 GNSS RTK + IMU 融合输出
// 加 5cm 位置噪声 + 0.5° 角度噪声 + 20ms 随机延迟模拟真实 INS
```

---

## launch 文件设计

每个赛道一个 launch 文件。以 trackdrive 为例：

```python
# launch/trackdrive.launch.py
def generate_launch_description():
    return LaunchDescription([
        # 1. 启动 Gazebo 世界
        ExecuteProcess(cmd=['ign', 'gazebo', 'worlds/trackdrive.sdf']),

        # 2. 生成车辆到 Gazebo
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'wuta_car', '-file', 'urdf/wuta_car.urdf',
                        '-x', '0', '-y', '0', '-z', '0.3']),

        # 3. INS 桥接
        Node(package='wuta_gazebo', executable='ins_bridge'),

        # 4. 命令转换
        Node(package='wuta_gazebo', executable='cmd_converter'),

        # 5. 启动 FSD 管线（复用已有 launch）
        IncludeLaunchDescription('localization_manager/launch/localization.launch.py'),
        Node(package='lidar_detection', executable='lidar_detection_node'),
        Node(package='cone_map_builder', executable='cone_map_builder_node'),
        Node(package='boundary_detector', executable='boundary_detector_node'),
        Node(package='path_generator', executable='path_generator_node'),
        Node(package='controller', executable='controller_node'),
    ])
```

---

## 与原方案（自定义轻量仿真）对比

| | 自定义轻量方案 | Gazebo 方案 |
|------|------|------|
| 开发工作量 | 低（写 Python 脚本） | 中（写 URDF/SDF 模型 + 插件） |
| LiDAR 仿真 | 人工射线投射，简化 | GPU 光线追踪，逼真 |
| 相机仿真 | 不支持 | 原生支持，可调 FOV/分辨率/畸变 |
| 车辆模型 | 自行车运动学 | 完整刚体动力学（质量/惯量/摩擦/悬挂） |
| 碰撞模拟 | 无 | 碰撞检测 + 物理响应 |
| 硬件要求 | 无 GPU | **必须有 NVIDIA GPU** |
| 工控机可跑 | 是 | 否 |
| 适合场景 | 纯算法逻辑验证 | 传感器级仿真 + camera_detection 开发 |

---

## 开发计划

### 第一阶段：静态世界 + 简单车辆（3 天）

1. 安装 gazebo_ros_pkgs: `sudo apt install ros-humble-gazebo-ros-pkgs`
2. 创建锥筒 SDF 模型（3 个，简单 box 即可）
3. 创建 trackdrive.sdf 世界（手动排列 20 个锥筒）
4. 创建车辆 URDF（用 box 简化的车身 + 4 个 wheel link）
5. 接入 ackermann_drive 插件
6. 接入 LiDAR 传感器
7. 写 cmd_converter 转换节点
8. 验证：Gazebo 中的车能收到 /control/command 动起来

### 第二阶段：全链路闭环（2 天）

1. LiDAR 话题接通，验证 lidar_detection_node 能否检测到锥筒
2. 写 ins_bridge，提供 CG-410 模拟
3. 启动完整 FSD 管线，验证闭环
4. 调参（LiDAR 分辨率、噪声、路面摩擦）

### 第三阶段：精细车辆模型 + camera（2 天）

1. 用实车 CAD 数据替换简化 URDF
2. 接入 Camera 传感器，为 camera_detection 开发铺路
3. 生成全尺寸赛道（Trackdrive 完整一圈约 500m）
4. 加指标收集（单圈时间、路径偏差）

---

## 需要的硬件

| 项目 | 最低要求 |
|------|------|
| GPU | NVIDIA GTX 1060 或以上 |
| VRAM | 4GB+ |
| RAM | 16GB+ |
| OS | Ubuntu 22.04 |

> 如果只有工控机（Nuvo-7000 无独显），此方案无法实施，只能用自定义轻量方案。
