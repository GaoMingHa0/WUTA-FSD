# WUTA-FSD

武汉理工大学无人驾驶方程式赛车自动驾驶算法栈。
由BITFSD https://github.com/bitfsd/fsd_algorithm.git 二次开发而来。

---

## 硬件平台

| 设备 | 型号 |
|------|------|
| 工控机 | NUVO-7160GC（配置待定） |
| 激光雷达 | RS-LiDAR-M1 |
| 组合导航 | 华测 CG-410（GNSS + IMU） |
| 相机 | 待定（接口已预留） |

---

## 系统架构

```
传感器层
  RS-LiDAR-M1 ──→ lidar_detection  ──→ ConeArray
  相机(预留) ──→ camera_detection ──→ ConeArray  ──→ detection_fusion
  华测CG-410 ──→ /chcnav/odometry, /chcnav/velocity ──→ 定位 / 控制

定位层（双模式）
  EXPLORE: KISS-ICP + EKF(CG-410) ──┐
  RACE:    NDT 地图匹配            ──┴──→ /localization/pose

建图层
  ConeArray + pose ──→ cone_map_builder ──→ ConeMap（loop closure检测）

规划层
  ConeMap ──→ boundary_detector(局部路径/冻结全局中心线) ──→ path_generator ──→ Lane
  三模式：trackdrive / skidpad / acceleration

控制层
  Lane + pose ──→ controller(Pure Pursuit) ──→ Command → VCU

系统管理
  mission_manager：唯一状态机发布者，IDLE→READY→EXPLORE→…→FINISH
```

---

## 目录结构

```
WUTA-FSD/
├── REFACTOR.md              # 详细重构计划和进度
├── ros2_ws/
│   └── src/
│       ├── common/
│       │   ├── wuta_msgs/           # 自定义消息定义
│       │   └── wuta_tools/          # 工具库
│       ├── perception/
│       │   ├── lidar_detection/     # LiDAR锥桶检测（PCL/DL双后端）
│       │   ├── camera_detection/    # 相机检测（预留）
│       │   └── detection_fusion/    # 多源融合（预留）
│       ├── localization/
│       │   ├── kiss-icp/            # [submodule] KISS-ICP
│       │   ├── robot_localization/  # [submodule] EKF/UKF融合
│       │   ├── kiss_icp_wrapper/    # RS-LiDAR-M1 参数 + KISS里程计净化器（INS交叉校验）
│       │   ├── localization_manager/# 双模式切换，统一输出/localization/pose
│       │   └── ndt_localization/    # NDT地图匹配 + 地图保存
│       ├── mapping/
│       │   └── cone_map_builder/    # 锥桶地图构建，loop closure检测
│       ├── planning/
│       │   ├── boundary_detector/   # Delaunay三角剖分中心线提取
│       │   └── path_generator/      # 三模式路径生成
│       ├── control/
│       │   └── controller/          # Pure Pursuit横纵向控制
│       └── system/
│           ├── can_interface/       # CAN 链路节点（0x210/0x501 心跳收发，SocketCAN）
│           └── mission_manager/     # 任务状态机
└── ros/                     # 原ROS1代码（见master分支）
```
