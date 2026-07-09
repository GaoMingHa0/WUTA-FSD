# WUTA-FSD ROS2 编译修复记录 — 环境迁移与 package.xml 修复

> 日期: 2026-07-09
> 环境: Ubuntu 22.04 (WSL2), ROS 2 Humble, CMake 4.3.4 (pip)
> 工作空间: `/home/g2909/WUTA1/WUTA-FSD/ros2_ws`
> 分支: `小登测试`

---

## 前言

本文档是 [编译修复记录.md](./编译修复记录.md) 和 [install路径修复记录.md](./install路径修复记录.md) 的**补充篇**。前两份记录描述了 2026-06-29 期间的修复，但经 2026-07-09 逐项核对源码后发现：

1. 原记录中部分修复**未实际写入** WUTA 工作空间源码（但已写入 WUTA1 工作空间）
2. 两个工作空间都缺少 **package.xml 依赖声明**，导致 `autoware_msgs` 头文件找不到
3. 原记录缺少**环境搭建**的完整步骤，换电脑/换用户后无法复现编译

本文档记录：(1) 如何区分 WUTA 和 WUTA1 两个副本；(2) 本次新修复的 package.xml 缺陷；(3) 从零开始搭建编译环境的完整流程。

---

## 一、两个工作空间副本的关系

| 对比项 | WUTA 副本 | WUTA1 副本（本文档基于此） |
|--------|----------|------------------------|
| 路径 | `/home/g2909/WUTA-FSD/ros2_ws` | `/home/g2909/WUTA1/WUTA-FSD/ros2_ws` |
| 分支 | `wuta0318` | `小登测试`（多 1 个 commit `4968e7d`） |
| 源码修复状态 | 4 项关键修复**未生效**（见下文 §2） | 4 项关键修复**已生效** |
| 能否直接编译 | ❌ 必然失败 | ✅ 修复 package.xml 后可通过 |

**结论：推荐使用 WUTA1 副本进行后续开发。** WUTA 副本的修复是半成品，其 `thirdparty/pathplanning` 库的修复尚未写入。

---

## 二、原记录中"已修复但 WUTA 未生效"的 4 项

以下修复在 WUTA1 中已生效，在 WUTA 中未生效。如果你使用 WUTA 副本，需要手动补上。

### 2.1 ndt_localization 缺少 visualization_msgs 链接

**原记录 §6 声称已修复，WUTA 实际未修。**

文件: `localization/ndt_localization/CMakeLists.txt`

```diff
 ament_target_dependencies(ndt_localization_node
-  rclcpp sensor_msgs geometry_msgs nav_msgs wuta_msgs pcl_conversions
+  rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs wuta_msgs pcl_conversions
 )
```

原因: `ndt_localization.hpp:8` 引用了 `<visualization_msgs/msg/marker_array.hpp>`，但 `ament_target_dependencies` 漏了该包，编译报 `marker_array.hpp: No such file or directory`。

### 2.2 boundary_detector HPL 头文件 include 路径缺失

**原记录 §7 声称已修复，WUTA 实际未修。**

文件: `planning/boundary_detector/CMakeLists.txt` 第 17 行

```diff
-target_include_directories(pathplanning PUBLIC thirdparty/pathplanning/include)
+target_include_directories(pathplanning PUBLIC thirdparty/pathplanning/include include)
```

原因: `GlobalVariables.cpp:1-2` 引用 `hpl/hpl_param.hpp` 和 `hpl/hpl_log.hpp`，这两个头文件位于 `include/hpl/` 目录。不加 `include` 路径，编译报 `hpl/hpl_param.hpp: No such file or directory`。

### 2.3 GlobalVariables 缺少成员变量和 getter

**原记录 §8.1 声称已修复，WUTA 实际未修。**

文件: `planning/boundary_detector/thirdparty/pathplanning/include/pathplanning/GlobalVariables.h`

```diff
 // 新增 getter 方法
+double GetRedundancy() { return Redundancy; }
+double GetStartPointRedundancy() { return StartPointRedundancy; }
+int GetPlugPointSize() { return PlugPointSize; }
+double GetDistanceToBoundary() { return DistanceToBoundary; }
+int GetIsUseFirstLapPoints() { return IsUseFirstLapPoints; }
+int GetIsUseFirstLapPointsPro() { return IsUseFirstLapPointsPro; }

 // 新增 private 成员
+double Redundancy;
+double StartPointRedundancy;
+int PlugPointSize;
+double DistanceToBoundary;
+int IsUseFirstLapPoints;
+int IsUseFirstLapPointsPro;
```

原因: `GlobalVariables.cpp` 构造函数中对这些成员调用了 `param_node.declParam()`，但头文件未声明，编译报 `Redundancy was not declared in this scope`。

### 2.4 Path.cpp 缺标准库头文件 + 三个头文件只有前向声明

**原记录 §8.2/§8.3 声称已修复，WUTA 实际未修。**

**Path.cpp** — 加标准库头文件:
```diff
+#include <cmath>
+#include <cassert>
 #include "pathplanning/structs/MidPoint.h"
```

**PathSearch.h** — 前向声明改为 #include:
```diff
 #include <iostream>
 #include <vector>
-class MidPoint;
-class Vect;
+#include "pathplanning/structs/Point2d.h"
+#include "pathplanning/structs/MidPoint.h"
+#include "pathplanning/structs/Vect.h"
+#include "pathplanning/structs/Triangle.h"
 class Evaluation;
```

**Path.h** — 前向声明改为 #include:
```diff
 #include <memory>
-class MidPoint;
+#include "pathplanning/structs/Point2d.h"
+#include "pathplanning/structs/MidPoint.h"
 class PathSearch;
```

**Tool.h** — 前向声明改为 #include:
```diff
 #include <vector>
+#include <memory>
-class Point2d;
-class MidPoint;
-class Vect;
-class Triangle;
+#include "pathplanning/structs/Point2d.h"
+#include "pathplanning/structs/MidPoint.h"
+#include "pathplanning/structs/Vect.h"
+#include "pathplanning/structs/Triangle.h"
```

原因: `Path.cpp` 使用 `cos()`/`sin()`（需 `<cmath>`）和 `assert()`（需 `<cassert>`）；三个头文件用 `class Xxx;` 前向声明后，`std::vector<std::shared_ptr<Xxx>>` 需要完整类型，编译报 `incomplete type`。

---

## 三、本次新发现的修复（2026-07-09）

### 🔴 关键修复：package.xml 缺少 autoware_msgs 依赖声明

**问题现象:**
```
boundary_detector_node.hpp:15:10: fatal error: autoware_msgs/msg/lane.hpp: No such file or directory
```

**根因:** ROS 2 的 colcon 构建系统根据每个包的 `package.xml` 决定其 `AMENT_PREFIX_PATH`。`boundary_detector` 和 `path_generator` 的 `CMakeLists.txt` 里写了 `find_package(autoware_msgs REQUIRED)`，但 `package.xml` 里**没有声明 `<depend>autoware_msgs</depend>`**，导致 colcon 不把 autoware_msgs 的 include 路径传给这两个包。

**验证方法:** 查看 `build/boundary_detector/CMakeFiles/boundary_detector_node.dir/flags.make` 中的 `CXX_INCLUDES`，会发现缺少 autoware_msgs 路径。

#### 修复 1: `planning/boundary_detector/package.xml`

```diff
 <depend>rclcpp</depend>
 <depend>wuta_msgs</depend>
+<depend>autoware_msgs</depend>
 <export>
```

#### 修复 2: `planning/path_generator/package.xml`

```diff
 <depend>rclcpp</depend>
 <depend>wuta_msgs</depend>
+<depend>autoware_msgs</depend>
 <export>
```

**注意:** `controller/package.xml` 已正确声明了 `<depend>autoware_msgs</depend>`，无需修改。

**修复后需要清除 cmake 缓存才能生效:**
```bash
rm -rf build/boundary_detector build/path_generator
colcon build --symlink-install --packages-select boundary_detector path_generator
```

---

## 四、换环境从零编译完整指南

以下步骤适用于：新装 Ubuntu 22.04、新建 WSL 实例、或切换到另一个用户。

### 4.1 系统级依赖（需要 sudo）

```bash
# 基础工具
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv git curl

# ROS 2 Humble 核心（如未安装）
sudo apt-get install -y ros-humble-desktop

# 项目所需的 ROS 2 系统包
sudo apt-get install -y \
  ros-humble-diagnostic-updater \
  ros-humble-geographic-msgs \
  ros-humble-visualization-msgs \
  ros-humble-pcl-conversions \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs

# 第三方 C++ 库
sudo apt-get install -y \
  libgeographic-dev \
  libeigen3-dev \
  libyaml-cpp-dev \
  libceres-dev \
  libsuitesparse-dev \
  libpcl-dev \
  robin-map-dev
```

### 4.2 CMake 升级（必须 3.24+）

> ⚠️ Ubuntu 22.04 官方源 cmake 仅 3.22.1。kiss_icp 通过 FetchContent 下载 Sophus 1.24.6，**要求 cmake 3.24+**。

```bash
# 安装/升级 cmake 到最新版（装到 ~/.local/bin/）
pip install --upgrade cmake

# 确认版本（应显示 4.x）
~/.local/bin/cmake --version

# 确保 ~/.local/bin 在 PATH 最前面（覆盖系统旧版 /usr/bin/cmake）
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# 最终验证
which cmake          # 应输出 /home/<用户>/.local/bin/cmake
cmake --version     # 应输出 4.x
```

### 4.3 ROS 2 环境初始化

```bash
# 每次新终端都需要 source（或写入 ~/.bashrc 自动生效）
source /opt/ros/humble/setup.bash

# 验证
echo $ROS_DISTRO     # 应输出 humble
```

### 4.4 获取源码

```bash
# 克隆项目（如 git 代理不通，参见 §4.5）
git clone <项目地址> WUTA1
cd WUTA1

# 初始化子模块（kiss-icp, robot_localization 等）
git submodule update --init --recursive
```

### 4.5 网络/代理注意事项

如果 `git submodule update` 报连接错误（`Failed to connect to 127.0.0.1:7890`），说明 git 配置了代理但代理未运行：

```bash
# 检查代理配置
git config --global http.proxy
git config --global https.proxy

# 方案 A：启动代理（如 Clash for Windows GUI）
# 方案 B：临时取消代理
git config --global --unset http.proxy
git config --global --unset https.proxy
```

### 4.6 编译

```bash
cd /home/<用户>/WUTA1/WUTA-FSD/ros2_ws

# 首次编译前确认：无陈旧 build/ 缓存
# 如果之前失败过，先清理：
rm -rf build/ install/ log/

# 全量编译
colcon build --symlink-install
```

**预期输出:**
```
Summary: 16 packages finished
```

16 个包清单:
```
autoware_msgs  controller  mission_manager  ndt_localization
wuta_msgs      path_generator  boundary_detector  cone_map_builder
lidar_detection  camera_detection  detection_fusion  kiss_icp
kiss_icp_wrapper  robot_localization  localization_manager  wuta_tools
```

### 4.7 验证编译结果

```bash
# 加载工作空间
source install/setup.bash

# 检查包是否可被识别
colcon list | wc -l    # 应输出 16

# 检查 autoware_msgs 消息是否可用
ros2 interface show autoware_msgs/msg/Lane
ros2 interface show autoware_msgs/msg/Waypoint
ros2 interface show autoware_msgs/msg/Command
```

---

## 五、常见编译错误速查

| 错误信息 | 原因 | 解决 |
|---------|------|------|
| `CMake 3.24 or higher is required` | cmake 版本过低 | `pip install --upgrade cmake` + PATH |
| `autoware_msgs/msg/lane.hpp: No such file` | package.xml 缺依赖 | 加 `<depend>autoware_msgs</depend>` + 清缓存 |
| `hpl/hpl_param.hpp: No such file` | boundary_detector CMakeLists 缺 include | 加 `include` 到 target_include_directories |
| `visualization_msgs/msg/marker_array.hpp: No such file` | ndt CMakeLists 缺依赖 | ament_target_dependencies 加 visualization_msgs |
| `cos/sin/assert was not declared` | Path.cpp 缺标准库 | 加 `<cmath>` + `<cassert>` |
| `incomplete type 'MidPoint'` | 头文件只有前向声明 | 替换为 #include 实际 struct 头文件 |
| `Redundancy was not declared` | GlobalVariables.h 缺成员 | 补充成员变量和 getter |
| `CMakeCache.txt directory is different` | 陈旧 build 缓存 | `rm -rf build/ install/ log/` |
| `Failed to connect to 127.0.0.1:7890` | git 代理未运行 | 启动代理或取消代理配置 |
| `which: no colcon` | 不在 WSL 环境内 | 通过 `wsl` 命令进入 Ubuntu |

---

## 六、遗留注意事项（与原记录一致）

1. **config/ 和 launch/ 目录为空**: 运行节点前需要补充 YAML 配置和 launch 启动脚本
2. **HPL 库是哑实现**: `hpl::ParamNode::declParam()` 不读取 YAML，pathplanning 的参数使用代码中的默认值
3. **CMake 4.x 来自 pip**: 系统包管理器不管理它，升级需手动 `pip install --upgrade cmake`
4. **kiss_icp_wrapper 无实际节点**: 其 CMakeLists 仅有 `ament_package()` + install，TODO 待补充
