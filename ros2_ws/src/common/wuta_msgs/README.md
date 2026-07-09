# wuta_msgs

WUTA-FSD ROS2 自定义消息定义包。

## 消息列表

### Cone.msg
单个锥桶检测结果。

| 字段 | 类型 | 说明 |
|------|------|------|
| `position` | `geometry_msgs/Point` | 锥桶中心位置（传感器坐标系或 map 坐标系） |
| `color` | `uint8` | 颜色（见常量） |
| `confidence` | `float32` | 检测置信度 [0.0, 1.0] |

颜色常量：`COLOR_UNKNOWN=0`, `COLOR_BLUE=1`, `COLOR_YELLOW=2`, `COLOR_ORANGE=3`

---

### ConeArray.msg
感知层统一输出格式，单帧锥桶检测结果。

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳 + 坐标系（通常为传感器 frame） |
| `cones` | `Cone[]` | 检测到的锥桶列表 |

**发布者**：`lidar_detection_node`, `camera_detection_node`
**订阅者**：`detection_fusion_node`, `cone_map_builder`

---

### ConeMap.msg
全局锥桶地图，建图完成后持续发布。

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | frame_id = "map" |
| `blue_cones` | `Cone[]` | 左边界（蓝色） |
| `yellow_cones` | `Cone[]` | 右边界（黄色） |
| `orange_cones` | `Cone[]` | 起终点（橙色） |
| `unknown_cones` | `Cone[]` | 未分类 |
| `is_closed` | `bool` | true = 第一圈完成，地图闭合 |

**发布者**：`cone_map_builder`
**订阅者**：`boundary_detector`, `mission_manager`

---

### MissionState.msg
系统任务状态，由 MissionManager 发布，所有节点可订阅以感知当前模式。

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳 |
| `state` | `uint8` | 任务状态（见常量） |
| `localization_mode` | `uint8` | 定位模式（见常量） |
| `description` | `string` | 可读描述 |

状态常量：`IDLE=0`, `READY=1`, `EXPLORE=2`, `MAPPING_DONE=3`, `RACE=4`, `FINISH=5`, `EMERGENCY=6`
定位模式常量：`LOC_KISS_ICP=0`, `LOC_NDT=1`
