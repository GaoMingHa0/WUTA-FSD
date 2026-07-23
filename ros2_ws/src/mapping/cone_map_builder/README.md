# cone_map_builder

探索阶段（第一圈）的锥桶地图构建节点。将传感器坐标系下的锥桶检测结果累积为全局 map 坐标系下的锥桶地图，检测一圈完成后触发地图闭合。

## 核心流程

```
/perception/lidar/cones (sensor frame)
/localization/pose (map frame)
        │
        ▼ TF2 坐标变换
  逐帧锥桶 → map 坐标系
        │
        ▼ 去重合并
  distance < merge_distance(0.5m) → 加权平均更新位置
  否则 → 新增锥桶
        │
        ▼ 颜色分配（默认左右法）
  传感器坐标系 y>0 → COLOR_BLUE
  传感器坐标系 y<0 → COLOR_YELLOW
  同一锥桶多次观测采用颜色投票，避免单帧误染长期保留
        │
        ▼ hit_count >= min_hit_count(2) → 发布
        │
        ▼ Loop Closure 检测
  已移动 > start_skip_distance(5m)
  AND 返回起点距离 < loop_closure_distance(3m)
  AND 已确认锥桶数 >= min_cones_for_closure(10)
        │
        ▼ is_closed = true → 保存 YAML → 通知 MissionManager
```

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/perception/lidar/cones` | `ConeArray` | 传感器坐标系锥桶 |
| 订阅 | `/localization/pose` | `PoseStamped` | 当前位姿（map frame） |
| 发布 | `/mapping/cone_map` | `ConeMap` | 全局锥桶地图，5Hz |
| 发布 | `/mapping/cone_map_viz` | `MarkerArray` | 可视化 |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `merge_distance` | 1.2m | 同一锥桶合并距离；Mode B 中用于吸收 INS/EKF 与检测噪声导致的重复锥桶，仍需小于相邻锥桶间距 |
| `min_hit_count` | 2 | 发布前的最低检测次数，过滤单帧噪声 |
| `loop_closure_distance` | 3.0m | 判定回到起点的距离阈值 |
| `assign_colors` | true | true 时按 LiDAR/body 坐标系左右分色；false 时保留上游 detection/fusion 给出的颜色 |
| `map_save_path` | `/tmp/wuta_cone_map.yaml` | 地图保存路径 |

## 线程模型

使用 `MultiThreadedExecutor` + 两个 `MutuallyExclusiveCallbackGroup`：
- **pose_cbg**：处理 `/localization/pose`（50Hz，仅存储，极轻）
- **cones_cbg**：处理 `/perception/lidar/cones`（10Hz，含 TF 变换和去重计算）

`ConeMapBuilder` 对检测消息先按其采样时间查询 `map <- lidar` TF，等待
`tf_lookup_timeout_sec`（默认 0.1 s）。检测消息在队列中等待精确采样时刻的 TF，最长
`pending_detection_timeout_sec`（默认 0.5 s），随后丢弃并记录警告。默认关闭
`use_latest_tf_fallback`，避免车辆运动时用最新 TF 转换旧点云造成地图偏移；该参数仅
用于兼容旧配置，不建议在建图时开启。

避免锥桶处理耗时时阻塞 pose 更新。

## 地图文件格式（YAML）

```yaml
cone_map:
  - x: 10.5
    y: 3.2
    z: 0.1
    color: 1      # 1=BLUE, 2=YELLOW
    hit_count: 8
  - ...
```

## 待完善

- [ ] 上相机后将 `assign_colors` 改为 false，由 detection_fusion 提供颜色；builder 将保留上游颜色并继续做合并投票
- [ ] 地图加载接口（供 NDT 模式初始化使用）
- [ ] 橙色锥桶（起终点）的特殊处理
