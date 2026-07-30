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
  当前帧每个轨迹最多匹配一个检测
  最近且颜色兼容、distance < merge_distance(0.5m) → 加权平均更新位置
  否则 → 新增锥桶
        │
        ▼ 同帧共视保护
  靠近且同一 ConeArray 中同时检测到 → 标记为真实独立锥桶
        │
        ▼ 在线收敛合并
  兼容、从未同帧共视的轨迹 distance < consolidation_distance(1.0m)
  → 按 hit_count 加权合并，清除定位修正形成的平行重复轨迹
        │
        ▼ 颜色融合
  上游已知颜色 → 多帧投票并始终优先
  上游未知颜色 → 使用距离最近的一次车体左右观测
  （y>0 为 BLUE，y<0 为 YELLOW）
        │
        ▼ hit_count >= min_hit_count(3) → 发布
        │
        ▼ 地图闭合（任一条件）
  正式 /system/lap_count >= mapping_laps(1)
  OR 几何回环兜底：
  累计行驶 > start_skip_distance(30m)
  AND 返回起点距离 < loop_closure_distance(3m)
  AND 当前朝向与起点朝向差 <= 60°
  AND 已确认锥桶数 >= min_cones_for_closure(10)
        │
        ▼ 冻结前再次收敛合并
        │
        ▼ is_closed = true → 保存 YAML → 通知 MissionManager
```

`merge_distance` 只负责把当前检测关联到已有轨迹；更大的
`consolidation_distance` 专门合并已收敛的重复轨迹，并在每个成功处理的检测帧后执行。
为了不误合并密集弯道的真实相邻同色锥桶，两个轨迹只要曾在同一检测帧中作为两个独立目标
出现，就会被永久排除在该宽半径合并之外。闭环冻结前再执行一次同样受共视保护的传递式合并。

## Topics

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/perception/lidar/cones` | `ConeArray` | 传感器坐标系锥桶 |
| 订阅 | `/localization/pose` | `PoseStamped` | 当前位姿（map frame） |
| 订阅 | `/system/lap_count` | `UInt32` | 正式定位圈次；达到 `mapping_laps` 后冻结地图 |
| 发布 | `/mapping/cone_map` | `ConeMap` | 全局锥桶地图，5Hz |
| 发布 | `/mapping/cone_map_viz` | `MarkerArray` | 可视化 |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `merge_distance` | 0.5m | 同一锥桶合并距离；用于吸收检测与定位小噪声，同时避免 Trackdrive 密集弯道把相邻锥桶融合掉 |
| `consolidation_distance` | 1.0m | 已有轨迹收敛后的在线重复清理半径；只作用于从未同帧共视的轨迹 |
| `min_hit_count` | 3 | 发布前的最低检测次数，过滤短寿命定位/检测轨迹 |
| `localization_jump_threshold` | 1.0m | 相邻定位回调超过该距离时暂停建图，防止错误位姿写入地图 |
| `localization_jump_cooldown_sec` | 2.0s | 定位跳变后的建图冷却时间 |
| `loop_closure_distance` | 3.0m | 判定回到起点的距离阈值 |
| `mapping_laps` | 1 | 正式圈次达到该值时冻结地图；几何闭环仍作为兜底 |
| `assign_colors` | true | true 时按 LiDAR/body 坐标系左右分色；false 时保留上游 detection/fusion 给出的颜色 |
| `map_save_path` | `/tmp/wuta_cone_map.yaml` | 地图保存路径 |

Note: `assign_colors=true` only fills in `COLOR_UNKNOWN` detections. If an
upstream detector or fusion node already provides blue/yellow/orange, the
builder preserves that semantic color and uses it in the merge vote. For
unknown detections, the closest observation is used instead of all-frame
majority voting so that distant visible sections do not dominate the side label.

## 线程模型

使用 `MultiThreadedExecutor` + 两个 `MutuallyExclusiveCallbackGroup`：
- **pose_cbg**：处理 `/localization/pose`（50Hz，仅存储，极轻）
- **cones_cbg**：串行处理 `/perception/lidar/cones`、`/system/lap_count` 和地图发布，
  避免在线合并删除轨迹时与可视化遍历并发

`ConeMapBuilder` 对检测消息先按其采样时间查询 `map <- lidar` TF，等待
`tf_lookup_timeout_sec`（默认 0.1 s）。检测消息在队列中等待精确采样时刻的 TF，最长
`pending_detection_timeout_sec`（默认 0.5 s），随后丢弃并记录警告。默认关闭
`use_latest_tf_fallback`，避免车辆运动时用最新 TF 转换旧点云造成地图偏移；该参数仅
用于兼容旧配置，不建议在建图时开启。

若相邻两条 `/localization/pose` 相距超过 `localization_jump_threshold`（默认 1 m），
builder 会清空待处理检测，并在 `localization_jump_cooldown_sec`（默认 2 s）内拒绝新的
锥筒帧。该保护针对 KISS-ICP 在重复赛段错误重定位：精确时间戳 TF 仍会存在，但其对应的
地图位姿已经错误，继续融合只会生成整段平移的重复锥桶。

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
