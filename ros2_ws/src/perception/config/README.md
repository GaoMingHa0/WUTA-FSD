# 实车感知调试记录与总参数表（2026-09-25）

## 启动与实时位置

在仓库根目录运行：

```bash
./start_hardware_fusion.sh --debug-orange --rviz
```

调试模式只显示当前橙锥：RViz 订阅 `/perception/debug/orange_markers`，每次输出先删除上一帧标记，再绘制当前 XYZ，标记 0.25 秒后也会自动过期。调试模式不启动累积式 `cone_map_builder`，因此搬动锥桶后不会留下旧地图位置。`/perception/debug/orange_cones` 的坐标系为 `rslidar`，时间戳为原始点云时间戳；`/perception/debug/orange_status` 给出 XYZ、来源和延迟：

```bash
source /opt/ros/humble/setup.bash
source WUTA-FSD/ros2_ws/install/setup.bash
ros2 topic echo /perception/debug/orange_status
```

`source=local_cluster` 表示原始点云到达后立即用相机橙框做局部聚类；深度缺失或局部聚类失败时，`source=fusion_fallback` 表示使用该帧正常融合的橙锥位置。`end_to_end_ms` 从相机和雷达中较早的采集时间戳计到输出，`lidar_to_output_ms` 只从雷达帧时间戳计。调试数据不进入地图。正常建图时不加 `--debug-orange`。

## 一个文件调整实地参数

所有常用相机、全局雷达检测、正常融合、局部聚类和调试门限集中在 [field_tuning.yaml](field_tuning.yaml)。启动脚本默认读取它；自定义文件用 `--field-config PATH`。命令行 `name:=value` 优先于文件，例如：

```bash
./start_hardware_fusion.sh --debug-orange \
  --field-config WUTA-FSD/ros2_ws/src/perception/config/field_tuning.yaml \
  guided_depth_tolerance:=0.45
```

修改 YAML 后重启启动脚本生效。普通局部聚类当前是 **5 cm 体素、15 cm 连接距离**；4 cm 连接距离未启用。调试局部聚类只在调试输出中放宽深度差、尺寸和最少点数门限；颜色置信度仍由文件中的 `fusion_min_color_probability` 控制。当前不使用高度 ROI。外参和 ZED/M1 驱动设置分别位于 `../calibration/camera_lidar.yaml`、`../detection_fusion/config/zed_hardware.yaml` 和 `../detection_fusion/config/rsm1_hardware.yaml`。

## 今天的实测与限制

测量环境为当前工控机、RoboSense M1、ZED 和 YOLOv8s INT8 TensorRT；数字来自静止或近静止的单橙锥场景，不代表赛道上的召回率。

| 项目 | 中位数或计数 |
| --- | ---: |
| 原始点云时间戳 → 点云话题到达 | 93.7 ms |
| 点云话题到达 → 全局雷达检测结果 | 29.9 ms |
| 全局检测内聚类（优化配置的分段日志） | 约 31 ms |
| 正常融合内局部聚类 | 约 1.5 ms |
| 正常融合整体端到端（较早传感器时间戳 → 输出） | 约 133 ms |
| 橙锥调试输出：雷达时间戳 → 输出 | 93.1 ms；P95 117.9 ms |
| 橙锥调试输出：整体端到端 | 116.2 ms；P95 149.6 ms |
| 橙锥调试输出频率与检出 | 9.9 Hz；114/119 帧 |

M1 驱动目前约 10 Hz 成整帧发布，并以第一点为帧时间戳；点云到达前的约 94 ms 与等待整帧相符。调试模式绕开全局检测的等待，但仍需完整点云。把时间戳改成帧尾不会让实际输出提前。

静止橙锥的 20 秒采样中，相机 295/295 帧检测到橙锥、正常融合 190/197 帧输出橙锥，190/190 个投影落在对应橙框内。雷达投影深度与双目深度约差 0.26 m；没有测绘真值，不能据此保证绝对距离精度。搬动锥桶后曾在累积地图中出现两条橙锥记录，但实时相机和融合话题每帧各至多只有一个。调试模式已改为只显示当前位置。
