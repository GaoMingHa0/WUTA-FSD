# 实机相机 / 雷达外参

将运行时外参保存为 `camera_lidar.yaml`，绝对路径为
`/home/wuta/lyx/WUTA/WUTA-FSD/ros2_ws/src/perception/calibration/camera_lidar.yaml`。
填写格式见 [camera_lidar.example.yaml](camera_lidar.example.yaml)。

矩阵方向必须是 **LiDAR -> 校正左目光学坐标**：
`p_camera = R * p_lidar + t`，平移单位米，R 按行排列。
默认帧名为 `rslidar` 与 `zed_left_camera_optical_frame`；若驱动实际帧名不同，
填写实际消息 header.frame_id。MATLAB 导出结果需明确矩阵方向、行/列向量
约定和长度单位，不能直接凭变量名判断。

如果暂时只有 MATLAB `.mat`、导出的 `.txt` 或其他原始文件，可直接放在
此目录，保留原名，后续转换为上述 YAML。模板中的 null 表示尚未标定，
不能用零平移或单位旋转替代真实结果。

本目录为感知栈共享标定资料目录，不是独立 ROS 包。
相机内参从 ZED 的 CameraInfo 读取，无需另行复制；此外参仅描述相机和
雷达之间的关系，车辆坐标系外参与动态定位 TF 仍需分别验证。
运行时 `camera_lidar.yaml` 已被 Git 忽略，模板与格式说明可提交。
