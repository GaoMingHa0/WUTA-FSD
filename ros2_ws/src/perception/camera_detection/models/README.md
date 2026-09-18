# 实机相机模型

1280x760 矩形训练权重为 `best-new.pt`，对应的固定 1280x768、batch 1、
FP16 TensorRT 文件为 `best-new.engine`。硬件启动脚本默认加载 engine；仍可通过
`--model PATH` 显式选择 PT 或 ONNX。

工控机绝对路径：
`/home/wuta/lyx/WUTA/WUTA-FSD/ros2_ws/src/perception/camera_detection/models/best-new.engine`。
启动配置支持显式模型路径，不依赖当前终端目录。
权重文件由 `.gitignore` 排除，不提交到 Git。

当前已检查的模型：输入 float32 `[1, 3, 640, 640]`，输出
`[1, 7, 8400]`，未内置 NMS；类别为 `0=red, 1=yellow, 2=blue`。
PT/ONNX/TensorRT 后端均撤销 letterbox 并输出校正左目原图坐标，保留完整类别证据。
yellow 对应 YELLOW，blue 对应 BLUE；用户已确认 red 对应 ORANGE。
