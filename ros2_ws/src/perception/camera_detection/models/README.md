# 实机相机模型

将 YOLOv8 PyTorch 权重放在本目录，命名为 `best.pt`；启动脚本默认加载该文件。
仍可通过 `--model PATH` 显式选择旧的 `best.onnx`。

工控机绝对路径：
`/home/wuta/lyx/WUTA/WUTA-FSD/ros2_ws/src/perception/camera_detection/models/best.pt`。
启动配置支持显式模型路径，不依赖当前终端目录。
权重文件由 `.gitignore` 排除，不提交到 Git。

当前已检查的模型：输入 float32 `[1, 3, 640, 640]`，输出
`[1, 7, 8400]`，未内置 NMS；类别为 `0=red, 1=yellow, 2=blue`。
PT/ONNX 节点均撤销 letterbox 并输出校正左目原图坐标，保留完整类别证据。
yellow 对应 YELLOW，blue 对应 BLUE；用户已确认 red 对应 ORANGE。
