# 实机相机模型

当前硬件启动默认加载 `yolov8sp2-int8.engine`：YOLOv8s-P2、固定输入
`[1,3,768,1280]`、用 32 张相机图像校准的 TensorRT INT8 引擎。
1280x720 相机图像会按比例缩放并在上下补边，输出框映射回原图。
权重 `yolov8sp2.pt` 的类别映射为 `0=red -> ORANGE, 1=yellow -> YELLOW,
2=blue -> BLUE`。

重新量化脚本位于仓库根目录 `tools/build_yolov8s_int8.py`。构建出的 ONNX、
engine、标定缓存和模型权重均不提交。LW-DETR 可用 `--model PATH` 显式选择。

YOLOv8 的旧权重 `best-new.pt` 和固定 1280x768 FP16 引擎 `best-new.engine` 仍可
通过 `--model PATH` 显式选择作对照。

旧 YOLO 模型输入 float32 `[1, 3, 640, 640]`，输出
`[1, 7, 8400]`，未内置 NMS；类别为 `0=red, 1=yellow, 2=blue`。
PT/ONNX/TensorRT 后端均撤销 letterbox 并输出校正左目原图坐标，保留完整类别证据。
yellow 对应 YELLOW，blue 对应 BLUE；用户已确认 red 对应 ORANGE。
