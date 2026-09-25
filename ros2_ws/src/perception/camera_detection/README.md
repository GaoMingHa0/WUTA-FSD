# Camera cone detector / stereo adapter

The hardware launch defaults to the YOLOv8s-P2 INT8 engine
`models/yolov8sp2-int8.engine`, loaded by the retained Python/TensorRT
`yolov8_node`. `detector_backend:=auto` selects the C++/TensorRT
`lwdetr_tensorrt_node` only for a LW-DETR `.engine`; YOLO models use Python.
Use `detector_backend:=cpp` to require the C++ node or `python` to select Python.
Both nodes publish the same detection, annotated image, and status topics.
LW-DETR engines use a DETR-specific decoder; other `.engine` files retain the YOLO decoder.
`.pt` weights remain supported through native PyTorch CUDA and `.onnx` through ONNX Runtime.
`stereo_detection_adapter_cpp` adds registered ZED depth to those detections by
default. The original Python `stereo_detection_adapter` remains available with
`adapter_backend:=python` in the hardware launch.
Driver acquisition is supplied by the standalone hardware fusion launch.

Store hardware engines in `models/` and shared camera-from-LiDAR calibration at
`../calibration/camera_lidar.yaml`. The supplied model classes are red/yellow/blue;
the user confirmed red means ORANGE. Model IDs are mapped explicitly.

The detector node consumes the latest image in a bounded worker, reverses letterbox,
and publishes `/camera/yolo/cones` with the original exposure stamp/frame.
`/camera/yolo/image_annotated` is a full-resolution bgr8 image with boxes, semantic
color names and confidence; it is rendered only while subscribers exist.
Use Best Effort QoS in RViz/rqt_image_view. No detections produces an unmarked
image. `/perception/camera/yolo/status` reports detection count, source image
timestamp, inference time, detection publishing time, annotation rendering time,
image-data conversion time, annotated image publishing time and total cycle time.
The image timestamp is not the camera shutter duration.
Parameters: `model_path`, `image_topic`, `output_topic`, `confidence_threshold`,
`nms_iou_threshold`, `red_color` (default 3), `inference_threads` (default 4),
`annotated_topic`, `publish_annotated_image` (default true), `model_input_width`,
and `model_input_height`. Set both dimensions for rectangular PT/engine input validation;
1280x760 is stride-aligned to a 1280x768 inference tensor. Leave both zero for
the legacy/model default.
`device` defaults to `cuda`, `gpu_device_id` to 0. CUDA provider loading is
checked, and failure raises an error; CPU execution requires explicit `device:=cpu`.
The annotated image includes the device, detected-cone count and inference time.

The LW-DETR engine expects fixed float input `[1,3,768,1280]`, RGB ImageNet normalization,
and emits 300 normalized boxes plus class logits. A 1280x720 camera frame is letterboxed
with 24-pixel top and bottom padding. Class IDs map as `0=red -> ORANGE`,
`1=yellow -> YELLOW`, `2=blue -> BLUE`. The model-specific engine is detected from the
`lwdetr` filename prefix. The default engine is generated locally and is not committed.

To regenerate it, put the checkpoint at `models/checkpoint_best_ema.pth.1` and run:

```bash
PYTHONNOUSERSITE=1 /home/wuta/miniconda3/envs/tensorrt/bin/python tools/export_lwdetr_onnx.py
PYTHONNOUSERSITE=1 /home/wuta/miniconda3/envs/tensorrt/bin/python tools/build_lwdetr_engine.py
```

The build script calibrates using up to 32 PNG images under `/home/wuta/BiaoDing/data/camera`.
For another camera domain, provide a representative calibration directory with `--calibration-dir`.

PT and TensorRT inference use the existing Python 3.10 PyTorch/TensorRT packages at
`/home/wuta/miniconda3/envs/tensorrt/lib/python3.10/site-packages` (validated:
PyTorch 2.13.0+cu130, Ultralytics 8.4.104, GTX 1660 SUPER). Override this package
directory with `YOLO_PYTHON_PACKAGES` when deploying elsewhere; packages must
match ROS Python 3.10. The path is added only inside the YOLO process.
TensorRT uses the engine's fixed FP16 1280x768 input. PT uses float32 at its
configured input size. Both use the same raw class-score decoding/NMS as ONNX;
exposure headers, source-pixel boxes and red-to-ORANGE mapping are retained.
CUDA availability is checked before loading weights; there is no CPU fallback.

For optional ONNX weights, ONNX Runtime must be installed for ROS system Python.
The GPU deployment uses ONNX Runtime GPU 1.23.2, CUDA 12.8 and cuDNN 9.10.2.21.
Install `onnxruntime-gpu==1.23.2` and `nvidia-cudnn-cu12==9.10.2.21` with system
pip `--target .hardware_deps --no-deps` from the repository root, alongside the
existing compatible SciPy. Other Python dependencies come from system/user site
packages. CUDA toolkit libraries are preloaded from `/usr/local/cuda-12.8/targets/x86_64-linux/lib`;
cuDNN 9 is preloaded from the GPU package's sibling `nvidia/cudnn/lib` directory.
These libraries are loaded only inside the YOLO process, leaving the ZED
driver's CUDA/cuDNN environment untouched.
On this industrial PC, system SciPy is incompatible with NumPy 2.2.6. The root
hardware script adds `.hardware_deps` to PYTHONPATH, containing SciPy 1.15.3;
this directory is local and not versioned. It can be restored using system pip
with `--target .hardware_deps --no-deps scipy==1.15.3` from the repository root.

## Driver contract

* `/camera/yolo/cones`: `wuta_msgs/msg/CameraConeDetectionArray`, exposure stamp,
  rectified LEFT optical frame. YOLO boxes must be converted from resized/letterbox
  inference coordinates to original rectified-image pixels. `bbox_xyxy` is
  `[xmin,ymin,xmax,ymax]`. Map model class IDs to `[UNKNOWN,BLUE,YELLOW,ORANGE]`
  probabilities explicitly; model class IDs are not Cone enum values. A top-1
  score may be represented as class probability with residual UNKNOWN mass.
* `/camera/left/depth_registered`: `sensor_msgs/msg/Image`, 32FC1 metres or
  16UC1 millimetres, depth registered to the same left image. The stereo driver
  computes disparity/depth; this adapter does not match raw left/right images.
* `/camera/left/camera_info`: `sensor_msgs/msg/CameraInfo`, same frame/resolution,
  calibrated rectified P. Publish periodically for late subscribers.

The three input topic names are parameters `boxes_topic`, `depth_topic`,
`info_topic`. The C++ node also accepts `output_topic` and `output_info_topic`
for isolated interface tests. Default outputs `/perception/camera/cones` and
`/perception/camera/camera_info` match the fusion contract. Inputs use sensor-data
QoS; outputs Reliable/Volatile depth 10.

Depth within 10 ms is accepted; wait at most 50 ms, otherwise preserve boxes/color
and mark depth invalid. Queue/history length 20. ROI is central 40% of the box;
require >=6 samples, >=60% valid depth, range 0.3–40 m and small 10–90% spread.
Output is a visible-surface position with propagated pixel/depth uncertainty,
not a cone base centre. Foreground segmentation, depth calibration and common
LiDAR/stereo reference calibration are prerequisites before enabling hardware
position fusion. A uniformly wrong/background depth can pass ROI checks, so
fusion also checks against LiDAR geometry.

```bash
ros2 run camera_detection stereo_detection_adapter_cpp
# Python backup: ros2 run camera_detection stereo_detection_adapter
PYTHONPATH=. python3 -m pytest test -q
```

High/low yellow cones share the current YELLOW enum; size classification is not
introduced by this adapter. Physical stereo baseline/extrinsics remain external.
