# YOLOv8 / stereo adapter

`stereo_detection_adapter` is implemented; camera acquisition and YOLOv8 inference
are reserved for the supplied hardware/weights. This node is not a YOLO runner.

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
`info_topic`. Outputs `/perception/camera/cones` and
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
ros2 run camera_detection stereo_detection_adapter
PYTHONPATH=. python3 -m pytest test -q
```

High/low yellow cones share the current YELLOW enum; size classification is not
introduced by this adapter. Physical stereo baseline/extrinsics remain external.
