# Stereo / LiDAR late fusion

`detection_fusion_node` is a Python/rclpy implementation in this ament_cmake
package. It consumes detection results, not raw camera images or track files.
NumPy and SciPy (Hungarian assignment) are runtime dependencies.

## Inputs and outputs

| Topic | Type | Semantics |
| --- | --- | --- |
| `/perception/lidar/cones_raw` | `wuta_msgs/msg/ConeArray` | LiDAR cluster centroids, original acquisition stamp/frame |
| `/perception/camera/cones` | `wuta_msgs/msg/CameraConeDetectionArray` | Rectified left-image YOLO boxes and color probabilities, optional stereo position/covariance |
| `/perception/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Rectified left image calibration; P used for projection; publish periodically |
| `/perception/fused/cones` | `wuta_msgs/msg/ConeArray` | One result per accepted LiDAR scan, original LiDAR frame/stamp |
| `/perception/fusion/status` | `std_msgs/msg/String` | JSON stamp_ns, reason, lidar_cones, matches, colored |

Inputs use sensor-data QoS (Best Effort / Volatile); outputs Reliable / Volatile,
depth 10. Reliable input publishers are compatible. TF must cover both acquisition
times; `fixed_frame=odom` must be continuous. Camera optical axes are right/down/forward.

## Algorithm and bounded fallback

LiDAR triggers a bounded queue (20 frames); wait at most 100 ms for camera/TF.
Nearest camera exposure must be within 60 ms. TF full-time lookup moves LiDAR
points from scan time to camera exposure time through odom. No latest-TF fallback.
Driver-side LiDAR deskew and a shared clock are required for moving hardware.

Project into rectified image P; reject boxes with invalid geometry, points behind
camera, 3D distance >0.8 m, or 3D Mahalanobis distance squared >11.345. Reject
ambiguous rows/columns before Hungarian one-to-one matching, allowing unmatched
cones. Thresholds are initial engineering defaults, not hardware calibration.

With reliable stereo depth, conservative equal-weight covariance intersection
fuses XY, retains LiDAR centroid Z, and limits the XY correction to 0.25 m. An
extra 0.15 m reference uncertainty models differing visible-surface centroids.
Bad/absent stereo depth allows image-only color matching. Low-confidence color
stays UNKNOWN. Unmatched LiDAR survives; camera-only detections do not create map
landmarks. Each camera frame contributes color evidence at most once, even if
LiDAR runs faster. Duplicate/out-of-order LiDAR stamps are dropped. Restart the
node after a replay clock reset.

`ConeArray` compatibility output deliberately does not expose covariance or color
probabilities: the existing builder retains hit-count position averaging, with
new optional color confirmation/correction. A full covariance landmark filter
is not implemented. `confidence` remains LiDAR detection confidence, not a color
probability. Colored observations have passed the probability threshold.

## Entry points

Fusion only (external detections and TF):

```bash
ros2 launch detection_fusion fusion.launch.py
```

Hardware-neutral LiDAR detection + stereo adapter + fusion + builder:

```bash
ros2 launch detection_fusion fusion_mapping.launch.py
```

Supply LiDAR driver `/hesai/pandar`, YOLO adapter `/camera/yolo/cones`, registered
depth `/camera/left/depth_registered`, CameraInfo `/camera/left/camera_info`, and
existing localization pose/TF. No real drivers, inference weights, camera intrinsics
or physical extrinsics are invented. See `../camera_detection/README.md`.
The hardware entry defaults to `fuse_positions=false` until centroid reference
and uncertainty are calibrated; colors still fuse. Simulation enables XY fusion.

From the parent WUTA repository: `./start_fusion_simulator.sh --rviz` starts
the independent synthetic-camera experiment. It cannot be used as real-camera
accuracy evidence. Do not launch it alongside the standard simulator.

## Tests

```bash
PYTHONPATH=. python3 -m pytest test -q
```

Tests cover unique/ambiguous matches, wrong-depth adjacent sections, covariance
validation, optical transform axes, and weak-stereo position weighting.
