# Stereo / LiDAR late fusion

The hardware launch defaults to `detection_fusion_node_cpp` (C++/rclcpp).
The original Python/rclpy `detection_fusion_node` remains available through
`fusion_backend:=python`. Both consume detection results, not raw camera images
or track files. The Python backup uses NumPy and SciPy for assignment.

## Inputs and outputs

| Topic | Type | Semantics |
| --- | --- | --- |
| `/perception/lidar/cones_raw` | `wuta_msgs/msg/ConeArray` | LiDAR cluster centroids, original acquisition stamp/frame |
| `/perception/camera/cones` | `wuta_msgs/msg/CameraConeDetectionArray` | Rectified left-image YOLO boxes and color probabilities, optional stereo position/covariance |
| `/perception/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Rectified left image calibration; P used for projection; publish periodically |
| `/perception/fused/cones` | `wuta_msgs/msg/ConeArray` | One result per accepted LiDAR scan, original LiDAR frame/stamp |
| `/perception/fusion/status` | `std_msgs/msg/String` | JSON stamp_ns, reason, lidar_cones, matches, colored, camera_delta_ms, association_ms, mutual_matches, hungarian_rows |
| `/perception/debug/orange_cones` | `wuta_msgs/msg/ConeArray` | Optional orange cone XYZ in the original LiDAR frame/stamp, published directly from each raw cloud |
| `/perception/debug/orange_status` | `std_msgs/msg/String` | Optional JSON with orange count, XYZ, image time offset, local clustering time, `lidar_to_output_ms`, `camera_to_output_ms`, and `end_to_end_ms` |
| `/perception/debug/orange_markers` | `visualization_msgs/msg/MarkerArray` | Current orange cone cylinder and XYZ label; prior markers deleted every frame |

The C++ node also accepts `status_topic` for isolated ROS interface tests.

For a live orange-cone position check, run `./start_hardware_fusion.sh --debug-orange`.
Field tuning values and the dated measurements are collected in
[`../config/README.md`](../config/README.md) and
[`../config/field_tuning.yaml`](../config/field_tuning.yaml).
This C++-only mode uses the newest camera orange box and stereo depth to cluster
the raw LiDAR cloud as soon as it arrives. Its image time gate is 60 ms and its
image margin is 8 px. Cluster depth, width, height, and minimum size gates are
relaxed only for the debug output; color confidence is unchanged. Inspect
`/perception/debug/orange_status` for XYZ in metres and `end_to_end_ms` (from the
earlier camera/LiDAR acquisition stamp to debug publication). The ordinary
`/perception/fused/cones` path keeps its existing gates. If stereo depth is
missing or the local cluster fails, a colored normal-fusion cone supplies the
current position at its normal latency; the status `source` is then
`fusion_fallback` instead of `local_cluster`. The debug RViz preset displays
only `/perception/debug/orange_markers`. In debug mode the accumulating map
builder is not started, so a moved cone does not leave an old map marker.
An empty debug `ConeArray` means no candidate passed the local cluster check
for that matched camera/cloud pair. This mode still waits for a full M1 frame;
it removes the global detector dependency from the debug output path.

Inputs use sensor-data QoS (Best Effort / Volatile); outputs Reliable / Volatile,
depth 10. Reliable input publishers are compatible. TF must cover both acquisition
times; `fixed_frame=odom` must be continuous. Camera optical axes are right/down/forward.

## Algorithm and bounded fallback

LiDAR triggers a bounded queue (20 frames); wait at most 100 ms for camera/TF.
Nearest camera exposure must be within 30 ms. TF full-time lookup moves LiDAR
points from scan time to camera exposure time through odom. No latest-TF fallback.
The C++ matcher accepts isolated one-to-one candidate edges directly and calls
Hungarian only for unresolved conflicts. Fusion status reports the signed camera/LiDAR offset,
association time, direct-match count, and residual Hungarian size. Driver-side
LiDAR deskew and a shared clock are required for moving hardware.

Project into rectified image P; reject boxes with invalid geometry, points behind
camera, 3D distance >0.8 m, or 3D Mahalanobis distance squared >11.345. Reject
ambiguous rows/columns before Hungarian one-to-one matching, allowing unmatched
cones. Thresholds are initial engineering defaults, not hardware calibration.

With reliable stereo depth, conservative equal-weight covariance intersection
fuses XY, retains LiDAR centroid Z, and limits the XY correction to 0.25 m. An
extra 0.15 m reference uncertainty models differing visible-surface centroids.
Bad/absent stereo depth allows image-only color matching. Low-confidence color
stays UNKNOWN. Unmatched LiDAR survives when `publish_unmatched_lidar=true`;
the hardware launch sets it to false. With `guided_clustering=true`, an unmatched
camera box with valid stereo depth can add a landmark only when nearby raw LiDAR
points form a size-gated 3D cluster. Each camera frame contributes color evidence
at most once, even if
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
