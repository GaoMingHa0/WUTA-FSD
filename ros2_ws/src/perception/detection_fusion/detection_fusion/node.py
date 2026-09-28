"""Bounded, LiDAR-triggered late fusion. Drivers/YOLO run outside this node."""

from collections import deque
from copy import deepcopy
import json
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from wuta_msgs.msg import CameraConeDetectionArray, Cone, ConeArray

from detection_fusion.core import associate, covariance, fuse_position, guided_cluster, transform_matrix


def stamp_ns(stamp):
    return stamp.sec*1000000000 + stamp.nanosec


class DetectionFusion(Node):
    def __init__(self):
        super().__init__('detection_fusion_node')
        defaults = {
            'lidar_topic': '/perception/lidar/cones_raw',
            'camera_topic': '/perception/camera/cones',
            'camera_info_topic': '/perception/camera/camera_info',
            'output_topic': '/perception/fused/cones',
            'fixed_frame': 'odom', 'sync_slop_sec': 0.03,
            'max_wait_sec': 0.10, 'max_queue': 20,
            'max_match_distance': 0.8, 'mahalanobis_gate': 11.345,
            'pixel_margin': 8.0, 'ambiguity_margin': 0.15,
            'min_detection_confidence': 0.5, 'min_color_probability': 0.8,
            'lidar_sigma': 0.12, 'reference_sigma': 0.15,
            'max_position_shift': 0.25, 'fuse_positions': True,
            'publish_unmatched_lidar': True,
            'pointcloud_topic': '/rslidar_points', 'guided_clustering': False,
            'guided_voxel_size': 0.05, 'guided_cluster_tolerance': 0.15,
            'guided_depth_tolerance': 0.4, 'guided_min_cluster_size': 5,
            'guided_max_cluster_size': 200, 'guided_max_width': 0.5,
            'guided_min_height': 0.08, 'guided_max_height': 0.7,
        }
        self.cfg = {name: self.declare_parameter(name, value).value
                    for name, value in defaults.items()}
        for name in ('sync_slop_sec', 'max_wait_sec', 'max_match_distance',
                     'mahalanobis_gate', 'lidar_sigma', 'reference_sigma', 'max_position_shift'):
            if not np.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name + ' must be positive and finite')
        if self.cfg['max_queue'] < 1:
            raise ValueError('max_queue must be positive')
        self.cameras = deque(maxlen=self.cfg['max_queue'])
        self.clouds = deque(maxlen=self.cfg['max_queue'])
        self.pending = deque()
        self.info = None
        self.last_lidar_stamp = -1
        self.last_camera_stamp = -1
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(ConeArray, self.cfg['output_topic'], 10)
        self.status = self.create_publisher(String, '/perception/fusion/status', 10)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'],
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(CameraConeDetectionArray, self.cfg['camera_topic'],
                                 self.on_camera, qos_profile_sensor_data)
        self.create_subscription(ConeArray, self.cfg['lidar_topic'],
                                 self.on_lidar, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cfg['pointcloud_topic'],
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.01, self.process)

    def on_info(self, msg):
        p = np.asarray(msg.p).reshape(3, 4)
        if msg.header.frame_id and np.all(np.isfinite(p)) and p[0, 0] > 0 and p[1, 1] > 0:
            self.info = msg

    def on_camera(self, msg):
        if msg.header.frame_id and stamp_ns(msg.header.stamp) > self.last_camera_stamp:
            self.last_camera_stamp = stamp_ns(msg.header.stamp)
            self.cameras.append(msg)

    def on_lidar(self, msg):
        ns = stamp_ns(msg.header.stamp)
        # Duplicate/out-of-order scans cannot add extra map hits.
        if not msg.header.frame_id or ns <= self.last_lidar_stamp:
            return
        self.last_lidar_stamp = ns
        if len(self.pending) >= self.cfg['max_queue']:
            old, _ = self.pending.popleft()
            self.emit(old, None, 'queue_overflow')
        self.pending.append((msg, time.monotonic()))
        self.process()

    def on_cloud(self, msg):
        if msg.header.frame_id:
            self.clouds.append(msg)

    def matrix(self, target, target_stamp, source, source_stamp):
        t = self.tf.lookup_transform_full(
            target, Time.from_msg(target_stamp), source, Time.from_msg(source_stamp),
            self.cfg['fixed_frame'], timeout=Duration(seconds=0)).transform
        return transform_matrix([t.translation.x, t.translation.y, t.translation.z],
                                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    def process(self):
        while self.pending:
            lidar, queued = self.pending[0]
            ns = stamp_ns(lidar.header.stamp)
            candidates = [c for c in self.cameras
                          if abs(stamp_ns(c.header.stamp)-ns) <= self.cfg['sync_slop_sec']*1e9]
            camera = min(candidates, key=lambda c: abs(stamp_ns(c.header.stamp)-ns)) if candidates else None
            reason = 'camera_timeout'
            if camera is not None and self.info is not None:
                if self.info.header.frame_id == camera.header.frame_id:
                    try:
                        # LiDAR at tL -> camera at exposure tC, compensating ego motion.
                        matrix = self.matrix(camera.header.frame_id, camera.header.stamp,
                                             lidar.header.frame_id, lidar.header.stamp)
                        clouds = [c for c in self.clouds if c.header.frame_id == lidar.header.frame_id
                                  and abs(stamp_ns(c.header.stamp)-ns) <= 1000000]
                        cloud = min(clouds, key=lambda c: abs(stamp_ns(c.header.stamp)-ns)) if clouds else None
                        self.emit(lidar, (camera, matrix, cloud), 'fused')
                        self.pending.popleft()
                        # Each camera frame contributes semantic evidence at most once.
                        self.cameras = deque((c for c in self.cameras
                                              if stamp_ns(c.header.stamp) > stamp_ns(camera.header.stamp)),
                                             maxlen=self.cfg['max_queue'])
                        continue
                    except (TransformException, ValueError):
                        reason = 'transform_unavailable'
                else:
                    reason = 'camera_info_frame_mismatch'
            elif camera is not None:
                reason = 'camera_info_missing'
            if time.monotonic()-queued < self.cfg['max_wait_sec']:
                break
            self.pending.popleft()
            self.emit(lidar, None, reason)

    def emit(self, lidar, camera_data, reason):
        output = ConeArray()
        output.header = deepcopy(lidar.header)
        # Raw source color is ignored; this path must not inherit geometric side guesses.
        for raw in lidar.cones:
            if np.all(np.isfinite([raw.position.x, raw.position.y, raw.position.z])):
                cone = deepcopy(raw)
                cone.color = 0
                output.cones.append(cone)
        matches = []
        colored = 0
        raw_lidar_cones = len(output.cones)
        if camera_data is not None:
            camera, matrix, cloud = camera_data
            lidar_points = np.array([[c.position.x, c.position.y, c.position.z] for c in output.cones])
            projected_points = lidar_points @ matrix[:3, :3].T + matrix[:3, 3]
            observations, detections = [], []
            for detection in camera.detections:
                if not np.isfinite(detection.confidence) or detection.confidence < self.cfg['min_detection_confidence']:
                    continue
                point, cov = None, None
                if detection.position_valid:
                    try:
                        cov = covariance(detection.position_covariance)
                        cov += np.eye(3)*self.cfg['reference_sigma']**2
                        point = np.array([detection.position.x, detection.position.y, detection.position.z])
                        if not np.all(np.isfinite(point)) or point[2] <= 0:
                            raise ValueError('Invalid stereo point')
                    except ValueError:
                        point, cov = None, None
                observations.append((detection.bbox_xyxy, point, cov))
                detections.append(detection)
            matches = associate(projected_points, observations, np.asarray(self.info.p).reshape(3, 4),
                                **{k: self.cfg[k] for k in ('mahalanobis_gate', 'pixel_margin',
                                   'ambiguity_margin', 'lidar_sigma')},
                                max_distance=self.cfg['max_match_distance'])
            guided = 0
            if self.cfg['guided_clustering'] and cloud is not None:
                raw_points = point_cloud2.read_points_numpy(
                    cloud, field_names=['x', 'y', 'z'], skip_nans=True)
                raw_points = np.asarray(raw_points, dtype=float).reshape(-1, 3)
                matched_detections = {j for _, j in matches}
                for j, (box, point, _) in enumerate(observations):
                    if j in matched_detections or point is None:
                        continue
                    centre = guided_cluster(raw_points, matrix,
                        np.asarray(self.info.p).reshape(3, 4), box, point,
                        **{name.removeprefix('guided_'): self.cfg[name] for name in (
                            'guided_voxel_size', 'guided_cluster_tolerance',
                            'guided_depth_tolerance', 'guided_min_cluster_size',
                            'guided_max_cluster_size', 'guided_max_width',
                            'guided_min_height', 'guided_max_height')})
                    if centre is not None:
                        cone = Cone()
                        cone.position.x, cone.position.y, cone.position.z = map(float, centre)
                        cone.color = 0
                        cone.confidence = detections[j].confidence
                        output.cones.append(cone)
                        matches.append((len(output.cones)-1, j))
                        guided += 1
            inverse = np.linalg.inv(matrix)
            for i, j in matches:
                probs = np.asarray(detections[j].color_probabilities)
                if np.all(np.isfinite(probs)) and np.all(probs >= 0) and probs.sum() > 0:
                    probs = probs/probs.sum()
                    color = int(np.argmax(probs))
                    if color > 0 and probs[color] >= self.cfg['min_color_probability']:
                        output.cones[i].color = color
                        colored += 1
                _, point, cov = observations[j]
                if point is not None and self.cfg['fuse_positions']:
                    transformed = inverse[:3, :3]@point + inverse[:3, 3]
                    rotated_cov = inverse[:3, :3]@cov@inverse[:3, :3].T
                    fused = fuse_position(lidar_points[i], transformed, rotated_cov,
                                          self.cfg['lidar_sigma'], self.cfg['max_position_shift'])
                    output.cones[i].position.x = float(fused[0])
                    output.cones[i].position.y = float(fused[1])
        guided = guided if camera_data is not None else 0
        if not self.cfg['publish_unmatched_lidar']:
            matched_indices = {i for i, _ in matches}
            output.cones = [cone for i, cone in enumerate(output.cones) if i in matched_indices]
        self.pub.publish(output)
        self.status.publish(String(data=json.dumps({
            'stamp_ns': stamp_ns(output.header.stamp), 'reason': reason,
            'lidar_cones': raw_lidar_cones, 'matches': len(matches), 'colored': colored,
            'published_cones': len(output.cones),
            'unmatched_filtered': raw_lidar_cones-sum(i < raw_lidar_cones for i, _ in matches),
            'guided_clusters': guided})))


def main(args=None):
    rclpy.init(args=args)
    node = DetectionFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
