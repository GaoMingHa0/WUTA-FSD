"""ROS interface regression for model IDs, fusion, mapping and red/blue planning.

Run after building the affected FSD packages, on an isolated ROS_DOMAIN_ID:
  /usr/bin/python3 -m pytest test/test_cone_colors_ros.py -q
"""
import math
from copy import deepcopy
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster
from wuta_msgs.msg import CameraConeDetection, CameraConeDetectionArray, Cone, ConeArray, ConeMap, MissionState


@pytest.fixture
def ros(tmp_path):
    rclpy.init()
    node = rclpy.create_node('cone_color_test_' + uuid.uuid4().hex[:8])
    processes = []
    logs = []
    observations = {}
    broadcasters = []

    class Harness:
        camera_frame = node.get_name() + '_camera'
        lidar_frame = node.get_name() + '_lidar'

        def fusion_tf(self):
            broadcaster = StaticTransformBroadcaster(node)
            broadcasters.append(broadcaster)
            transforms = []
            for child in (self.camera_frame, self.lidar_frame):
                t = TransformStamped()
                t.header.frame_id, t.child_frame_id = 'map', child
                t.header.stamp = self.stamp()
                t.transform.rotation.w = 1.
                transforms.append(t)
            broadcaster.sendTransform(transforms)

        def topic(self, name):
            return '/' + node.get_name() + '/' + name

        def start(self, package, executable, params=None, remaps=None):
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            args = [str(binary), '--ros-args']
            for key, value in (params or {}).items():
                args += ['-p', key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))]
            for old, new in (remaps or {}).items():
                args += ['-r', old + ':=' + self.topic(new)]
            log = (tmp_path / (executable + str(len(processes)) + '.log')).open('w+')
            processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy()))
            logs.append(log)

        def publisher(self, cls, name, latched=False):
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else 10
            return node.create_publisher(cls, self.topic(name), qos)

        def received(self, cls, name):
            received = []
            observations[name] = received
            node.create_subscription(cls, self.topic(name), received.append, 10)
            return received

        def wait(self, predicate, send=None, timeout=8):
            deadline = time.monotonic() + timeout
            next_send = 0
            while time.monotonic() < deadline:
                if any(p.poll() is not None for p in processes):
                    pytest.fail('ROS node exited: ' + self.output())
                if predicate():
                    return
                if send and time.monotonic() >= next_send:
                    send()
                    next_send = time.monotonic() + .08
                rclpy.spin_once(node, timeout_sec=.02)
            pytest.fail('ROS condition timed out: ' + self.output())

        def spin(self, seconds=.15):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)

        def output(self):
            output = []
            for log in logs:
                log.flush()
                log.seek(0)
                output.append(log.read())
            output.extend(name + ': ' + str(messages[-1]) for name, messages in observations.items() if messages)
            return '\n'.join(output)

        def stamp(self):
            return node.get_clock().now().to_msg()

    try:
        yield Harness()
    finally:
        for proc in processes:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def cone(x, y, color=Cone.COLOR_UNKNOWN, z=.2):
    result = Cone()
    result.position.x, result.position.y, result.position.z = float(x), float(y), float(z)
    result.color = color
    result.confidence = .9
    return result


def pose(ros):
    result = PoseStamped()
    result.header.frame_id = 'map'
    result.header.stamp = ros.stamp()
    result.pose.orientation.w = 1.
    return result


def test_message_ids_and_unknown_default():
    assert (Cone.COLOR_RED, Cone.COLOR_YELLOW, Cone.COLOR_BLUE, Cone.COLOR_ORANGE, Cone.COLOR_UNKNOWN) == (0, 1, 2, 3, 4)
    assert Cone().color == Cone.COLOR_UNKNOWN
    assert len(CameraConeDetection().color_probabilities) == 5
    assert ConeMap().red_cones == []


@pytest.mark.parametrize('executable', ['detection_fusion_node_cpp', 'detection_fusion_node'])
def test_fusion_keeps_red_zero_and_all_other_colors(ros, executable):
    ros.fusion_tf()
    ros.start('detection_fusion', executable, {'fixed_frame': 'map', 'min_color_probability': .6,
        'publish_unmatched_lidar': True, 'fuse_positions': False}, {
        '/perception/lidar/cones_raw': 'lidar', '/perception/camera/cones': 'camera',
        '/perception/camera/camera_info': 'info', '/perception/fused/cones': 'fused',
        '/perception/fusion/status': 'status'})
    lidar_pub = ros.publisher(ConeArray, 'lidar')
    camera_pub = ros.publisher(CameraConeDetectionArray, 'camera')
    info_pub = ros.publisher(CameraInfo, 'info')
    output = ros.received(ConeArray, 'fused')
    ros.received(String, 'status')
    ros.wait(lambda: all(p.get_subscription_count() for p in (lidar_pub, camera_pub, info_pub)))

    def send():
        stamp = ros.stamp()
        info = CameraInfo()
        info.header.frame_id, info.header.stamp = ros.camera_frame, stamp
        info.width, info.height = 1280, 720
        info.p = [1000., 0., 640., 0., 0., 1000., 360., 0., 0., 0., 1., 0.]
        lidar, camera = ConeArray(), CameraConeDetectionArray()
        camera.header = deepcopy(info.header)
        lidar.header = deepcopy(info.header)
        lidar.header.frame_id = ros.lidar_frame
        for color in range(5):
            x = color - 2.
            lidar.cones.append(cone(x, 0., z=5.))
            detection = CameraConeDetection()
            detection.detection_id = color
            u = 640. + 200. * x
            detection.bbox_xyxy = [u - 12., 348., u + 12., 372.]
            probabilities = [0.] * 5
            probabilities[color] = .9
            if color != Cone.COLOR_UNKNOWN:
                probabilities[Cone.COLOR_UNKNOWN] = .1
            detection.color_probabilities = probabilities
            detection.confidence = .9
            camera.detections.append(detection)
        info_pub.publish(info)
        camera_pub.publish(camera)
        lidar_pub.publish(lidar)

    ros.wait(lambda: any([c.color for c in m.cones] == list(range(5)) for m in output), send)
    fused = next(m for m in output if [c.color for c in m.cones] == list(range(5)))
    assert fused.header.frame_id == ros.lidar_frame
    assert fused.header.stamp.sec > 0
    assert [c.position.x for c in fused.cones] == [-2., -1., 0., 1., 2.]
    assert all(c.confidence == pytest.approx(.9) for c in fused.cones)


def test_map_confirms_red_blue_and_keeps_orange_separate(ros):
    ros.start('cone_map_builder', 'cone_map_builder_node', {'assign_colors': False,
        'allow_semantic_color_correction': True, 'semantic_color_confirmation_hits': 3,
        'min_hit_count': 1}, {'/perception/lidar/cones': 'input', '/localization/pose': 'pose',
        '/mapping/cone_map': 'map', '/mapping/cone_map_viz': 'markers'})
    inp, poses = ros.publisher(ConeArray, 'input'), ros.publisher(PoseStamped, 'pose')
    maps = ros.received(ConeMap, 'map')
    markers = ros.received(MarkerArray, 'markers')
    ros.wait(lambda: inp.get_subscription_count() and poses.get_subscription_count())
    poses.publish(pose(ros))
    ros.spin()

    def scan():
        msg = ConeArray()
        msg.header.frame_id, msg.header.stamp = 'map', ros.stamp()
        msg.cones = [cone(3, 2, Cone.COLOR_RED), cone(3, -2, Cone.COLOR_BLUE),
                     cone(10, 0, Cone.COLOR_ORANGE), cone(15, 0, Cone.COLOR_YELLOW), cone(20, 0)]
        inp.publish(msg)
        ros.spin()

    scan()
    scan()
    ros.wait(lambda: maps and len(maps[-1].unknown_cones) == 5)
    assert not maps[-1].red_cones and not maps[-1].orange_cones
    scan()
    ros.wait(lambda: maps and all(len(getattr(maps[-1], c + '_cones')) == 1
                                  for c in ('red', 'blue', 'yellow', 'orange', 'unknown')))
    assert maps[-1].red_cones[0].position.y == 2.
    assert maps[-1].blue_cones[0].position.y == -2.
    assert maps[-1].header.frame_id == 'map'
    ros.wait(lambda: markers and any(m.color.r == 1. and m.color.g == 0. and m.color.b == 0.
                                     for m in markers[-1].markers if m.type == m.CYLINDER))


@pytest.mark.parametrize('assign_colors,expected', [(True, (0, 2)), (False, (4, 4))])
def test_unknown_observations_use_red_left_blue_right_only_when_enabled(ros, assign_colors, expected):
    ros.start('cone_map_builder', 'cone_map_builder_node', {'assign_colors': assign_colors,
        'min_hit_count': 1}, {'/perception/lidar/cones': 'input', '/localization/pose': 'pose',
        '/mapping/cone_map': 'map'})
    inp, poses = ros.publisher(ConeArray, 'input'), ros.publisher(PoseStamped, 'pose')
    maps = ros.received(ConeMap, 'map')
    ros.wait(lambda: inp.get_subscription_count() and poses.get_subscription_count())

    def send():
        poses.publish(pose(ros))
        msg = ConeArray()
        msg.header.frame_id, msg.header.stamp = 'map', ros.stamp()
        msg.cones = [cone(3, 2), cone(3, -2)]
        inp.publish(msg)

    ros.wait(lambda: maps and len(maps[-1].red_cones + maps[-1].blue_cones + maps[-1].unknown_cones) == 2, send)
    cones = maps[-1].red_cones + maps[-1].blue_cones + maps[-1].unknown_cones
    assert tuple(c.color for c in sorted(cones, key=lambda c: -c.position.y)) == expected


def test_boundary_pairs_red_left_and_blue_right(ros):
    ros.start('boundary_detector', 'boundary_detector_node', remaps={
        '/mapping/cone_map': 'input', '/localization/pose': 'pose', '/system/mission_state': 'mission',
        '/planning/centerline': 'lane'})
    maps, poses = ros.publisher(ConeMap, 'input'), ros.publisher(PoseStamped, 'pose')
    missions = ros.publisher(MissionState, 'mission')
    lanes = ros.received(Lane, 'lane')
    ros.wait(lambda: all(p.get_subscription_count() for p in (maps, poses, missions)))

    def send():
        poses.publish(pose(ros))
        mission = MissionState()
        mission.mission_mode, mission.state = MissionState.MISSION_TRACKDRIVE, MissionState.EXPLORE
        missions.publish(mission)
        m = ConeMap()
        m.header.frame_id, m.header.stamp = 'map', ros.stamp()
        m.red_cones = [cone(x, 2, Cone.COLOR_RED) for x in (2, 5, 8, 11)]
        m.blue_cones = [cone(x, -2, Cone.COLOR_BLUE) for x in (2, 5, 8, 11)]
        # Special markers sit on another line and must not distort red/blue pairing.
        m.orange_cones = [cone(x, 5, Cone.COLOR_ORANGE) for x in (2, 5, 8, 11)]
        m.yellow_cones = [cone(x, 5, Cone.COLOR_YELLOW) for x in (2, 5, 8, 11)]
        maps.publish(m)

    ros.wait(lambda: lanes and len(lanes[-1].waypoints) >= 3, send)
    assert lanes[-1].header.frame_id == 'map'
    assert all(abs(w.pose.pose.position.y) < .1 for w in lanes[-1].waypoints)


@pytest.mark.parametrize('debug_color', ['red', 'orange'])
def test_debug_filters_red_and_orange_independently(ros, debug_color):
    ros.fusion_tf()
    ros.start('detection_fusion', 'detection_fusion_node_cpp', {'fixed_frame': 'map',
        'debug_' + debug_color: True, 'min_color_probability': .6}, {
        '/perception/lidar/cones_raw': 'lidar', '/perception/camera/cones': 'camera',
        '/perception/camera/camera_info': 'info', '/perception/debug/' + debug_color + '_cones': 'debug'})
    lidar_pub = ros.publisher(ConeArray, 'lidar')
    camera_pub = ros.publisher(CameraConeDetectionArray, 'camera')
    info_pub = ros.publisher(CameraInfo, 'info')
    output = ros.received(ConeArray, 'debug')
    ros.wait(lambda: all(p.get_subscription_count() for p in (lidar_pub, camera_pub, info_pub)))

    def send():
        info = CameraInfo()
        info.header.frame_id, info.header.stamp = ros.camera_frame, ros.stamp()
        info.p = [1000., 0., 640., 0., 0., 1000., 360., 0., 0., 0., 1., 0.]
        lidar, camera = ConeArray(), CameraConeDetectionArray()
        camera.header = deepcopy(info.header)
        lidar.header = deepcopy(info.header)
        lidar.header.frame_id = ros.lidar_frame
        for color, x in ((Cone.COLOR_RED, -1.), (Cone.COLOR_ORANGE, 1.)):
            lidar.cones.append(cone(x, 0., z=5.))
            detection = CameraConeDetection()
            detection.bbox_xyxy = [640. + x * 200. - 10., 350., 640. + x * 200. + 10., 370.]
            detection.confidence = .9
            probs = [0.] * 5
            probs[color], probs[Cone.COLOR_UNKNOWN] = .9, .1
            detection.color_probabilities = probs
            camera.detections.append(detection)
        info_pub.publish(info)
        camera_pub.publish(camera)
        lidar_pub.publish(lidar)

    expected = Cone.COLOR_RED if debug_color == 'red' else Cone.COLOR_ORANGE
    ros.wait(lambda: any(len(m.cones) == 1 and m.cones[0].color == expected for m in output), send)


def test_closed_red_blue_map_produces_counterclockwise_global_centerline(ros):
    ros.start('boundary_detector', 'boundary_detector_node', remaps={
        '/mapping/cone_map': 'input', '/localization/pose': 'pose', '/system/mission_state': 'mission',
        '/planning/centerline': 'lane', '/planning/global_centerline_ready': 'ready'})
    maps, poses = ros.publisher(ConeMap, 'input'), ros.publisher(PoseStamped, 'pose')
    missions = ros.publisher(MissionState, 'mission')
    lanes, ready = ros.received(Lane, 'lane'), ros.received(Bool, 'ready')
    ros.wait(lambda: all(p.get_subscription_count() for p in (maps, poses, missions)))

    def send():
        p = pose(ros)
        p.pose.position.x = 10.
        p.pose.orientation.z = math.sin(math.pi / 4)
        p.pose.orientation.w = math.cos(math.pi / 4)
        poses.publish(p)
        mission = MissionState()
        mission.mission_mode, mission.state = MissionState.MISSION_TRACKDRIVE, MissionState.MAPPING_DONE
        missions.publish(mission)
        m = ConeMap()
        m.header.frame_id, m.header.stamp, m.is_closed = 'map', ros.stamp(), True
        for index in range(40):
            theta = index * 2 * math.pi / 40
            m.red_cones.append(cone(8 * math.cos(theta), 8 * math.sin(theta), Cone.COLOR_RED))
            m.blue_cones.append(cone(12 * math.cos(theta), 12 * math.sin(theta), Cone.COLOR_BLUE))
        maps.publish(m)

    ros.wait(lambda: ready and ready[-1].data and lanes and len(lanes[-1].waypoints) >= 20, send)
    for waypoint in lanes[-1].waypoints:
        p, q = waypoint.pose.pose.position, waypoint.pose.pose.orientation
        assert math.hypot(p.x, p.y) == pytest.approx(10., abs=.3)
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        assert -p.y * math.cos(yaw) + p.x * math.sin(yaw) > 0


@pytest.mark.parametrize('left_color,expected_quality', [('red', 'PASS'), ('yellow', 'FAIL')])
def test_mission_quality_requires_red_and_blue_boundaries(ros, left_color, expected_quality):
    ros.start('mission_manager', 'mission_manager_node', {'min_red_cones': 1, 'min_blue_cones': 1}, {
        '/mapping/cone_map': 'map', '/system/mission_state': 'mission',
        '/system/start_command': 'start', '/system/lidar_ready': 'lidar_ready',
        '/system/localization_ready': 'localization_ready'})
    maps = ros.publisher(ConeMap, 'map')
    start, lidar_ready, loc_ready = [ros.publisher(Bool, name) for name in ('start', 'lidar_ready', 'localization_ready')]
    states = ros.received(MissionState, 'mission')
    ros.wait(lambda: all(p.get_subscription_count() for p in (maps, start, lidar_ready, loc_ready)))

    def begin():
        for publisher in (start, lidar_ready, loc_ready):
            publisher.publish(Bool(data=True))

    ros.wait(lambda: states and states[-1].state == MissionState.EXPLORE, begin)
    m = ConeMap()
    m.header.frame_id, m.header.stamp, m.is_closed = 'map', ros.stamp(), True
    getattr(m, left_color + '_cones').append(cone(3, 2, Cone.COLOR_RED if left_color == 'red' else Cone.COLOR_YELLOW))
    m.blue_cones.append(cone(3, -2, Cone.COLOR_BLUE))
    ros.wait(lambda: states and states[-1].state == MissionState.MAPPING_DONE, lambda: maps.publish(m))
    assert 'quality=' + expected_quality in ros.output()
