"""External YOLOv8 boxes + registered stereo depth -> fusion input."""
from collections import deque
from copy import deepcopy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from wuta_msgs.msg import CameraConeDetectionArray
from camera_detection.stereo import depth_image, estimate_position


def stamp_ns(header):
    return header.stamp.sec*1000000000 + header.stamp.nanosec


class StereoAdapter(Node):
    def __init__(self):
        super().__init__('stereo_detection_adapter')
        self.declare_parameter('boxes_topic', '/camera/yolo/cones')
        self.declare_parameter('depth_topic', '/camera/left/depth_registered')
        self.declare_parameter('info_topic', '/camera/left/camera_info')
        self.depths = deque(maxlen=20)
        self.boxes = deque(maxlen=20)
        self.info = None
        self.pub = self.create_publisher(CameraConeDetectionArray, '/perception/camera/cones', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/perception/camera/camera_info', 10)
        self.create_subscription(CameraConeDetectionArray, self.get_parameter('boxes_topic').value,
                                 self.on_boxes, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_timer(0.01, self.process)

    def on_boxes(self, msg):
        self.boxes.append((msg, time.monotonic()))

    def on_depth(self, msg):
        self.depths.append(msg)

    def on_info(self, msg):
        self.info = msg
        self.info_pub.publish(msg)

    def process(self):
        while self.boxes:
            source, queued = self.boxes[0]
            candidates = [d for d in self.depths if d.header.frame_id == source.header.frame_id
                          and abs(stamp_ns(d.header)-stamp_ns(source.header)) <= 10000000]
            if not candidates and time.monotonic()-queued < 0.05:
                break
            self.boxes.popleft()
            result = deepcopy(source)
            for detection in result.detections:
                detection.position_valid = False
            if candidates and self.info is not None and self.info.header.frame_id == source.header.frame_id:
                image = min(candidates, key=lambda d: abs(stamp_ns(d.header)-stamp_ns(source.header)))
                try:
                    depth = depth_image(image)
                    if (image.width, image.height) != (self.info.width, self.info.height):
                        raise ValueError('Depth must be registered to rectified left image')
                    for detection in result.detections:
                        estimate = estimate_position(depth, detection.bbox_xyxy, self.info.p)
                        if estimate is not None:
                            point, cov = estimate
                            detection.position.x, detection.position.y, detection.position.z = map(float, point)
                            detection.position_covariance = cov.flatten().tolist()
                            detection.position_valid = True
                except (ValueError, TypeError):
                    pass  # Color-only fallback, never fabricate valid depth.
            self.pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = StereoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
