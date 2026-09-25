"""Bounded latest-image inference; publish the original exposure header."""
from array import array
from copy import deepcopy
import json
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from wuta_msgs.msg import CameraConeDetection, CameraConeDetectionArray

from camera_detection.yolo import YoloModel, annotate, image_bgr


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        defaults = {'model_path': '', 'image_topic': '/zed/zed_node/rgb/image_rect_color',
                    'annotated_topic': '/camera/yolo/image_annotated', 'publish_annotated_image': True,
                    'device': 'cuda', 'gpu_device_id': 0,
                    'model_input_width': 0, 'model_input_height': 0,
                    'output_topic': '/camera/yolo/cones', 'confidence_threshold': 0.5,
                    'nms_iou_threshold': 0.45, 'red_color': 3, 'inference_threads': 4}
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for name in ('confidence_threshold', 'nms_iou_threshold'):
            if not 0 < self.cfg[name] < 1:
                raise ValueError(name + ' must be between zero and one')
        if self.cfg['inference_threads'] < 1:
            raise ValueError('inference_threads must be positive')
        dimensions = (self.cfg['model_input_height'], self.cfg['model_input_width'])
        if any(value < 0 for value in dimensions) or ((dimensions[0] == 0) != (dimensions[1] == 0)):
            raise ValueError('model_input_width and model_input_height must both be zero or positive')
        input_size = dimensions if dimensions[0] > 0 else None
        self.model = YoloModel(self.cfg['model_path'], self.cfg['red_color'], self.cfg['inference_threads'],
                               self.cfg['device'], self.cfg['gpu_device_id'], input_size=input_size)
        self.get_logger().info('Loaded model classes: ' + str(self.model.names))
        self.get_logger().info('Detector device: ' + str(getattr(self.model, 'device', 'cuda')) +
                               '; backend: ' + self.model.backend + '; providers: ' + str(self.model.providers))
        self.pub = self.create_publisher(CameraConeDetectionArray, self.cfg['output_topic'], 10)
        self.image_pub = self.create_publisher(Image, self.cfg['annotated_topic'], qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/perception/camera/yolo/status', 10)
        self.lock = threading.Lock()
        self.latest = None
        self.stopping = threading.Event()
        self.create_subscription(Image, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def on_image(self, msg):
        with self.lock:
            self.latest = msg

    def run(self):
        while not self.stopping.wait(0.002):
            with self.lock:
                source, self.latest = self.latest, None
            if source is None:
                continue
            started = time.monotonic()
            try:
                if not source.header.frame_id:
                    raise ValueError('Image frame_id is empty')
                image = image_bgr(source)
                predictions = self.model.predict(image, self.cfg['confidence_threshold'],
                                                 self.cfg['nms_iou_threshold'])
                inference_ms = (time.monotonic() - started) * 1000
                result = CameraConeDetectionArray()
                result.header = deepcopy(source.header)
                for index, (box, probabilities, confidence) in enumerate(predictions):
                    detection = CameraConeDetection()
                    detection.detection_id = index
                    detection.bbox_xyxy = box
                    detection.color_probabilities = probabilities
                    detection.confidence = confidence
                    detection.position_valid = False
                    result.detections.append(detection)
                self.pub.publish(result)
                detection_publish_ms = (time.monotonic() - started) * 1000 - inference_ms
                annotation_ms = 0.0
                render_ms = 0.0
                image_data_ms = 0.0
                annotated_publish_ms = 0.0
                if self.cfg['publish_annotated_image'] and self.image_pub.get_subscription_count():
                    annotation_started = time.monotonic()
                    device_label = (f'CUDA GPU {self.model.gpu_device_id}'
                                    if self.model.device == 'cuda' else 'CPU')
                    annotated = annotate(image, predictions,
                        f'{self.model.backend} {device_label} | cones: {len(predictions)} | {inference_ms:.0f} ms')
                    render_ms = (time.monotonic() - annotation_started) * 1000
                    image_msg = Image()
                    image_msg.header = deepcopy(source.header)
                    image_msg.height, image_msg.width = annotated.shape[:2]
                    image_msg.encoding = 'bgr8'
                    image_msg.is_bigendian = 0
                    image_msg.step = image_msg.width * 3
                    data_started = time.monotonic()
                    # The ROS 2 generated setter validates every byte in a
                    # plain bytes object in Python. array('B') takes its fast
                    # path and preserves the exact BGR byte sequence.
                    image_msg.data = array('B', annotated.tobytes())
                    image_data_ms = (time.monotonic() - data_started) * 1000
                    annotation_ms = (time.monotonic() - annotation_started) * 1000
                    publish_started = time.monotonic()
                    self.image_pub.publish(image_msg)
                    annotated_publish_ms = (time.monotonic() - publish_started) * 1000
                self.status.publish(String(data=json.dumps({'detections': len(result.detections),
                    'inference_ms': inference_ms, 'device': self.model.device,
                    'backend': self.model.backend, 'providers': self.model.providers,
                    'detection_publish_ms': detection_publish_ms,
                    'annotation_ms': annotation_ms,
                    'render_ms': render_ms,
                    'image_data_ms': image_data_ms,
                    'annotated_publish_ms': annotated_publish_ms,
                    'cycle_ms': (time.monotonic() - started) * 1000,
                    'stamp_ns': source.header.stamp.sec * 1000000000 + source.header.stamp.nanosec})))
            except Exception as error:
                self.get_logger().error('Detector inference failed: ' + str(error))

    def close(self):
        self.stopping.set()
        self.worker.join()


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
