"""YOLOv8 PT/ONNX inference in rectified source-image pixels."""
import ast
import os
import sys
from pathlib import Path

import cv2
import numpy as np
try:
    import onnxruntime as ort
except ImportError:
    ort = None


def image_bgr(msg):
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}
    if msg.encoding not in channels:
        raise ValueError('Unsupported color encoding: ' + msg.encoding)
    count = channels[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * count or len(msg.data) < msg.step * msg.height:
        raise ValueError('Invalid image dimensions/stride/buffer')
    pixels = np.ndarray((msg.height, msg.width, count), np.uint8,
                        buffer=bytes(msg.data), strides=(msg.step, count, 1))[:, :, :3]
    if msg.encoding in ('rgb8', 'rgba8'):
        pixels = pixels[:, :, ::-1]
    return np.ascontiguousarray(pixels)


def letterbox(image, size):
    height, width = image.shape[:2]
    target_h, target_w = size
    scale = min(target_w / width, target_h / height)
    resized_w, resized_h = round(width * scale), round(height * scale)
    left = (target_w - resized_w) // 2
    top = (target_h - resized_h) // 2
    canvas = np.full((target_h, target_w, 3), 114, np.uint8)
    canvas[top:top + resized_h, left:left + resized_w] = cv2.resize(image, (resized_w, resized_h))
    tensor = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return tensor, scale, (left, top)


def decode(output, scale, padding, image_shape, color_ids, confidence=0.5, iou=0.45):
    rows = np.asarray(output)
    if rows.ndim != 3 or rows.shape[0] != 1 or rows.shape[1] != 4 + len(color_ids):
        raise ValueError('Expected YOLOv8 output [1, 4 + classes, anchors], without NMS')
    rows = rows[0].T
    rows = rows[np.all(np.isfinite(rows), axis=1)]
    scores = rows[:, 4:].max(axis=1)
    selected = scores >= confidence
    rows, scores = rows[selected], scores[selected]
    if not len(rows):
        return []
    classes = rows[:, 4:].argmax(axis=1)
    boxes = np.column_stack((rows[:, :2] - rows[:, 2:4] / 2,
                             rows[:, :2] + rows[:, 2:4] / 2))
    boxes = (boxes - np.array([*padding, *padding])) / scale
    height, width = image_shape[:2]
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, width)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, height)
    valid = np.all(boxes[:, 2:] > boxes[:, :2], axis=1)
    rows, scores, classes, boxes = rows[valid], scores[valid], classes[valid], boxes[valid]
    # Class-agnostic suppression: one physical cone must not vote two colors.
    xywh = np.column_stack((boxes[:, :2], boxes[:, 2:] - boxes[:, :2]))
    indices = np.asarray(cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), confidence, iou)).reshape(-1)
    detections = []
    for index in indices:
        # Independent class scores are evidence, not calibrated probabilities.
        # Retain unassigned confidence as UNKNOWN rather than making weak scores certain.
        probabilities = np.zeros(4)
        evidence = np.clip(rows[index, 4:], 0, 1)
        evidence /= max(1.0, evidence.sum())
        for class_id, color_id in enumerate(color_ids):
            probabilities[color_id] += evidence[class_id]
        probabilities[0] += max(0.0, 1.0 - probabilities.sum())
        detections.append((boxes[index].tolist(), probabilities.tolist(), float(scores[index])))
    return detections


def execution_providers(device, gpu_device_id):
    if device not in ('cuda', 'cpu') or gpu_device_id < 0:
        raise ValueError('device must be cuda/cpu and gpu_device_id must be nonnegative')
    if ort is None:
        raise RuntimeError('ONNX weights require onnxruntime; use PT weights with PyTorch otherwise')
    if device == 'cpu':
        return ['CPUExecutionProvider']
    if 'CUDAExecutionProvider' not in ort.get_available_providers():
        raise RuntimeError('CUDAExecutionProvider unavailable; install onnxruntime-gpu for ROS Python')
    return [('CUDAExecutionProvider', {'device_id': gpu_device_id,
        'gpu_mem_limit': 2 * 1024 ** 3, 'cudnn_conv_algo_search': 'HEURISTIC',
        'cudnn_conv_use_max_workspace': '0', 'do_copy_in_default_stream': '1'}),
        'CPUExecutionProvider']


class YoloModel:
    def __new__(cls, path, *args, **kwargs):
        if Path(path).suffix.lower() == '.pt':
            return PtYoloModel(path, *args, **kwargs)
        return super().__new__(cls)

    backend = 'onnxruntime'

    @property
    def providers(self):
        return self.session.get_providers()

    def __init__(self, path, red_color=0, threads=4, device='cuda', gpu_device_id=0, profile_prefix=None):
        providers = execution_providers(device, gpu_device_id)
        if device == 'cuda' and hasattr(ort, 'preload_dlls'):
            # Preload in this process only; do not change the ZED driver's library environment.
            cuda_lib = Path('/usr/local/cuda-12.8/targets/x86_64-linux/lib')
            ort.preload_dlls(cuda=True, cudnn=False,
                            directory=str(cuda_lib) if cuda_lib.is_dir() else None)
            cudnn_lib = Path(ort.__file__).resolve().parent.parent / 'nvidia/cudnn/lib'
            ort.preload_dlls(cuda=False, cudnn=True,
                            directory=str(cudnn_lib) if cudnn_lib.is_dir() else None)
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        if profile_prefix:
            options.enable_profiling = True
            options.profile_file_prefix = str(profile_prefix)
        self.session = ort.InferenceSession(path, sess_options=options, providers=providers)
        if device == 'cuda' and 'CUDAExecutionProvider' not in self.session.get_providers():
            raise RuntimeError('CUDA provider failed to load; check CUDA 12/cuDNN 9 libraries')
        self.session.disable_fallback()
        self.device = device
        self.gpu_device_id = gpu_device_id
        inputs = self.session.get_inputs()
        if len(inputs) != 1 or inputs[0].type != 'tensor(float)' or inputs[0].shape[:2] != [1, 3]:
            raise ValueError('Expected one float32 image input [1, 3, H, W]')
        self.size = inputs[0].shape[2:]
        if len(self.size) != 2 or any(not isinstance(n, int) or n <= 0 for n in self.size):
            raise ValueError('Model must have fixed positive input dimensions')
        self.input_name = inputs[0].name
        names = ast.literal_eval(self.session.get_modelmeta().custom_metadata_map.get('names', '{}'))
        if not isinstance(names, dict) or set(names) != set(range(len(names))) or not names:
            raise ValueError('Model must provide contiguous class names metadata')
        mapping = {'blue': 1, 'yellow': 2, 'orange': 3, 'red': red_color}
        if red_color not in (0, 3) or any(name not in mapping for name in names.values()):
            raise ValueError('Unsupported model class mapping')
        self.color_ids = [mapping[names[i]] for i in range(len(names))]
        self.names = names

    def predict(self, image, confidence=0.5, iou=0.45):
        tensor, scale, padding = letterbox(image, self.size)
        output = self.session.run(None, {self.input_name: tensor})[0]
        return decode(output, scale, padding, image.shape, self.color_ids, confidence, iou)


class PtYoloModel:
    """Native PyTorch inference with the same preprocessing and color evidence as ONNX."""
    backend = 'pytorch'

    def __init__(self, path, red_color=0, threads=4, device='cuda', gpu_device_id=0, profile_prefix=None):
        if device not in ('cuda', 'cpu') or gpu_device_id < 0:
            raise ValueError('device must be cuda/cpu and gpu_device_id must be nonnegative')
        # Scope the compatible Python packages to the inference process, not the ZED driver.
        packages = os.environ.get('YOLO_PYTHON_PACKAGES',
            '/home/wuta/miniconda3/envs/tensorrt/lib/python3.10/site-packages')
        if Path(packages).is_dir() and packages not in sys.path:
            sys.path.insert(0, packages)
        import torch
        from ultralytics import YOLO
        if device == 'cuda' and (not torch.cuda.is_available() or gpu_device_id >= torch.cuda.device_count()):
            raise RuntimeError('PyTorch CUDA unavailable for the requested GPU; CPU fallback is disabled')
        torch.set_num_threads(threads)
        self.torch = torch
        self.device = device
        self.gpu_device_id = gpu_device_id
        self.torch_device = torch.device(f'cuda:{gpu_device_id}' if device == 'cuda' else 'cpu')
        loaded = YOLO(str(path), task='detect')
        if loaded.task != 'detect':
            raise ValueError('PT weights must be a YOLO detection model')
        self.model = loaded.model.to(self.torch_device).float().eval()
        self.names = loaded.names
        mapping = {'blue': 1, 'yellow': 2, 'orange': 3, 'red': red_color}
        if (red_color not in (0, 3) or not isinstance(self.names, dict) or not self.names
                or set(self.names) != set(range(len(self.names)))
                or any(name not in mapping for name in self.names.values())):
            raise ValueError('Unsupported model class mapping')
        self.color_ids = [mapping[self.names[i]] for i in range(len(self.names))]
        self.size = (640, 640)
        self.providers = [f'PyTorch:{self.torch_device}']

    def raw_output(self, image):
        tensor, scale, padding = letterbox(image, self.size)
        with self.torch.inference_mode():
            output = self.model(self.torch.from_numpy(tensor).to(self.torch_device))
            if isinstance(output, tuple):
                output = output[0]
            output = output.detach().cpu().numpy()
        return output, scale, padding

    def predict(self, image, confidence=0.5, iou=0.45):
        output, scale, padding = self.raw_output(image)
        return decode(output, scale, padding, image.shape, self.color_ids, confidence, iou)


def annotate(image, predictions, status_text=None):
    """Draw decoded boxes on a copy of the matching source image (BGR)."""
    result = image.copy()
    colors = [(180, 180, 180), (255, 80, 0), (0, 255, 255), (0, 140, 255)]
    names = ['unknown', 'blue', 'yellow', 'orange']
    height, width = result.shape[:2]
    if status_text:
        cv2.rectangle(result, (0, 0), (width - 1, min(height - 1, 35)), (0, 0, 0), -1)
        cv2.putText(result, status_text, (8, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)
    for box, probabilities, confidence in predictions:
        color_id = int(np.argmax(probabilities))
        color = colors[color_id]
        x0, y0, x1, y1 = np.rint(box).astype(int)
        x0, x1 = np.clip([x0, x1], 0, width - 1)
        y0, y1 = np.clip([y0, y1], 0, height - 1)
        cv2.rectangle(result, (int(x0), int(y0)), (int(x1), int(y1)), color, 2)
        label = f'{names[color_id]} {confidence:.2f}'
        (_, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        origin = (int(x0), int(min(height - 1, max(label_height + 2, y0 - 6))))
        cv2.putText(result, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(result, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return result
