"""YOLOv8 PT/ONNX/TensorRT inference in rectified source-image pixels."""
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


def _letterbox_canvas(image, size):
    height, width = image.shape[:2]
    target_h, target_w = size
    scale = min(target_w / width, target_h / height)
    resized_w, resized_h = round(width * scale), round(height * scale)
    left = (target_w - resized_w) // 2
    top = (target_h - resized_h) // 2
    canvas = np.full((target_h, target_w, 3), 114, np.uint8)
    canvas[top:top + resized_h, left:left + resized_w] = cv2.resize(image, (resized_w, resized_h))
    return canvas, scale, (left, top)


def letterbox(image, size):
    canvas, scale, padding = _letterbox_canvas(image, size)
    tensor = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return tensor, scale, padding


def checkpoint_input_size(value, stride):
    """Convert an Ultralytics checkpoint imgsz value to a valid (H, W)."""
    if isinstance(value, int):
        size = (value, value)
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        size = tuple(value)
    else:
        raise ValueError('PT weights must provide imgsz as an integer or [height, width]')
    if any(not isinstance(number, int) or isinstance(number, bool) or number <= 0
           for number in size):
        raise ValueError('PT checkpoint imgsz values must be positive integers')
    stride = int(stride)
    if stride <= 0:
        raise ValueError('PT model stride must be positive')
    # A rectangular training height such as 760 is not divisible by the YOLO
    # stride of 32. Pad it to 768 to prevent feature-map shape mismatch.
    return tuple((number + stride - 1) // stride * stride for number in size)


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
        suffix = Path(path).suffix.lower()
        if suffix == '.pt':
            return PtYoloModel(path, *args, **kwargs)
        if suffix == '.engine':
            if Path(path).stem.lower().startswith('lwdetr'):
                return LwDetrEngineModel(path, *args, **kwargs)
            return EngineYoloModel(path, *args, **kwargs)
        return super().__new__(cls)

    backend = 'onnxruntime'

    @property
    def providers(self):
        return self.session.get_providers()

    def __init__(self, path, red_color=0, threads=4, device='cuda', gpu_device_id=0,
                 profile_prefix=None, input_size=None):
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
        if input_size is not None and tuple(input_size) != tuple(self.size):
            raise ValueError('ONNX model input size is fixed and cannot be overridden')
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

    def __init__(self, path, red_color=0, threads=4, device='cuda', gpu_device_id=0,
                 profile_prefix=None, input_size=None):
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
        checkpoint_size = self.model.args.get('imgsz')
        # Existing best.pt records the scalar training value 1280, but the
        # deployed model was validated at 640x640 and performs poorly when that
        # scalar is applied here. A rectangular value explicitly describes the
        # camera-shaped input used by the new weights; otherwise retain the
        # established deployment size.
        self.size = (checkpoint_input_size(input_size, self.model.stride.max().item())
                     if input_size is not None else
                     checkpoint_input_size(checkpoint_size, self.model.stride.max().item())
                     if isinstance(checkpoint_size, (list, tuple)) and len(checkpoint_size) == 2
                     else (640, 640))
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


class EngineYoloModel:
    """Fixed-shape TensorRT FP16 inference with the common WUTA decoder."""
    backend = 'tensorrt'

    def __init__(self, path, red_color=0, threads=4, device='cuda', gpu_device_id=0,
                 profile_prefix=None, input_size=None):
        del profile_prefix
        if device != 'cuda' or gpu_device_id < 0:
            raise ValueError('TensorRT engine inference requires device=cuda and a nonnegative gpu_device_id')
        packages = os.environ.get('YOLO_PYTHON_PACKAGES',
            '/home/wuta/miniconda3/envs/tensorrt/lib/python3.10/site-packages')
        if Path(packages).is_dir() and packages not in sys.path:
            sys.path.insert(0, packages)
        import torch
        if not torch.cuda.is_available() or gpu_device_id >= torch.cuda.device_count():
            raise RuntimeError('TensorRT CUDA unavailable for the requested GPU; CPU fallback is disabled')
        from ultralytics.nn.autobackend import AutoBackend

        torch.set_num_threads(threads)
        self.torch = torch
        self.device = device
        self.gpu_device_id = gpu_device_id
        self.torch_device = torch.device(f'cuda:{gpu_device_id}')
        self.model = AutoBackend(model=str(path), device=self.torch_device,
                                 fp16=True, verbose=False)
        binding = self.model.bindings.get('images')
        shape = tuple(binding.shape) if binding is not None else ()
        if len(shape) != 4 or shape[:2] != (1, 3) or any(number <= 0 for number in shape):
            raise ValueError('TensorRT engine must have a fixed image input [1, 3, H, W]')
        self.size = shape[2:]
        if input_size is not None:
            requested = checkpoint_input_size(input_size, int(self.model.stride))
            if requested != self.size:
                raise ValueError(
                    f'TensorRT engine input is {self.size}, but requested input aligns to {requested}')
        self.names = self.model.names
        mapping = {'blue': 1, 'yellow': 2, 'orange': 3, 'red': red_color}
        if (red_color not in (0, 3) or not isinstance(self.names, dict) or not self.names
                or set(self.names) != set(range(len(self.names)))
                or any(name not in mapping for name in self.names.values())):
            raise ValueError('Unsupported model class mapping')
        self.color_ids = [mapping[self.names[i]] for i in range(len(self.names))]
        import tensorrt as trt
        self.providers = [f'TensorRT:{trt.__version__}:cuda:{gpu_device_id}']
        self.model.warmup((1, 3, *self.size))

    def raw_output(self, image):
        canvas, scale, padding = _letterbox_canvas(image, self.size)
        # Transfer compact uint8 pixels and normalize on the GPU. TensorRT
        # consumes the binding pointer as dense BCHW memory and does not honor
        # PyTorch strides, so materialize RGB BCHW before the device transfer.
        tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None])
        tensor = self.torch.from_numpy(tensor).to(self.torch_device)
        tensor = tensor.half() if self.model.fp16 else tensor.float()
        tensor /= 255.0
        with self.torch.inference_mode():
            output = self.model(tensor)
            if isinstance(output, (list, tuple)):
                if len(output) != 1:
                    raise ValueError('Expected one TensorRT detection output')
                output = output[0]
            output = output.float().cpu().numpy()
        return output, scale, padding

    def predict(self, image, confidence=0.5, iou=0.45):
        output, scale, padding = self.raw_output(image)
        return decode(output, scale, padding, image.shape, self.color_ids, confidence, iou)


class LwDetrEngineModel:
    """Fixed-shape TensorRT LW-DETR with its own preprocessing and DETR decoder."""
    backend = 'tensorrt-int8-lwdetr'

    def __init__(self, path, red_color=0, threads=4, device='cuda', gpu_device_id=0,
                 profile_prefix=None, input_size=None):
        del profile_prefix
        if device != 'cuda' or gpu_device_id < 0:
            raise ValueError('LW-DETR TensorRT requires device=cuda and a nonnegative GPU ID')
        packages = os.environ.get('YOLO_PYTHON_PACKAGES',
            '/home/wuta/miniconda3/envs/tensorrt/lib/python3.10/site-packages')
        if Path(packages).is_dir() and packages not in sys.path:
            sys.path.insert(0, packages)
        import torch
        import tensorrt as trt
        if not torch.cuda.is_available() or gpu_device_id >= torch.cuda.device_count():
            raise RuntimeError('CUDA unavailable for LW-DETR TensorRT inference')
        if red_color not in (0, 3):
            raise ValueError('red_color must be 0 (UNKNOWN) or 3 (ORANGE)')
        torch.set_num_threads(threads)
        self.torch = torch
        self.device = 'cuda'
        self.gpu_device_id = gpu_device_id
        self.torch_device = torch.device(f'cuda:{gpu_device_id}')
        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:
            raise RuntimeError(f'Failed to deserialize TensorRT engine: {path}')
        self.context = self.engine.create_execution_context()
        self.inputs = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)
                       if self.engine.get_tensor_mode(self.engine.get_tensor_name(i)) == trt.TensorIOMode.INPUT]
        self.outputs = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)
                        if self.engine.get_tensor_mode(self.engine.get_tensor_name(i)) == trt.TensorIOMode.OUTPUT]
        if len(self.inputs) != 1 or len(self.outputs) != 2:
            raise ValueError(f'Expected one input and DETR boxes/logits outputs, got {self.inputs}, {self.outputs}')
        self.input_name = self.inputs[0]
        input_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        if len(input_shape) != 4 or input_shape[:2] != (1, 3) or min(input_shape) <= 0:
            raise ValueError(f'Expected fixed [1,3,H,W] LW-DETR input, got {input_shape}')
        self.size = input_shape[2:]
        if input_size is not None and tuple(input_size) != tuple(self.size):
            raise ValueError(f'Engine input {self.size} does not match configured input {tuple(input_size)}')
        self.torch_dtype = {trt.DataType.FLOAT: torch.float32,
                            trt.DataType.HALF: torch.float16}[self.engine.get_tensor_dtype(self.input_name)]
        self.output_buffers = []
        for name in self.outputs:
            shape = tuple(self.engine.get_tensor_shape(name))
            if any(dimension <= 0 for dimension in shape):
                raise ValueError(f'LW-DETR output must have a fixed shape, got {name}: {shape}')
            dtype = {trt.DataType.FLOAT: torch.float32, trt.DataType.HALF: torch.float16,
                     trt.DataType.INT32: torch.int32}[self.engine.get_tensor_dtype(name)]
            self.output_buffers.append((name, torch.empty(shape, dtype=dtype, device=self.torch_device)))
        output_shapes = [tuple(buffer.shape) for _, buffer in self.output_buffers]
        if not any(shape[-1:] == (4,) for shape in output_shapes) or not any(shape[-1:] == (3,) for shape in output_shapes):
            raise ValueError(f'Expected DETR output boxes [...,4] and logits [...,3], got {output_shapes}')
        self.boxes_name = next(name for name, buffer in self.output_buffers if buffer.shape[-1] == 4)
        self.logits_name = next(name for name, buffer in self.output_buffers if buffer.shape[-1] == 3)
        self.names = {0: 'red', 1: 'yellow', 2: 'blue'}
        self.class_to_color = [red_color, 2, 1]
        self.providers = [f'TensorRT:{trt.__version__}:cuda:{gpu_device_id}:INT8+FP16']
        self.torch.cuda.synchronize(self.torch_device)
        self.raw_output(np.zeros((self.size[0], self.size[1], 3), dtype=np.uint8))

    def raw_output(self, image):
        height, width = self.size
        source_h, source_w = image.shape[:2]
        scale = min(width / source_w, height / source_h)
        resized_w, resized_h = round(source_w * scale), round(source_h * scale)
        left, top = (width - resized_w) // 2, (height - resized_h) // 2
        canvas = np.full((height, width, 3), 114, np.uint8)
        canvas[top:top + resized_h, left:left + resized_w] = cv2.resize(
            image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        pixels = canvas[:, :, ::-1].astype(np.float32) / 255.0
        pixels = (pixels - np.array([0.485, 0.456, 0.406], np.float32)) / np.array(
            [0.229, 0.224, 0.225], np.float32)
        tensor = self.torch.from_numpy(np.ascontiguousarray(
            pixels.transpose(2, 0, 1)[None])).to(device=self.torch_device, dtype=self.torch_dtype)
        self.context.set_tensor_address(self.input_name, int(tensor.data_ptr()))
        for name, buffer in self.output_buffers:
            self.context.set_tensor_address(name, int(buffer.data_ptr()))
        stream = self.torch.cuda.current_stream(self.torch_device)
        if not self.context.execute_async_v3(stream.cuda_stream):
            raise RuntimeError('TensorRT LW-DETR inference execution failed')
        outputs = {name: buffer.float().cpu().numpy() for name, buffer in self.output_buffers}
        return outputs[self.boxes_name], outputs[self.logits_name], scale, (left, top)

    def predict(self, image, confidence=0.5, iou=0.45):
        coords, logits, scale, (left, top) = self.raw_output(image)
        boxes = np.asarray(coords).reshape(-1, 4)
        scores_by_class = 1.0 / (1.0 + np.exp(-np.clip(np.asarray(logits).reshape(-1, 3), -80, 80)))
        count = min(300, scores_by_class.size)
        flat = scores_by_class.reshape(-1)
        chosen = np.argpartition(flat, -count)[-count:]
        chosen = chosen[np.argsort(flat[chosen])[::-1]]
        query_ids, class_ids = chosen // 3, chosen % 3
        scores = flat[chosen]
        cxcy, wh = boxes[query_ids, :2], boxes[query_ids, 2:]
        xyxy = np.concatenate((cxcy - wh / 2, cxcy + wh / 2), axis=1)
        xyxy *= np.array([self.size[1], self.size[0], self.size[1], self.size[0]])
        xyxy -= np.array([left, top, left, top])
        xyxy /= scale
        source_h, source_w = image.shape[:2]
        xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, source_w)
        xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, source_h)
        keep = (scores >= confidence) & (xyxy[:, 2] > xyxy[:, 0]) & (xyxy[:, 3] > xyxy[:, 1])
        indices = np.flatnonzero(keep)
        if not len(indices):
            return []
        nms_boxes = np.column_stack((xyxy[indices, :2], xyxy[indices, 2:] - xyxy[indices, :2]))
        nms = np.asarray(cv2.dnn.NMSBoxes(nms_boxes.tolist(), scores[indices].tolist(),
                                         confidence, iou)).reshape(-1)
        results = []
        for index in indices[nms]:
            probabilities = np.zeros(4, dtype=np.float32)
            probabilities[self.class_to_color[class_ids[index]]] = scores[index]
            probabilities[0] = 1.0 - scores[index]
            results.append((xyxy[index].tolist(), probabilities.tolist(), float(scores[index])))
        return results


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
