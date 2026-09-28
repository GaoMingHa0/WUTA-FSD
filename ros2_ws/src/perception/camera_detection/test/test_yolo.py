from types import SimpleNamespace

import numpy as np
import pytest

from camera_detection.yolo import (annotate, checkpoint_input_size, decode,
                                   EngineYoloModel, execution_providers, image_bgr,
                                   letterbox, YoloModel)


def test_bgra_with_row_padding():
    msg = SimpleNamespace(width=2, height=2, step=12, encoding='bgra8',
        data=bytes([1, 2, 3, 255, 4, 5, 6, 255, 0, 0, 0, 0,
                    7, 8, 9, 255, 10, 11, 12, 255, 0, 0, 0, 0]))
    assert image_bgr(msg).tolist() == [[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]]


def test_letterbox_decode_original_pixels_and_duplicate_color_suppression():
    image = np.zeros((720, 1280, 3), np.uint8)
    tensor, scale, padding = letterbox(image, (640, 640))
    assert tensor.shape == (1, 3, 640, 640)
    assert padding == (0, 140)
    output = np.array([[[320, 320, 100, 100, .01, .01, .95],
                        [320, 320, 100, 100, .01, .85, .01]]], dtype=float).transpose(0, 2, 1)
    detections = decode(output, scale, padding, image.shape, [0, 2, 1])
    assert len(detections) == 1
    box, probabilities, confidence = detections[0]
    assert box == [540, 260, 740, 460]
    assert np.isclose(sum(probabilities), 1)
    assert probabilities[1] == .95
    assert confidence == .95


def test_checkpoint_input_size_preserves_rectangular_training_shape_and_stride():
    assert checkpoint_input_size([760, 1280], 32) == (768, 1280)
    assert checkpoint_input_size(1280, 32) == (1280, 1280)
    with pytest.raises(ValueError):
        checkpoint_input_size([720], 32)


def test_weak_red_stays_unknown_and_empty_output():
    output = np.array([[[20, 20, 10, 10, .7, .1, .1]]]).transpose(0, 2, 1)
    detections = decode(output, 1, (0, 0), (100, 100, 3), [0, 2, 1])
    assert detections[0][1][0] == pytest.approx(.8)
    assert decode(output, 1, (0, 0), (100, 100, 3), [0, 2, 1], confidence=.9) == []


def test_reject_end_to_end_output():
    with pytest.raises(ValueError):
        decode(np.zeros((1, 300, 6)), 1, (0, 0), (100, 100, 3), [0, 2, 1])


def test_annotations_keep_source_unchanged_and_use_source_pixels():
    image = np.zeros((100, 120, 3), np.uint8)
    rendered = annotate(image, [([30, 40, 60, 80], [0., 0., 0., 1.], .95)])
    assert not image.any()
    assert rendered.shape == image.shape
    assert rendered[60, 30].tolist() == [0, 140, 255]
    assert not rendered[90, 90].any()
    assert np.array_equal(annotate(image, []), image)


def test_gpu_request_does_not_silently_become_cpu(monkeypatch):
    monkeypatch.setattr('camera_detection.yolo.ort.get_available_providers',
                        lambda: ['CPUExecutionProvider'])
    with pytest.raises(RuntimeError, match='CUDAExecutionProvider unavailable'):
        execution_providers('cuda', 0)
    assert execution_providers('cpu', 0) == ['CPUExecutionProvider']


def test_pt_gpu_request_fails_before_loading_weights_when_cuda_unavailable(monkeypatch):
    import sys
    from camera_detection.yolo import YoloModel
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False)))
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=lambda *args, **kwargs: pytest.fail('Must not load weights')))
    with pytest.raises(RuntimeError, match='CPU fallback is disabled'):
        YoloModel('best.pt', device='cuda')


def test_engine_suffix_selects_tensorrt_backend(monkeypatch):
    monkeypatch.setattr(EngineYoloModel, '__init__', lambda self, *args, **kwargs: None)
    assert isinstance(YoloModel('best.engine'), EngineYoloModel)
