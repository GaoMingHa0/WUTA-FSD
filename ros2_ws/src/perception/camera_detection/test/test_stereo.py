from types import SimpleNamespace
import numpy as np
from camera_detection.stereo import depth_image, estimate_position


def test_depth_and_covariance():
    result = estimate_position(np.full((100, 100), 5.), [30, 30, 70, 70],
                               [100, 0, 50, 0, 0, 100, 50, 0, 0, 0, 1, 0])
    point, cov = result
    assert abs(point[0]) < .1 and point[2] == 5.
    assert np.all(np.linalg.eigvalsh(cov) > 0)


def test_absent_and_mixed_background_depth_rejected():
    p = [100, 0, 50, 0, 0, 100, 50, 0, 0, 0, 1, 0]
    assert estimate_position(np.zeros((100, 100)), [30, 30, 70, 70], p) is None
    image = np.full((100, 100), 5.)
    image[:, 50:] = 15.
    assert estimate_position(image, [30, 30, 70, 70], p) is None


def test_depth_mm_big_endian_with_row_padding():
    msg = SimpleNamespace(encoding='16UC1', is_bigendian=True, width=2, height=2,
                          step=6, data=np.array([[1000, 2000, 999], [3000, 4000, 999]], dtype='>u2').tobytes())
    assert np.allclose(depth_image(msg), [[1., 2.], [3., 4.]])
