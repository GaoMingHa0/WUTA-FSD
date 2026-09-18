"""Validate the camera-from-LiDAR convention before publishing static TF."""
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def load_calibration(path):
    with open(path, encoding='utf-8') as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict) or values.get('schema_version') != 1:
        raise ValueError('Calibration schema_version must be 1')
    for name in ('lidar_frame', 'camera_frame'):
        frame = values.get(name)
        if not isinstance(frame, str) or not frame or frame.startswith('/'):
            raise ValueError(name + ' must be a nonempty TF frame without leading slash')
    if values['lidar_frame'] == values['camera_frame']:
        raise ValueError('Camera and LiDAR frames must differ')
    transform = values['transform_camera_from_lidar']
    rotation = np.asarray(transform['rotation'], dtype=float)
    translation = np.asarray(transform['translation_m'], dtype=float)
    if rotation.shape != (3, 3) or translation.shape != (3,) or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise ValueError('Calibration must contain finite 3x3 rotation and 3-vector translation')
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError('Calibration rotation must be orthonormal with determinant +1')
    # TF parent camera, child lidar stores exactly p_camera = R p_lidar + t.
    return values['camera_frame'], values['lidar_frame'], translation, Rotation.from_matrix(rotation).as_quat()
