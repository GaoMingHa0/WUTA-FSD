import numpy as np
import pytest
import yaml

from detection_fusion.calibration import load_calibration
from detection_fusion.core import transform_matrix


def test_camera_parent_lidar_child_preserves_transform_direction(tmp_path):
    path = tmp_path / 'calibration.yaml'
    rotation = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    values = dict(schema_version=1, camera_frame='camera_optical', lidar_frame='rslidar',
                  transform_camera_from_lidar=dict(rotation=rotation, translation_m=[.1, .2, .3]))
    path.write_text(yaml.safe_dump(values))
    camera, lidar, translation, quaternion = load_calibration(path)
    assert (camera, lidar) == ('camera_optical', 'rslidar')
    matrix = transform_matrix(translation, quaternion)
    assert np.allclose(matrix @ [2, 3, 4, 1], [-2.9, -3.8, 2.3, 1])
    values['transform_camera_from_lidar']['rotation'][0][0] = 1
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError):
        load_calibration(path)
