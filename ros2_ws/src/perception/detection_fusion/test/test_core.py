import numpy as np
import pytest
from detection_fusion.core import associate, covariance, fuse_position, guided_cluster, project, transform_matrix

P = np.array([[700., 0., 640., 0.], [0., 700., 360., 0.], [0., 0., 1., 0.]])


def test_unique_color_only_and_behind_camera():
    boxes = [([630, 350, 650, 370], None, None)]
    assert associate([[0, 0, 10], [0, 0, -10]], boxes, P) == [(0, 0)]


def test_overlapping_boxes_and_multiple_cones_are_rejected():
    box = ([600, 330, 680, 390], None, None)
    assert associate([[0, 0, 10]], [box, box], P) == []
    assert associate([[-0.01, 0, 10], [0.01, 0, 10]], [box], P) == []


def test_depth_resolves_same_image_ray_but_rejects_wrong_segment():
    observation = ([600, 330, 680, 390], np.array([0, 0, 10]), np.eye(3)*0.01)
    assert associate([[0, 0, 10], [0, 0, 13]], [observation], P) == [(0, 0)]
    assert associate([[0, 0, 13]], [observation], P) == []


def test_invalid_covariance():
    with pytest.raises(ValueError):
        covariance(np.zeros(9))
    with pytest.raises(ValueError):
        covariance([float('nan')]*9)


def test_uncertain_camera_keeps_lidar_position_and_z():
    result = fuse_position(np.array([10., 0., .1]), np.array([10.5, .1, .3]), np.eye(3)*100)
    assert abs(result[0]-10) < .001
    assert result[2] == .1


def test_optical_frame_and_translation():
    matrix = transform_matrix([0, 0, .5], [-.5, .5, -.5, .5])
    assert np.allclose(matrix@np.array([0, .34, 10, 1]), [10, 0, .16, 1])
    assert np.allclose(np.linalg.inv(matrix)@np.array([10, 0, .16, 1]), [0, .34, 10, 1])


def test_assignment_is_one_to_one():
    observations = [([630, 350, 650, 370], None, None), ([700, 350, 720, 370], None, None)]
    assert associate([[0, 0, 10], [1, 0, 10]], observations, P) == [(0, 0), (1, 1)]


def test_conflicting_candidates_use_global_assignment():
    observations = [([523.5, 345, 756.5, 375], None, None),
                    ([508, 345, 702, 375], None, None)]
    assert associate([[0, 0, 10], [1, 0, 10]], observations, P) == [(0, 1), (1, 0)]


def test_camera_guided_cluster_uses_box_and_depth_layer():
    rng = np.random.default_rng(4)
    cone = rng.uniform([-.12, -.12, 9.8], [.12, .12, 10.2], (120, 3))
    background = rng.uniform([-2, -2, 12], [2, 2, 13], (500, 3))
    result = guided_cluster(np.vstack([cone, background]), np.eye(4), P,
                            [625, 345, 655, 375], np.array([0, 0, 10]),
                            voxel_size=.04, cluster_tolerance=.12)
    assert result is not None
    assert np.linalg.norm(result - [0, 0, 10]) < .15


def test_camera_guided_cluster_requires_real_points():
    assert guided_cluster([[2, 2, 15]], np.eye(4), P, [630, 350, 650, 370],
                          np.array([0, 0, 10])) is None
