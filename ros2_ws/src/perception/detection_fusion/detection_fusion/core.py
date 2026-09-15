"""ROS-independent geometry, uncertainty and one-to-one association."""

import numpy as np
from scipy.optimize import linear_sum_assignment


def transform_matrix(translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = translation
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite transform')
    return result


def covariance(values):
    matrix = np.asarray(values, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(matrix)) or not np.allclose(matrix, matrix.T):
        raise ValueError('Invalid covariance')
    if np.min(np.linalg.eigvalsh(matrix)) <= 0:
        raise ValueError('Covariance must be positive definite')
    return matrix


def project(points, projection):
    """Use rectified CameraInfo.P, including its translation column."""
    points = np.asarray(points).reshape(-1, 3)
    pixels = np.column_stack([points, np.ones(len(points))]) @ projection.T
    valid = (points[:, 2] > 0) & (pixels[:, 2] > 1e-6)
    result = np.full((len(points), 2), np.nan)
    result[valid] = pixels[valid, :2] / pixels[valid, 2, None]
    return result


def associate(lidar_camera, observations, projection, max_distance=0.8,
              mahalanobis_gate=11.345, pixel_margin=8.0,
              ambiguity_margin=0.15, lidar_sigma=0.12):
    """observations: (bbox, optional camera point, optional covariance).

    Reject ambiguous rows AND columns before assignment; a forced assignment
    must not turn two overlapping YOLO boxes into a confident color label.
    """
    points = np.asarray(lidar_camera).reshape(-1, 3)
    pixels = project(points, projection)
    cost = np.full((len(points), len(observations)), np.inf)
    for j, (bbox, camera_point, camera_cov) in enumerate(observations):
        box = np.asarray(bbox, dtype=float)
        if not np.all(np.isfinite(box)) or np.any(box[2:] <= box[:2]):
            continue
        for i, uv in enumerate(pixels):
            if not np.all(np.isfinite(uv)):
                continue
            if np.any(uv < box[:2]-pixel_margin) or np.any(uv > box[2:]+pixel_margin):
                continue
            image_cost = np.linalg.norm((uv-(box[:2]+box[2:])/2) / (box[2:]-box[:2]))
            if camera_point is None:
                cost[i, j] = image_cost
                continue
            residual = points[i] - camera_point
            if np.linalg.norm(residual) > max_distance:
                continue
            distance = float(residual @ np.linalg.solve(
                camera_cov + np.eye(3)*lidar_sigma**2, residual))
            if distance <= mahalanobis_gate:
                cost[i, j] = distance/mahalanobis_gate + 0.25*image_cost
    if not cost.size:
        return []
    rows_bad, cols_bad = set(), set()
    for axis, rejected in ((1, rows_bad), (0, cols_bad)):
        for k, values in enumerate(cost if axis == 1 else cost.T):
            finite = np.sort(values[np.isfinite(values)])
            if len(finite) > 1 and finite[1]-finite[0] < ambiguity_margin:
                rejected.add(k)
    for i in rows_bad:
        cost[i, :] = np.inf
    for j in cols_bad:
        cost[:, j] = np.inf
    # Dummy columns represent unmatched LiDAR cones.
    augmented = np.concatenate([np.where(np.isfinite(cost), cost, 1e6),
                                np.full((len(points), len(points)), 10.0)], axis=1)
    rows, cols = linear_sum_assignment(augmented)
    return [(int(i), int(j)) for i, j in zip(rows, cols)
            if j < len(observations) and np.isfinite(cost[i, j])]


def fuse_position(lidar, camera, camera_cov, lidar_sigma=0.12, max_shift=0.25):
    """Conservative covariance intersection in XY; keep LiDAR centroid Z.

    Fixed 0.5 information weights avoid assuming independent errors following
    shared motion compensation. Extra centroid bias belongs in camera_cov.
    """
    lidar = np.asarray(lidar, dtype=float)
    precision_l = np.eye(2)/lidar_sigma**2
    precision_c = np.linalg.inv(camera_cov[:2, :2])
    fused = np.linalg.solve(precision_l+precision_c,
                            precision_l@lidar[:2] + precision_c@camera[:2])
    if np.linalg.norm(fused-lidar[:2]) > max_shift:
        return lidar.copy()
    return np.array([fused[0], fused[1], lidar[2]])
