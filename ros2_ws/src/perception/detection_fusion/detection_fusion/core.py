"""ROS-independent geometry, uncertainty and one-to-one association."""

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree


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

    # A mutual row/column minimum is an unambiguous one-to-one match. Resolve
    # those directly and run Hungarian only on the remaining conflict subset.
    row_best = np.argmin(cost, axis=1)
    col_best = np.argmin(cost, axis=0)
    row_valid = np.any(np.isfinite(cost), axis=1)
    col_valid = np.any(np.isfinite(cost), axis=0)
    row_degree = np.sum(np.isfinite(cost), axis=1)
    col_degree = np.sum(np.isfinite(cost), axis=0)
    matches = []
    used_rows, used_cols = set(), set()
    for i, j in enumerate(row_best):
        j = int(j)
        if (row_valid[i] and col_valid[j] and row_degree[i] == 1 and
                col_degree[j] == 1 and col_best[j] == i):
            matches.append((int(i), j))
            used_rows.add(int(i))
            used_cols.add(j)

    remaining_rows = [i for i in range(len(points)) if i not in used_rows]
    remaining_cols = [j for j in range(len(observations)) if j not in used_cols]
    if remaining_rows and remaining_cols:
        residual = cost[np.ix_(remaining_rows, remaining_cols)]
        active_rows = np.flatnonzero(np.any(np.isfinite(residual), axis=1))
        active_cols = np.flatnonzero(np.any(np.isfinite(residual), axis=0))
        if len(active_rows) and len(active_cols):
            reduced = residual[np.ix_(active_rows, active_cols)]
            # Dummy columns represent unmatched LiDAR cones.
            augmented = np.concatenate([np.where(np.isfinite(reduced), reduced, 1e6),
                                        np.full((len(active_rows), len(active_rows)), 10.0)], axis=1)
            rows, cols = linear_sum_assignment(augmented)
            for row, col in zip(rows, cols):
                if col < len(active_cols) and np.isfinite(reduced[row, col]):
                    matches.append((remaining_rows[active_rows[row]],
                                    remaining_cols[active_cols[col]]))
    return sorted(matches)


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


def guided_cluster(points_lidar, lidar_to_camera, projection, bbox, camera_point=None,
                   voxel_size=0.05, cluster_tolerance=0.15, depth_tolerance=0.4,
                   min_cluster_size=5, max_cluster_size=200,
                   max_width=0.5, min_height=0.08, max_height=0.7):
    """Extract a real 3D point cluster inside a camera box and depth layer."""
    points = np.asarray(points_lidar, dtype=float).reshape(-1, 3)
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if not len(points):
        return None
    camera = points @ lidar_to_camera[:3, :3].T + lidar_to_camera[:3, 3]
    pixels = project(camera, projection)
    box = np.asarray(bbox, dtype=float)
    inside = ((camera[:, 2] > 0.3) & np.all(np.isfinite(pixels), axis=1)
              & (pixels[:, 0] >= box[0]) & (pixels[:, 0] <= box[2])
              & (pixels[:, 1] >= box[1]) & (pixels[:, 1] <= box[3]))
    reference = None
    if camera_point is not None:
        reference = np.asarray(camera_point, dtype=float)
        if reference.shape != (3,) or not np.all(np.isfinite(reference)) or reference[2] <= 0:
            reference = None
        else:
            inside &= np.abs(camera[:, 2] - reference[2]) <= depth_tolerance
    selected = points[inside]
    if len(selected) < min_cluster_size:
        return None

    # Voxel centroids preserve geometry better than taking an arbitrary point.
    keys = np.floor(selected / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    voxels = np.column_stack([
        np.bincount(inverse, weights=selected[:, axis]) / counts for axis in range(3)])
    if len(voxels) < min_cluster_size:
        return None

    pairs = cKDTree(voxels).query_pairs(cluster_tolerance)
    parents = np.arange(len(voxels))

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for first, second in pairs:
        first, second = root(first), root(second)
        if first != second:
            parents[second] = first
    groups = {}
    for index in range(len(voxels)):
        groups.setdefault(root(index), []).append(index)

    candidates = []
    for indices in groups.values():
        cluster = voxels[indices]
        dimensions = np.ptp(cluster, axis=0)
        if (min_cluster_size <= len(cluster) <= max_cluster_size
                and dimensions[0] < max_width and dimensions[1] < max_width
                and min_height < dimensions[2] < max_height):
            centre = cluster.mean(axis=0)
            camera_centre = centre @ lidar_to_camera[:3, :3].T + lidar_to_camera[:3, 3]
            score = (np.linalg.norm(camera_centre-reference) if reference is not None
                     else camera_centre[2])
            candidates.append((score, -len(cluster), centre))
    return min(candidates, key=lambda item: (item[0], item[1]))[2] if candidates else None
