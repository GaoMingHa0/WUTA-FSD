"""Robust registered-depth sampling for rectified YOLO bounding boxes."""
import numpy as np


def depth_image(msg):
    if msg.encoding not in ('32FC1', '16UC1'):
        raise ValueError('Depth must be 32FC1 metres or 16UC1 millimetres')
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('f4' if msg.encoding == '32FC1' else 'u2'))
    if msg.step < msg.width*dtype.itemsize or len(msg.data) < msg.height*msg.step:
        raise ValueError('Invalid depth image stride/buffer')
    values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                        strides=(msg.step, dtype.itemsize)).astype(float)
    return values * (0.001 if msg.encoding == '16UC1' else 1.0)


def estimate_position(depth, box, projection):
    """Return (point, covariance) or None for absent/heterogeneous ROI depth.

    Uses central 40% box region; assumes this region is on the cone surface.
    Does not claim to recover a cone base centre or perform raw stereo matching.
    """
    box = np.asarray(box, dtype=float)
    p = np.asarray(projection).reshape(3, 4)
    if not np.all(np.isfinite(box)) or np.any(box[2:] <= box[:2]) or p[0, 0] <= 0 or p[1, 1] <= 0:
        return None
    centre, extent = (box[:2]+box[2:])/2, (box[2:]-box[:2])*0.2
    x0, y0 = np.maximum(np.floor(centre-extent).astype(int), [0, 0])
    x1, y1 = np.minimum(np.ceil(centre+extent).astype(int), [depth.shape[1], depth.shape[0]])
    if x1 <= x0 or y1 <= y0:
        return None
    roi = depth[y0:y1, x0:x1]
    valid = np.isfinite(roi) & (roi > 0.3) & (roi < 40.)
    values = roi[valid]
    if len(values) < 6 or len(values) < 0.6*roi.size:
        return None
    z = float(np.median(values))
    spread = float(np.percentile(values, 90)-np.percentile(values, 10))
    if spread > max(0.3, z*0.08):
        return None
    # Median pixel coordinates of valid samples, rather than a padded ROI centre.
    ys, xs = np.nonzero(valid)
    u, v = float(np.median(xs)+x0), float(np.median(ys)+y0)
    fx, fy, cx, cy = p[0, 0], p[1, 1], p[0, 2], p[1, 2]
    point = np.array([((u-cx)*z-p[0, 3])/fx, ((v-cy)*z-p[1, 3])/fy, z])
    sigma_z = max(0.03+0.0015*z*z, spread/2)
    jacobian = np.array([[z/fx, 0, (u-cx)/fx], [0, z/fy, (v-cy)/fy], [0, 0, 1.]])
    cov = jacobian @ np.diag([4., 4., sigma_z**2]) @ jacobian.T + np.eye(3)*0.025**2
    return point, cov
