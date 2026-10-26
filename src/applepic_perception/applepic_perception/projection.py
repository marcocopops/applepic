"""Pixel -> ground projection under a flat-ground assumption (pure numpy, no ROS).

Camera convention: ROS optical frame (z forward, x right, y down) and pinhole intrinsics K.
The camera pose is given as a rotation matrix R and translation t mapping optical-frame
points to the world frame: p_world = R @ p_cam + t.
"""
import numpy as np


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    """Rotation matrix of a unit quaternion (x, y, z, w)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def pixel_to_ray(u, v, K) -> np.ndarray:
    """Direction (unnormalized, z = 1) in the optical frame of the ray through pixel (u, v)."""
    K = np.asarray(K, dtype=float).reshape(3, 3)
    return np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])


def project_pixel_to_plane(u, v, K, R, t, plane_z=0.0, max_range=None):
    """World (x, y) where the ray through pixel (u, v) hits the plane z = plane_z.

    Returns None if the ray points away from the plane (above the horizon) or if the
    hit is farther than max_range (horizontal distance from the camera).
    """
    R = np.asarray(R, dtype=float)
    t = np.asarray(t, dtype=float)
    d = R @ pixel_to_ray(u, v, K)
    if abs(d[2]) < 1e-9:
        return None
    s = (plane_z - t[2]) / d[2]
    if s <= 0:
        return None
    hit = t + s * d
    if max_range is not None and np.hypot(hit[0] - t[0], hit[1] - t[1]) > max_range:
        return None
    return float(hit[0]), float(hit[1])


def project_point_to_pixel(p_world, K, R, t):
    """Inverse model: pixel (u, v) of a world point, or None if behind the camera."""
    K = np.asarray(K, dtype=float).reshape(3, 3)
    p_cam = np.asarray(R, dtype=float).T @ (np.asarray(p_world, dtype=float) - np.asarray(t))
    if p_cam[2] <= 0:
        return None
    uvw = K @ p_cam
    return float(uvw[0] / uvw[2]), float(uvw[1] / uvw[2])
