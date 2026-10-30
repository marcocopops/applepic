"""Tree trunk foot detection: position of a trunk axis on the ground (pure OpenCV/numpy, no ROS).

The camera looks down: only the lower part of a trunk is in the image, usually cut by the top
border. The lower edge of the brown blob is the trunk foot, a circle of known radius on the ground.
Each edge pixel is projected on the ground plane and a circle of that radius is fitted to them, which
gives the axis without the bias of a blob centroid (perspective, viewing from the side).
"""
from dataclasses import dataclass

import cv2
import numpy as np

from applepic_perception.projection import project_pixel_to_plane


@dataclass
class TrunkParams:
    hue_band: tuple = (12, 25)   # brown; apples are below, grass above
    sat_band: tuple = (80, 200)
    val_band: tuple = (40, 130)
    open_kernel: int = 5         # px
    radius: float = 0.10         # m, trunk radius
    min_width: int = 15          # px, narrower blobs give too short an arc
    trim: float = 0.1            # fraction of the arc dropped on each side (silhouette edges)
    max_residual: float = 0.01   # m, RMS distance of the edge points to the fitted circle
    fit_rounds: int = 6          # fits, dropping the worst points between them
    min_inliers: float = 0.4     # fraction of the edge points the final circle must explain


def trunk_mask(bgr: np.ndarray, params: TrunkParams = TrunkParams()) -> np.ndarray:
    """Binary mask (uint8, 0/255) of trunk-colored pixels."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    p = params
    mask = cv2.inRange(hsv, (p.hue_band[0], p.sat_band[0], p.val_band[0]),
                       (p.hue_band[1], p.sat_band[1], p.val_band[1]))
    if p.open_kernel > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p.open_kernel, p.open_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def fit_circle_known_radius(points, radius, viewpoint, iterations=10):
    """Center of the circle of `radius` best fitting `points` (seen from `viewpoint`, N x 2 array).

    Returns (center, RMS residual). Gauss-Newton on the radial distances, started behind the arc.
    """
    points = np.asarray(points, dtype=float)
    mean = points.mean(axis=0)
    away = mean - np.asarray(viewpoint, dtype=float)
    center = mean + radius * away / max(np.linalg.norm(away), 1e-9)
    for _ in range(iterations):
        delta = points - center
        dist = np.maximum(np.linalg.norm(delta, axis=1), 1e-9)
        J = -delta / dist[:, None]
        step, *_ = np.linalg.lstsq(J, radius - dist, rcond=None)
        center = center + step
    res = np.linalg.norm(points - center, axis=1) - radius
    return center, float(np.sqrt(np.mean(res ** 2)))


def _fit_foot(ground, viewpoint, params):
    """Circle center of the foot arc, or None. Seen off-axis the trunk leans in the image, so that
    some columns have the silhouette as lower edge: these points, off the circle, are dropped."""
    points = ground
    for _ in range(params.fit_rounds):
        center, residual = fit_circle_known_radius(points, params.radius, viewpoint)
        if residual <= params.max_residual:
            return center if len(points) >= params.min_inliers * len(ground) else None
        errors = np.abs(np.linalg.norm(points - center, axis=1) - params.radius)
        points = points[errors <= np.quantile(errors, 0.8)]
        if len(points) < 5:
            return None
    return None


def detect_trunks(bgr, K, R, t, params: TrunkParams = TrunkParams()) -> list:
    """Trunk axes (x, y) on the plane z = 0 of the frame of the camera pose (R, t).

    Blobs cut by the side or bottom image borders are rejected (partial arc), as are those whose
    lower edge does not fit a circle of the trunk radius (e.g. an apple in front of the foot).
    """
    mask = trunk_mask(bgr, params)
    height, width = mask.shape
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    trunks = []
    for i in range(1, count):
        x, y, w, h, _ = stats[i]
        if w < params.min_width or x == 0 or x + w >= width or y + h >= height:
            continue
        blob = labels[y:y + h, x:x + w] == i
        columns = np.nonzero(blob.any(axis=0))[0]
        # Lowest blob pixel of each column (argmax on the flipped column finds the last True).
        rows = h - 1 - np.argmax(blob[::-1, columns], axis=0)
        drop = int(len(columns) * params.trim)
        columns, rows = columns[drop:len(columns) - drop], rows[drop:len(columns) - drop]
        keep = y + rows > 0       # an edge on the top border is the image limit, not the foot
        if keep.sum() < params.min_width * (1.0 - 2.0 * params.trim):
            continue
        ground = [project_pixel_to_plane(x + c, y + r + 0.5, K, R, t)
                  for c, r in zip(columns[keep], rows[keep])]
        if any(g is None for g in ground):
            continue
        center = _fit_foot(np.array(ground), t[:2], params)
        if center is not None:
            trunks.append((float(center[0]), float(center[1])))
    return trunks

