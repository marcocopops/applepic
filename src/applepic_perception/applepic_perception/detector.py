"""HSV color segmentation of red apples (pure OpenCV/numpy, no ROS)."""
from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass
class HsvParams:
    # Red wraps around hue 0 in OpenCV's 0-180 hue scale, hence two hue bands.
    # Rendered colors: ripe 0, orange (unripe) 12, tree trunks (brown) 18, which must stay outside.
    hue_bands: tuple = ((0, 14), (170, 180))
    sat_min: int = 100
    val_min: int = 50
    min_area: float = 30.0       # px^2, rejects noise and far-away specks
    max_area: float = 60000.0    # px^2
    min_fill: float = 0.4        # blob area / enclosing circle area
    open_kernel: int = 3         # morphological opening kernel size (px), 0 disables it
    close_kernel: int = 7        # morphological closing (px): joins an apple cut by a grass blade
    border_margin: int = 1       # blobs touching the image border are cut: centroid unreliable


@dataclass
class Detection:
    u: float          # centroid column (px)
    v: float          # centroid row (px)
    radius: float     # radius of the minimum enclosing circle (px)
    area: float       # contour area (px^2)
    bbox: tuple = field(default=(0, 0, 0, 0))  # x, y, w, h


def red_mask(bgr: np.ndarray, params: HsvParams = HsvParams()) -> np.ndarray:
    """Binary mask (uint8, 0/255) of red pixels."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in params.hue_bands:
        mask |= cv2.inRange(hsv, (lo, params.sat_min, params.val_min), (hi, 255, 255))
    if params.open_kernel > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.open_kernel, params.open_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if params.close_kernel > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.close_kernel, params.close_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_apples(bgr: np.ndarray, params: HsvParams = HsvParams()) -> list:
    """Return red blobs as Detection objects, sorted from bottom of image (closest) to top."""
    mask = red_mask(bgr, params)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < params.min_area or area > params.max_area:
            continue
        (_, _), radius = cv2.minEnclosingCircle(contour)
        if radius <= 0 or area / (math.pi * radius * radius) < params.min_fill:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        b = params.border_margin
        if b > 0 and (x < b or y < b or x + w > mask.shape[1] - b or y + h > mask.shape[0] - b):
            continue
        m = cv2.moments(contour)
        if m['m00'] == 0:
            continue
        detections.append(Detection(
            u=m['m10'] / m['m00'], v=m['m01'] / m['m00'], radius=radius, area=area,
            bbox=(x, y, w, h)))
    detections.sort(key=lambda d: -d.v)
    return detections


def annotate(bgr: np.ndarray, detections, labels=None) -> np.ndarray:
    """Copy of the image with detection circles (and optional text labels) drawn."""
    out = bgr.copy()
    for i, d in enumerate(detections):
        center = (int(round(d.u)), int(round(d.v)))
        cv2.circle(out, center, int(round(d.radius)) + 2, (255, 255, 0), 2)
        cv2.drawMarker(out, center, (255, 255, 0), cv2.MARKER_CROSS, 8, 1)
        if labels is not None and i < len(labels) and labels[i]:
            cv2.putText(out, labels[i], (center[0] + int(d.radius) + 4, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return out
