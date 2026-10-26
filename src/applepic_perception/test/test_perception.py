import math

import cv2
import numpy as np
import pytest

from applepic_perception.detector import HsvParams, annotate, detect_apples, red_mask
from applepic_perception.projection import (
    pixel_to_ray, project_pixel_to_plane, project_point_to_pixel, quaternion_to_matrix)

GRASS_BGR = (101, 176, 116)   # sampled from a sim camera frame
APPLE_BGR = (45, 45, 179)
TRUNK_BGR = (38, 62, 78)
GRIPPER_BGR = (194, 194, 194)

# Same intrinsics as the robot camera: 640x480, HFOV 1.3 rad.
W, H = 640, 480
F = (W / 2) / math.tan(1.3 / 2)
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def robot_camera_pose(x=0.0, y=0.0, yaw=0.0, cam_x=0.35, cam_z=0.5, pitch=0.85):
    """World pose of the optical frame for a robot at (x, y, yaw), as in the xacro."""
    r_base = _rot_z(yaw)
    r_link = r_base @ _rot_y(pitch)
    r_opt = r_link @ _rot_z(-math.pi / 2) @ _rot_x(-math.pi / 2)
    t = np.array([x, y, 0.0]) + r_base @ np.array([cam_x, 0.0, cam_z])
    return r_opt, t


def _blank():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = GRASS_BGR
    return img


# --- detector ---------------------------------------------------------------

def test_detects_red_disks_and_ignores_background():
    img = _blank()
    cv2.circle(img, (100, 300), 25, APPLE_BGR, -1)
    cv2.circle(img, (500, 120), 10, APPLE_BGR, -1)
    cv2.rectangle(img, (280, 0), (360, 80), TRUNK_BGR, -1)        # brown trunk
    cv2.rectangle(img, (220, 430), (420, 480), GRIPPER_BGR, -1)   # grey gripper
    dets = detect_apples(img)
    assert len(dets) == 2
    # Sorted closest (bottom of the image) first.
    assert dets[0].u == pytest.approx(100, abs=1) and dets[0].v == pytest.approx(300, abs=1)
    assert dets[0].radius == pytest.approx(25, abs=2)
    assert dets[1].u == pytest.approx(500, abs=1) and dets[1].v == pytest.approx(120, abs=1)


def test_red_wraparound_hue_is_detected():
    img = _blank()
    cv2.circle(img, (200, 200), 20, (60, 20, 200), -1)   # hue ~ 175 (magenta-ish red)
    cv2.circle(img, (400, 200), 20, (20, 40, 200), -1)   # hue ~ 3 (orange-ish red)
    assert len(detect_apples(img)) == 2


def test_dark_shaded_apple_still_detected():
    img = _blank()
    cv2.circle(img, (320, 240), 20, (25, 25, 112), -1)
    assert len(detect_apples(img)) == 1


def test_tiny_noise_and_thin_lines_rejected():
    img = _blank()
    img[50:52, 50:52] = APPLE_BGR                       # 4 px speck
    cv2.line(img, (100, 400), (600, 400), APPLE_BGR, 3)  # long thin red line: low fill ratio
    assert detect_apples(img) == []


def test_blobs_cut_by_image_border_rejected():
    img = _blank()
    cv2.circle(img, (300, 5), 20, APPLE_BGR, -1)      # cut by the top edge
    cv2.circle(img, (635, 240), 20, APPLE_BGR, -1)    # cut by the right edge
    cv2.circle(img, (320, 240), 20, APPLE_BGR, -1)
    dets = detect_apples(img)
    assert len(dets) == 1 and dets[0].u == pytest.approx(320, abs=1)
    assert len(detect_apples(img, HsvParams(border_margin=0))) == 3


def test_mask_is_binary_and_annotate_keeps_shape():
    img = _blank()
    cv2.circle(img, (320, 240), 20, APPLE_BGR, -1)
    mask = red_mask(img)
    assert set(np.unique(mask)) <= {0, 255}
    out = annotate(img, detect_apples(img), labels=['a'])
    assert out.shape == img.shape and not np.array_equal(out, img)


def test_min_area_param():
    img = _blank()
    cv2.circle(img, (320, 240), 5, APPLE_BGR, -1)
    assert len(detect_apples(img)) == 1
    assert detect_apples(img, HsvParams(min_area=200)) == []


# --- projection -------------------------------------------------------------

def test_quaternion_to_matrix_yaw():
    a = 0.7
    q = (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))
    assert np.allclose(quaternion_to_matrix(*q), _rot_z(a))


def test_principal_ray_is_optical_axis():
    assert np.allclose(pixel_to_ray(W / 2, H / 2, K), [0, 0, 1])


def test_image_center_hits_ground_ahead():
    r, t = robot_camera_pose()
    x, y = project_pixel_to_plane(W / 2, H / 2, K, r, t, plane_z=0.0)
    assert x == pytest.approx(0.35 + 0.5 / math.tan(0.85), abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_pixels_above_horizon_and_beyond_range_rejected():
    r, t = robot_camera_pose()
    assert project_pixel_to_plane(W / 2, 0, K, r, t, max_range=1.0) is None
    # With the 0.85 rad tilt the top image row is still below the horizon; tilt the camera up.
    r_up, t_up = robot_camera_pose(pitch=-0.2)
    assert project_pixel_to_plane(W / 2, 0, K, r_up, t_up) is None


@pytest.mark.parametrize('pose', [(0.0, 0.0, 0.0), (2.6, 4.0, 0.0), (10.0, 7.0, 2.3)])
def test_point_round_trip(pose):
    r, t = robot_camera_pose(*pose)
    x0, y0, yaw = pose
    for dx, dy in [(1.0, 0.0), (1.5, 0.4), (0.8, -0.3)]:
        p = np.array([x0, y0, 0.04]) + _rot_z(yaw) @ np.array([dx, dy, 0.0])
        u, v = project_point_to_pixel(p, K, r, t)
        assert 0 <= u < W and 0 <= v < H
        x, y = project_pixel_to_plane(u, v, K, r, t, plane_z=0.04)
        assert (x, y) == pytest.approx((p[0], p[1]), abs=1e-6)


def test_synthetic_render_detect_project():
    """Draw apples where the pinhole model puts them, detect, project back to the world."""
    r, t = robot_camera_pose(2.6, 4.0, 0.0)
    apples = [(3.6, 4.0), (3.9, 4.5), (4.1, 3.7)]
    img = _blank()
    for ax, ay in apples:
        u, v = project_point_to_pixel((ax, ay, 0.04), K, r, t)
        dist = np.linalg.norm(np.array([ax, ay, 0.04]) - t)
        radius = max(2, int(round(F * 0.04 / dist)))
        cv2.circle(img, (int(round(u)), int(round(v))), radius, APPLE_BGR, -1)
    found = [project_pixel_to_plane(d.u, d.v, K, r, t, plane_z=0.04) for d in detect_apples(img)]
    assert len(found) == len(apples)
    for ax, ay in apples:
        err = min(math.hypot(ax - fx, ay - fy) for fx, fy in found)
        # Disks are drawn on integer pixels; 1 px ~ 2-3 cm on the ground at 1.7 m.
        assert err < 0.06
