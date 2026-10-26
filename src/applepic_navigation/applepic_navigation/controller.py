"""Pure-Python point-to-point controller for a differential-drive base (no rclpy).

Strategy: rotate in place while the heading error is large, then drive forward with a
speed proportional to the distance (scaled by cos(heading error)) while steering.
"""
import math
from dataclasses import dataclass


def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class ControllerParams:
    max_linear: float = 0.5        # m/s
    max_angular: float = 1.2       # rad/s
    k_linear: float = 0.8          # 1/s
    k_angular: float = 2.0         # 1/s
    rotate_threshold: float = 0.6  # rad: above this, rotate in place
    goal_tolerance: float = 0.15   # m


def _clip(v, limit):
    return max(-limit, min(limit, v))


def compute_command(x, y, yaw, gx, gy, params=ControllerParams()):
    """Return (linear, angular, reached) to drive the pose (x, y, yaw) to the goal (gx, gy)."""
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist < params.goal_tolerance:
        return 0.0, 0.0, True
    err = wrap_angle(math.atan2(dy, dx) - yaw)
    angular = _clip(params.k_angular * err, params.max_angular)
    if abs(err) > params.rotate_threshold:
        return 0.0, angular, False
    linear = min(params.max_linear, params.k_linear * dist) * math.cos(err)
    # Keep a small minimum speed so the robot does not crawl just outside the tolerance.
    linear = max(linear, 0.05)
    return linear, angular, False
