"""Pure-Python approach controller: bring a gripper mounted ahead of the base onto a point.

The gripper sits `offset` m ahead of the base on its x axis, so it reaches the target when the
robot faces the target at a distance `offset`. The robot rotates in place until roughly aligned,
then creeps forward (steering) and stops `offset` m short; it backs up if it overshot.
"""
import math
from dataclasses import dataclass

from applepic_navigation.controller import wrap_angle


@dataclass
class ApproachParams:
    offset: float = 0.46           # m, gripper ahead of base_footprint
    max_linear: float = 0.25       # m/s
    min_linear: float = 0.04       # m/s
    max_angular: float = 0.8       # rad/s
    k_linear: float = 0.8          # 1/s
    k_angular: float = 2.0         # 1/s
    rotate_threshold: float = 0.2  # rad: above this, rotate in place
    tolerance: float = 0.03        # m, gripper-to-target distance considered reached


def _clip(v, limit):
    return max(-limit, min(limit, v))


def gripper_error(x, y, yaw, tx, ty, offset):
    """Distance between the gripper and the target point."""
    gx, gy = x + offset * math.cos(yaw), y + offset * math.sin(yaw)
    return math.hypot(tx - gx, ty - gy)


def approach_command(x, y, yaw, tx, ty, params=ApproachParams()):
    """Return (linear, angular, reached) to bring the gripper onto the target (tx, ty)."""
    if gripper_error(x, y, yaw, tx, ty, params.offset) < params.tolerance:
        return 0.0, 0.0, True
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    along = dist - params.offset            # remaining travel; < 0 means overshoot
    err = wrap_angle(math.atan2(dy, dx) - yaw)
    if dist < 0.5 * params.offset:
        # Target is under the chassis (should not happen): just back up straight.
        return -params.min_linear, 0.0, False
    angular = _clip(params.k_angular * err, params.max_angular)
    if abs(err) > params.rotate_threshold:
        return 0.0, angular, False
    speed = min(params.max_linear, params.k_linear * abs(along))
    speed = max(speed, params.min_linear)
    linear = math.copysign(speed, along) * math.cos(err)
    return linear, angular, False
