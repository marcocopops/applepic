"""Pure-Python geometry for the "magic" gripper: where it is and which apple it can grab."""
import math


def gripper_position(x, y, yaw, offset):
    """World position of a gripper mounted `offset` m ahead of the base on its x axis."""
    return x + offset * math.cos(yaw), y + offset * math.sin(yaw)


def apple_in_reach(gripper_xy, apples, radius):
    """Return the name of the closest apple within `radius` (horizontal) of the gripper.

    `apples` maps name -> (x, y). Returns None if no apple is close enough.
    """
    gx, gy = gripper_xy
    best, best_d = None, radius
    for name, (ax, ay) in apples.items():
        d = math.hypot(ax - gx, ay - gy)
        if d <= best_d:
            best, best_d = name, d
    return best
