"""Pure-Python ground coverage of the camera around a tree (no rclpy).

`camera_footprint` projects the (slightly shrunk) image corners onto the apple plane to get the
ground quadrilateral seen by the camera, in the base frame (x forward, y left). `CoverageGrid`
discretises the annulus around a trunk where apples can lie and records which cells the camera
has looked at; the mission uses it to skip search points that would add nothing and to leave a
tree once its ground has been seen.
"""
import math
from dataclasses import dataclass


@dataclass
class CameraParams:
    x: float = 0.35          # m, camera ahead of base_footprint (URDF camera_x)
    z: float = 0.50          # m, camera height (URDF camera_z)
    pitch: float = 0.85      # rad, tilted down (URDF camera_pitch)
    hfov: float = 1.3        # rad
    width: int = 640
    height: int = 480
    plane_z: float = 0.04    # m, apple centre height
    margin: float = 0.85     # fraction of the image kept (blobs cut by the border are rejected)


def camera_footprint(cam=None):
    """Ground quadrilateral [(x, y)] * 4 seen by the camera, in the base frame (far side first)."""
    cam = cam or CameraParams()
    th = math.tan(cam.hfov / 2.0) * cam.margin
    tv = math.tan(cam.hfov / 2.0) * cam.height / cam.width * cam.margin
    cp, sp = math.cos(cam.pitch), math.sin(cam.pitch)
    quad = []
    for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1)):   # image left/right, top/bottom
        # Ray in camera_link (x forward, y left, z up), then pitched down about y.
        rx, ry, rz = 1.0, -u * th, -v * tv
        gx, gz = rx * cp + rz * sp, -rx * sp + rz * cp
        if gz >= 0.0:
            raise ValueError('image corner above the horizon: footprint unbounded')
        t = (cam.z - cam.plane_z) / -gz
        quad.append((cam.x + t * gx, t * ry))
    return quad


def point_in_convex(poly, x, y):
    sign = 0
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        c = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if c != 0.0:
            s = 1 if c > 0 else -1
            if sign and s != sign:
                return False
            sign = s
    return True


class CoverageGrid:
    """Cells of the annulus r_min..r_max around `center`, marked once seen by the camera."""

    def __init__(self, center, r_min=0.4, r_max=1.7, resolution=0.1, footprint=None,
                 trunk_radius=0.15):
        self.center = tuple(center)
        self.footprint = footprint or camera_footprint()
        self.trunk_radius = trunk_radius
        self.r_max = r_max
        self.reach = max(math.hypot(x, y) for x, y in self.footprint)
        cx, cy = self.center
        n = int(math.ceil(r_max / resolution))
        self.cells = []
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                x, y = i * resolution, j * resolution
                if r_min <= math.hypot(x, y) <= r_max:
                    self.cells.append((cx + x, cy + y))
        self.seen = [False] * len(self.cells)

    def visible(self, pose):
        """Indices of the cells inside the camera footprint at `pose` (x, y, yaw)."""
        x, y, yaw = pose
        if math.dist((x, y), self.center) > self.reach + self.r_max:
            return []
        c, s = math.cos(yaw), math.sin(yaw)
        # Cells hidden behind the trunk: farther than it, within its angular half-width.
        tdx, tdy = self.center[0] - x, self.center[1] - y
        tdist = math.hypot(tdx, tdy)
        out = []
        for k, (px, py) in enumerate(self.cells):
            dx, dy = px - x, py - y
            if dx * dx + dy * dy > self.reach * self.reach:
                continue
            if not point_in_convex(self.footprint, dx * c + dy * s, -dx * s + dy * c):
                continue
            if tdist > self.trunk_radius:
                along = (dx * tdx + dy * tdy) / tdist
                if along > tdist and abs(dx * tdy - dy * tdx) / tdist < self.trunk_radius:
                    continue
            out.append(k)
        return out

    def gain(self, pose):
        """Fraction of all cells that a view from `pose` would newly cover."""
        return sum(1 for k in self.visible(pose) if not self.seen[k]) / len(self.cells)

    def mark(self, pose):
        """Record the view from `pose`; return the fraction of cells newly covered."""
        new = 0
        for k in self.visible(pose):
            if not self.seen[k]:
                self.seen[k] = True
                new += 1
        return new / len(self.cells)

    @property
    def fraction(self):
        return sum(self.seen) / len(self.cells)
