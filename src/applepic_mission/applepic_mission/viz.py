"""Pure helpers for the demo visualisation (no rclpy): trajectory decimation, scene, HUD text."""
import math


class PathRecorder:
    """Keeps a decimated trajectory: a point is stored once it is `min_step` m from the last one."""

    def __init__(self, min_step=0.05, max_points=20000):
        self.min_step = min_step
        self.max_points = max_points
        self.points = []

    def add(self, x, y):
        if self.points:
            lx, ly = self.points[-1]
            if math.hypot(x - lx, y - ly) < self.min_step:
                return False
        self.points.append((x, y))
        if len(self.points) > self.max_points:
            # Drop every other point of the oldest half to bound memory.
            half = len(self.points) // 2
            self.points = self.points[:half:2] + self.points[half:]
        return True


def scene_items(config, search_radius):
    """Static scene derived from the orchard config, as plain dicts.

    Each item: kind ('trunk' | 'canopy' | 'search_ring' | 'depot' | 'label'), x, y and, depending
    on the kind, radius, size or text.
    """
    tree_cfg = config.get('tree', {})
    trunk_r = float(tree_cfg.get('trunk_radius', 0.1))
    canopy_r = float(tree_cfg.get('canopy_radius', 1.0))
    items = []
    for i, (tx, ty) in enumerate(config['trees']):
        items.append({'kind': 'trunk', 'x': tx, 'y': ty, 'radius': trunk_r})
        items.append({'kind': 'canopy', 'x': tx, 'y': ty, 'radius': canopy_r})
        items.append({'kind': 'search_ring', 'x': tx, 'y': ty, 'radius': search_radius})
        items.append({'kind': 'label', 'x': tx, 'y': ty, 'text': f'tree {i}'})
    depot = config['depot']
    dx, dy = depot['position']
    items.append({'kind': 'depot', 'x': dx, 'y': dy, 'size': list(depot.get('size', [1.0, 1.0]))})
    items.append({'kind': 'label', 'x': dx, 'y': dy, 'text': 'depot'})
    return items


def circle_points(x, y, radius, n=48):
    """Closed polyline approximating a circle (n + 1 points, first == last)."""
    return [(x + radius * math.cos(2 * math.pi * k / n), y + radius * math.sin(2 * math.pi * k / n))
            for k in range(n + 1)]


def hud_text(stats, basket_count=None, capacity=None):
    """Multi-line status shown above the robot in RViz."""
    stats = stats or {}
    state = stats.get('state') or 'WAITING'
    lines = [f'{state} (tree {stats.get("tree", "-")})']
    if basket_count is not None:
        lines.append(f'basket {basket_count}/{capacity}' if capacity else f'basket {basket_count}')
    lines.append(f'picked {stats.get("picked", 0)}  unloaded {stats.get("unloaded", 0)}')
    lines.append(f'{stats.get("distance_m", 0.0):.0f} m  {stats.get("elapsed_s", 0.0):.0f} s')
    return '\n'.join(lines)
