"""Build the waypoint route around the orchard from the orchard YAML config (no rclpy).

Each tree yields one approach waypoint, offset from the trunk towards the centroid of the
tree loop (so the robot stays outside the apple ring). Segments passing too close to a trunk
get a detour waypoint. The route optionally ends on the depot.
"""
import math

import yaml


def load_orchard(path):
    with open(path) as f:
        return yaml.safe_load(f)


def approach_points(trees, distance):
    """One point per tree, `distance` metres from the trunk towards the loop centroid."""
    cx = sum(t[0] for t in trees) / len(trees)
    cy = sum(t[1] for t in trees) / len(trees)
    points = []
    for tx, ty in trees:
        dx, dy = cx - tx, cy - ty
        n = math.hypot(dx, dy)
        if n < 1e-9:
            dx, dy, n = 1.0, 0.0, 1.0
        points.append((tx + distance * dx / n, ty + distance * dy / n))
    return points


def _closest_on_segment(a, b, p):
    ax, ay = a
    vx, vy = b[0] - ax, b[1] - ay
    vv = vx * vx + vy * vy
    t = 0.0 if vv < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / vv))
    return ax + t * vx, ay + t * vy


def segment_clearance(a, b, obstacles):
    """Minimum distance from segment [a, b] to any obstacle point."""
    return min((math.dist(_closest_on_segment(a, b, o), o) for o in obstacles), default=math.inf)


def _detour(a, b, obstacles, clearance, depth):
    """Return the intermediate points to insert between a and b (excluding both)."""
    if depth <= 0:
        return []
    worst, worst_d = None, clearance
    for o in obstacles:
        d = math.dist(_closest_on_segment(a, b, o), o)
        if d < worst_d:
            worst, worst_d = o, d
    if worst is None:
        return []
    cx, cy = _closest_on_segment(a, b, worst)
    nx, ny = cx - worst[0], cy - worst[1]
    n = math.hypot(nx, ny)
    if n < 1e-6:  # segment crosses the trunk: go around on the left side of a -> b
        vx, vy = b[0] - a[0], b[1] - a[1]
        nx, ny, n = -vy, vx, math.hypot(vx, vy)
    margin = clearance * 1.25
    via = (worst[0] + margin * nx / n, worst[1] + margin * ny / n)
    return (_detour(a, via, obstacles, clearance, depth - 1) + [via]
            + _detour(via, b, obstacles, clearance, depth - 1))


def build_route(config, approach_distance=2.2, clearance=2.0, start=None, end_at_depot=True):
    """Return a list of (x, y, label) waypoints for one orchard loop.

    `start` is the robot position the route begins from (defaults to the depot); it is used
    only to insert a detour on the first segment if needed.
    """
    trees = [tuple(t) for t in config['trees']]
    depot = tuple(config['depot']['position'])
    start = tuple(start) if start is not None else depot
    goals = [(x, y, f'tree_{i}') for i, (x, y) in enumerate(approach_points(trees, approach_distance))]
    if end_at_depot:
        goals.append((depot[0], depot[1], 'depot'))
    route = []
    prev = start
    for gx, gy, label in goals:
        for vx, vy in _detour(prev, (gx, gy), trees, clearance, depth=3):
            route.append((vx, vy, 'via'))
        route.append((gx, gy, label))
        prev = (gx, gy)
    return route
