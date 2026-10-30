"""Build the waypoint route around the orchard from the orchard YAML config (no rclpy).

Each tree yields one approach waypoint, offset from the trunk towards the centroid of the
tree loop (so the robot stays outside the apple ring). Segments passing too close to a trunk
get a detour waypoint. The route optionally ends on the depot.
"""
import heapq
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


def _detour(a, b, keepouts, depth):
    """Return the intermediate points to insert between a and b (excluding both).

    `keepouts` is a list of (point, clearance): the worst violation is handled first.
    """
    if depth <= 0:
        return []
    worst, clearance, worst_gap = None, 0.0, 0.0
    for o, c in keepouts:
        gap = c - math.dist(_closest_on_segment(a, b, o), o)
        if gap > worst_gap:
            worst, clearance, worst_gap = o, c, gap
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
    return (_detour(a, via, keepouts, depth - 1) + [via]
            + _detour(via, b, keepouts, depth - 1))


def _path_clearance(points, bumps):
    return min((segment_clearance(a, b, bumps) for a, b in zip(points, points[1:])),
               default=math.inf)


def _usable_keepouts(a, b, keepouts):
    """Keepouts shrunk so that neither end of the move is inside one: the robot has to leave it."""
    return [(o, min(c, math.dist(a, o), math.dist(b, o))) for o, c in keepouts]


def _grid_search(a, b, keepouts, step, margin):
    """Shortest 8-connected path from a to b outside the keepouts, None if there is none."""
    far = margin + max((c for _, c in keepouts), default=0.0)
    x0, y0 = min(a[0], b[0]) - far, min(a[1], b[1]) - far
    nx = int((max(a[0], b[0]) + far - x0) / step) + 1
    ny = int((max(a[1], b[1]) + far - y0) / step) + 1
    blocked = set()
    for (ox, oy), c in keepouts:
        n = int(c / step) + 1
        i0, j0 = int((ox - x0) / step), int((oy - y0) / step)
        for i in range(max(i0 - n, 0), min(i0 + n + 2, nx)):
            for j in range(max(j0 - n, 0), min(j0 + n + 2, ny)):
                if math.hypot(x0 + i * step - ox, y0 + j * step - oy) < c:
                    blocked.add((i, j))
    def cell(p):
        return round((p[0] - x0) / step), round((p[1] - y0) / step)
    start, goal = cell(a), cell(b)
    blocked -= {start, goal}
    best = {start: 0.0}
    parent = {}
    heap = [(math.dist(start, goal), start)]
    while heap:
        _, u = heapq.heappop(heap)
        if u == goal:
            path = []
            while u in parent:
                u = parent[u]
                path.append((x0 + u[0] * step, y0 + u[1] * step))
            return [tuple(a)] + path[-2::-1] + [tuple(b)]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            v = (u[0] + di, u[1] + dj)
            if v in blocked or not (0 <= v[0] < nx and 0 <= v[1] < ny):
                continue
            cost = best[u] + math.hypot(di, dj)
            if cost < best.get(v, math.inf):
                best[v], parent[v] = cost, u
                heapq.heappush(heap, (cost + math.dist(v, goal), v))
    return None


def _shortcut(path, keepouts):
    """Drop the waypoints whose neighbours see each other."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and any(segment_clearance(path[i], path[j], [o]) < c for o, c in keepouts):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_around_bumps(a, b, obstacles, clearance, bumps, bump_clearance, min_clearance,
                      min_bump_clearance, step=0.2, margin=4.0):
    """Waypoints from a to b (a excluded) on a grid, keeping away from trunks and bump points; both
    clearances are lowered to the minimum ones if there is no way otherwise. None if still no way."""
    for trunk_c, bump_c in ((clearance, bump_clearance), (min_clearance, min_bump_clearance)):
        keepouts = _usable_keepouts(a, b, [(o, min(clearance, trunk_c)) for o in obstacles]
                                    + [(o, bump_c + step) for o in bumps])
        path = _grid_search(a, b, keepouts, step, margin)
        if path is not None:
            return _shortcut(path, keepouts)[1:]
    return None


def plan_path(a, b, obstacles, clearance, depth=3, bumps=(), bump_clearance=0.0,
              min_clearance=0.7, min_bump_clearance=0.55):
    """Return the waypoints from a to b (b included, a excluded) keeping `clearance` from obstacles.

    Detours are only added for obstacles the straight segment passes too close to. `bumps` are
    points hit by the robot, to keep `bump_clearance` from: if the detours do not, a grid search
    replaces them (a detour away from a trunk can lead straight back to what the robot just hit).
    """
    keepouts = [(o, clearance) for o in obstacles] + [(o, bump_clearance) for o in bumps]
    path = _detour(tuple(a), tuple(b), keepouts, depth) + [tuple(b)]
    bumps = [o for o in bumps if math.dist(a, o) >= bump_clearance
             and math.dist(b, o) >= bump_clearance]
    if bumps and _path_clearance([tuple(a)] + path, bumps) < bump_clearance:
        path = plan_around_bumps(a, b, obstacles, clearance, bumps, bump_clearance,
                                 min_clearance, min_bump_clearance) or path
    return path


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
        for vx, vy in _detour(prev, (gx, gy), [(t, clearance) for t in trees], 3):
            route.append((vx, vy, 'via'))
        route.append((gx, gy, label))
        prev = (gx, gy)
    return route
