import math
import os

import pytest

from applepic_navigation.controller import (
    ControllerParams, compute_command, wrap_angle, yaw_from_quaternion)
from applepic_navigation.route import (
    approach_points, build_route, load_orchard, plan_path, segment_clearance)

CONFIG = os.path.join(
    os.path.dirname(__file__), '..', '..', 'applepic_gazebo', 'config', 'orchard.yaml')


@pytest.fixture
def config():
    return load_orchard(CONFIG)


def simulate(route, start=(0.0, 0.0, 0.0), dt=0.05, max_time=600.0, params=ControllerParams()):
    """Integrate a unicycle model driven by the controller; return (trajectory, reached)."""
    x, y, yaw = start
    traj, reached, i, t = [(x, y)], [], 0, 0.0
    while i < len(route) and t < max_time:
        v, w, ok = compute_command(x, y, yaw, route[i][0], route[i][1], params)
        if ok:
            reached.append(route[i][2])
            i += 1
            continue
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        traj.append((x, y))
        t += dt
    return traj, reached


def test_wrap_and_yaw():
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(0.5) == pytest.approx(0.5)
    s = math.sin(0.7 / 2)
    assert yaw_from_quaternion(0, 0, s, math.cos(0.7 / 2)) == pytest.approx(0.7)


def test_rotates_in_place_when_goal_behind():
    v, w, ok = compute_command(0, 0, 0, -5, 0.1)
    assert not ok and v == 0.0 and abs(w) > 0


def test_drives_forward_and_stops_at_goal():
    v, w, ok = compute_command(0, 0, 0, 5, 0)
    assert not ok and v == pytest.approx(0.5) and w == pytest.approx(0.0)
    assert compute_command(1.0, 0.05, 0, 1.0, 0.0) == (0.0, 0.0, True)


def test_approach_points_distance(config):
    trees = config['trees']
    pts = approach_points(trees, 2.2)
    for (tx, ty), p in zip(trees, pts):
        assert math.dist((tx, ty), p) == pytest.approx(2.2)


def test_route_order_and_clearance(config):
    route = build_route(config)
    labels = [lbl for _, _, lbl in route if lbl != 'via']
    assert labels == [f'tree_{i}' for i in range(len(config['trees']))] + ['depot']
    trees = [tuple(t) for t in config['trees']]
    pts = [tuple(config['depot']['position'])] + [(x, y) for x, y, _ in route]
    for a, b in zip(pts, pts[1:]):
        assert segment_clearance(a, b, trees) >= 2.0 - 1e-6


def test_detour_when_segment_crosses_trunk():
    cfg = {'trees': [[5.0, 0.0], [10.0, 0.0]], 'depot': {'position': [0.0, 0.0]}}
    route = build_route(cfg, approach_distance=2.2, clearance=2.0, start=(0.0, 0.0))
    assert any(lbl == 'via' for _, _, lbl in route)
    pts = [(0.0, 0.0)] + [(x, y) for x, y, _ in route]
    for a, b in zip(pts, pts[1:]):
        assert segment_clearance(a, b, [(5.0, 0.0), (10.0, 0.0)]) >= 2.0 - 1e-6


def test_unicycle_completes_loop_without_hitting_trees(config):
    route = build_route(config)
    traj, reached = simulate(route)
    assert reached == [lbl for _, _, lbl in route]
    trees = [tuple(t) for t in config['trees']]
    min_d = min(math.dist(p, t) for p in traj for t in trees)
    assert min_d > 1.8  # controller overshoot stays small


TREES = [(4, 4), (10, 4), (16, 4), (16, 10), (10, 10), (4, 10)]
LOG = [(10.1 + 0.3 * i, 11.75) for i in range(-3, 4)]


def test_trunk_detour_does_not_cross_the_points_just_hit():
    # Seen in simulation: 1.5 m from a trunk, the detour went straight back through the log.
    start = (10.4, 11.5)
    path = plan_path(start, (0.0, 0.0), TREES, 1.9, bumps=LOG, bump_clearance=0.7)
    points = [start] + path
    others = [o for o in LOG if math.dist(o, start) >= 0.7]
    assert min(segment_clearance(a, b, others) for a, b in zip(points, points[1:])) >= 0.7
    assert path[-1] == (0.0, 0.0)


def test_detours_kept_when_they_avoid_the_bump_points():
    start, goal = (0.0, 0.0), (8.0, 0.0)
    assert plan_path(start, goal, [], 1.9, bumps=[(4.0, 5.0)], bump_clearance=0.7) == [goal]


def test_bump_clearance_lowered_to_pass_a_narrow_gap():
    # Wall with a 1.3 m gap: free at 0.55 m, not at 0.7 m + grid margin.
    wall = [(5.0, y * 0.2) for y in range(-20, 21) if abs(y * 0.2) >= 0.65]
    path = plan_path((0.0, 0.0), (10.0, 0.0), [], 1.9, bumps=wall, bump_clearance=0.7)
    points = [(0.0, 0.0)] + path
    assert min(segment_clearance(a, b, wall) for a, b in zip(points, points[1:])) >= 0.5
    assert max(abs(y) for _, y in points) < 2.0


def test_no_way_through_falls_back_on_a_path_to_the_goal():
    ring = [(5.0 + 2.0 * math.cos(i * 0.1), 2.0 * math.sin(i * 0.1)) for i in range(63)]
    path = plan_path((0.0, 0.0), (5.0, 0.0), [], 1.9, bumps=ring, bump_clearance=0.7)
    assert path[-1] == (5.0, 0.0)
