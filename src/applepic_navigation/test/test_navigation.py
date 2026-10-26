import math
import os

import pytest

from applepic_navigation.controller import (
    ControllerParams, compute_command, wrap_angle, yaw_from_quaternion)
from applepic_navigation.route import (
    approach_points, build_route, load_orchard, segment_clearance)

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
