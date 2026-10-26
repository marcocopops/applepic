import math

import pytest

from applepic_mission.basket import Basket
from applepic_mission.mission import (
    DONE, GOTO_BASE, GOTO_TREE, PICK, SEARCH, UNLOAD, Mission, MissionParams)
from applepic_mission.picking import apple_in_reach, gripper_position
from applepic_navigation.controller import wrap_angle

TREES = [(4.0, 4.0), (10.0, 4.0)]
DEPOT = (0.0, 0.0)


def make_mission(**kw):
    return Mission(TREES, DEPOT, MissionParams(**kw))


def at_search_point(m, tree=0, index=4):
    """Put the mission in SEARCH at a given search point, facing the trunk."""
    x, y = m.search_point(tree, index)
    yaw = math.atan2(TREES[tree][1] - y, TREES[tree][0] - x)
    m.step((x, y, yaw), 0.0)            # GOTO_TREE to the closest point = here -> SEARCH
    assert m.state == SEARCH
    return x, y, yaw


def confirm(m, xy, n=2):
    for i in range(n):
        m.on_detections([xy], stamp=float(i))


def test_starts_going_to_closest_search_point_of_first_tree():
    m = make_mission()
    v, w = m.step((0.0, 0.0, 0.0), 0.0)
    assert m.state == GOTO_TREE and m.tree == 0
    goal = m.path[-1]
    assert math.dist(goal, TREES[0]) == pytest.approx(2.2)
    assert goal == m.search_point(0, 5)   # 225 deg: the side facing the depot


def test_arrival_starts_search_then_faces_and_looks():
    m = make_mission(look_time=1.0)
    x, y, _ = at_search_point(m)
    v, w = m.step((x, y, math.pi), 0.2)  # facing -x while the trunk is at +x: rotate
    assert m.phase == 'face' and v == 0.0 and abs(w) > 0
    yaw = math.atan2(TREES[0][1] - y, TREES[0][0] - x)
    m.step((x, y, yaw), 0.3)
    assert m.phase == 'look'
    m.step((x, y, yaw), 1.0)
    assert m.k == 0
    m.step((x, y, yaw), 1.4)             # look time over -> next search point
    assert m.state == SEARCH and m.k == 1 and m.phase == 'move'


def test_confirmed_apple_near_tree_triggers_pick_then_resumes_search():
    m = make_mission()
    x, y, yaw = at_search_point(m)       # index 4: west of the trunk, facing east
    apple = (TREES[0][0] - 1.0, TREES[0][1] + 0.2)
    confirm(m, apple, n=1)
    m.step((x, y, yaw), 0.2)
    assert m.state == SEARCH             # one view is not enough
    confirm(m, apple)
    v, w = m.step((x, y, yaw), 0.3)
    assert m.state == PICK and v > 0
    m.on_picked(apple)
    assert m.state == SEARCH and m.k == 0 and m.phase == 'move'
    assert m.stats.picked == 1 and not m.tracker.confirmed()


def test_apples_of_other_trees_are_ignored():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    confirm(m, (x + 0.8, y - 2.5))       # > apple_radius from trunk 0
    m.step((x, y, yaw), 0.2)
    assert m.state == SEARCH


def test_apple_behind_trunk_is_not_approached():
    m = make_mission()
    x, y, yaw = at_search_point(m)       # (1.8, 4.0) facing the trunk at (4, 4)
    confirm(m, (4.5, 4.0))               # straight approach would drive through the trunk
    m.step((x, y, yaw), 0.2)
    assert m.state == SEARCH
    assert not m.approach_feasible(4.5, 4.0)
    assert m.approach_feasible(4.0 - 0.6, 4.0)


def test_pick_timeout_marks_failed():
    m = make_mission(pick_timeout=2.0)
    x, y, yaw = at_search_point(m)
    apple = (x + 0.46 * math.cos(yaw), y + 0.46 * math.sin(yaw))   # already under the gripper
    confirm(m, apple)
    m.step((x, y, yaw), 1.0)
    assert m.state == PICK
    m.step((x, y, yaw), 1.1)             # reached
    m.step((x, y, yaw), 3.5)             # never picked
    assert m.state == SEARCH and m.stats.failed == 1 and not m.tracker.confirmed()


def test_basket_full_goes_to_base_unloads_and_returns_to_same_tree():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    m.on_basket_full(True)
    m.step((x, y, yaw), 0.5)
    assert m.state == GOTO_BASE and m.resume_tree
    m.step((0.0, 0.0, 0.0), 1.0)
    assert m.state == UNLOAD and m.unload_requested
    m.on_unloaded(10)
    assert m.state == GOTO_TREE and m.tree == 0 and not m.unload_requested
    assert m.stats.unloaded == 10 and m.stats.loops_done == 0


def test_end_of_loop_goes_to_base_then_done():
    m = make_mission(loops=1, search_points=1, look_time=0.5)
    pose, t = (0.0, 0.0, 0.0), 0.0
    while m.state != GOTO_BASE and t < 100.0:
        if m.path:                       # teleport along the planned path
            pose = (*m.path[0], pose[2])
        elif m.state == SEARCH and m.phase == 'face':
            tx, ty = TREES[m.tree]
            pose = (pose[0], pose[1], math.atan2(ty - pose[1], tx - pose[0]))
        m.step(pose, t)
        t += 0.1
    assert m.state == GOTO_BASE and not m.resume_tree
    assert m.stats.trees_searched == len(TREES)
    assert [s for s, _, _ in m.stats.transitions] == [
        GOTO_TREE, SEARCH, GOTO_TREE, SEARCH, GOTO_BASE]
    while m.path:
        m.step((*m.path[0], 0.0), t)
        t += 0.1
    m.step((0.0, 0.0, 0.0), t)
    assert m.state == UNLOAD
    m.on_unloaded(0)
    assert m.state == DONE and m.stats.loops_done == 1
    assert m.step((0.0, 0.0, 0.0), t + 1.0) == (0.0, 0.0)


def test_new_loop_when_loops_not_reached():
    m = make_mission(loops=2)
    m.step((0.0, 0.0, 0.0), 0.0)
    m._enter_goto_base(resume=False)
    m.step((0.0, 0.0, 0.0), 0.1)
    m.on_unloaded(0)
    assert m.state == GOTO_TREE and m.tree == 0 and m.stats.loops_done == 1


def _visible(pose, apple):
    """Crude camera footprint: 0.4-2.3 m ahead, widening with distance."""
    x, y, yaw = pose
    dx, dy = apple[0] - x, apple[1] - y
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return 0.4 <= fwd <= 2.3 and abs(lat) <= 0.6 * (fwd + 0.3)


def run_world(apples, trees=TREES, capacity=10, max_time=1500.0, dt=0.1):
    """Unicycle robot + fake detector + magic gripper driven by the mission until DONE."""
    m = Mission(trees, DEPOT, MissionParams(loops=1))
    basket = Basket(capacity)
    remaining = dict(apples)
    pose, t, dist, min_trunk = (0.0, 0.0, 0.0), 0.0, 0.0, math.inf
    while m.state != DONE and t < max_time:
        m.on_detections([xy for xy in remaining.values() if _visible(pose, xy)], t)
        v, w = m.step(pose, t)
        if m.unload_requested:
            m.on_unloaded(basket.unload())
            m.on_basket_full(False)
        x, y, yaw = pose
        pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, wrap_angle(yaw + w * dt))
        dist += abs(v) * dt
        min_trunk = min(min_trunk, min(math.dist(pose[:2], tr) for tr in trees))
        g = gripper_position(*pose, 0.46)
        name = apple_in_reach(g, remaining, 0.07)
        if name is not None and not basket.full:
            del remaining[name]
            basket.add()
            m.on_picked(g)
            m.on_basket_full(basket.full)
        t += dt
    return m, basket, remaining, t, min_trunk


def ring_apples(trees, per_tree=6, seed=0):
    import random
    rng = random.Random(seed)
    apples = {}
    for i, (tx, ty) in enumerate(trees):
        for j in range(per_tree):
            r, a = rng.uniform(0.45, 1.6), rng.uniform(0, 2 * math.pi)
            apples[f'apple_t{i}_{j}'] = (tx + r * math.cos(a), ty + r * math.sin(a))
    return apples


def test_full_mission_picks_and_unloads_all_apples():
    apples = ring_apples(TREES, per_tree=6)
    m, basket, remaining, t, min_trunk = run_world(apples, capacity=20)
    assert m.state == DONE, f'stuck in {m.state} at t={t:.0f}'
    assert not remaining
    assert basket.total_unloaded == len(apples) == m.stats.unloaded
    assert min_trunk > 0.44


def test_full_mission_with_small_basket_unloads_mid_loop():
    apples = ring_apples(TREES, per_tree=6, seed=3)
    m, basket, remaining, t, _ = run_world(apples, capacity=4)
    assert m.state == DONE, f'stuck in {m.state} at t={t:.0f}'
    assert not remaining and basket.total_unloaded == len(apples)
    unloads = [s for s, _, _ in m.stats.transitions if s == UNLOAD]
    assert len(unloads) >= 3


def test_full_mission_on_generated_orchard():
    import os
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', '..', 'applepic_gazebo', 'worlds',
                        'orchard_apples.yaml')
    with open(path) as f:
        manifest = yaml.safe_load(f)
    trees = [tuple(t) for t in manifest['trees']]
    apples = {a['name']: (a['x'], a['y']) for a in manifest['apples']}
    m, basket, remaining, t, min_trunk = run_world(apples, trees=trees, capacity=10,
                                                    max_time=3000.0)
    assert m.state == DONE, f'stuck in {m.state} at t={t:.0f}'
    assert not remaining and basket.total_unloaded == len(apples)
    assert min_trunk > 0.44
