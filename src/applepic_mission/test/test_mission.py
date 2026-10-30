import math

import pytest

from applepic_mission.basket import Basket
from applepic_mission.coverage import CameraParams, camera_footprint, point_in_convex
from applepic_mission.mission import (
    ANCHOR, DONE, GOTO_BASE, GOTO_TREE, PICK, SEARCH, UNLOAD, Mission, MissionParams)
from applepic_mission.picking import apple_in_reach, gripper_position
from applepic_navigation.controller import wrap_angle
from applepic_navigation.route import segment_clearance

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
    assert math.dist(goal, TREES[0]) == pytest.approx(m.p.search_radius)
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
    m = make_mission(coverage=False)
    x, y, yaw = at_search_point(m)       # index 4: west of the trunk, facing east
    apple = (TREES[0][0] - 1.0, TREES[0][1] + 0.15)
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


def test_late_pick_report_retires_the_apple_it_names_not_the_target():
    # gz may confirm a removal late, while the gripper is already on the next apple.
    m = make_mission(coverage=False)
    at_search_point(m)
    a, b = (TREES[0][0] - 1.0, TREES[0][1] + 0.1), (TREES[0][0] - 0.8, TREES[0][1] - 0.6)
    confirm(m, a)
    confirm(m, b)
    m.step(m._pose, 0.3)
    assert m.state == PICK
    target = m.tracker.get(m.target)
    other = b if math.dist((target.x, target.y), a) < 0.01 else a
    m.on_picked(other)
    assert m.state == PICK and m.target == target.id and target.state == 'active'


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
            pose = (pose[0], pose[1], m.order[m.k][2])
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


def cover_all(m, tree=0):
    grid = m.grid(tree)
    grid.seen = [True] * len(grid.cells)


def test_covered_tree_is_left_right_after_arrival():
    m = make_mission()
    m.step((0.0, 0.0, 0.0), 0.0)
    cover_all(m)
    x, y = m.path[-1]
    m.step((x, y, 0.0), 0.1)                 # arrival at tree 0: nothing left to see
    assert m.state == GOTO_TREE and m.tree == 1
    assert [s for s, _, _ in m.stats.transitions] == [GOTO_TREE, SEARCH, GOTO_TREE]


def test_known_apple_keeps_the_search_going_despite_coverage():
    m = make_mission()
    x, y, yaw = at_search_point(m)           # west of the trunk
    m.on_detections([(4.5, 4.0)], 0.1)       # seen once, behind the trunk
    cover_all(m)
    m.step((x, y, yaw), 0.2)
    m.step((x, y, yaw), 1.5)                 # look over
    assert m.state == SEARCH and m.tree == 0 and m.k == 1


def test_search_points_already_seen_are_skipped():
    m = make_mission()
    x, y, yaw = at_search_point(m)           # order: 4, 5, 6, ...
    m.grid(0).mark(m.order[1])               # the next point has nothing new to show
    m.step((x, y, yaw), 0.2)
    m.step((x, y, yaw), 1.5)
    assert m.state == SEARCH and m.k == 2 and m.phase == 'move'
    assert m.path[-1] == pytest.approx(m.search_point(0, 6))


def test_fill_in_view_added_when_circle_done_but_ground_unseen():
    m = make_mission(search_points=1)
    x, y, yaw = at_search_point(m, index=0)
    m.step((x, y, yaw), 0.2)
    m.step((x, y, yaw), 1.5)                 # single point looked at: ground mostly unseen
    assert m.state == SEARCH and m.tree == 0 and m.fill_ins == 1 and m.k == 1
    fx, fy, fyaw = m.order[1]
    r = math.dist((fx, fy), TREES[0])
    assert m.p.circle_clearance < r <= m.p.search_radius + m.p.fill_in_radial + 1e-9
    assert m.grid(0).gain(m.order[1]) >= m.p.fill_in_gain


def test_coverage_resets_on_a_new_loop():
    m = make_mission(loops=2)
    m.step((0.0, 0.0, 0.0), 0.0)
    cover_all(m)
    m._enter_goto_base(resume=False)
    m.step((0.0, 0.0, 0.0), 0.1)
    m.on_unloaded(0)
    assert m.state == GOTO_TREE and m.grid(0).fraction == 0.0


def test_new_loop_when_loops_not_reached():
    m = make_mission(loops=2)
    m.step((0.0, 0.0, 0.0), 0.0)
    m._enter_goto_base(resume=False)
    m.step((0.0, 0.0, 0.0), 0.1)
    m.on_unloaded(0)
    assert m.state == GOTO_TREE and m.tree == 0 and m.stats.loops_done == 1


# Fake detector: the real camera footprint, a bit larger than the one the mission plans with.
FOOTPRINT = camera_footprint(CameraParams(margin=0.95))


def _visible(pose, apple, footprint=FOOTPRINT):
    x, y, yaw = pose
    dx, dy = apple[0] - x, apple[1] - y
    return point_in_convex(footprint, dx * math.cos(yaw) + dy * math.sin(yaw),
                           -dx * math.sin(yaw) + dy * math.cos(yaw))


def _chassis_hits(pose, crates):
    """True if the chassis rectangle overlaps one of the square crates (x, y, half side)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(x + lon * c - lat * s, y + lon * s + lat * c)
               for lon in (-0.2, 0.4) for lat in (-0.3, 0.3)]
    for cx, cy, half in crates:
        # Separating axes: those of the crate, then those of the chassis.
        if (min(p[0] for p in corners) > cx + half or max(p[0] for p in corners) < cx - half
                or min(p[1] for p in corners) > cy + half or max(p[1] for p in corners) < cy - half):
            continue
        square = [(cx + i * half, cy + j * half) for i in (-1, 1) for j in (-1, 1)]
        lons = [(px - x) * c + (py - y) * s for px, py in square]
        lats = [-(px - x) * s + (py - y) * c for px, py in square]
        if min(lons) <= 0.4 and max(lons) >= -0.2 and min(lats) <= 0.3 and max(lats) >= -0.3:
            return True
    return False


def run_world(apples, trees=TREES, capacity=10, max_time=1500.0, dt=0.1, footprint=FOOTPRINT,
              crates=(), **params):
    """Unicycle robot + fake detector + magic gripper driven by the mission until DONE.

    `crates` are unknown to the mission: they block the robot, which only feels the bump.
    """
    m = Mission(trees, DEPOT, MissionParams(loops=1, **params))
    basket = Basket(capacity)
    remaining = dict(apples)
    pose, t, dist, min_trunk = (0.0, 0.0, 0.0), 0.0, 0.0, math.inf
    while m.state != DONE and t < max_time:
        m.on_detections([xy for xy in remaining.values() if _visible(pose, xy, footprint)], t)
        v, w = m.step(pose, t)
        if m.unload_requested:
            m.on_unloaded(basket.unload())
            m.on_basket_full(False)
        x, y, yaw = pose
        moved = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, wrap_angle(yaw + w * dt))
        if _chassis_hits(moved, crates):
            m.on_bump(t)
        else:
            pose = moved
        dist += abs(v) * dt
        min_trunk = min(min_trunk, min(math.dist(pose[:2], tr) for tr in trees))
        g = gripper_position(*pose, 0.46)
        name = apple_in_reach(g, remaining, 0.07)
        if name is not None and not basket.full:
            xy = remaining.pop(name)
            basket.add()
            m.on_picked(xy)
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


def test_coverage_search_is_faster_and_misses_nothing():
    trees = [(4.0, 4.0), (10.0, 4.0), (10.0, 10.0)]
    for seed in range(3):
        apples = ring_apples(trees, per_tree=6, seed=seed)
        fast = run_world(apples, trees=trees, capacity=20)
        slow = run_world(apples, trees=trees, capacity=20, coverage=False)
        for m, _, remaining, t, _ in (fast, slow):
            assert m.state == DONE and not remaining, f'seed {seed}: {len(remaining)} left'
        assert fast[3] < 0.9 * slow[3], f'seed {seed}: {fast[3]:.0f} s vs {slow[3]:.0f} s'


@pytest.mark.parametrize('seed', [1, 6, 11, 2024])
def test_strict_detector_misses_nothing_on_generated_orchards(seed):
    # Seeds where the camera never saw some apples near the trunk or at the ring edge (session 11),
    # with a fake detector that sees exactly the footprint the mission plans with.
    orchard = pytest.importorskip('applepic_gazebo.orchard')
    import os
    cfg = orchard.load_config(os.path.join(os.path.dirname(__file__), '..', '..',
                                           'applepic_gazebo', 'config', 'orchard.yaml'))
    cfg['seed'] = seed
    apples = {a.name: (a.x, a.y) for a in orchard.generate_apples(cfg)}
    m, basket, remaining, t, _ = run_world(apples, trees=[tuple(t) for t in cfg['trees']],
                                           max_time=3000.0, footprint=camera_footprint())
    assert m.state == DONE and not remaining, f'{sorted(remaining)} left'


def _anchoring_mission(**kw):
    m = make_mission(anchor=True, **kw)
    x, y = m.search_point(0, 4)
    m.step((x, y, 0.0), 0.0)
    assert m.state == ANCHOR
    return m, x, y


def test_anchor_creeps_to_the_trunk_until_a_fix():
    m, x, y = _anchoring_mission()
    v, w = m.step((x, y, 0.8), 0.1)        # not facing the trunk: turn first
    assert v == 0.0 and w < 0.0
    v, w = m.step((x, y, 0.0), 0.2)
    assert v == m.p.anchor_speed and w == 0.0
    m.on_trunk_fix()
    m.step((x + 0.3, y, 0.0), 0.3)
    assert m.state == SEARCH and m.stats.transitions[-1][2] == 'trunk fix'


def test_anchor_gives_up_after_the_longest_creep():
    m, x, y = _anchoring_mission()
    m.step((x, y, 0.0), 0.1)
    x += m.p.anchor_travel + 0.01
    v, w = m.step((x, y, 0.0), 0.2)      # creep over: look to one side, then the other
    assert m.state == ANCHOR and v == 0.0 and w > 0.0
    v, w = m.step((x, y, m.p.anchor_scan), 0.3)
    assert m.state == ANCHOR and w < 0.0
    m.step((x, y, -m.p.anchor_scan), 0.4)
    assert m.state == SEARCH and m.stats.transitions[-1][2] == 'no trunk fix'


def test_anchor_stops_near_the_estimated_trunk_and_on_a_full_basket():
    m, x, y = _anchoring_mission(anchor_travel=5.0)
    m.step((x, y, 0.0), 0.1)
    v, w = m.step((TREES[0][0] - m.p.anchor_stop + 0.01, TREES[0][1], 0.0), 0.2)
    assert m.state == ANCHOR and v == 0.0 and w != 0.0
    m, x, y = _anchoring_mission()
    m.on_basket_full(True)
    m.step((x, y, 0.0), 0.1)
    assert m.state == GOTO_BASE


def test_apple_seen_once_gets_one_view_aimed_at_it_before_leaving():
    m = make_mission(coverage=False)
    x, y, yaw = at_search_point(m)
    m.k = len(m.order) - 1
    apple = (TREES[0][0] + 1.2, TREES[0][1] + 0.5)
    m.on_detections([apple], stamp=0.0)         # one view only, never confirmed
    m.phase, m.look_until = 'look', 0.0
    m.step((x, y, yaw), 1.0)
    assert m.state == SEARCH and m.k == len(m.order) - 1 and m.phase == 'move'
    view = m.order[-1]
    mid = (view[0] + m.p.relook_ahead * math.cos(view[2]),
           view[1] + m.p.relook_ahead * math.sin(view[2]))
    assert math.dist(mid, apple) < 0.5
    # Still not confirmed after that view: the tree is left, the view is not requested twice.
    m.phase, m.look_until = 'look', 0.0
    m.step((x, y, yaw), 2.0)
    assert m.state == GOTO_TREE and m.tree == 1


def test_bump_backs_up_then_skips_the_search_view():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    k = m.k
    m.on_bump(1.0)
    v, w = m.step((x, y, yaw), 1.0)
    assert v < 0.0 and w == 0.0 and m.stats.bumps == 1
    assert m.step((x, y, yaw), 2.0)[0] < 0.0
    m.step((x, y, yaw), 3.5)            # free again
    assert m.state == SEARCH and m.k > k and m.stats.bumps == 1


def test_bump_drives_forward_when_still_blocked_behind():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    m.on_bump(1.0)
    assert m.step((x, y, yaw), 1.0)[0] < 0.0
    m.on_bump(3.0)
    assert m.step((x, y, yaw), 3.1)[0] > 0.0
    assert m.stats.bumps == 1


def test_bump_during_pick_gives_up_the_apple():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    confirm(m, (x + 0.8, y))
    m.step((x, y, yaw), 0.5)
    assert m.state == PICK
    target = m.target
    near_trunk = (TREES[0][0] - 0.45, TREES[0][1], 0.0)
    m.on_bump(1.0)
    m.step(near_trunk, 1.0)
    m.step(near_trunk, 3.5)
    assert m.state == SEARCH and m.stats.failed == 1 and not m.obstacles
    assert m.tracker.get(target).state == 'failed'


def test_old_contact_does_not_trigger_an_escape():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    m.on_bump(1.0)
    m.step((x, y, yaw), 5.0)
    assert m.stats.bumps == 0


def test_search_view_not_reached_in_time_is_skipped():
    m = make_mission(coverage=False)
    x, y, yaw = at_search_point(m)
    far = (x + 1.0, y, yaw)
    m.k, m.phase = 0, None
    m._start_move()
    m.path = [(x + 5.0, y)]
    m.step(far, 1.0)
    assert m.k == 0
    m.step(far, 1.0 + m.p.view_timeout + 0.1)
    assert m.k == 1 and m.phase == 'move'


def _moving_bump(m, pose, now, v=0.3):
    """The robot driving at speed `v` hits something at time `now`, then backs away freely."""
    m.last_v = v
    m.on_bump(now)
    assert m.step(pose, now)[0] != 0.0
    m.step(pose, now + 2.5)


def test_bump_away_from_trunks_remembers_the_contact_ahead_of_the_robot():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    pose = (x + 3.0, y, 0.0)
    _moving_bump(m, pose, 1.0)
    assert m.obstacles[0] == pytest.approx((pose[0] + 0.40, pose[1]))
    assert sorted(o[1] for o in m.obstacles) == pytest.approx([pose[1] - 0.3, pose[1], pose[1] + 0.3])
    assert m.stats.obstacles == 1


def test_bump_while_reversing_is_behind_and_pivot_records_nothing():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    _moving_bump(m, (x + 3.0, y, 0.0), 1.0, v=-0.2)
    assert m.obstacles[0] == pytest.approx((x + 3.0 - 0.20, y))
    _moving_bump(m, (x + 5.0, y, 0.0), 5.0, v=0.0)
    assert m.stats.obstacles == 1 and len(m.obstacles) == 3


def test_bump_on_a_trunk_is_not_an_obstacle():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    tx, ty = TREES[0]
    _moving_bump(m, (tx - 0.5, ty, 0.0), 1.0)
    assert m.obstacles == [] and m.stats.bumps == 1


def test_repeated_contact_at_the_same_place_counts_once():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    _moving_bump(m, (x + 3.0, y, 0.0), 1.0)
    _moving_bump(m, (x + 3.05, y, 0.0), 5.0)
    assert m.stats.obstacles == 1 and m.stats.bumps == 2


def test_paths_go_around_a_known_obstacle():
    m = make_mission()
    m.step((0.0, 0.0, 0.0), 0.0)
    target = m.path[-1]
    middle = (target[0] / 2, target[1] / 2)
    m.obstacles.append(middle)
    m._enter_goto_tree()
    assert segment_clearance((0.0, 0.0), m.path[0], [middle]) >= 0.69
    assert m.path[-1] == pytest.approx(target)
    assert len(m.path) > 1


def test_bump_on_the_way_to_a_search_view_retries_it_once_around_the_contact():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    k = m.k
    goal = m.order[k][:2]
    _moving_bump(m, (goal[0] - 2.0, goal[1], 0.0), 1.0)
    m.step((goal[0] - 2.0, goal[1], 0.0), 3.5)
    assert m.state == SEARCH and m.k == k and m.stats.obstacles == 1
    _moving_bump(m, (goal[0] - 1.0, goal[1], 0.0), 5.0)
    m.step((goal[0] - 1.0, goal[1], 0.0), 7.5)
    assert m.k > k


def test_view_next_to_a_known_obstacle_is_dropped():
    m = make_mission(coverage=False)
    x, y, yaw = at_search_point(m)
    next_view = m.order[m.k + 1]
    m.obstacles.append(next_view[:2])
    m.k += 1
    assert not m._skip_covered()
    assert m.order[m.k] != next_view


def test_blocked_search_point_is_not_the_entry_of_a_tree():
    m = make_mission()
    m._pose = (0.0, 0.0, 0.0)
    entry = m._closest_search_index(0, (0.0, 0.0))
    m.obstacles.append(m.search_point(0, entry))
    assert m._closest_search_index(0, (0.0, 0.0)) != entry


def test_apple_kept_after_a_first_bump_then_given_up():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    confirm(m, (x + 0.8, y))
    m.step((x, y, yaw), 0.5)
    assert m.state == PICK
    target = m.target
    far = (x + 3.0, y, 0.0)
    _moving_bump(m, far, 1.0)
    m.step(far, 3.5)
    assert m.tracker.get(target).state == 'active' and m.stats.failed == 0
    m.state, m.target, m.target_since = PICK, target, 4.0
    _moving_bump(m, far, 5.0)
    m.step(far, 7.5)
    assert m.tracker.get(target).state == 'failed' and m.stats.failed == 1


def test_apple_behind_a_known_obstacle_is_not_approached_from_here():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    assert m.approach_feasible(x, y - 2.0)
    m.obstacles.append((x, y - 0.8))
    assert not m.approach_feasible(x, y - 2.0)


def test_obstacles_follow_the_map_shift():
    m = make_mission()
    m.obstacles.append((1.0, 2.0))
    m.on_map_shift(lambda p: (p[0] + 0.5, p[1]))
    assert m.obstacles == [(1.5, 2.0)]


def _bump_until_tree_changes(m, pose, t):
    tree = m.tree
    while m.tree == tree and m.state == GOTO_TREE:
        _moving_bump(m, pose, t)
        m.step(pose, t + 2.5)
        t += 4.0
    return t


def test_tree_out_of_reach_is_put_off_to_the_end_of_the_loop_then_left_out():
    trees = [(4.0, 4.0), (10.0, 4.0), (16.0, 4.0)]
    m = Mission(trees, DEPOT, MissionParams(loops=1, max_goto_bumps=2))
    pose = (0.0, 0.0, 0.0)
    m.step(pose, 0.0)
    t = _bump_until_tree_changes(m, pose, 1.0)
    assert m.tree == 1 and m.stats.trees_skipped == 0 and m.put_off == [0]
    t = _bump_until_tree_changes(m, pose, t)
    assert m.tree == 2 and m.put_off == [0, 1]
    t = _bump_until_tree_changes(m, pose, t)
    assert m.tree == 0 and m.put_off == [1, 2] and m.stats.trees_skipped == 0
    t = _bump_until_tree_changes(m, pose, t)
    assert m.tree == 1 and m.stats.trees_skipped == 1
    t = _bump_until_tree_changes(m, pose, t)
    assert m.tree == 2 and m.stats.trees_skipped == 2
    _bump_until_tree_changes(m, pose, t)
    assert m.state == GOTO_BASE and m.stats.trees_skipped == 3


def test_loop_goes_on_after_the_tree_it_had_reached_when_a_put_off_tree_is_done():
    m = Mission([(4.0, 4.0), (10.0, 4.0), (16.0, 4.0)], DEPOT, MissionParams(loops=1, max_goto_bumps=2))
    pose = (0.0, 0.0, 0.0)
    m.step(pose, 0.0)
    _bump_until_tree_changes(m, pose, 1.0)
    assert m.tree == 1
    m._next_tree()
    assert m.tree == 2
    m._next_tree()
    assert m.tree == 0 and m.put_off == []
    m._next_tree()
    assert m.state == GOTO_BASE


def test_full_mission_goes_around_unknown_crates():
    apples = ring_apples(TREES)
    # One crate on the way to the first tree, one on a search point of the second.
    tx, ty = TREES[1]
    crates = [(TREES[0][0] / 2, TREES[0][1] / 2, 0.25), (tx, ty - 1.9, 0.2)]
    crates = [c for c in crates if all(_clear(c, a) for a in apples.values())]
    assert crates
    m, basket, remaining, t, _ = run_world(apples, crates=crates, max_time=2500.0)
    assert m.state == DONE and not remaining
    assert m.stats.bumps >= 1 and m.stats.obstacles >= 1


def _clear(crate, apple):
    return max(abs(apple[0] - crate[0]), abs(apple[1] - crate[1])) > crate[2] + 0.1


def test_robot_blocked_both_ways_ends_up_pivoting():
    m = make_mission()
    x, y, yaw = at_search_point(m)
    m.on_bump(1.0)
    m.step((x, y, yaw), 1.0)
    m.on_bump(3.0)
    assert m.step((x, y, yaw), 3.1)[0] > 0.0
    m.on_bump(5.2)
    v, w = m.step((x, y, yaw), 5.3)
    assert v == 0.0 and w != 0.0


def _wall(n):
    """Diagonal wall of 2 n + 1 crates across the straight way from the depot to (4, 4)."""
    return [(1.0 + 0.2 * i, 1.8 - 0.2 * i, 0.15) for i in range(-n, n + 1)]


def test_full_mission_gets_past_a_long_wall():
    trees = [(4.0, 4.0)]
    apples = ring_apples(trees, per_tree=3)
    m, basket, remaining, t, _ = run_world(apples, trees=trees, crates=_wall(5), max_time=1500.0)
    assert m.state == DONE and not remaining
    assert m.stats.bumps >= 2 and m.stats.trees_skipped == 0


def test_long_wall_costs_few_bumps():
    trees = [(4.0, 4.0)]
    apples = ring_apples(trees, per_tree=3)
    m, basket, remaining, t, _ = run_world(apples, trees=trees, crates=_wall(8), max_time=1500.0)
    assert m.state == DONE and not remaining
    assert m.stats.bumps <= 6 and m.stats.trees_skipped == 0


def test_tree_walled_in_is_left_out_and_the_mission_ends():
    trees = [(4.0, 4.0)]
    apples = ring_apples(trees, per_tree=3)
    # Ring of crates, 3 m around the trunk.
    crates = [(4.0 + 3.0 * math.cos(i * 0.1), 4.0 + 3.0 * math.sin(i * 0.1), 0.15) for i in range(63)]
    m, basket, remaining, t, _ = run_world(apples, trees=trees, crates=crates, max_time=2500.0)
    assert m.state == DONE and m.stats.trees_skipped == 1 and len(remaining) == 3
