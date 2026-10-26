"""Pure-Python mission state machine for the apple picking robot (no rclpy).

States and transitions:

    GOTO_TREE --arrived--> SEARCH --feasible confirmed apple--> PICK --picked/failed--> SEARCH
    SEARCH --all search points looked at--> GOTO_TREE (next tree) or GOTO_BASE (end of loop)
    any working state --basket full--> GOTO_BASE (then back to the same tree)
    GOTO_BASE --arrived--> UNLOAD --unloaded--> GOTO_TREE (resume / new loop) or DONE

SEARCH visits `search_points` points on a circle of radius `search_radius` around the trunk
(starting with the one closest to the robot), faces the trunk at each one and looks for
`look_time` s. A CoverageGrid per tree records the ground the camera has seen (in SEARCH and
PICK): search points whose view would add less than `min_gain` are skipped, and the tree is left
once `coverage_goal` of its ground has been seen, unless a known apple is still waiting for a
feasible approach. Apples (fused by an AppleTracker) within `apple_radius` of the trunk are picked
nearest first, provided the straight approach keeps the base away from the trunk.

`step(pose, now)` returns the velocity command for the current state; the caller feeds events
with `on_detections`, `on_picked`, `on_basket_full` and `on_unloaded`, and calls the unload
service while `unload_requested` is set.
"""
import math
from dataclasses import dataclass, field

from applepic_mission.apple_tracker import AppleTracker
from applepic_mission.approach import ApproachParams, approach_command
from applepic_mission.coverage import CoverageGrid, camera_footprint
from applepic_navigation.controller import ControllerParams, compute_command, wrap_angle
from applepic_navigation.route import plan_path, segment_clearance

GOTO_TREE = 'GOTO_TREE'
SEARCH = 'SEARCH'
PICK = 'PICK'
GOTO_BASE = 'GOTO_BASE'
UNLOAD = 'UNLOAD'
DONE = 'DONE'


@dataclass
class MissionParams:
    search_radius: float = 2.0      # m, circle of search points around the trunk
    search_points: int = 8
    apple_radius: float = 1.9       # m, apples farther from the trunk belong to no tree here
    look_time: float = 1.0          # s, stopped facing the trunk at each search point
    face_tolerance: float = 0.1     # rad
    face_gain: float = 2.0          # 1/s
    max_face_rate: float = 1.2      # rad/s
    travel_clearance: float = 1.9   # m, distance kept from trunks when travelling between trees
    circle_clearance: float = 1.7   # m, distance kept from the trunk between search points
    trunk_path_clearance: float = 0.45  # m, base path to an apple vs trunk axis
    trunk_stop_clearance: float = 0.5   # m, base stop point vs trunk axis (rotation in place)
    approach_timeout: float = 30.0  # s, give up an apple not reached in time
    pick_timeout: float = 3.0       # s, give up an apple reached but not picked
    pick_gate: float = 0.2          # m, match a reported pick to a track
    coverage: bool = True           # skip search points / leave trees using camera coverage
    coverage_goal: float = 0.98     # fraction of the tree ground to see before leaving it
    min_gain: float = 0.01          # skip a search point whose view adds less than this
    max_fill_in: int = 3            # extra best-gain views on the circle once it is done
    fill_in_gain: float = 0.02      # minimum coverage added by a fill-in view
    fill_in_angles: int = 24        # candidate positions of a fill-in view on the circle
    fill_in_offset: float = 0.6     # rad, candidate headings: trunk and trunk +/- this
    coverage_r_min: float = 0.4     # m, covered annulus around the trunk (apple ring + margin)
    coverage_r_max: float = 1.7     # m
    loops: int = 1                  # 0 = forever


@dataclass
class MissionStats:
    picked: int = 0
    failed: int = 0
    unloaded: int = 0
    loops_done: int = 0
    trees_searched: int = 0
    transitions: list = field(default_factory=list)


class Mission:

    def __init__(self, trees, depot, params=None, nav=None, approach=None, tracker=None,
                 camera=None):
        self.trees = [tuple(t) for t in trees]
        self.depot = tuple(depot)
        self.p = params or MissionParams()
        self.nav = nav or ControllerParams()
        self.approach = approach or ApproachParams()
        self.tracker = tracker or AppleTracker()
        self.stats = MissionStats()
        self.footprint = camera_footprint(camera)
        self.grids = {}             # tree index -> CoverageGrid of the current loop
        self.state = None
        self.tree = 0
        self.path = []              # pending (x, y) waypoints of the current move
        self.basket_full = False
        self.resume_tree = False    # unloading because the basket was full mid-loop
        self.unload_requested = False
        self.carried = 0            # apples picked since the last unload
        # SEARCH bookkeeping
        self.order = []             # views (x, y, yaw) to visit, in order
        self.fill_ins = 0
        self.k = 0                  # position in `order`
        self.phase = None           # move | face | look
        self.look_until = None
        # PICK bookkeeping
        self.target = None
        self.target_since = None
        self.reached_since = None
        self._pose = None

    # ---------------------------------------------------------------- events

    def on_detections(self, points, stamp=0.0):
        self.tracker.update(points, stamp)

    def on_basket_full(self, full):
        self.basket_full = bool(full)

    def on_picked(self, gripper_xy):
        """The gripper picked an apple at `gripper_xy`: retire the matching track."""
        self.stats.picked += 1
        self.carried += 1
        gx, gy = gripper_xy
        cands = [t for t in self.tracker.tracks if t.state == 'active'
                 and math.hypot(t.x - gx, t.y - gy) <= self.p.pick_gate]
        hit = min(cands, key=lambda t: math.hypot(t.x - gx, t.y - gy), default=None)
        if hit is not None:
            self.tracker.mark_picked(hit.id)
        if self.state == PICK and (hit is None or hit.id == self.target):
            if hit is None:
                self.tracker.mark_picked(self.target)
            self._enter_search(resume=True)

    def on_unloaded(self, count=None):
        if self.state != UNLOAD:
            return
        self.unload_requested = False
        self.stats.unloaded += self.carried if count is None else count
        self.carried = 0
        self.basket_full = False
        if self.resume_tree:
            self.resume_tree = False
            self._enter_goto_tree()
            return
        self.stats.loops_done += 1
        if self.p.loops and self.stats.loops_done >= self.p.loops:
            self._set_state(DONE)
            return
        self.tree = 0
        self.grids = {}
        self._enter_goto_tree()

    # ---------------------------------------------------------------- helpers

    def _set_state(self, state, note=''):
        self.state = state
        self.stats.transitions.append((state, self.tree, note))

    def search_point(self, tree, index):
        tx, ty = self.trees[tree]
        a = 2.0 * math.pi * index / self.p.search_points
        return tx + self.p.search_radius * math.cos(a), ty + self.p.search_radius * math.sin(a)

    def _closest_search_index(self, tree, xy):
        return min(range(self.p.search_points),
                   key=lambda i: math.dist(self.search_point(tree, i), xy))

    def grid(self, tree):
        if tree not in self.grids:
            self.grids[tree] = CoverageGrid(
                self.trees[tree], self.p.coverage_r_min, self.p.coverage_r_max,
                footprint=self.footprint)
        return self.grids[tree]

    def view_pose(self, tree, angle, offset=0.0):
        """Point of the search circle of `tree` at `angle`, facing the trunk (+ `offset` rad)."""
        tx, ty = self.trees[tree]
        r = self.p.search_radius
        return (tx + r * math.cos(angle), ty + r * math.sin(angle),
                wrap_angle(angle + math.pi + offset))

    def fill_in_view(self):
        """Search-circle view adding the most coverage of the current tree, or None if too little."""
        grid = self.grid(self.tree)
        best, best_gain = None, self.p.fill_in_gain
        for i in range(self.p.fill_in_angles):
            a = 2.0 * math.pi * i / self.p.fill_in_angles
            for off in (0.0, -self.p.fill_in_offset, self.p.fill_in_offset):
                view = self.view_pose(self.tree, a, off)
                g = grid.gain(view)
                if g >= best_gain:
                    best, best_gain = view, g
        return best

    def pending(self):
        """Active apples of the current tree, confirmed (not approachable from here) or seen once."""
        trunk = self.trees[self.tree]
        return [t for t in self.tracker.tracks if t.state == 'active'
                and math.dist((t.x, t.y), trunk) <= self.p.apple_radius]

    def _skip_covered(self):
        """Advance self.k past views adding no coverage, append fill-in views once the circle is
        done; return True when the tree is finished."""
        if not self.p.coverage or self.pending():
            return self.k >= len(self.order)
        grid = self.grid(self.tree)
        if grid.fraction >= self.p.coverage_goal:
            return True
        while self.k < len(self.order) and grid.gain(self.order[self.k]) < self.p.min_gain:
            self.k += 1
        if self.k >= len(self.order) and self.fill_ins < self.p.max_fill_in:
            view = self.fill_in_view()
            if view is not None:
                self.order.append(view)
                self.fill_ins += 1
        return self.k >= len(self.order)

    def _travel_path(self, goal, clearance):
        return plan_path(self._pose[:2], goal, self.trees, clearance)

    def _enter_goto_tree(self):
        self._set_state(GOTO_TREE)
        entry = self.search_point(self.tree, self._closest_search_index(self.tree, self._pose[:2]))
        self.path = self._travel_path(entry, self.p.travel_clearance)

    def _enter_goto_base(self, resume):
        self.resume_tree = resume
        self._clear_target()
        self._set_state(GOTO_BASE, 'basket full' if resume else 'end of loop')
        self.path = self._travel_path(self.depot, self.p.travel_clearance)

    def _enter_search(self, resume=False):
        """Start the search pattern of the current tree, or resume it at the current point."""
        self._clear_target()
        if not resume:
            start = self._closest_search_index(self.tree, self._pose[:2])
            n = self.p.search_points
            self.order = [self.view_pose(self.tree, 2.0 * math.pi * ((start + i) % n) / n)
                          for i in range(n)]
            self.k = 0
            self.fill_ins = 0
        self._set_state(SEARCH)
        if self._skip_covered():
            self._next_tree()
            return
        self._start_move()

    def _start_move(self):
        goal = self.order[self.k][:2]
        self.path = plan_path(self._pose[:2], goal, [self.trees[self.tree]],
                              self.p.circle_clearance)
        self.phase = 'move'
        self.look_until = None

    def _next_tree(self):
        self.stats.trees_searched += 1
        self.tree += 1
        if self.tree >= len(self.trees):
            self.tree = len(self.trees) - 1
            self._enter_goto_base(resume=False)
        else:
            self._enter_goto_tree()

    def _clear_target(self):
        self.target = None
        self.target_since = None
        self.reached_since = None

    def _follow_path(self):
        """Drive along self.path; return (linear, angular, arrived)."""
        x, y, yaw = self._pose
        while self.path:
            v, w, reached = compute_command(x, y, yaw, *self.path[0], self.nav)
            if not reached:
                return v, w, False
            self.path.pop(0)
        return 0.0, 0.0, True

    def approach_feasible(self, ax, ay):
        """True if the straight gripper approach to (ax, ay) keeps the base off the trunk."""
        x, y = self._pose[:2]
        trunk = self.trees[self.tree]
        d = math.hypot(ax - x, ay - y)
        if d < 1e-6:
            return False
        ux, uy = (ax - x) / d, (ay - y) / d
        stop = (ax - self.approach.offset * ux, ay - self.approach.offset * uy)
        return (math.dist(stop, trunk) >= self.p.trunk_stop_clearance
                and segment_clearance((x, y), stop, [trunk]) >= self.p.trunk_path_clearance)

    def candidates(self):
        """Confirmed apples of the current tree that can be approached from here, nearest first."""
        trunk = self.trees[self.tree]
        x, y = self._pose[:2]
        cands = [t for t in self.tracker.confirmed()
                 if math.dist((t.x, t.y), trunk) <= self.p.apple_radius
                 and self.approach_feasible(t.x, t.y)]
        return sorted(cands, key=lambda t: math.hypot(t.x - x, t.y - y))

    # ---------------------------------------------------------------- tick

    def step(self, pose, now):
        """Advance the state machine; return the (linear, angular) velocity command."""
        self._pose = tuple(pose)
        if self.p.coverage and self.state in (SEARCH, PICK):
            self.grid(self.tree).mark(self._pose)
        if self.state is None:
            self._enter_goto_tree()
        if self.basket_full and self.state in (GOTO_TREE, SEARCH, PICK):
            self._enter_goto_base(resume=True)
        handler = {
            GOTO_TREE: self._step_goto_tree,
            SEARCH: self._step_search,
            PICK: self._step_pick,
            GOTO_BASE: self._step_goto_base,
            UNLOAD: self._step_unload,
            DONE: lambda now: (0.0, 0.0),
        }[self.state]
        return handler(now)

    def _step_goto_tree(self, now):
        v, w, arrived = self._follow_path()
        if not arrived:
            return v, w
        self._enter_search()
        return 0.0, 0.0

    def _step_search(self, now):
        cands = self.candidates()
        if cands:
            self.target, self.target_since = cands[0].id, now
            self._set_state(PICK, f'track {cands[0].id}')
            return self._step_pick(now)
        if self.phase == 'move':
            v, w, arrived = self._follow_path()
            if not arrived:
                return v, w
            self.phase = 'face'
        if self.phase == 'face':
            err = wrap_angle(self.order[self.k][2] - self._pose[2])
            if abs(err) > self.p.face_tolerance:
                w = max(-self.p.max_face_rate, min(self.p.max_face_rate, self.p.face_gain * err))
                return 0.0, w
            self.phase, self.look_until = 'look', now + self.p.look_time
        if now < self.look_until:
            return 0.0, 0.0
        self.k += 1
        if self._skip_covered():
            self._next_tree()
        else:
            self._start_move()
        return 0.0, 0.0

    def _step_pick(self, now):
        track = self.tracker.get(self.target)
        timed_out = now - self.target_since > self.p.approach_timeout or (
            self.reached_since is not None and now - self.reached_since > self.p.pick_timeout)
        if track is None or track.state != 'active' or timed_out:
            if track is not None and track.state == 'active':
                self.tracker.mark_failed(track.id)
                self.stats.failed += 1
            self._enter_search(resume=True)
            return 0.0, 0.0
        v, w, reached = approach_command(*self._pose, track.x, track.y, self.approach)
        if reached and self.reached_since is None:
            self.reached_since = now
        return v, w

    def _step_goto_base(self, now):
        v, w, arrived = self._follow_path()
        if not arrived:
            return v, w
        self._set_state(UNLOAD)
        self.unload_requested = True
        return 0.0, 0.0

    def _step_unload(self, now):
        return 0.0, 0.0
