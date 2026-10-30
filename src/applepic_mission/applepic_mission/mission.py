"""Pure-Python mission state machine for the apple picking robot (no rclpy).

States and transitions:

    GOTO_TREE --arrived--> [ANCHOR --trunk fix or nothing seen-->] SEARCH --feasible confirmed apple--> PICK --picked/failed--> SEARCH
    SEARCH --all search points looked at--> GOTO_TREE (next tree) or GOTO_BASE (end of loop)
    any working state --basket full--> GOTO_BASE (then back to the same tree)
    GOTO_BASE --arrived--> UNLOAD --unloaded--> GOTO_TREE (resume / new loop) or DONE

SEARCH visits `search_points` points on a circle of radius `search_radius` around the trunk
(starting with the one closest to the robot), faces the trunk at each one and looks for
`look_time` s. A CoverageGrid per tree records the ground the camera has seen (in SEARCH and
PICK): search points whose view would add less than `min_gain` are skipped, and the tree is left
once `coverage_goal` of its ground has been seen, unless a known apple is still waiting for a
feasible approach; an apple seen only once gets one more view aimed at it before leaving. Apples (fused by an AppleTracker) within `apple_radius` of the trunk are picked
nearest first, provided the straight approach keeps the base away from the trunk.

ANCHOR (optional, for a coarse GPS): a pose error would shift the search around the tree, so the
robot first faces the trunk and creeps towards it until the localization reports a trunk fix (the
trunk foot is just out of the camera view from the search circle), then looks to both sides.

A bump (the pose estimate led the robot into a trunk) interrupts any moving state: the robot drives
back, or forward if it still touches something, and gives up what it was doing: apple, search view.
A bump away from any trunk reveals an unknown obstacle: the edge of the chassis facing the direction of
travel touched it, somewhere along its width, so three points of that edge are remembered for the whole
mission; every move plans around them
and views too close to them are dropped; the interrupted apple or view gets one more try. A tree still
not reached after `max_goto_bumps` bumps is put off to the end of the loop, then left out.

`step(pose, now)` returns the velocity command for the current state; the caller feeds events
with `on_detections`, `on_picked`, `on_bump`, `on_basket_full` and `on_unloaded`, and calls the unload
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
ANCHOR = 'ANCHOR'
SEARCH = 'SEARCH'
PICK = 'PICK'
GOTO_BASE = 'GOTO_BASE'
UNLOAD = 'UNLOAD'
DONE = 'DONE'


@dataclass
class MissionParams:
    search_radius: float = 1.9      # m, circle of search points around the trunk
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
    coverage_goal: float = 0.995    # fraction of the tree ground to see before leaving it
    min_gain: float = 0.01          # skip a search point whose view adds less than this
    max_fill_in: int = 4            # extra best-gain views near the circle once it is done
    fill_in_gain: float = 0.005     # minimum coverage added by a fill-in view
    fill_in_angles: int = 24        # candidate angular positions of a fill-in view
    fill_in_offset: float = 0.6     # rad, candidate headings: trunk, +/- half and +/- this
    fill_in_radial: float = 0.2     # m, candidate fill-in views also this much inside/outside the
                                    # circle (sees the trunk foot / the outer ring edge)
    relook_ahead: float = 1.0       # m, an apple to look at again is placed this far ahead (mid view)
    coverage_r_min: float = 0.4     # m, covered annulus around the trunk (apple ring + margin)
    coverage_r_max: float = 1.7     # m
    loops: int = 1                  # 0 = forever
    anchor: bool = False            # get a trunk fix on arrival at each tree (ANCHOR state)
    anchor_speed: float = 0.2       # m/s
    anchor_travel: float = 1.2      # m, longest creep towards the trunk
    anchor_stop: float = 0.7        # m, estimated distance to the trunk axis where the creep ends
    anchor_scan: float = 0.8        # rad, look this much to each side after the creep
    view_timeout: float = 40.0      # s, give up a search view not reached in time
    escape_speed: float = 0.2       # m/s, driving away from a bump
    escape_time: float = 2.0        # s, then the robot checks that it is free
    bump_memory: float = 0.3        # s, a contact this recent means the robot is touching something
    bump_front: float = 0.40        # m, base origin to the front of the chassis
    bump_rear: float = 0.20         # m, base origin to the rear of the chassis
    bump_half_width: float = 0.3    # m, half width of the bumper (the contact is somewhere along it)
    bump_clearance: float = 0.7     # m, base path vs contact point (0.5 = pivot radius of the chassis)
    bump_pick_clearance: float = 0.55  # m, same for a straight approach to an apple
    bump_merge: float = 0.1         # m, a contact this close to a known one is the same point
    bump_trunk_gate: float = 0.5    # m, a contact this close to a trunk axis is that trunk
    max_goto_bumps: int = 8         # bumps on the way to a tree before putting it off to the end of
                                    # the loop, then leaving it out if that happens again


@dataclass
class MissionStats:
    picked: int = 0
    failed: int = 0
    unloaded: int = 0
    loops_done: int = 0
    trees_searched: int = 0
    bumps: int = 0
    obstacles: int = 0
    trees_skipped: int = 0
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
        self.relooked = set()       # ids of the once-seen tracks already given an extra view
        self.k = 0                  # position in `order`
        self.phase = None           # move | face | look
        self.look_until = None
        self.move_since = None
        # Bump escape
        self.last_bump = None
        self.escape_until = None
        self.escape_flips = 0
        self.last_v = 0.0
        self.obstacles = []         # (x, y) contact points with what the map does not know
        self.hit_obstacle = None    # contact point of the current escape
        self.bump_tried = set()     # tracks and views already given one more try after a bump
        self.goto_bumps = 0
        self.furthest_tree = 0      # the loop goes on from there after a put off tree
        self.put_off = []           # trees to come back to at the end of the loop
        self.retried = set()        # trees already put off once in this loop
        # ANCHOR bookkeeping
        self.anchor_fixed = False
        self.anchor_start = None
        self.anchor_scan = []       # headings still to look at once the creep is over
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

    def on_map_shift(self, fn):
        """The localization moved the map relative to the odometry: fn maps old map (x, y) to new.

        Apples are anchored to the continuous odometry frame, so the robot-to-apple geometry stays
        odometric while a GPS-driven estimate wanders (no ghost duplicate tracks, no misses).
        """
        self.tracker.transform(fn)
        self.obstacles = [tuple(fn(o)) for o in self.obstacles]

    def on_trunk_fix(self):
        """The localization has just been corrected on a trunk seen by the camera."""
        self.anchor_fixed = True

    def on_bump(self, now):
        """The bumper touches something (not the ground nor an apple)."""
        self.last_bump = now

    def on_picked(self, apple_xy):
        """The gripper picked the apple that was at `apple_xy`: retire the matching track."""
        self.stats.picked += 1
        self.carried += 1
        gx, gy = apple_xy
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
        self.furthest_tree = 0
        self.retried = set()
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
        indices = range(self.p.search_points)
        free = [i for i in indices if not self._blocked(self.search_point(tree, i))]
        return min(free or indices, key=lambda i: math.dist(self.search_point(tree, i), xy))

    def grid(self, tree):
        if tree not in self.grids:
            self.grids[tree] = CoverageGrid(
                self.trees[tree], self.p.coverage_r_min, self.p.coverage_r_max,
                footprint=self.footprint)
        return self.grids[tree]

    def view_pose(self, tree, angle, offset=0.0, radius=None):
        """Point of the search circle of `tree` at `angle`, facing the trunk (+ `offset` rad)."""
        tx, ty = self.trees[tree]
        r = self.p.search_radius if radius is None else radius
        return (tx + r * math.cos(angle), ty + r * math.sin(angle),
                wrap_angle(angle + math.pi + offset))

    def _candidate_views(self):
        """fill_in_angles positions x 3 radii x 5 headings near the search circle."""
        r, dr, off = self.p.search_radius, self.p.fill_in_radial, self.p.fill_in_offset
        # Keep the views off the trunk-avoidance circle used to move between them.
        radii = {max(r - dr, self.p.circle_clearance + 0.05), r, r + dr}
        for i in range(self.p.fill_in_angles):
            a = 2.0 * math.pi * i / self.p.fill_in_angles
            for radius in sorted(radii):
                if self._blocked(self.view_pose(self.tree, a, 0.0, radius)):
                    continue
                for o in (0.0, -off / 2, off / 2, -off, off):
                    yield self.view_pose(self.tree, a, o, radius)

    def fill_in_view(self):
        """View near the search circle adding the most coverage of the current tree, or None if
        too little."""
        grid = self.grid(self.tree)
        best, best_gain = None, self.p.fill_in_gain
        for view in self._candidate_views():
            g = grid.gain(view)
            if g > best_gain:
                best, best_gain = view, g
        return best

    def relook_view(self):
        """View with an apple of this tree seen only once in the middle of the image, or None."""
        once = [t for t in self.pending()
                if t.views < self.tracker.min_views and t.id not in self.relooked]
        if not once:
            return None
        self.relooked.add(once[0].id)
        target = (once[0].x, once[0].y)

        def miss(view):
            mid = (view[0] + self.p.relook_ahead * math.cos(view[2]),
                   view[1] + self.p.relook_ahead * math.sin(view[2]))
            return math.dist(mid, target)
        return min(self._candidate_views(), key=miss, default=None)

    def pending(self):
        """Active apples of the current tree, confirmed (not approachable from here) or seen once."""
        trunk = self.trees[self.tree]
        return [t for t in self.tracker.tracks if t.state == 'active'
                and math.dist((t.x, t.y), trunk) <= self.p.apple_radius]

    def _skip_covered(self):
        """Advance self.k past views adding no coverage, append fill-in views once the circle is
        done; return True when the tree is finished."""
        while self.k < len(self.order) and self._blocked(self.order[self.k]):
            self.k += 1
        if self.k >= len(self.order):
            view = self.relook_view()
            if view is not None:
                self.order.append(view)
                return False
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

    def _blocked(self, xy):
        """True if the base standing at xy would be too close to a known obstacle."""
        return any(math.dist(o, xy[:2]) < self.p.bump_clearance for o in self.obstacles)

    def _plan(self, goal, trunks, clearance, start=None):
        return plan_path(start or self._pose[:2], goal, trunks, clearance, bumps=self.obstacles,
                         bump_clearance=self.p.bump_clearance)

    def _travel_path(self, goal, clearance):
        return self._plan(goal, self.trees, clearance)

    def _enter_goto_tree(self, note=''):
        self._set_state(GOTO_TREE, note)
        entry = self.search_point(self.tree, self._closest_search_index(self.tree, self._pose[:2]))
        self.path = self._travel_path(entry, self.p.travel_clearance)

    def _enter_goto_base(self, resume, note=''):
        self.resume_tree = resume
        self._clear_target()
        self._set_state(GOTO_BASE, ('basket full' if resume else 'end of loop') + note)
        self.path = self._travel_path(self.depot, self.p.travel_clearance)

    def _enter_anchor(self):
        self.anchor_fixed = False
        self.anchor_start = None
        self.anchor_scan = None
        self._set_state(ANCHOR)

    def _enter_search(self, resume=False, note=''):
        """Start the search pattern of the current tree, or resume it at the current point."""
        self._clear_target()
        if not resume:
            start = self._closest_search_index(self.tree, self._pose[:2])
            n = self.p.search_points
            self.order = [self.view_pose(self.tree, 2.0 * math.pi * ((start + i) % n) / n)
                          for i in range(n)]
            self.k = 0
            self.fill_ins = 0
        self._set_state(SEARCH, note)
        if self._skip_covered():
            self._next_tree()
            return
        self._start_move()

    def _start_move(self):
        goal = self.order[self.k][:2]
        self.path = self._plan(goal, [self.trees[self.tree]], self.p.circle_clearance)
        self.phase = 'move'
        self.look_until = None
        self.move_since = None

    def _leave_note(self):
        """Why the current tree is left: its coverage and the active tracks it still has."""
        note = f'left tree {self.tree}'
        if self.p.coverage:
            note += f', coverage {self.grid(self.tree).fraction:.2f}'
        pending = [f'{t.id}x{t.views}@({t.x:.2f}, {t.y:.2f})' for t in self.pending()]
        return note + (f', pending {" ".join(pending)}' if pending else '')

    def _next_tree(self):
        note = self._leave_note()
        self.stats.trees_searched += 1
        self._advance_tree(note)

    def _put_off_tree(self):
        """Too many bumps on the way to this tree: come back at the end of the loop, from another
        side and knowing more obstacles; leave it out if that fails too."""
        if self.tree in self.retried:
            self.stats.trees_skipped += 1
            self._advance_tree(f'tree {self.tree} out of reach')
        else:
            self.retried.add(self.tree)
            self.put_off.append(self.tree)
            self._advance_tree(f'tree {self.tree} put off')

    def _advance_tree(self, note):
        self.goto_bumps = 0
        self.tree = max(self.tree, self.furthest_tree) + 1
        self.furthest_tree = self.tree
        if self.tree >= len(self.trees) and self.put_off:
            self.tree = self.put_off.pop(0)
            self._enter_goto_tree(note)
        elif self.tree >= len(self.trees):
            self.tree = len(self.trees) - 1
            self._enter_goto_base(resume=False, note=f' ({note})')
        else:
            self._enter_goto_tree(note)

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
                and segment_clearance((x, y), stop, [trunk]) >= self.p.trunk_path_clearance
                and segment_clearance((x, y), stop, self.obstacles) >= self.p.bump_pick_clearance)

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
        cmd = self._tick(pose, now)
        self.last_v = cmd[0]
        return cmd

    def _tick(self, pose, now):
        self._pose = tuple(pose)
        if self.p.coverage and self.state in (SEARCH, PICK):
            self.grid(self.tree).mark(self._pose)
        if self.state is None:
            self._enter_goto_tree()
        if self.basket_full and self.state in (GOTO_TREE, ANCHOR, SEARCH, PICK):
            self._enter_goto_base(resume=True)
        escape = self._step_escape(now)
        if escape is not None:
            return escape
        handler = {
            GOTO_TREE: self._step_goto_tree,
            ANCHOR: self._step_anchor,
            SEARCH: self._step_search,
            PICK: self._step_pick,
            GOTO_BASE: self._step_goto_base,
            UNLOAD: self._step_unload,
            DONE: lambda now: (0.0, 0.0),
        }[self.state]
        return handler(now)

    def _touching(self, now):
        return self.last_bump is not None and now - self.last_bump <= self.p.bump_memory

    def _step_escape(self, now):
        """Velocity command while driving away from a bump, None when not escaping."""
        if self.escape_until is None:
            if self.state in (UNLOAD, DONE) or not self._touching(now):
                return None
            self.stats.bumps += 1
            self.hit_obstacle = self._remember_obstacle()
            self.escape_flips = 0
            self.escape_until = now + self.p.escape_time
        elif now >= self.escape_until:
            if not self._touching(now):
                self.escape_until = None
                self._after_escape(now)
                return 0.0, 0.0
            self.escape_flips += 1                # blocked this way too
            self.escape_until = now + self.p.escape_time
        phase = self.escape_flips % 3             # back, forward, then pivot to change the angle
        if phase == 2:
            return 0.0, self.p.max_face_rate / 2
        return (-1.0 if phase == 0 else 1.0) * self.p.escape_speed, 0.0

    def _remember_obstacle(self):
        """Remember the bumper edge that hit something; return its middle, None if it is a trunk
        or when pivoting (no direction: the contact can be anywhere around the chassis).
        """
        if abs(self.last_v) < 0.01:
            return None
        x, y, yaw = self._pose
        reach = self.p.bump_front if self.last_v > 0.0 else -self.p.bump_rear
        c, s = math.cos(yaw), math.sin(yaw)
        middle = (x + reach * c, y + reach * s)
        half = self.p.bump_half_width
        edge = [middle, (middle[0] - half * s, middle[1] + half * c),
                (middle[0] + half * s, middle[1] - half * c)]
        if any(math.dist(p, t) < self.p.bump_trunk_gate for p in edge for t in self.trees):
            return None
        new = [p for p in edge if all(math.dist(p, o) >= self.p.bump_merge for o in self.obstacles)]
        self.obstacles.extend(new)
        self.stats.obstacles += middle in new
        return middle

    def _after_escape(self, now):
        """Go on around what the robot hit, or give up what led it there if that is not possible."""
        known = self.hit_obstacle is not None
        where = f' at ({self.hit_obstacle[0]:.2f}, {self.hit_obstacle[1]:.2f})' if known else ''
        self.hit_obstacle = None
        if self.state == PICK:
            track = self.tracker.get(self.target)
            if known and track is not None and track.id not in self.bump_tried:
                # Stays a candidate if another approach avoids the contact point.
                self.bump_tried.add(track.id)
                self._enter_search(resume=True, note=f'bump{where}: apple kept')
                return
            if track is not None and track.state == 'active':
                self.tracker.mark_failed(track.id)
                self.stats.failed += 1
            self._enter_search(resume=True, note=f'bump{where}: gave up the apple')
        elif self.state == SEARCH:
            view = (self.tree, self.k)
            if (known and view not in self.bump_tried and self.k < len(self.order)
                    and not self._blocked(self.order[self.k])):
                self.bump_tried.add(view)
                self._set_state(SEARCH, f'bump{where}: going around')
                self._start_move()
            else:
                self._skip_view(f'bump{where}: skipped a view')
        elif self.state == ANCHOR:
            self._enter_search(note=f'bump{where}')
        elif self.state == GOTO_TREE:
            self.goto_bumps += 1
            if self.goto_bumps >= self.p.max_goto_bumps:
                self._put_off_tree()
            else:
                self._enter_goto_tree(f'bump{where}')
        elif self.state == GOTO_BASE:
            self.goto_bumps += 1
            self._enter_goto_base(self.resume_tree, f' (bump{where})')

    def _skip_view(self, note):
        self.k += 1
        self._set_state(SEARCH, note)
        if self._skip_covered():
            self._next_tree()
        else:
            self._start_move()

    def _step_goto_tree(self, now):
        v, w, arrived = self._follow_path()
        if not arrived:
            return v, w
        self.goto_bumps = 0
        if self.p.anchor:
            self._enter_anchor()
        else:
            self._enter_search()
        return 0.0, 0.0

    def _step_anchor(self, now):
        if self.anchor_fixed:
            self._enter_search(note='trunk fix')
            return 0.0, 0.0
        x, y, yaw = self._pose
        tx, ty = self.trees[self.tree]
        if self.anchor_start is None:
            w = self._face(math.atan2(ty - y, tx - x))
            if w is not None:
                return 0.0, w
            self.anchor_start = (x, y)
        if self.anchor_scan is None:
            if (math.dist(self.anchor_start, (x, y)) < self.p.anchor_travel
                    and math.dist((x, y), (tx, ty)) > self.p.anchor_stop):
                return self.p.anchor_speed, 0.0
            self.anchor_scan = [yaw + self.p.anchor_scan, yaw - self.p.anchor_scan]
        while self.anchor_scan:
            w = self._face(self.anchor_scan[0])
            if w is not None:
                return 0.0, w
            self.anchor_scan.pop(0)
        self._enter_search(note='no trunk fix')
        return 0.0, 0.0

    def _face(self, heading):
        """Rotation rate towards `heading`, None once facing it."""
        err = wrap_angle(heading - self._pose[2])
        if abs(err) <= self.p.face_tolerance:
            return None
        return max(-self.p.max_face_rate, min(self.p.max_face_rate, self.p.face_gain * err))

    def _step_search(self, now):
        cands = self.candidates()
        if cands:
            self.target, self.target_since = cands[0].id, now
            self._set_state(PICK, f'track {cands[0].id}')
            return self._step_pick(now)
        if self.phase == 'move':
            if self.move_since is None:
                self.move_since = now
            elif now - self.move_since > self.p.view_timeout:
                self._skip_view('view not reached in time')
                return 0.0, 0.0
            v, w, arrived = self._follow_path()
            if not arrived:
                return v, w
            self.phase = 'face'
        if self.phase == 'face':
            w = self._face(self.order[self.k][2])
            if w is not None:
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
            note = ''
            if track is not None and track.state == 'active':
                self.tracker.mark_failed(track.id)
                self.stats.failed += 1
                note = f'gave up track {track.id} at ({track.x:.2f}, {track.y:.2f})'
            self._enter_search(resume=True, note=note)
            return 0.0, 0.0
        v, w, reached = approach_command(*self._pose, track.x, track.y, self.approach)
        if reached and self.reached_since is None:
            self.reached_since = now
        return v, w

    def _step_goto_base(self, now):
        v, w, arrived = self._follow_path()
        if not arrived:
            return v, w
        self.goto_bumps = 0
        self._set_state(UNLOAD)
        self.unload_requested = True
        return 0.0, 0.0

    def _step_unload(self, now):
        return 0.0, 0.0
