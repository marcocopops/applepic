import math

import pytest

from applepic_mission.coverage import (
    CameraParams, CoverageGrid, camera_footprint, point_in_convex)


def test_footprint_matches_camera_geometry():
    far_l, far_r, near_r, near_l = camera_footprint(CameraParams(margin=1.0))
    # Image top edge: 0.85 - vfov/2 = 0.333 rad below the horizon, seen from 0.46 m above apples.
    vfov = 2 * math.atan(math.tan(0.65) * 480 / 640)
    assert far_l[0] == pytest.approx(0.35 + 0.46 / math.tan(0.85 - vfov / 2), rel=0.02)
    assert near_l[0] == pytest.approx(0.35 + 0.46 / math.tan(0.85 + vfov / 2), rel=0.02)
    assert far_l[1] == pytest.approx(-far_r[1]) and far_l[1] > near_l[1] > 0
    assert 1.6 < far_l[0] < 1.8 and 0.4 < near_l[0] < 0.5


def test_margin_shrinks_footprint():
    full, shrunk = camera_footprint(CameraParams(margin=1.0)), camera_footprint()
    assert all(point_in_convex(full, x, y) for x, y in shrunk)


def test_point_in_convex():
    square = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
    assert point_in_convex(square, 0.5, 0.2)
    assert not point_in_convex(square, 1.5, 0.0)


def test_grid_marks_only_the_cells_in_view():
    grid = CoverageGrid((0.0, 0.0))
    assert grid.fraction == 0.0
    pose = (-2.0, 0.0, 0.0)                    # west of the trunk, facing it
    g = grid.gain(pose)
    assert 0.05 < g < 0.3
    assert grid.mark(pose) == pytest.approx(g)
    assert grid.gain(pose) == 0.0 and grid.fraction == pytest.approx(g)
    seen = [c for c, s in zip(grid.cells, grid.seen) if s]
    assert all(c[0] < 0.0 for c in seen)       # only the near side
    assert grid.gain((10.0, 10.0, 0.0)) == 0.0  # far away
    assert grid.gain((-2.0, 0.0, math.pi)) == 0.0  # facing away


def test_trunk_hides_cells_behind_it():
    grid = CoverageGrid((0.0, 0.0), r_max=1.7, trunk_radius=0.15)
    pose = (-1.0, 0.0, 0.0)                    # 1 m from the trunk: the far side is in view
    behind = [k for k, (x, y) in enumerate(grid.cells) if x > 0.2 and abs(y) < 0.05]
    beside = [k for k, (x, y) in enumerate(grid.cells) if x > 0.2 and 0.25 < abs(y) < 0.35]
    vis = set(grid.visible(pose))
    assert behind and not vis & set(behind)
    assert vis & set(beside)


def test_eight_views_cover_most_but_not_all_of_the_ring():
    grid = CoverageGrid((0.0, 0.0))
    for i in range(8):
        a = 2 * math.pi * i / 8
        grid.mark((2.0 * math.cos(a), 2.0 * math.sin(a), a + math.pi))
    assert 0.6 < grid.fraction < 0.98
