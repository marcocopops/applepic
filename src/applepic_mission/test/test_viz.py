import math

from applepic_mission.viz import circle_points, hud_text, PathRecorder, scene_items

CONFIG = {
    'depot': {'position': [0.0, 0.0], 'size': [1.5, 1.5]},
    'tree': {'trunk_radius': 0.1, 'canopy_radius': 1.1},
    'trees': [[4.0, 4.0], [10.0, 4.0]],
}


def test_path_recorder_decimates_and_bounds_memory():
    rec = PathRecorder(min_step=0.1, max_points=100)
    assert rec.add(0.0, 0.0)
    assert not rec.add(0.05, 0.0)
    assert rec.add(0.1, 0.0)
    for i in range(1000):
        rec.add(0.2 + 0.15 * i, 0.0)
    assert len(rec.points) <= 100
    assert rec.points[0] == (0.0, 0.0)
    assert rec.points[-1] == (0.2 + 0.15 * 999, 0.0)


def test_scene_items_cover_trees_and_depot():
    items = scene_items(CONFIG, 2.2)
    kinds = [it['kind'] for it in items]
    assert kinds.count('trunk') == 2 and kinds.count('canopy') == 2
    assert kinds.count('search_ring') == 2 and kinds.count('depot') == 1
    labels = [it['text'] for it in items if it['kind'] == 'label']
    assert labels == ['tree 0', 'tree 1', 'depot']
    ring = next(it for it in items if it['kind'] == 'search_ring')
    assert ring['radius'] == 2.2


def test_circle_points_closed_on_radius():
    pts = circle_points(1.0, 2.0, 0.5, n=12)
    assert len(pts) == 13
    assert math.isclose(pts[0][0], pts[-1][0]) and math.isclose(pts[0][1], pts[-1][1])
    assert all(math.isclose(math.hypot(x - 1.0, y - 2.0), 0.5) for x, y in pts)


def test_hud_text():
    assert hud_text(None).startswith('WAITING')
    text = hud_text({'state': 'SEARCH', 'tree': 3, 'picked': 7, 'unloaded': 2,
                     'distance_m': 41.2, 'elapsed_s': 120.0}, 5, 10)
    assert 'SEARCH (tree 3)' in text and 'basket 5/10' in text
    assert 'picked 7  unloaded 2' in text and '41 m' in text
