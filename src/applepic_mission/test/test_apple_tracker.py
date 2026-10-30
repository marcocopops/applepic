import random

from applepic_mission.apple_tracker import AppleTracker


def test_noisy_detections_fuse_into_one_confirmed_track_per_apple():
    rng = random.Random(0)
    truth = [(1.0, 1.0), (1.2, 1.0), (3.0, -1.0)]   # first two only 20 cm apart
    tr = AppleTracker(gate=0.08, min_views=2)
    for k in range(10):
        dets = [(x + rng.gauss(0, 0.01), y + rng.gauss(0, 0.01)) for x, y in truth]
        rng.shuffle(dets)
        tr.update(dets, stamp=k * 0.1)
    conf = tr.confirmed()
    assert len(conf) == 3
    for x, y in truth:
        assert min(abs(t.x - x) + abs(t.y - y) for t in conf) < 0.02


def test_single_view_is_not_confirmed():
    tr = AppleTracker(min_views=2)
    tr.update([(0.0, 0.0)])
    assert tr.confirmed() == []
    tr.update([(0.01, 0.0)])
    assert len(tr.confirmed()) == 1


def test_out_of_view_tracks_are_kept():
    tr = AppleTracker()
    tr.update([(1.0, 0.0)])
    tr.update([(1.0, 0.0)])
    for _ in range(5):
        tr.update([])
    assert len(tr.confirmed()) == 1


def test_picked_track_is_not_recreated_and_nearest_skips_it():
    tr = AppleTracker()
    for _ in range(2):
        tr.update([(1.0, 0.0), (2.0, 0.0)])
    first = tr.nearest(0.0, 0.0)
    assert (first.x, first.y) == (1.0, 0.0)
    tr.mark_picked(first.id)
    tr.update([(1.02, 0.0)])          # stale detection of the picked apple
    assert len(tr.tracks) == 2
    assert tr.nearest(0.0, 0.0).x == 2.0
    tr.mark_failed(tr.nearest(0.0, 0.0).id)
    assert tr.nearest(0.0, 0.0) is None


def test_tracks_follow_map_shift():
    tr = AppleTracker()
    tr.update([(1.0, 0.0)])
    tr.transform(lambda p: (p[0] + 0.3, p[1] - 0.1))
    tr.update([(1.29, -0.1)])         # same apple seen after the shift: no duplicate
    assert len(tr.tracks) == 1 and tr.tracks[0].views == 2
