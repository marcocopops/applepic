from applepic_mission.metrics import apple_tree, evaluate, per_tree

MANIFEST = ['apple_t0_0', 'apple_t0_1', 'apple_t1_0', 'apple_t1_1', 'apple_t2_0']
WORLD = {'ground', 'applepic'}


def _stats(**kw):
    s = dict(state='DONE', picked=4, unloaded=4, failed=0, loops_done=1, distance_m=50.0)
    s.update(kw)
    return s


def test_apple_tree_parsing():
    assert apple_tree('apple_t12_3') == 12
    assert apple_tree('tree_0') is None
    assert per_tree(['apple_t1_0', 'apple_t0_2', 'apple_t1_1']) == {0: 1, 1: 2}


def test_pass_when_enough_apples_unloaded_in_time():
    picked = MANIFEST[:4]
    m = evaluate(_stats(), MANIFEST, picked, WORLD | {'apple_t2_0'}, 600.0, 200.0, 0.8, 900.0)
    assert m['result'] == 'PASS'
    assert m['unloaded_ratio'] == 0.8
    assert m['apples_left'] == ['apple_t2_0']
    assert m['effective_rtf'] == 3.0
    assert m['picked_per_tree'] == {0: 2, 1: 2}


def test_fail_on_low_ratio_timeout_or_not_done():
    picked = MANIFEST[:4]
    models = WORLD | {'apple_t2_0'}
    assert evaluate(_stats(), MANIFEST, picked, models, 600, 200, 0.9, 900)['result'] == 'FAIL'
    late = evaluate(_stats(), MANIFEST, picked, models, 1000, 300, 0.8, 900)
    assert late['result'] == 'FAIL' and not late['checks']['within_sim_limit']
    running = evaluate(_stats(state='SEARCH'), MANIFEST, picked, models, 600, 200, 0.8, 900)
    assert not running['checks']['mission_done']


def test_fail_when_picked_apple_still_in_world_or_counts_disagree():
    picked = MANIFEST[:4]
    ghost = evaluate(_stats(), MANIFEST, picked, WORLD | {'apple_t0_0', 'apple_t2_0'},
                     600, 200, 0.8, 900)
    assert ghost['picked_but_still_in_gz'] == ['apple_t0_0'] and ghost['result'] == 'FAIL'
    off = evaluate(_stats(picked=5), MANIFEST, picked, WORLD, 600, 200, 0.8, 900)
    assert not off['checks']['counts_consistent']
    nogz = evaluate(_stats(), MANIFEST, picked, set(), 600, 200, 0.8, 900)
    assert not nogz['checks']['gz_queried']
