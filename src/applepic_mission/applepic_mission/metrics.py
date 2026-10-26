"""Pure-Python end-to-end mission scoring (no rclpy), used by the M7 scenario.

`evaluate` merges the mission's own stats with what an external observer saw (apple names
reported on /basket/picked, apple models still present in gz, sim/wall durations) and decides
PASS/FAIL against the scenario thresholds.
"""
import re

_TREE_RE = re.compile(r'^apple_t(\d+)_\d+$')


def apple_tree(name):
    """Tree index encoded in a generated apple model name (`apple_t<tree>_<i>`), else None."""
    m = _TREE_RE.match(name)
    return int(m.group(1)) if m else None


def per_tree(names):
    """Count apple names per tree index: {tree: count}."""
    counts = {}
    for n in names:
        t = apple_tree(n)
        if t is not None:
            counts[t] = counts.get(t, 0) + 1
    return dict(sorted(counts.items()))


def evaluate(stats, manifest, picked, models, sim_s, wall_s, min_ratio, sim_limit):
    """Score one run.

    stats:     last /mission/stats dict (state, picked, unloaded, failed, distance_m, ...)
    manifest:  names of all apples spawned in the world
    picked:    apple names reported by the gripper on /basket/picked
    models:    model names currently in gz (empty if the query failed)
    sim_s:     simulated seconds since the mission started; wall_s: wall seconds
    min_ratio: required fraction of manifest apples unloaded at the depot
    sim_limit: maximum simulated duration (s) for the mission to reach DONE
    """
    manifest = list(manifest)
    total = len(manifest)
    unloaded = int(stats.get('unloaded', 0))
    ratio = unloaded / total if total else 0.0
    left = sorted(n for n in manifest if n in models)
    ghost = sorted(n for n in set(picked) if n in models)  # reported picked but still in gz
    checks = {
        'mission_done': stats.get('state') == 'DONE',
        'ratio_ok': ratio >= min_ratio,
        'within_sim_limit': sim_s <= sim_limit,
        'gz_queried': len(models) > 0,
        'picked_removed_from_gz': not ghost,
        'counts_consistent': len(set(picked)) == int(stats.get('picked', 0)) == len(picked),
    }
    return {
        'result': 'PASS' if all(checks.values()) else 'FAIL',
        'checks': checks,
        'apples_total': total,
        'apples_picked': int(stats.get('picked', 0)),
        'apples_unloaded': unloaded,
        'unloaded_ratio': round(ratio, 3),
        'min_ratio': min_ratio,
        'pick_failures': int(stats.get('failed', 0)),
        'distance_m': float(stats.get('distance_m', 0.0)),
        'sim_time_s': round(sim_s, 1),
        'sim_limit_s': sim_limit,
        'wall_time_s': round(wall_s, 1),
        'effective_rtf': round(sim_s / wall_s, 2) if wall_s > 0 else 0.0,
        'final_state': stats.get('state'),
        'loops_done': int(stats.get('loops_done', 0)),
        'picked_per_tree': per_tree(picked),
        'total_per_tree': per_tree(manifest),
        'apples_left': left,
        'picked_but_still_in_gz': ghost,
    }
