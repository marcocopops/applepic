#!/usr/bin/env python3
"""M7 end-to-end scoring: wait for the mission to finish, then score it against the gz world.

Listens to /mission/stats (JSON), /basket/picked and /clock until the mission state is DONE,
--sim-limit simulated seconds elapse or --timeout wall seconds elapse. Then queries the gz model
list and scores the run with `applepic_mission.metrics.evaluate` (written to --out as JSON).
Exits 0 on PASS.
"""
import argparse
import json
import subprocess
import sys
import time

from applepic_mission.metrics import evaluate
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import yaml


def gz_models():
    try:
        out = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True,
                             timeout=20)
    except subprocess.TimeoutExpired:
        return set()
    return {line.strip().lstrip('- ').strip() for line in out.stdout.splitlines()
            if line.strip().startswith('-')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', default='src/applepic_gazebo/worlds/orchard_apples.yaml')
    ap.add_argument('--min-ratio', type=float, default=0.8)
    ap.add_argument('--sim-limit', type=float, default=1200.0, help='simulated seconds')
    ap.add_argument('--timeout', type=float, default=1800.0, help='wall seconds')
    ap.add_argument('--out', default='artifacts/e2e_metrics.json')
    args = ap.parse_args()
    with open(args.manifest) as f:
        manifest = [a['name'] for a in yaml.safe_load(f)['apples']]

    rclpy.init()
    node = Node('eval_e2e')
    picked, stats, clock = [], [{}], [None, None]  # clock = [first, last] sim time
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if stats[0] and clock[0] is None:  # count from the mission's first transition
            clock[0] = t
        clock[1] = t

    node.create_subscription(String, '/basket/picked', lambda m: picked.append(m.data), 10)
    node.create_subscription(String, '/mission/stats',
                             lambda m: stats.__setitem__(0, json.loads(m.data)), latched)
    node.create_subscription(Clock, '/clock', on_clock, 10)

    def sim_s():
        return clock[1] - clock[0] if clock[0] is not None else 0.0

    start = time.monotonic()
    last_print = start
    while (time.monotonic() - start < args.timeout and sim_s() <= args.sim_limit
           and stats[0].get('state') != 'DONE'):
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() - last_print > 60:
            last_print = time.monotonic()
            print(f'[wall {last_print - start:.0f} s, sim {sim_s():.0f} s] picked {len(picked)}, '
                  f'state {stats[0].get("state")}, tree {stats[0].get("tree")}', flush=True)
    wall = time.monotonic() - start
    models = gz_models()
    node.destroy_node()
    rclpy.try_shutdown()

    m = evaluate(stats[0], manifest, picked, models, sim_s(), wall, args.min_ratio,
                 args.sim_limit)
    with open(args.out, 'w') as f:
        json.dump(m, f, indent=2)
    print(json.dumps(m, indent=2))
    print(f'unloaded {m["apples_unloaded"]}/{m["apples_total"]} = {m["unloaded_ratio"]:.0%} '
          f'(min {args.min_ratio:.0%}), sim {m["sim_time_s"]} s (limit {args.sim_limit:.0f} s), '
          f'{m["distance_m"]} m -> {m["result"]}')
    return 0 if m['result'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
