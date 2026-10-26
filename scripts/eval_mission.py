#!/usr/bin/env python3
"""Wait for the mission to finish, then report metrics and cross-check them with the gz world.

Listens to /mission/stats (JSON) and /basket/picked until the mission state is DONE or
--timeout s (wall) elapse. Exits 0 if the mission is DONE, at least --min-ratio of the manifest
apples were unloaded at the depot and no picked apple model remains in gz.
"""
import argparse
import json
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml


def gz_models():
    out = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True, timeout=20)
    return {line.strip().lstrip('- ').strip() for line in out.stdout.splitlines()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', default='src/applepic_gazebo/worlds/orchard_apples.yaml')
    ap.add_argument('--min-ratio', type=float, default=0.8)
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('--out', default='artifacts/mission_metrics.json')
    args = ap.parse_args()
    with open(args.manifest) as f:
        apples = [a['name'] for a in yaml.safe_load(f)['apples']]

    rclpy.init()
    node = Node('eval_mission')
    picked, stats = [], [{}]
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/basket/picked', lambda m: picked.append(m.data), 10)
    node.create_subscription(String, '/mission/stats',
                             lambda m: stats.__setitem__(0, json.loads(m.data)), latched)
    start = time.monotonic()
    last_print = start
    while time.monotonic() - start < args.timeout and stats[0].get('state') != 'DONE':
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() - last_print > 60:
            last_print = time.monotonic()
            print(f'[{last_print - start:.0f} s] picked {len(picked)}, stats {stats[0]}', flush=True)
    wall = time.monotonic() - start
    models = gz_models()
    node.destroy_node()
    rclpy.try_shutdown()

    s = stats[0]
    left = sorted(m for m in models if m.startswith('apple_'))
    still_there = [n for n in picked if n in models]
    metrics = dict(s, apples_total=len(apples), picked_msgs=len(picked),
                   apples_left_in_gz=len(left), apples_left=left, wall_s=round(wall, 1))
    with open(args.out, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))
    ratio = s.get('unloaded', 0) / len(apples)
    ok = (s.get('state') == 'DONE' and ratio >= args.min_ratio and not still_there
          and len(models) > 0)
    print(f'unloaded {s.get("unloaded", 0)}/{len(apples)} = {ratio:.0%} '
          f'(min {args.min_ratio:.0%}) -> {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
