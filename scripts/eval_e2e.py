#!/usr/bin/env python3
"""M7 end-to-end scoring: wait for the mission to finish, then score it against the gz world.

Listens to /mission/stats (JSON), /basket/picked, /clock and the pose estimate
(/localization/odom, compared with the ground truth /odom) until the mission state is DONE,
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
from applepic_navigation.controller import yaw_from_quaternion
from applepic_navigation.localization import PoseErrorTracker
from nav_msgs.msg import Odometry
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


def gz_pose(name):
    """[x, y] of a gz model from `gz model -m <name> -p`, or None."""
    try:
        out = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True,
                             timeout=20).stdout
    except subprocess.TimeoutExpired:
        return None
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if 'XYZ' in line and i + 1 < len(lines):
            vals = lines[i + 1].strip().strip('[]').split()
            try:
                return [round(float(v), 3) for v in vals[:2]]
            except ValueError:
                return None
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', default='src/applepic_gazebo/worlds/orchard_apples.yaml')
    ap.add_argument('--min-ratio', type=float, default=0.8)
    ap.add_argument('--sim-limit', type=float, default=1200.0, help='simulated seconds')
    ap.add_argument('--timeout', type=float, default=1800.0, help='wall seconds')
    ap.add_argument('--seed', type=int, default=None, help='orchard seed, recorded in the metrics')
    ap.add_argument('--localization', default='truth', help='recorded in the metrics')
    ap.add_argument('--gps-noise', type=float, default=None)
    ap.add_argument('--gps-bias', type=float, default=None)
    ap.add_argument('--truth-csv', default='', help='ground-truth trajectory (t, x, y) at 2 Hz')
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
    loc_error = PoseErrorTracker()

    def odom_sample(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    truth_rows = []

    def on_truth(msg):
        t, pose = odom_sample(msg)
        loc_error.add_truth(t, pose)
        if not truth_rows or t - truth_rows[-1][0] >= 0.5:
            truth_rows.append((t, pose[0], pose[1]))

    node.create_subscription(Odometry, '/odom', on_truth, 50)
    node.create_subscription(Odometry, '/localization/odom',
                             lambda m: loc_error.add_estimate(*odom_sample(m)), 50)

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
    left_poses = {n: gz_pose(n) for n in manifest if n in models}
    node.destroy_node()
    rclpy.try_shutdown()

    m = evaluate(stats[0], manifest, picked, models, sim_s(), wall, args.min_ratio,
                 args.sim_limit)
    if args.seed is not None:
        m['seed'] = args.seed
    m['apples_left_final_xy'] = left_poses
    m['localization'] = {'mode': args.localization, 'gps_noise_m': args.gps_noise,
                         'gps_bias_m': args.gps_bias, **loc_error.summary()}
    with open(args.out, 'w') as f:
        json.dump(m, f, indent=2)
    if args.truth_csv:
        with open(args.truth_csv, 'w') as f:
            f.write('t,x,y\n')
            f.writelines(f'{t:.2f},{x:.3f},{y:.3f}\n' for t, x, y in truth_rows)
    print(json.dumps(m, indent=2))
    print(f'unloaded {m["apples_unloaded"]}/{m["apples_total"]} = {m["unloaded_ratio"]:.0%} '
          f'(min {args.min_ratio:.0%}), sim {m["sim_time_s"]} s (limit {args.sim_limit:.0f} s), '
          f'{m["distance_m"]} m -> {m["result"]}')
    return 0 if m['result'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
