#!/usr/bin/env python3
"""Wait for the gripper to fill the basket, then cross-check /basket/count with the gz world.

Exits 0 if at least --min-picked apples were picked, the count matches the number of
/basket/picked messages, and every picked apple model is absent from `gz model --list`.
"""
import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String


def gz_models():
    out = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True, timeout=20)
    return {line.strip().lstrip('- ').strip() for line in out.stdout.splitlines()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--min-picked', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--settle', type=float, default=5.0, help='s to wait after reaching min')
    args = ap.parse_args()

    rclpy.init()
    node = Node('eval_picking')
    picked, count = [], [None]
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/basket/picked', lambda m: picked.append(m.data), 10)
    node.create_subscription(Int32, '/basket/count', lambda m: count.__setitem__(0, m.data),
                             latched)
    start = time.monotonic()
    reached_at = None
    while time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if reached_at is None and len(picked) >= args.min_picked:
            reached_at = time.monotonic()
        if reached_at is not None and time.monotonic() - reached_at > args.settle:
            break
    elapsed = time.monotonic() - start
    models = gz_models()
    node.destroy_node()
    rclpy.try_shutdown()

    still_there = [n for n in picked if n in models]
    apples_left = sorted(m for m in models if m.startswith('apple_'))
    print(f'picked {len(picked)} apples in {elapsed:.0f} s (wall): {picked}')
    print(f'/basket/count = {count[0]}; apple models left in gz: {len(apples_left)}')
    ok = (len(picked) >= args.min_picked and count[0] == len(picked) and not still_there
          and len(models) > 0)
    if still_there:
        print(f'FAIL: picked apples still in gz: {still_there}')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
