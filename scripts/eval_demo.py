#!/usr/bin/env python3
"""M8 check: waits for the demo visualisation topics and validates their content.

Passes once /mission_viz/markers holds the full orchard scene plus the HUD, /mission_viz/path has
grown to --min-path points and /mission/stats reports at least --min-picked apples.
"""
import argparse
import json
import sys
import time

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class DemoProbe(Node):

    def __init__(self):
        super().__init__('demo_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = None
        self.path_len = 0
        self.stats = {}
        self.create_subscription(MarkerArray, '/mission_viz/markers', self._on_markers, latched)
        self.create_subscription(Path, '/mission_viz/path', self._on_path, 10)
        self.create_subscription(String, '/mission/stats', self._on_stats, latched)

    def _on_markers(self, msg):
        self.markers = msg

    def _on_path(self, msg):
        self.path_len = len(msg.poses)

    def _on_stats(self, msg):
        self.stats = json.loads(msg.data)


def summarize(markers):
    counts, hud = {}, None
    for m in markers.markers:
        counts[m.ns] = counts.get(m.ns, 0) + 1
        if m.ns == 'hud':
            hud = m.text
    return counts, hud


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trees', type=int, default=6)
    ap.add_argument('--min-path', type=int, default=20)
    ap.add_argument('--min-picked', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    node = DemoProbe()
    expected = {'trunk': args.trees, 'canopy': args.trees, 'search_ring': args.trees,
                'label': args.trees + 1, 'depot': 1, 'hud': 1}
    deadline = time.monotonic() + args.timeout
    ok = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.markers is None:
            continue
        counts, _ = summarize(node.markers)
        if (counts == expected and node.path_len >= args.min_path
                and node.stats.get('picked', 0) >= args.min_picked):
            ok = True
            break
    counts, hud = summarize(node.markers) if node.markers else ({}, None)
    print(f'markers per namespace: {counts} (expected {expected})')
    print(f'path points: {node.path_len} (min {args.min_path})')
    print(f'mission stats: {node.stats}')
    print('HUD:\n  ' + (hud or '<none>').replace('\n', '\n  '))
    print('PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
