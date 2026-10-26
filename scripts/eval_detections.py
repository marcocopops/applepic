#!/usr/bin/env python3
"""Compare /apples/detections with the ground-truth apple manifest (sim must be running).

Collects detections for a few seconds, matches each one to the nearest manifest apple,
prints per-apple position errors and false positives, saves the debug image as PNG.
Exit code 0 if at least --min-matched apples are matched with mean error < --max-error.

Usage: python3 scripts/eval_detections.py [--seconds 5] [--out artifacts/perception_debug.png]
"""
import argparse
import math
import sys

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
import rclpy
from sensor_msgs.msg import Image
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', default='src/applepic_gazebo/worlds/orchard_apples.yaml')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--match-radius', type=float, default=0.3)
    ap.add_argument('--max-error', type=float, default=0.10)
    ap.add_argument('--min-matched', type=int, default=1)
    ap.add_argument('--out', default='artifacts/perception_debug.png')
    args = ap.parse_args()

    with open(args.manifest) as f:
        apples = yaml.safe_load(f)['apples']

    rclpy.init()
    node = rclpy.create_node('eval_detections')
    msgs, debug = [], []
    node.create_subscription(PoseArray, '/apples/detections', msgs.append, 10)
    node.create_subscription(Image, '/apples/debug_image', debug.append, 1)
    # Wall-clock collection window, after the first detection message arrives.
    start = None
    deadline_first = node.get_clock().now().nanoseconds + 30e9
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        now = node.get_clock().now().nanoseconds
        if start is None:
            if msgs:
                start = now
            elif now > deadline_first:
                break
        elif now - start > args.seconds * 1e9:
            break
    if not msgs:
        print('no /apples/detections received', file=sys.stderr)
        sys.exit(1)

    errors = {}       # apple name -> list of errors
    false_pos = 0
    total = 0
    for m in msgs:
        for p in m.poses:
            total += 1
            best = min(apples, key=lambda a: math.hypot(a['x'] - p.position.x,
                                                         a['y'] - p.position.y))
            e = math.hypot(best['x'] - p.position.x, best['y'] - p.position.y)
            if e <= args.match_radius:
                errors.setdefault(best['name'], []).append(e)
            else:
                false_pos += 1
                print(f'  unmatched detection ({p.position.x:.2f}, {p.position.y:.2f}), '
                      f'nearest {best["name"]} at {e:.2f} m')

    print(f'{len(msgs)} messages, {total} detections, {false_pos} unmatched')
    all_err = []
    for name, errs in sorted(errors.items()):
        all_err += errs
        print(f'  {name}: {len(errs)} hits, mean error {sum(errs) / len(errs):.3f} m, '
              f'max {max(errs):.3f} m')
    mean = sum(all_err) / len(all_err) if all_err else float('inf')
    print(f'matched apples: {len(errors)}, mean error {mean:.3f} m')

    if debug:
        img = CvBridge().imgmsg_to_cv2(debug[-1], desired_encoding='bgr8')
        cv2.imwrite(args.out, img)
        print(f'saved {args.out}')
    node.destroy_node()
    rclpy.shutdown()
    ok = len(errors) >= args.min_matched and mean < args.max_error and false_pos == 0
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
