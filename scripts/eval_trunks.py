#!/usr/bin/env python3
"""Error of the trunk positions measured by the camera, against the ground truth.

Run next to a simulation: /trees/trunks (relative to the robot) is placed in the world with the
ground-truth /odom and compared with the nearest tree. SIGINT or SIGTERM prints the statistics.
Usage: scripts/eval_trunks.py [--output file]
"""
import argparse
import json
import math
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node

from applepic_gazebo.orchard import load_config

TREES = None


class Eval(Node):

    def __init__(self):
        super().__init__('eval_trunks')
        self.pose = None
        self.errors = []
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseArray, '/trees/trunks', self._on_trunks, 10)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))

    def _on_trunks(self, msg):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        for p in msg.poses:
            world = (x + c * p.position.x - s * p.position.y,
                     y + s * p.position.x + c * p.position.y)
            self.errors.append(min(math.dist(world, t) for t in TREES))


def main():
    global TREES
    parser = argparse.ArgumentParser()
    parser.add_argument('--output')
    parser.add_argument('--config', default='src/applepic_gazebo/config/orchard.yaml')
    args = parser.parse_args()
    TREES = [tuple(t) for t in load_config(args.config)['trees']]
    rclpy.init()
    node = Eval()
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.2)
    errors = sorted(node.errors)
    n = len(errors)
    result = {'samples': n}
    if n:
        result.update(mean_m=round(sum(errors) / n, 4), p95_m=round(errors[int(0.95 * n)], 4),
                      max_m=round(errors[-1], 4), over_30cm=sum(e > 0.3 for e in errors))
    print(json.dumps(result))
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(result, f)


if __name__ == '__main__':
    main()
