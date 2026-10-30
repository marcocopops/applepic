#!/usr/bin/env python3
"""Log each bumper contact episode next to a running sim: what was hit, where around the chassis.

Diagnostic only, it reads the ground truth. One CSV line per new contact (gap > 0.5 s), stops on SIGTERM.
Usage: scripts/log_bumps.py [--output artifacts/bumps.csv]
"""
import argparse
import math
import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts


class BumpLogger(Node):

    def __init__(self, output):
        super().__init__('bump_logger')
        self.out = open(output, 'w', buffering=1)
        self.out.write('sim_t,truth_x,truth_y,truth_yaw,v,w,fwd,left,other\n')
        self.odom = None
        self.last = -1e9
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper', self._on_bumper, 10)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_bumper(self, msg):
        if self.odom is None:
            return
        for c in msg.contacts:
            names = (c.collision1.name, c.collision2.name)
            if any('ground' in n for n in names) or not c.positions:
                continue
            o = self.odom
            t = o.header.stamp.sec + 1e-9 * o.header.stamp.nanosec
            gap, self.last = t - self.last, t
            if gap < 0.5:
                return
            q = o.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            x, y = o.pose.pose.position.x, o.pose.pose.position.y
            dx, dy = c.positions[0].x - x, c.positions[0].y - y
            fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            other = names[1] if 'applepic' in names[0] else names[0]
            self.out.write(f'{t:.2f},{x:.2f},{y:.2f},{yaw:.2f},{o.twist.twist.linear.x:.2f},'
                           f'{o.twist.twist.angular.z:.2f},{fwd:.2f},{left:.2f},{other}\n')
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='artifacts/bumps.csv')
    args = ap.parse_args()
    rclpy.init()
    node = BumpLogger(args.output)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    node.out.close()


if __name__ == '__main__':
    main()
