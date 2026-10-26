"""ROS 2 node: follow the orchard waypoint loop using /odom (world frame) and /cmd_vel."""
import csv
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from applepic_navigation.controller import ControllerParams, compute_command, yaw_from_quaternion
from applepic_navigation.route import build_route, load_orchard


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        default_cfg = os.path.join(
            get_package_share_directory('applepic_gazebo'), 'config', 'orchard.yaml')
        self.declare_parameter('orchard_config', default_cfg)
        self.declare_parameter('approach_distance', 2.2)
        self.declare_parameter('clearance', 2.0)
        self.declare_parameter('loops', 1)            # 0 = loop forever
        self.declare_parameter('dwell', 1.0)          # s stopped at each tree waypoint
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('trajectory_csv', '')  # optional pose log for verification
        gp = self.get_parameter

        self.config = load_orchard(gp('orchard_config').value)
        self.approach_distance = gp('approach_distance').value
        self.clearance = gp('clearance').value
        self.loops = gp('loops').value
        self.dwell = gp('dwell').value
        self.params = ControllerParams(
            max_linear=gp('max_linear').value,
            max_angular=gp('max_angular').value,
            goal_tolerance=gp('goal_tolerance').value)

        self.route = None
        self.index = 0
        self.loop_count = 0
        self.pose = None
        self.dwell_until = None
        self.done = False

        self.csv_file = None
        path = gp('trajectory_csv').value
        if path:
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['t', 'x', 'y', 'yaw', 'waypoint'])

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(0.1, self._step)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.route is None:
            self.route = build_route(self.config, self.approach_distance, self.clearance,
                                     start=self.pose[:2])
            desc = ', '.join(f'{lbl}({x:.1f},{y:.1f})' for x, y, lbl in self.route)
            self.get_logger().info(f'Route: {desc}')
        if self.csv_file:
            label = self.route[self.index][2] if not self.done else 'done'
            self.csv.writerow([f'{self._now():.2f}', f'{p.x:.3f}', f'{p.y:.3f}',
                               f'{self.pose[2]:.3f}', label])

    def _status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _step(self):
        if self.pose is None or self.route is None or self.done:
            return
        if self.dwell_until is not None:
            if self._now() < self.dwell_until:
                self.cmd_pub.publish(Twist())
                return
            self.dwell_until = None
        gx, gy, label = self.route[self.index]
        v, w, reached = compute_command(*self.pose, gx, gy, self.params)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)
        if not reached:
            return
        self._status(f'reached {label} ({gx:.2f}, {gy:.2f}) at '
                     f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')
        if label != 'via':
            self.dwell_until = self._now() + self.dwell
        self.index += 1
        if self.index < len(self.route):
            return
        self.loop_count += 1
        if self.loops and self.loop_count >= self.loops:
            self.done = True
            self.cmd_pub.publish(Twist())
            self._status(f'done: {self.loop_count} loop(s) completed')
            return
        self.index = 0


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv_file:
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
