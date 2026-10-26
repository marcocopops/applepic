"""ROS node: run the orchard mission (trees -> search -> pick -> depot -> unload) with Mission.

Wraps the pure state machine `applepic_mission.mission.Mission`: feeds it /odom, apple detections
and basket events, publishes its velocity commands and calls /basket/unload at the depot. The
magic pick itself is done by the `gripper` node.

Subscribes: /odom, /apples/detections (PoseArray, world frame), /basket/picked, /basket/full.
Publishes:  /cmd_vel, ~/state (std_msgs/String, latched), ~/status (std_msgs/String),
            ~/stats (std_msgs/String, JSON, latched, refreshed on every transition).
Client:     /basket/unload (std_srvs/Trigger).
"""
import csv
import json
import os
import re

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from applepic_mission.apple_tracker import AppleTracker
from applepic_mission.approach import ApproachParams
from applepic_mission.mission import DONE, Mission, MissionParams
from applepic_mission.picking import gripper_position
from applepic_navigation.controller import ControllerParams, yaw_from_quaternion
from applepic_navigation.route import load_orchard


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        default_cfg = os.path.join(
            get_package_share_directory('applepic_gazebo'), 'config', 'orchard.yaml')
        self.declare_parameter('orchard_config', default_cfg)
        self.declare_parameter('offset', 0.46)
        self.declare_parameter('gate', 0.15)
        self.declare_parameter('min_views', 2)
        self.declare_parameter('trajectory_csv', '')
        self.declare_parameter('exit_when_done', False)
        defaults = MissionParams()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        gp = self.get_parameter

        config = load_orchard(gp('orchard_config').value)
        params = MissionParams(**{n: type(v)(gp(n).value) for n, v in vars(defaults).items()})
        self.offset = float(gp('offset').value)
        self.mission = Mission(
            config['trees'], config['depot']['position'], params, ControllerParams(),
            ApproachParams(offset=self.offset),
            AppleTracker(gate=float(gp('gate').value), min_views=int(gp('min_views').value)))
        self.exit_when_done = bool(gp('exit_when_done').value)

        self.pose = None
        self.start_time = None
        self.distance = 0.0
        self.unload_future = None
        self.last_state = None
        self.last_transition_count = 0

        self.csv_file = None
        path = gp('trajectory_csv').value
        if path:
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['t', 'x', 'y', 'yaw', 'state'])

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '~/state', latched)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.stats_pub = self.create_publisher(String, '~/stats', latched)
        self.unload_client = self.create_client(Trigger, '/basket/unload')
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseArray, '/apples/detections', self._on_detections, 10)
        self.create_subscription(String, '/basket/picked', self._on_picked, 10)
        self.create_subscription(Bool, '/basket/full', self._on_full, latched)
        self.create_timer(0.1, self._step)

    @property
    def done(self):
        return self.mission.state == DONE

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.pose is not None:
            self.distance += ((pose[0] - self.pose[0]) ** 2 + (pose[1] - self.pose[1]) ** 2) ** 0.5
        self.pose = pose
        if self.csv_file:
            self.csv.writerow([f'{self._now():.2f}', f'{p.x:.3f}', f'{p.y:.3f}',
                               f'{pose[2]:.3f}', self.mission.state or 'INIT'])

    def _on_detections(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.mission.on_detections([(p.position.x, p.position.y) for p in msg.poses], stamp)

    def _on_full(self, msg):
        self.mission.on_basket_full(msg.data)

    def _on_picked(self, msg):
        if self.pose is None:
            return
        self.mission.on_picked(gripper_position(*self.pose, self.offset))
        self._status(f'picked {msg.data} (total {self.mission.stats.picked}, '
                     f'carried {self.mission.carried})')

    def _on_unload_done(self, future):
        self.unload_future = None
        res = future.result()
        if res is None or not res.success:
            self.get_logger().warn('unload failed, will retry')
            return
        m = re.match(r'unloaded (\d+)', res.message)
        self._status(res.message)
        self.mission.on_unloaded(int(m.group(1)) if m else None)

    def _stats(self):
        s = self.mission.stats
        return {
            'state': self.mission.state, 'tree': self.mission.tree,
            'picked': s.picked, 'unloaded': s.unloaded, 'failed': s.failed,
            'loops_done': s.loops_done, 'trees_searched': s.trees_searched,
            'distance_m': round(self.distance, 2),
            'elapsed_s': round(self._now() - self.start_time, 1) if self.start_time else 0.0,
        }

    def _step(self):
        if self.pose is None:
            return
        now = self._now()
        if self.start_time is None:
            self.start_time = now
        v, w = self.mission.step(self.pose, now)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)

        if (self.mission.unload_requested and self.unload_future is None
                and self.unload_client.service_is_ready()):
            self.unload_future = self.unload_client.call_async(Trigger.Request())
            self.unload_future.add_done_callback(self._on_unload_done)

        transitions = self.mission.stats.transitions
        if len(transitions) != self.last_transition_count:
            for state, tree, note in transitions[self.last_transition_count:]:
                self._status(f'-> {state} (tree {tree}){": " + note if note else ""}')
            self.last_transition_count = len(transitions)
            self.state_pub.publish(String(data=self.mission.state))
            stats = json.dumps(self._stats())
            self.stats_pub.publish(String(data=stats))
            if self.done:
                self._status(f'mission done: {stats}')


def main():
    rclpy.init()
    node = MissionNode()
    try:
        while rclpy.ok() and not (node.exit_when_done and node.done):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        if node.csv_file:
            node.csv_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
