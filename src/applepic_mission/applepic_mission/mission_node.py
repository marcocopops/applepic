"""ROS node: run the orchard mission (trees -> search -> pick -> depot -> unload) with Mission.

Wraps the pure state machine `applepic_mission.mission.Mission`: feeds it the pose estimate, apple detections
and basket events, publishes its velocity commands and calls /basket/unload at the depot. The
magic pick itself is done by the `gripper` node.

Subscribes: /localization/odom (map frame), /apples/detections (PoseArray, map frame),
            /localization/trunk_fix, /basket/full, /basket/picked_point (where the picked apple was, relative to the
            robot when the pick was triggered: placed in the map with the pose at that time),
            TF map -> odom (apple tracks follow its changes: anchored to the odometry).
Publishes:  /cmd_vel, ~/state (std_msgs/String, latched), ~/status (std_msgs/String),
            ~/stats (std_msgs/String, JSON, latched, on every transition and every
            `stats_period` s so the HUD distance/time stay live).
Client:     /basket/unload (std_srvs/Trigger).
"""
import bisect
from collections import deque
import csv
import json
import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros

from applepic_mission.apple_tracker import AppleTracker
from applepic_mission.approach import ApproachParams
from applepic_mission.mission import DONE, Mission, MissionParams
from applepic_navigation.controller import ControllerParams, yaw_from_quaternion
from applepic_navigation.localization import map_shift
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
        self.declare_parameter('stats_period', 1.0)
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
        self.history = deque(maxlen=500)   # (t, pose, map -> odom) at the pose rate: ~10 s
        self.map_odom = (0.0, 0.0, 0.0)    # latest map -> odom (tx, ty, theta)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.start_time = None
        self.distance = 0.0
        self.unload_future = None
        self.last_state = None
        self.last_transition_count = 0
        self.stats_period = float(gp('stats_period').value)
        self.last_stats_time = None

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
        self.create_subscription(Odometry, '/localization/odom', self._on_odom, 10)
        self.create_subscription(PoseArray, '/apples/detections', self._on_detections, 10)
        self.create_subscription(PointStamped, '/basket/picked_point', self._on_picked_point, 10)
        self.create_subscription(Bool, '/basket/full', self._on_full, latched)
        self.create_subscription(PointStamped, '/localization/trunk_fix',
                                 lambda _: self.mission.on_trunk_fix(), 10)
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
        self.history.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, pose,
                             self.map_odom))
        if self.csv_file:
            self.csv.writerow([f'{self._now():.2f}', f'{p.x:.3f}', f'{p.y:.3f}',
                               f'{pose[2]:.3f}', self.mission.state or 'INIT'])

    def _on_detections(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.mission.on_detections([(p.position.x, p.position.y) for p in msg.poses], stamp)

    def _on_full(self, msg):
        self.mission.on_basket_full(msg.data)

    def _pose_at(self, t):
        """Estimated pose closest to time t (latest if t is newer than the history)."""
        if not self.history:
            return self.pose, self.map_odom
        times = [h[0] for h in self.history]
        i = bisect.bisect_left(times, t)
        if i == len(times):
            i -= 1
        elif i > 0 and t - times[i - 1] < times[i] - t:
            i -= 1
        return self.history[i][1:]

    def _on_picked_point(self, msg):
        x, y = msg.point.x, msg.point.y
        if msg.header.frame_id == 'base_footprint':
            pose, map_odom = self._pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if pose is None:
                return
            c, s = math.cos(pose[2]), math.sin(pose[2])
            # In the map as it was at pick time, then moved like the tracks since then.
            x, y = map_shift(map_odom, self.map_odom)(
                (pose[0] + c * x - s * y, pose[1] + s * x + c * y))
        self.mission.on_picked((x, y))
        self._status(f'picked apple at ({x:.2f}, {y:.2f}) (total {self.mission.stats.picked}, '
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

    def _update_map_odom(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        q = tf.transform.rotation
        new = (tf.transform.translation.x, tf.transform.translation.y,
               yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if new != self.map_odom:
            self.mission.on_map_shift(map_shift(self.map_odom, new))
            self.map_odom = new

    def _step(self):
        if self.pose is None:
            return
        self._update_map_odom()
        now = self._now()
        if self.start_time is None:
            self.start_time = self.last_stats_time = now
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
            stats = self._publish_stats(now)
            if self.done:
                self._status(f'mission done: {stats}')
        elif (self.stats_period > 0 and not self.done
              and now - self.last_stats_time >= self.stats_period):
            self._publish_stats(now)

    def _publish_stats(self, now):
        self.last_stats_time = now
        stats = json.dumps(self._stats())
        self.stats_pub.publish(String(data=stats))
        return stats


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
