"""ROS node: "magic" gripper and basket.

When the gripper point (`offset` m ahead of the base, ground-truth pose from /odom) comes within `pick_radius`
of an apple model (ground-truth poses from gz `/world/<world>/pose/info`), the apple is removed
from the simulation (gz service `/world/<world>/remove`, UserCommands plugin) and the basket
counter is incremented, unless the basket is full. The remove request runs in a
worker thread; its outcome is settled by the pose stream (gz executes it even when the reply times
out under load): the apple counts as picked once it disappears; if it is still there
`remove_timeout + confirm_timeout` s later it may be picked again.

Subscribes: /odom (nav_msgs/Odometry, world frame), gz /world/<world>/pose/info (gz.msgs.Pose_V).
Publishes:  /basket/count (std_msgs/Int32, latched), /basket/full (std_msgs/Bool, latched),
            /basket/picked (std_msgs/String, apple name),
            /basket/picked_point (geometry_msgs/PointStamped, frame base_footprint, stamped
            when the pick was triggered: where that apple was relative to the robot, so that
            the mission can place it with its own pose estimate; a late-confirmed pick may
            arrive once the gripper is already on another apple).
Services:   /basket/unload (std_srvs/Trigger): empty the basket (the mission calls it at the depot).
"""
import math
import subprocess
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from geometry_msgs.msg import PointStamped
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from applepic_mission.basket import Basket
from applepic_mission.picking import apple_in_reach, gripper_position
from applepic_navigation.controller import yaw_from_quaternion


class Gripper(Node):

    def __init__(self):
        super().__init__('gripper')
        self.declare_parameter('world', 'orchard')
        self.declare_parameter('offset', 0.46)        # m, gripper_link ahead of base_footprint
        self.declare_parameter('pick_radius', 0.07)   # m, horizontal gripper-to-apple distance
        self.declare_parameter('capacity', 10)
        self.declare_parameter('apple_prefix', 'apple_')
        self.declare_parameter('enabled', True)
        self.declare_parameter('remove_timeout', 3.0)   # s, gz remove service request
        self.declare_parameter('confirm_timeout', 3.0)  # s, after remove_timeout, before a retry
        gp = self.get_parameter
        world = gp('world').value
        self.offset = float(gp('offset').value)
        self.radius = float(gp('pick_radius').value)
        self.prefix = gp('apple_prefix').value
        self.enabled = bool(gp('enabled').value)
        self.basket = Basket(int(gp('capacity').value))
        self.remove_service = f'/world/{world}/remove'
        self.remove_timeout = float(gp('remove_timeout').value)
        self.remove_timeout_ms = int(1000 * self.remove_timeout)
        self.confirm_timeout = float(gp('confirm_timeout').value)

        self.lock = threading.Lock()
        self.apples = {}          # name -> (x, y), ground truth from gz
        self.removed = set()      # names already removed (pose/info may lag one message)
        self.requested = {}       # name -> ((x, y), monotonic time): removal pending
        self.pick_info = {}       # name -> ((x, y) in base_footprint, ROS stamp) of the request
        self.full_warned = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.count_pub = self.create_publisher(Int32, '/basket/count', latched)
        self.full_pub = self.create_publisher(Bool, '/basket/full', latched)
        self.picked_pub = self.create_publisher(String, '/basket/picked', 10)
        self.picked_point_pub = self.create_publisher(PointStamped, '/basket/picked_point', 10)
        self.create_service(Trigger, '/basket/unload', self._on_unload)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.gz = GzNode()
        topic = f'/world/{world}/pose/info'
        if not self.gz.subscribe(Pose_V, topic, self._on_poses):
            self.get_logger().error(f'cannot subscribe to gz topic {topic}')
        self._publish_count()
        self.get_logger().info(
            f'gripper ready: radius {self.radius} m, capacity {self.basket.capacity}')

    def _on_poses(self, msg):
        # Called from a gz-transport thread.
        apples = {p.name: (p.position.x, p.position.y)
                  for p in msg.pose if p.name.startswith(self.prefix)}
        gone, retry = [], []
        with self.lock:
            now = time.monotonic()
            # A removal counts as soon as the apple leaves the pose stream: gz executes it even
            # when the service reply times out under load, sometimes after the timeout.
            for name, (xy, since) in list(self.requested.items()):
                if name not in apples:
                    gone.append((name, xy))
                elif now - since > self.remove_timeout + self.confirm_timeout:
                    del self.requested[name]      # really not removed: allow a new attempt
                    retry.append(name)
            self.apples = {n: xy for n, xy in apples.items() if n not in self.removed}
        for name in retry:
            self.get_logger().warn(f'{name} still in gz after a remove request, will retry')
        for name, xy in gone:
            self._picked(name, xy)

    def _on_odom(self, msg):
        if not self.enabled:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        g = gripper_position(p.x, p.y, yaw, self.offset)
        with self.lock:
            name = apple_in_reach(
                g, {n: xy for n, xy in self.apples.items() if n not in self.requested},
                self.radius)
            if name is None:
                return
            if self.basket.count + len(self.requested) >= self.basket.capacity:
                if self.basket.full and not self.full_warned:
                    self.get_logger().warn(
                        f'basket full ({self.basket.count}), cannot pick {name}')
                    self.full_warned = True
                return
            xy = self.apples[name]
            dx, dy = xy[0] - p.x, xy[1] - p.y
            c, s = math.cos(yaw), math.sin(yaw)
            self.requested[name] = (xy, time.monotonic())
            self.pick_info[name] = ((c * dx + s * dy, -s * dx + c * dy), msg.header.stamp)
        # The gz request takes up to remove_timeout: never in the executor thread.
        threading.Thread(target=self._remove_async, args=(name, xy), daemon=True).start()

    def _remove_async(self, name, xy):
        if self._remove(name):
            self._picked(name, xy)

    def _picked(self, name, xy):
        with self.lock:
            if name in self.removed:
                return
            self.removed.add(name)
            self.requested.pop(name, None)
            rel, stamp = self.pick_info.pop(name)
            self.apples.pop(name, None)
            self.basket.add()
        self.get_logger().info(
            f'picked {name} -> basket {self.basket.count}/{self.basket.capacity}')
        self.picked_pub.publish(String(data=name))
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = 'base_footprint'
        point.point.x, point.point.y = float(rel[0]), float(rel[1])
        self.picked_point_pub.publish(point)
        self._publish_count()

    def _remove(self, name):
        # `gz service` in a subprocess, not GzNode.request: the Python binding keeps the GIL while
        # it waits, which froze the pose stream and /odom (hence every pick) for whole seconds.
        cmd = ['gz', 'service', '-s', self.remove_service, '--reqtype', 'gz.msgs.Entity',
               '--reptype', 'gz.msgs.Boolean', '--timeout', str(self.remove_timeout_ms),
               '--req', f'name: "{name}" type: MODEL']
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=self.remove_timeout + 5.0).stdout
        except (OSError, subprocess.TimeoutExpired):
            out = ''
        if 'data: true' not in out:
            self.get_logger().info(f'gz remove {name}: no reply, waiting for the pose stream',
                                   throttle_duration_sec=2.0)
            return False
        return True

    def _on_unload(self, request, response):
        with self.lock:   # picks are counted from the gz and worker threads
            n = self.basket.unload()
        self.full_warned = False
        self._publish_count()
        response.success = True
        response.message = f'unloaded {n} apples (total {self.basket.total_unloaded})'
        self.get_logger().info(response.message)
        return response

    def _publish_count(self):
        self.count_pub.publish(Int32(data=self.basket.count))
        self.full_pub.publish(Bool(data=self.basket.full))


def main():
    rclpy.init()
    node = Gripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
