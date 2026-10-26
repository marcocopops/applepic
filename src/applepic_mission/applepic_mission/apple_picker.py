"""ROS node: pick the apples seen by the camera, one after the other (M5 behaviour).

Fuses /apples/detections into tracks (AppleTracker), drives the gripper onto the nearest
confirmed apple (approach_command) and waits for the gripper node to report the pick on
/basket/picked. An apple not picked within `pick_timeout` s of reaching it (or of starting the
approach plus `approach_timeout`) is marked failed. Idles when the basket is full or when no
confirmed apple is known; the
mission (M6) will add the search pattern around each tree.

Subscribes: /odom, /apples/detections (PoseArray, world frame), /basket/picked, /basket/full.
Publishes:  /cmd_vel, ~/status (std_msgs/String).
"""
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from applepic_mission.apple_tracker import AppleTracker
from applepic_mission.approach import ApproachParams, approach_command
from applepic_navigation.controller import yaw_from_quaternion


class ApplePicker(Node):

    def __init__(self):
        super().__init__('apple_picker')
        self.declare_parameter('offset', 0.46)
        self.declare_parameter('gate', 0.15)
        self.declare_parameter('min_views', 2)
        self.declare_parameter('approach_timeout', 30.0)
        self.declare_parameter('pick_timeout', 3.0)
        gp = self.get_parameter
        self.params = ApproachParams(offset=float(gp('offset').value))
        self.tracker = AppleTracker(gate=float(gp('gate').value),
                                    min_views=int(gp('min_views').value))
        self.approach_timeout = float(gp('approach_timeout').value)
        self.pick_timeout = float(gp('pick_timeout').value)

        self.pose = None
        self.target = None          # track id
        self.target_since = None
        self.reached_since = None
        self.basket_full = False
        self.picked_total = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseArray, '/apples/detections', self._on_detections, 10)
        self.create_subscription(String, '/basket/picked', self._on_picked, 10)
        self.create_subscription(Bool, '/basket/full', self._on_full, latched)
        self.create_timer(0.1, self._step)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_detections(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tracker.update([(p.position.x, p.position.y) for p in msg.poses], stamp)

    def _on_full(self, msg):
        if msg.data and not self.basket_full:
            self._status('basket full: stop picking')
            self._clear_target()
        self.basket_full = msg.data

    def _on_picked(self, msg):
        self.picked_total += 1
        if self.target is not None:
            self.tracker.mark_picked(self.target)
            self._status(f'picked {msg.data} (track {self.target}), total {self.picked_total}')
            self._clear_target()
        else:
            self._status(f'picked {msg.data} on the way, total {self.picked_total}')

    def _clear_target(self):
        self.target = None
        self.target_since = None
        self.reached_since = None

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _step(self):
        if self.pose is None:
            return
        now = self._now()
        if self.basket_full:
            self._stop()
            return
        if self.target is None:
            t = self.tracker.nearest(*self.pose[:2])
            if t is None:
                self._stop()
                return
            self.target, self.target_since = t.id, now
            self._status(f'approaching apple track {t.id} at ({t.x:.2f}, {t.y:.2f})')
        track = self.tracker.get(self.target)
        if now - self.target_since > self.approach_timeout or (
                self.reached_since is not None and now - self.reached_since > self.pick_timeout):
            self._status(f'giving up on apple track {self.target} at ({track.x:.2f}, {track.y:.2f})')
            self.tracker.mark_failed(self.target)
            self._clear_target()
            self._stop()
            return
        lin, ang, reached = approach_command(*self.pose, track.x, track.y, self.params)
        if reached and self.reached_since is None:
            self.reached_since = now
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = lin, ang
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ApplePicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
