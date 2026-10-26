"""ROS node: "magic" gripper and basket.

When the gripper point (`offset` m ahead of the base, pose from /odom) comes within `pick_radius`
of an apple model (ground-truth poses from gz `/world/<world>/pose/info`), the apple is removed
from the simulation (gz service `/world/<world>/remove`, UserCommands plugin) and the basket
counter is incremented, unless the basket is full. A remove request that fails or times out
(gz may still have executed it under load) is settled by the pose stream: the apple counts as
picked if it disappears within `confirm_timeout` s, otherwise it may be picked again.

Subscribes: /odom (nav_msgs/Odometry, world frame), gz /world/<world>/pose/info (gz.msgs.Pose_V).
Publishes:  /basket/count (std_msgs/Int32, latched), /basket/full (std_msgs/Bool, latched),
            /basket/picked (std_msgs/String, apple name).
Services:   /basket/unload (std_srvs/Trigger): empty the basket (the mission calls it at the depot).
"""
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
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
        self.declare_parameter('confirm_timeout', 3.0)  # s, wait for an uncertain removal
        gp = self.get_parameter
        world = gp('world').value
        self.offset = float(gp('offset').value)
        self.radius = float(gp('pick_radius').value)
        self.prefix = gp('apple_prefix').value
        self.enabled = bool(gp('enabled').value)
        self.basket = Basket(int(gp('capacity').value))
        self.remove_service = f'/world/{world}/remove'
        self.remove_timeout_ms = int(1000 * float(gp('remove_timeout').value))
        self.confirm_timeout = float(gp('confirm_timeout').value)

        self.lock = threading.Lock()
        self.apples = {}          # name -> (x, y), ground truth from gz
        self.removed = set()      # names already removed (pose/info may lag one message)
        self.uncertain = {}       # name -> monotonic time of a failed remove request
        self.full_warned = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.count_pub = self.create_publisher(Int32, '/basket/count', latched)
        self.full_pub = self.create_publisher(Bool, '/basket/full', latched)
        self.picked_pub = self.create_publisher(String, '/basket/picked', 10)
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
        gone = []
        with self.lock:
            now = time.monotonic()
            for name, since in list(self.uncertain.items()):
                if name not in apples:
                    gone.append(name)
                    del self.uncertain[name]
                elif now - since > self.confirm_timeout:
                    del self.uncertain[name]      # really not removed: allow a new attempt
            self.apples = {n: xy for n, xy in apples.items() if n not in self.removed}
        for name in gone:
            self.get_logger().info(f'{name} removed from gz despite the failed request')
            self._picked(name)

    def _on_odom(self, msg):
        if not self.enabled:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        g = gripper_position(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), self.offset)
        with self.lock:
            name = apple_in_reach(
                g, {n: xy for n, xy in self.apples.items() if n not in self.uncertain},
                self.radius)
        if name is None:
            return
        if self.basket.full:
            if not self.full_warned:
                self.get_logger().warn(f'basket full ({self.basket.count}), cannot pick {name}')
                self.full_warned = True
            return
        if not self._remove(name):
            with self.lock:
                self.uncertain[name] = time.monotonic()
            return
        self._picked(name)

    def _picked(self, name):
        with self.lock:
            self.removed.add(name)
            self.apples.pop(name, None)
            self.basket.add()
        self.get_logger().info(
            f'picked {name} -> basket {self.basket.count}/{self.basket.capacity}')
        self.picked_pub.publish(String(data=name))
        self._publish_count()

    def _remove(self, name):
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        ok, rep = self.gz.request(self.remove_service, req, Entity, Boolean,
                                  self.remove_timeout_ms)
        if not (ok and rep.data):
            self.get_logger().warn(f'gz remove {name} failed', throttle_duration_sec=2.0)
            return False
        return True

    def _on_unload(self, request, response):
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
