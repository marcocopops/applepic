"""ROS node: "magic" gripper and basket.

When the gripper point (`offset` m ahead of the base, pose from /odom) comes within `pick_radius`
of an apple model (ground-truth poses from gz `/world/<world>/pose/info`), the apple is removed
from the simulation (gz service `/world/<world>/remove`, UserCommands plugin) and the basket
counter is incremented, unless the basket is full.

Subscribes: /odom (nav_msgs/Odometry, world frame), gz /world/<world>/pose/info (gz.msgs.Pose_V).
Publishes:  /basket/count (std_msgs/Int32, latched), /basket/full (std_msgs/Bool, latched),
            /basket/picked (std_msgs/String, apple name).
Services:   /basket/unload (std_srvs/Trigger): empty the basket (the mission calls it at the depot).
"""
import threading

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
        gp = self.get_parameter
        world = gp('world').value
        self.offset = float(gp('offset').value)
        self.radius = float(gp('pick_radius').value)
        self.prefix = gp('apple_prefix').value
        self.enabled = bool(gp('enabled').value)
        self.basket = Basket(int(gp('capacity').value))
        self.remove_service = f'/world/{world}/remove'

        self.lock = threading.Lock()
        self.apples = {}          # name -> (x, y), ground truth from gz
        self.removed = set()      # names already removed (pose/info may lag one message)
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
        with self.lock:
            self.apples = {n: xy for n, xy in apples.items() if n not in self.removed}

    def _on_odom(self, msg):
        if not self.enabled:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        g = gripper_position(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), self.offset)
        with self.lock:
            name = apple_in_reach(g, self.apples, self.radius)
        if name is None:
            return
        if self.basket.full:
            if not self.full_warned:
                self.get_logger().warn(f'basket full ({self.basket.count}), cannot pick {name}')
                self.full_warned = True
            return
        if not self._remove(name):
            return
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
        ok, rep = self.gz.request(self.remove_service, req, Entity, Boolean, 1000)
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
