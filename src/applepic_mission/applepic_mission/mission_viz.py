"""ROS node: RViz helpers for the demo (orchard markers, robot trajectory, mission HUD).

Subscribes: /odom, /mission/stats (JSON, latched), /basket/count (latched).
Publishes:  ~/path (nav_msgs/Path, frame odom), ~/markers (visualization_msgs/MarkerArray, latched).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from applepic_mission.viz import circle_points, hud_text, PathRecorder, scene_items
from applepic_navigation.route import load_orchard

COLORS = {
    'trunk': (0.45, 0.30, 0.15, 1.0),
    'canopy': (0.15, 0.55, 0.15, 0.35),
    'search_ring': (1.0, 0.85, 0.1, 0.8),
    'depot': (0.2, 0.4, 1.0, 0.6),
    'label': (1.0, 1.0, 1.0, 1.0),
    'hud': (1.0, 1.0, 0.3, 1.0),
}


def _color(kind):
    return ColorRGBA(**dict(zip('rgba', COLORS[kind])))


class MissionViz(Node):

    def __init__(self):
        super().__init__('mission_viz')
        default_cfg = os.path.join(
            get_package_share_directory('applepic_gazebo'), 'config', 'orchard.yaml')
        self.declare_parameter('orchard_config', default_cfg)
        self.declare_parameter('search_radius', 1.9)
        self.declare_parameter('capacity', 0)
        self.declare_parameter('frame', 'odom')
        gp = self.get_parameter
        self.frame = gp('frame').value
        self.capacity = int(gp('capacity').value)
        self.items = scene_items(load_orchard(gp('orchard_config').value),
                                 float(gp('search_radius').value))

        self.recorder = PathRecorder()
        self.pose = None
        self.stats = {}
        self.basket = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(String, '/mission/stats', self._on_stats, latched)
        self.create_subscription(Int32, '/basket/count', self._on_count, latched)
        self.marker_pub.publish(self._scene())
        self.create_timer(1.0, self._publish)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        self.recorder.add(p.x, p.y)

    def _on_stats(self, msg):
        try:
            self.stats = json.loads(msg.data)
        except ValueError:
            pass

    def _on_count(self, msg):
        self.basket = msg.data

    def _marker(self, ns, mid, mtype, kind):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = _color(kind)
        return m

    def _scene(self):
        arr = MarkerArray()
        for i, it in enumerate(self.items):
            kind = it['kind']
            if kind in ('trunk', 'canopy'):
                m = self._marker(kind, i, Marker.CYLINDER, kind)
                h = 1.6 if kind == 'trunk' else 0.02
                m.scale.x = m.scale.y = 2 * it['radius']
                m.scale.z = h
                m.pose.position.z = h / 2 if kind == 'trunk' else 0.01
            elif kind == 'search_ring':
                m = self._marker(kind, i, Marker.LINE_STRIP, kind)
                m.scale.x = 0.03
                m.points = [Point(x=x, y=y, z=0.02) for x, y in
                            circle_points(it['x'], it['y'], it['radius'])]
                it = dict(it, x=0.0, y=0.0)
            elif kind == 'depot':
                m = self._marker(kind, i, Marker.CUBE, kind)
                m.scale.x, m.scale.y = it['size']
                m.scale.z = 0.02
                m.pose.position.z = 0.01
            else:
                m = self._marker(kind, i, Marker.TEXT_VIEW_FACING, kind)
                m.scale.z = 0.4
                m.text = it['text']
                m.pose.position.z = 2.0
            m.pose.position.x, m.pose.position.y = float(it['x']), float(it['y'])
            arr.markers.append(m)
        return arr

    def _publish(self):
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.recorder.points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        if self.pose is None:
            return
        arr = self._scene()
        m = self._marker('hud', 0, Marker.TEXT_VIEW_FACING, 'hud')
        m.scale.z = 0.35
        m.text = hud_text(self.stats, self.basket, self.capacity)
        m.pose.position.x, m.pose.position.y = self.pose
        m.pose.position.z = 1.8
        arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = MissionViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
