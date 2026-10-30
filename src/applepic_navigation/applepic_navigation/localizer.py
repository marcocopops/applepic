"""ROS node: robot pose in the map (world) frame for navigation, perception and the mission.

mode `gps`:   dead reckoning (wheel speed from /wheel_odom + gyro yaw rate from /imu, frame odom
              at the start pose) fused with GPS fixes (/gps/fix, converted to the world ENU frame
              with the orchard datum) and with the tree trunks seen by the camera (/trees/trunks)
              by `localization.OffsetEkf`. Publishes TF map -> odom and odom -> base_footprint.
mode `truth`: republishes the ground-truth /odom (already in the world frame) and a static
              identity map -> odom, so that consumers are the same in both modes.

Subscribes: /wheel_odom + /imu + /gps/fix + /trees/trunks (gps) or /odom (truth).
Publishes:  /localization/odom (nav_msgs/Odometry, frame map, 50 Hz in gps mode),
            TF map -> odom (+ odom -> base_footprint in gps mode),
            /localization/trunk_fix (geometry_msgs/PointStamped, the tree just used, frame map).
"""
from collections import deque
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from applepic_navigation.gps import geodetic_to_enu
from applepic_navigation.localization import DeadReckoning, LocalizationParams, OffsetEkf, wrap
from applepic_navigation.route import load_orchard


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _set_yaw(q, yaw):
    q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Localizer(Node):

    def __init__(self):
        super().__init__('localizer')
        default_cfg = os.path.join(
            get_package_share_directory('applepic_gazebo'), 'config', 'orchard.yaml')
        self.declare_parameter('mode', 'truth')
        self.declare_parameter('orchard_config', default_cfg)
        self.declare_parameter('initial_yaw', 0.0)   # map heading at start (docked at the depot)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        defaults = LocalizationParams()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        gp = self.get_parameter
        self.mode = gp('mode').value
        self.map_frame = gp('map_frame').value
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.initial_yaw = float(gp('initial_yaw').value)
        config = load_orchard(gp('orchard_config').value)
        self.trees = [tuple(t) for t in config['trees']]
        sc = config['spherical_coordinates']
        self.datum = (float(sc['latitude_deg']), float(sc['longitude_deg']))
        self.ekf = OffsetEkf(LocalizationParams(
            **{n: type(v)(gp(n).value) for n, v in vars(defaults).items()}))
        self.dead_reckoning = DeadReckoning()
        self.history = deque(maxlen=250)   # (t, odom pose) at the IMU rate (50 Hz): 5 s
        self.pub = self.create_publisher(Odometry, '/localization/odom', 10)

        if self.mode == 'truth':
            identity = TransformStamped()
            identity.header.frame_id = self.map_frame
            identity.child_frame_id = self.odom_frame
            identity.transform.rotation.w = 1.0
            self.static_tf = StaticTransformBroadcaster(self)
            self.static_tf.sendTransform(identity)
            self.create_subscription(Odometry, '/odom', self._on_truth, 10)
        elif self.mode == 'gps':
            self.tf = TransformBroadcaster(self)
            self.create_subscription(Odometry, '/wheel_odom', self._on_wheel_odom, 20)
            self.create_subscription(Imu, '/imu', self._on_imu, 50)
            self.create_subscription(NavSatFix, '/gps/fix', self._on_fix, 10)
            self.create_subscription(PoseArray, '/trees/trunks', self._on_trunks, 10)
            self.trunk_pub = self.create_publisher(PointStamped, '/localization/trunk_fix', 10)
        else:
            raise ValueError(f'unknown localization mode {self.mode!r} (truth or gps)')
        self.get_logger().info(f'localization mode: {self.mode}')

    def _on_truth(self, msg):
        msg.header.frame_id = self.map_frame
        self.pub.publish(msg)

    def _odom_at(self, t):
        """Wheel odometry pose interpolated at time t (clamped to the buffer)."""
        if t <= self.history[0][0]:
            return self.history[0][1]
        for (t0, p0), (t1, p1) in zip(self.history, list(self.history)[1:]):
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]),
                        wrap(p0[2] + a * wrap(p1[2] - p0[2])))
        return self.history[-1][1]

    def _on_fix(self, msg):
        if not self.history or math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        xy = geodetic_to_enu(msg.latitude, msg.longitude, *self.datum)
        stddev = None
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            stddev = math.sqrt(max(0.5 * (msg.position_covariance[0]
                                          + msg.position_covariance[4]), 1e-8))
        pose = self._odom_at(_stamp(msg))
        if not self.ekf.initialized:
            self.ekf.initialize(xy, pose, self.initial_yaw, stddev)
            self.get_logger().info(f'initialized at ({xy[0]:.2f}, {xy[1]:.2f}), '
                                   f'heading {self.initial_yaw:.2f} rad')
        else:
            resets = self.ekf.resets
            if not self.ekf.update(xy, pose, stddev):
                self.get_logger().warn(f'GPS fix rejected (total {self.ekf.rejected})',
                                       throttle_duration_sec=5.0)
            if self.ekf.resets != resets:
                self.get_logger().warn(
                    f'{self.ekf.p.max_rejections} fixes rejected in a row: estimate reset on the '
                    f'GPS fix ({self.ekf.resets} resets)')

    def _on_trunks(self, msg):
        """Trunks measured relative to the robot, placed in odom with the pose at the image time."""
        if not self.history:
            return
        ox, oy, yaw = self._odom_at(_stamp(msg))
        c, s = math.cos(yaw), math.sin(yaw)
        for p in msg.poses:
            x, y = p.position.x, p.position.y
            tree = self.ekf.update_trunk((ox + c * x - s * y, oy + s * x + c * y), self.trees)
            if tree is not None:
                fix = PointStamped()
                fix.header.stamp = msg.header.stamp
                fix.header.frame_id = self.map_frame
                fix.point.x, fix.point.y = float(tree[0]), float(tree[1])
                self.trunk_pub.publish(fix)

    def _on_wheel_odom(self, msg):
        self.dead_reckoning.set_speed(msg.twist.twist.linear.x)

    def _transform(self, stamp, parent, child, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        _set_yaw(tf.transform.rotation, float(yaw))
        return tf

    def _on_imu(self, msg):
        t = _stamp(msg)
        wz = msg.angular_velocity.z
        pose = self.dead_reckoning.integrate(t, wz)
        self.history.append((t, pose))
        transforms = [self._transform(msg.header.stamp, self.odom_frame, self.base_frame, *pose)]
        if not self.ekf.initialized:
            self.tf.sendTransform(transforms)
            return
        self.ekf.predict(pose, t)
        transforms.append(self._transform(msg.header.stamp, self.map_frame, self.odom_frame,
                                          *self.ekf.x))
        self.tf.sendTransform(transforms)

        x, y, yaw = self.ekf.pose(pose)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        _set_yaw(out.pose.pose.orientation, yaw)
        var = self.ekf.position_stddev(pose) ** 2
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        cov[35] = float(self.ekf.P[2, 2])
        out.pose.covariance = cov
        out.twist.twist.linear.x = self.dead_reckoning.v   # body frame
        out.twist.twist.angular.z = wz
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Localizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
