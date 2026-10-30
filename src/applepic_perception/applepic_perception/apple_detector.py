"""ROS node: detect red apples in the camera image and publish their world positions.

Subscribes: /camera/image_raw, /camera/camera_info, TF (target_frame -> camera frame).
Publishes:  /apples/detections (geometry_msgs/PoseArray in target_frame),
            /apples/debug_image (sensor_msgs/Image, annotated).
"""
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from applepic_perception.detector import HsvParams, annotate, detect_apples
from applepic_perception.projection import project_pixel_to_plane, quaternion_to_matrix


class AppleDetector(Node):

    def __init__(self):
        super().__init__('apple_detector')
        defaults = HsvParams()
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('apple_radius', 0.04)   # apple centers lie on z = apple_radius
        self.declare_parameter('max_range', 4.0)       # m, horizontal distance from the camera
        self.declare_parameter('sat_min', defaults.sat_min)
        self.declare_parameter('val_min', defaults.val_min)
        self.declare_parameter('min_area', defaults.min_area)
        self.declare_parameter('publish_debug_image', True)
        self.params = HsvParams(
            sat_min=self.get_parameter('sat_min').value,
            val_min=self.get_parameter('val_min').value,
            min_area=float(self.get_parameter('min_area').value),
        )
        self.target_frame = self.get_parameter('target_frame').value
        self.plane_z = float(self.get_parameter('apple_radius').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.debug = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()
        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseArray, '/apples/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/apples/debug_image', 1)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_info, 1)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 1)

    def _on_info(self, msg):
        self.K = list(msg.k)

    def _camera_pose(self, frame, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, frame, Time.from_msg(stamp))
        except tf2_ros.ExtrapolationException:
            # Image newer than the last TF: fall back to the latest transform.
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, frame, Time())
            except tf2_ros.TransformException:
                return None
        except tf2_ros.TransformException:
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return quaternion_to_matrix(q.x, q.y, q.z, q.w), (p.x, p.y, p.z)

    def _on_image(self, msg):
        if self.K is None:
            return
        pose = self._camera_pose(msg.header.frame_id, msg.header.stamp)
        if pose is None:
            self.get_logger().warn(
                f'no TF {self.target_frame} -> {msg.header.frame_id}',
                throttle_duration_sec=5.0)
            return
        rot, trans = pose
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = detect_apples(img, self.params)

        out = PoseArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        labels = []
        for d in detections:
            hit = project_pixel_to_plane(
                d.u, d.v, self.K, rot, trans, plane_z=self.plane_z, max_range=self.max_range)
            if hit is None:
                labels.append('')
                continue
            p = Pose()
            p.position.x, p.position.y, p.position.z = hit[0], hit[1], self.plane_z
            p.orientation.w = 1.0
            out.poses.append(p)
            labels.append(f'{hit[0]:.2f},{hit[1]:.2f}')
        self.pub.publish(out)

        if self.debug and self.debug_pub.get_subscription_count() > 0:
            dbg = self.bridge.cv2_to_imgmsg(annotate(img, detections, labels), encoding='bgr8')
            dbg.header = msg.header
            self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = AppleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
