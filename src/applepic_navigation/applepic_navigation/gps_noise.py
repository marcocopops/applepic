"""ROS node: degrade the perfect gz NavSat fix with a seeded GPS error model (metres).

The gz NavSat noise is expressed in degrees on latitude and longitude alike (anisotropic in
metres) and is not seedable, so the gz sensor is noiseless and the error is added here.

Subscribes: /gps/raw (sensor_msgs/NavSatFix, perfect).
Publishes:  /gps/fix (sensor_msgs/NavSatFix, noisy, with its diagonal position covariance).
"""
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import NavSatFix

from applepic_navigation.gps import enu_to_geodetic, GpsErrorModel


class GpsNoise(Node):

    def __init__(self):
        super().__init__('gps_noise')
        self.declare_parameter('stddev', 0.02)        # m, white noise (RTK-grade by default)
        self.declare_parameter('bias_stddev', 0.0)    # m, wandering bias
        self.declare_parameter('bias_tau', 60.0)      # s, bias correlation time
        self.declare_parameter('seed', 0)             # < 0: random
        gp = self.get_parameter
        seed = int(gp('seed').value)
        self.model = GpsErrorModel(float(gp('stddev').value), float(gp('bias_stddev').value),
                                   float(gp('bias_tau').value), None if seed < 0 else seed)
        var = self.model.stddev ** 2 + self.model.bias_stddev ** 2
        self.cov = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4.0 * var]
        self.last_t = None
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.create_subscription(NavSatFix, '/gps/raw', self._on_fix, 10)
        self.get_logger().info(
            f'GPS error: white {self.model.stddev} m, bias {self.model.bias_stddev} m '
            f'(tau {self.model.tau} s), seed {seed}')

    def _on_fix(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.1 if self.last_t is None else t - self.last_t
        self.last_t = t
        east, north = self.model.sample(dt)
        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        out.latitude, out.longitude = enu_to_geodetic(east, north, msg.latitude, msg.longitude)
        out.altitude = msg.altitude
        out.position_covariance = self.cov
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GpsNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
