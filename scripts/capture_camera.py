#!/usr/bin/env python3
"""Save one frame of the robot camera (/camera/image_raw) as PNG.

Usage (with the sim running): python3 scripts/capture_camera.py [artifacts/robot_camera.png]
"""
import sys

import cv2
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'artifacts/robot_camera.png'
    rclpy.init()
    node = rclpy.create_node('capture_camera')
    frames = []
    node.create_subscription(Image, '/camera/image_raw', frames.append, 1)
    deadline = node.get_clock().now().nanoseconds + 20e9
    while not frames and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not frames:
        print('no image received', file=sys.stderr)
        sys.exit(1)
    img = CvBridge().imgmsg_to_cv2(frames[0], desired_encoding='bgr8')
    cv2.imwrite(out, img)
    print(f'saved {out} ({img.shape[1]}x{img.shape[0]})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
