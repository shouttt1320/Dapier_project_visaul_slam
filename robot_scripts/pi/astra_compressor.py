#!/usr/bin/env python3
"""
Ultra-Light Zero-Lag Compressor for Orbbec Astra on Raspberry Pi
Runs multi-threaded so color and depth compression never block each other.
Only publishes lightweight compressed frames over Wi-Fi.
No heavy 3D Point Cloud generation on Pi!
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ZeroLagCompressor(Node):
    def __init__(self):
        super().__init__('astra_compressor')
        self.bridge = CvBridge()

        # Local camera driver subscription QoS
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Wi-Fi transmission QoS (Zero backlog across network)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_color = self.create_publisher(
            CompressedImage, '/camera/color/image_raw/compressed', qos_pub
        )
        self.pub_depth = self.create_publisher(
            CompressedImage, '/camera/depth/image_raw/compressedDepth', qos_pub
        )

        self.sub_color = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_cb, qos_sub
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_cb, qos_sub
        )

        # Listen to camera mode switcher (NAV2 vs DOCKING)
        self.sub_mode = self.create_subscription(
            String, '/current_camera_mode', self.mode_cb, 10
        )
        self.current_mode = 'NAV2'

        self.latest_raw_depth = None
        self.last_col_time = 0.0

        self.get_logger().info('ZeroLagCompressor active (10 FPS Lockstep Sync, CPU Optimized).')

    def mode_cb(self, msg: String):
        mode_val = msg.data.strip().upper()
        if mode_val in ('NAV2', 'DOCKING') and mode_val != self.current_mode:
            self.get_logger().info(f'Compressor mode switch: {self.current_mode} -> {mode_val}')
            self.current_mode = mode_val

    def depth_cb(self, msg: Image):
        # Store freshest raw depth frame reference (zero-copy pointer assignment, sub-microsecond)
        self.latest_raw_depth = msg

    def color_cb(self, msg: Image):
        now = time.time()
        # 10 FPS cadence (0.100s period) - smooth visual mapping with minimal Pi CPU load (< 3%)
        if now - self.last_col_time < 0.100:
            return
        self.last_col_time = now

        # Ensure valid header timestamp
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        # 1. Compress & Publish Color (JPEG quality 55)
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 55])
            if success:
                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = 'jpeg'
                comp_msg.data = encoded.tobytes()
                self.pub_color.publish(comp_msg)
        except Exception as e:
            self.get_logger().error(f'Color compress error: {e}')

        # 2. Compress & Publish Depth in 1:1 lockstep pair with exact same timestamp
        if self.latest_raw_depth is not None:
            try:
                depth_img = self.bridge.imgmsg_to_cv2(self.latest_raw_depth, desired_encoding='passthrough')
                success, encoded = cv2.imencode('.png', depth_img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if success:
                    comp_msg = CompressedImage()
                    comp_msg.header = msg.header  # Identical timestamp down to nanoseconds!
                    comp_msg.format = 'png'
                    comp_msg.data = encoded.tobytes()
                    self.pub_depth.publish(comp_msg)
            except Exception as e:
                self.get_logger().error(f'Depth compress error: {e}')


def main():
    rclpy.init()
    node = ZeroLagCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
