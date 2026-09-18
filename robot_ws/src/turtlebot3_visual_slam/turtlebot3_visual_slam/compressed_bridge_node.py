#!/usr/bin/env python3
"""
Zero-Lag Decompression & Exact-Sync Bridge Node for PC Side.
Subscribes to ultra-lightweight compressed streams over Wi-Fi,
decompresses them in PC RAM, synchronizes timestamps,
and republishes clean uncompressed Image & CameraInfo topics for RTAB-Map & RViz.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


from std_msgs.msg import String


class CompressedBridgeNode(Node):
    def __init__(self):
        super().__init__('compressed_bridge_node')
        self.bridge = CvBridge()

        # QoS for receiving from Wi-Fi (sensor_data / BEST_EFFORT, depth=1)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_info_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # QoS for publishing locally to RTAB-Map (RELIABLE, depth=10)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions from Raspberry Pi over Wi-Fi
        self.sub_color = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.color_cb,
            qos_sub
        )
        self.sub_depth = self.create_subscription(
            CompressedImage,
            '/camera/depth/image_raw/compressedDepth',
            self.depth_cb,
            qos_sub
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.info_cb,
            qos_info_sub
        )

        # Camera mode listener (NAV2 vs DOCKING)
        self.sub_mode = self.create_subscription(
            String,
            '/current_camera_mode',
            self.mode_cb,
            10
        )
        self.current_mode = 'NAV2'

        # Local publishers for RTAB-Map & Point Cloud Generator
        self.pub_color = self.create_publisher(
            Image, '/camera/color/image_decompressed', qos_pub
        )
        self.pub_depth = self.create_publisher(
            Image, '/camera/depth/image_decompressed', qos_pub
        )
        self.pub_info = self.create_publisher(
            CameraInfo, '/camera/color/decompressed/camera_info', qos_pub
        )

        # Docking Debug Stream Bridge (ultra-low-latency compressed bridge for PC)
        self.sub_docking_debug = self.create_subscription(
            CompressedImage,
            '/docking/debug_image/compressed',
            self.docking_debug_cb,
            qos_sub
        )
        self.pub_docking_debug = self.create_publisher(
            Image, '/docking/debug_image_decompressed', qos_pub
        )

        self.latest_color = None
        self.latest_depth = None

        # Pre-initialize bootstrap CameraInfo for Astra S 320x240 to eliminate 10s wait
        self.latest_info = CameraInfo()
        self.latest_info.width = 320
        self.latest_info.height = 240
        self.latest_info.distortion_model = 'plumb_bob'
        self.latest_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_info.k = [285.17, 0.0, 159.5, 0.0, 285.17, 119.5, 0.0, 0.0, 1.0]
        self.latest_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.latest_info.p = [285.17, 0.0, 159.5, 0.0, 0.0, 285.17, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0]

        # Synchronized dispatch timer at 15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self.dispatch_sync_pair)

        self.get_logger().info('Compressed Bridge Node active: receiving compressed Wi-Fi, publishing synchronized decompressed images locally.')

    def mode_cb(self, msg: String):
        mode_val = msg.data.strip().upper()
        if mode_val in ('NAV2', 'DOCKING') and mode_val != self.current_mode:
            self.get_logger().info(f'Compressed bridge mode switch: {self.current_mode} -> {mode_val}')
            self.current_mode = mode_val

    def info_cb(self, msg: CameraInfo):
        self.latest_info = msg

    def color_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                # Convert BGR to RGB for ROS 2 standard
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                self.latest_color = (img_rgb, msg.header)
        except Exception as e:
            self.get_logger().error(f'Color decompress error: {e}')

    def depth_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is not None:
                self.latest_depth = (depth, msg.header)
        except Exception as e:
            self.get_logger().error(f'Depth decompress error: {e}')

    def docking_debug_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                out_msg.header = msg.header
                if not out_msg.header.frame_id:
                    out_msg.header.frame_id = 'camera_color_optical_frame'
                # Ensure local timestamp if header was unset
                if out_msg.header.stamp.sec == 0:
                    out_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_docking_debug.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'Docking debug decompress error: {e}', throttle_duration_sec=3.0)

    def _create_info_msg(self, stamp, frame_id):
        msg_i = CameraInfo()
        msg_i.header.stamp = stamp
        msg_i.header.frame_id = frame_id
        msg_i.height = self.latest_info.height
        msg_i.width = self.latest_info.width
        msg_i.distortion_model = self.latest_info.distortion_model
        msg_i.d = self.latest_info.d
        msg_i.k = self.latest_info.k
        msg_i.r = self.latest_info.r
        msg_i.p = self.latest_info.p
        return msg_i

    def dispatch_sync_pair(self):
        if self.latest_color is None or self.latest_info is None:
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = 'camera_color_optical_frame'

        # Case A: DOCKING Mode -> Dispatch Color & Info immediately (Zero waiting for depth!), and Depth if available
        if self.current_mode == 'DOCKING':
            img_rgb, _ = self.latest_color
            msg_c = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
            msg_c.header.stamp = stamp
            msg_c.header.frame_id = frame_id
            self.pub_color.publish(msg_c)

            msg_i = self._create_info_msg(stamp, frame_id)
            self.pub_info.publish(msg_i)

            if self.latest_depth is not None:
                img_depth, _ = self.latest_depth
                msg_d = self.bridge.cv2_to_imgmsg(img_depth, encoding='16UC1')
                msg_d.header.stamp = stamp
                msg_d.header.frame_id = frame_id
                self.pub_depth.publish(msg_d)
            return

        # Case B: NAV2 Mode -> Dispatch synchronized RGB-D pair for RTAB-Map SLAM
        if self.latest_depth is None:
            return

        img_rgb, _ = self.latest_color
        img_depth, _ = self.latest_depth

        # 1. Color Image
        msg_c = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
        msg_c.header.stamp = stamp
        msg_c.header.frame_id = frame_id
        self.pub_color.publish(msg_c)

        # 2. Depth Image (16UC1 mm uint16)
        msg_d = self.bridge.cv2_to_imgmsg(img_depth, encoding='16UC1')
        msg_d.header.stamp = stamp
        msg_d.header.frame_id = frame_id
        self.pub_depth.publish(msg_d)

        # 3. Camera Info
        msg_i = self._create_info_msg(stamp, frame_id)
        self.pub_info.publish(msg_i)


from rclpy.executors import ExternalShutdownException


def main(args=None):
    rclpy.init(args=args)
    node = CompressedBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
