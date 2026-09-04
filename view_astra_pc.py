#!/usr/bin/env python3
"""
Orbbec Astra S Live Network Viewer (PC Side)
Zero-backlog viewer using BEST_EFFORT QoS with depth=1.
Measures and displays real-time network latency (ms) and FPS directly on the HUD.
"""

import os
os.environ['ROS_DOMAIN_ID'] = '101'

import sys
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class AstraViewerNode(Node):
    def __init__(self):
        super().__init__('astra_pc_viewer')

        # BEST_EFFORT, KEEP_LAST, depth=1: Always delivers newest frame, zero buffer lag
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.color_frame = None
        self.depth_frame = None
        self.color_latency_ms = 0.0
        self.depth_latency_ms = 0.0
        self.lock = threading.Lock()

        # Only subscribe to compressed streams to keep Wi-Fi usage at < 3 Mbps
        self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.color_compressed_cb,
            qos
        )
        self.create_subscription(
            CompressedImage,
            '/camera/depth/image_raw/compressedDepth',
            self.depth_compressed_cb,
            qos
        )

        self.get_logger().info("Astra PC Viewer initialized with BEST_EFFORT (depth=1) QoS.")

    def color_compressed_cb(self, msg: CompressedImage):
        try:
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            latency = (time.time() - t_msg) * 1000.0 if t_msg > 0 else 0.0

            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                with self.lock:
                    self.color_frame = img
                    self.color_latency_ms = latency
        except Exception as e:
            self.get_logger().error(f"Color decode error: {e}")

    def depth_compressed_cb(self, msg: CompressedImage):
        try:
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            latency = (time.time() - t_msg) * 1000.0 if t_msg > 0 else 0.0

            np_arr = np.frombuffer(msg.data, np.uint8)
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is not None:
                with self.lock:
                    self.depth_frame = depth
                    self.depth_latency_ms = latency
        except Exception as e:
            self.get_logger().error(f"Depth decode error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AstraViewerNode()

    def spin_worker():
        try:
            rclpy.spin(node)
        except Exception:
            pass

    spin_thread = threading.Thread(target=spin_worker, daemon=True)
    spin_thread.start()

    window_name = "Orbbec Astra S: Zero-Lag Stream [BEST_EFFORT depth=1]"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 480)

    fps_start = time.time()
    frame_count = 0
    fps_text = "FPS: --"

    try:
        while rclpy.ok():
            with node.lock:
                color_curr = node.color_frame.copy() if node.color_frame is not None else None
                depth_curr = node.depth_frame.copy() if node.depth_frame is not None else None
                c_lat = node.color_latency_ms
                d_lat = node.depth_latency_ms

            # Prepare color display
            if color_curr is not None:
                disp_color = cv2.resize(color_curr, (640, 480))
            else:
                disp_color = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(disp_color, "Waiting for /camera/color/image_raw/compressed...", (20, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            # Prepare depth display
            if depth_curr is not None:
                depth_raw = depth_curr.astype(np.float32)

                # Center distance calculation
                ch, cw = depth_raw.shape[:2]
                center_val = depth_raw[ch // 2 - 3:ch // 2 + 3, cw // 2 - 3:cw // 2 + 3]
                valid_center = center_val[center_val > 0]
                if len(valid_center) > 0:
                    dist_m = np.median(valid_center) / 1000.0
                    dist_str = f"Center: {dist_m:.2f} m"
                else:
                    dist_str = "Center: Out of range"

                # Normalize 0.4m ~ 3.5m
                depth_clipped = np.clip(depth_raw, 400.0, 3500.0)
                depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                disp_depth = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                disp_depth[depth_raw == 0] = [0, 0, 0]

                disp_depth = cv2.resize(disp_depth, (640, 480))
                cv2.drawMarker(disp_depth, (320, 240), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.drawMarker(disp_color, (320, 240), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

                cv2.putText(disp_depth, "Depth Map (320x240 Native)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(disp_depth, dist_str, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(disp_depth, f"Depth Latency: {abs(d_lat):.1f} ms", (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                disp_depth = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(disp_depth, "Waiting for /camera/depth/image_raw/compressedDepth...", (20, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            # FPS calculation
            frame_count += 1
            now = time.time()
            if now - fps_start >= 1.0:
                fps_val = frame_count / (now - fps_start)
                fps_text = f"FPS: {fps_val:.1f}"
                fps_start = now
                frame_count = 0

            cv2.putText(disp_color, f"Astra S Color [{fps_text}]", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(disp_color, f"Color Latency: {abs(c_lat):.1f} ms", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            combined = np.hstack((disp_color, disp_depth))
            cv2.imshow(window_name, combined)

            key = cv2.waitKey(15) & 0xFF
            if key == ord('q') or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
