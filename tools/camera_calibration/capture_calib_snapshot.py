#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import sys

class CaptureCalibSnapshot(Node):
    def __init__(self, out_path):
        super().__init__('capture_calib_snapshot')
        self.out_path = out_path
        self.bridge = CvBridge()
        self.status_sub = self.create_subscription(String, '/docking/status', self.status_cb, 10)
        self.image_sub = self.create_subscription(Image, '/docking/debug_image', self.image_cb, 10)
        self.status_count = 0
        self.saved_img = False

    def status_cb(self, msg: String):
        self.status_count += 1
        print(f"[STATUS #{self.status_count}] {msg.data}")
        if self.status_count >= 5 and self.saved_img:
            rclpy.shutdown()

    def image_cb(self, msg: Image):
        if not self.saved_img:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(self.out_path, cv_img)
            print(f"[IMAGE] Saved debug image to {self.out_path}")
            self.saved_img = True
            if self.status_count >= 5:
                rclpy.shutdown()

def main():
    rclpy.init()
    out_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/calib_desk_edge_debug.png'
    node = CaptureCalibSnapshot(out_path)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
