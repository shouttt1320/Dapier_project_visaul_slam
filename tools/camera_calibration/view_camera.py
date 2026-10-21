#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys


class SimpleCameraViewer(Node):
    def __init__(self, topic: str = None):
        super().__init__('simple_camera_viewer')
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Candidate topics
        if topic:
            self.topics = [topic]
        else:
            self.topics = [
                '/camera/color/image_raw/compressed',
                '/camera/image_raw/compressed',
                '/camera/color/image_raw',
                '/camera/image_raw',
            ]

        self.subs = []
        for t in self.topics:
            if 'compressed' in t:
                self.subs.append(self.create_subscription(CompressedImage, t, lambda m, top=t: self.compressed_callback(m, top), qos))
            else:
                self.subs.append(self.create_subscription(Image, t, lambda m, top=t: self.raw_callback(m, top), qos))

        self.get_logger().info(f'Listening on camera topics: {self.topics}... (Press Q on window to quit)')

    def _show_frame(self, frame: np.ndarray, source_topic: str):
        if frame is not None:
            h, w = frame.shape[:2]
            cv2.putText(frame, f'{source_topic} ({w}x{h})', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow('TurtleBot3 Live Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def compressed_callback(self, msg: CompressedImage, source_topic: str):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self._show_frame(frame, source_topic)

    def raw_callback(self, msg: Image, source_topic: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._show_frame(frame, source_topic)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    custom_topic = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else None
    viewer = SimpleCameraViewer(topic=custom_topic)
    try:
        rclpy.spin(viewer)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()


if __name__ == '__main__':
    main()
