#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class SimpleCameraViewer(Node):
    def __init__(self):
        super().__init__('simple_camera_viewer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            qos
        )
        self.get_logger().info('Connecting to /camera/image_raw/compressed... (Press Q on video window to quit)')

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            # Display resolution and FPS info
            h, w = frame.shape[:2]
            cv2.putText(frame, f'TurtleBot3 Camera ({w}x{h})', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('TurtleBot3 Live Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    viewer = SimpleCameraViewer()
    try:
        rclpy.spin(viewer)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()


if __name__ == '__main__':
    main()
