#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped


class RedBoxDetectorNode(Node):
    def __init__(self):
        super().__init__('redbox_detector_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('min_area', 400.0)
        self.declare_parameter('lower_red1', [0, 100, 70])
        self.declare_parameter('upper_red1', [10, 255, 255])
        self.declare_parameter('lower_red2', [165, 100, 70])
        self.declare_parameter('upper_red2', [180, 255, 255])

        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.min_area = self.get_parameter('min_area').value
        self.lower_red1 = np.array(self.get_parameter('lower_red1').value, dtype=np.uint8)
        self.upper_red1 = np.array(self.get_parameter('upper_red1').value, dtype=np.uint8)
        self.lower_red2 = np.array(self.get_parameter('lower_red2').value, dtype=np.uint8)
        self.upper_red2 = np.array(self.get_parameter('upper_red2').value, dtype=np.uint8)

        self.bridge = CvBridge()

        # Publishers
        self.detection_pub = self.create_publisher(PointStamped, '/redbox/detection', 10)
        self.debug_image_pub = self.create_publisher(CompressedImage, '/redbox/debug_image/compressed', 10)

        # QoS for camera streaming (sensor data/best effort or reliable)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        if self.use_compressed or 'compressed' in self.image_topic:
            self.sub_img = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_image_callback,
                qos
            )
            self.get_logger().info(f'Subscribing to CompressedImage topic: {self.image_topic}')
        else:
            self.sub_img = self.create_subscription(
                Image,
                self.image_topic,
                self.raw_image_callback,
                qos
            )
            self.get_logger().info(f'Subscribing to Raw Image topic: {self.image_topic}')

        self.get_logger().info('RedBox Detector Node initialized successfully.')

    def raw_image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {e}')

    def compressed_image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self.process_image(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')

    def process_image(self, frame: np.ndarray, header):
        h, w = frame.shape[:2]

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create dual HSV masks for red
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Morphology operations to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        max_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area and area > max_area:
                # Calculate aspect ratio to filter abnormal shapes
                bx, by, bw, bh = cv2.boundingRect(cnt)
                aspect_ratio = float(bw) / float(bh)
                # Reasonable aspect ratio for box
                if 0.3 <= aspect_ratio <= 3.5:
                    max_area = area
                    best_contour = cnt

        det_msg = PointStamped()
        det_msg.header = header

        debug_frame = frame.copy()

        if best_contour is not None:
            bx, by, bw, bh = cv2.boundingRect(best_contour)
            cx = bx + bw / 2.0
            cy = by + bh / 2.0

            # Publish exact pixel centroid coordinates cx, cy
            det_msg.point.x = float(cx)
            det_msg.point.y = float(cy)
            det_msg.point.z = 1.0  # 1.0 = Detected

            # Draw annotations on debug frame
            norm_x = (cx - (w / 2.0)) / (w / 2.0)
            cv2.rectangle(debug_frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
            cv2.circle(debug_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(
                debug_frame,
                f'RED BOX ({int(cx)}, {int(cy)}) Area: {int(max_area)}px',
                (bx, max(20, by - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
        else:
            det_msg.point.x = 0.0
            det_msg.point.y = 0.0
            det_msg.point.z = 0.0  # 0.0 = Not detected

            cv2.putText(
                debug_frame,
                'Searching Red Box...',
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2
            )

        # Center line on image
        cv2.line(debug_frame, (int(w / 2), 0), (int(w / 2), h), (255, 255, 0), 1)

        # Publish detection result
        self.detection_pub.publish(det_msg)

        # Publish compressed debug image
        try:
            _, encoded_img = cv2.imencode('.jpg', debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            dbg_msg = CompressedImage()
            dbg_msg.header = header
            dbg_msg.format = 'jpeg'
            dbg_msg.data = encoded_img.tobytes()
            self.debug_image_pub.publish(dbg_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RedBoxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
