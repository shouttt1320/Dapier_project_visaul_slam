#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0, 0.0, 0.0, 1.0]
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q


class TargetDepthCoordinateNode(Node):
    def __init__(self):
        super().__init__('target_depth_coordinate_node')

        # Parameters
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_optical_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('depth_roi_radius', 10)
        self.declare_parameter('min_depth', 0.20)
        self.declare_parameter('max_depth', 2.50)

        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.stop_dist = self.get_parameter('stop_distance').value
        self.roi_r = self.get_parameter('depth_roi_radius').value
        self.min_d = self.get_parameter('min_depth').value
        self.max_d = self.get_parameter('max_depth').value

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage
        self.latest_detection = None
        self.latest_depth_img = None
        self.camera_intrinsics = None  # (fx, fy, cx, cy)

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.sub_detection = self.create_subscription(
            PointStamped,
            '/redbox/detection',
            self.detection_callback,
            10
        )
        self.sub_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_sensor
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.info_callback,
            10
        )

        # Publishers
        self.target_goal_pub = self.create_publisher(PoseStamped, '/redbox/target_goal', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/redbox/marker', 10)

        # Periodic 3D estimation loop (10 Hz)
        self.timer = self.create_timer(0.1, self.estimation_loop)

        self.get_logger().info('Pure Depth-based Target 3D Coordinate Node initialized.')

    def info_callback(self, msg: CameraInfo):
        if self.camera_intrinsics is None:
            # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            fx = msg.k[0]
            cx = msg.k[2]
            fy = msg.k[4]
            cy = msg.k[5]
            if fx > 0.0 and fy > 0.0:
                self.camera_intrinsics = (fx, fy, cx, cy)
                self.get_logger().info(f'Camera Intrinsics Loaded: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')

    def detection_callback(self, msg: PointStamped):
        self.latest_detection = msg

    def depth_callback(self, msg: Image):
        self.latest_depth_img = msg

    def estimation_loop(self):
        if self.latest_detection is None or self.latest_depth_img is None:
            return

        # Check if red box is detected (point.z == 1.0)
        if self.latest_detection.point.z < 0.5:
            return

        u = int(self.latest_detection.point.x)
        v = int(self.latest_detection.point.y)

        # Convert ROS Depth Image to numpy array
        try:
            if self.latest_depth_img.encoding == '16UC1':
                depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_img, desired_encoding='16UC1')
                # 16UC1 is in millimeters -> convert to meters
                depth_cv = depth_cv.astype(np.float32) / 1000.0
            elif self.latest_depth_img.encoding in ['32FC1', 'passthrough']:
                depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_img, desired_encoding='32FC1')
            else:
                depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_img)
                if depth_cv.dtype == np.uint16:
                    depth_cv = depth_cv.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
            return

        h, w = depth_cv.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return

        # Sample patch around centroid (u, v)
        u_min = max(0, u - self.roi_r)
        u_max = min(w, u + self.roi_r + 1)
        v_min = max(0, v - self.roi_r)
        v_max = min(h, v + self.roi_r + 1)

        patch = depth_cv[v_min:v_max, u_min:u_max]

        # Filter out invalid depth pixels (NaN, Inf, 0, out of range)
        valid_mask = np.isfinite(patch) & (patch >= self.min_d) & (patch <= self.max_d)
        valid_depths = patch[valid_mask]

        if len(valid_depths) < 5:
            return

        # Robust median depth Z
        z_opt = float(np.median(valid_depths))

        # Camera intrinsics (default fallback if camera_info not yet received)
        if self.camera_intrinsics is not None:
            fx, fy, cx, cy = self.camera_intrinsics
        else:
            # Fallback for 640x480 OS30A (HFOV 78 deg -> fx ≈ 400)
            fx = (w / 2.0) / math.tan(math.radians(78.0 / 2.0))
            fy = fx
            cx = w / 2.0
            cy = h / 2.0

        # Pinhole 3D Back-projection in optical frame
        x_opt = float((u - cx) * z_opt / fx)
        y_opt = float((v - cy) * z_opt / fy)

        # Optical frame: X right, Y down, Z forward
        # Create PointStamped in camera_optical_frame
        opt_point = PointStamped()
        opt_point.header.stamp = self.get_clock().now().to_msg()
        opt_point.header.frame_id = self.camera_optical_frame
        opt_point.point.x = x_opt
        opt_point.point.y = y_opt
        opt_point.point.z = z_opt

        # Transform to global map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_optical_frame,
                rclpy.time.Time()
            )
            global_box = tf2_geometry_msgs.do_transform_point(opt_point, transform)
        except Exception as e:
            self.get_logger().warn(f'TF lookup {self.camera_optical_frame} -> {self.target_frame} error: {e}', throttle_duration_sec=2.0)
            return

        box_x = global_box.point.x
        box_y = global_box.point.y

        # Get robot position in map frame
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
        except Exception:
            return

        dx = box_x - robot_x
        dy = box_y - robot_y
        dist_to_box = math.sqrt(dx * dx + dy * dy)

        if dist_to_box <= 0.05:
            return

        # Target stopping position: 0.5m in front of the box
        ux = dx / dist_to_box
        uy = dy / dist_to_box

        effective_stop = min(self.stop_dist, max(0.0, dist_to_box - 0.15))
        goal_x = box_x - ux * effective_stop
        goal_y = box_y - uy * effective_stop

        # Goal Yaw facing directly towards the red box
        goal_yaw = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, goal_yaw)

        # Publish Target Goal Pose
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.target_frame
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.target_goal_pub.publish(goal_pose)
        self.publish_markers(box_x, box_y, goal_pose)

    def publish_markers(self, box_x, box_y, goal_pose: PoseStamped):
        marker_array = MarkerArray()

        # Marker 1: Red Box (Cube)
        box_marker = Marker()
        box_marker.header.frame_id = self.target_frame
        box_marker.header.stamp = self.get_clock().now().to_msg()
        box_marker.ns = 'red_box'
        box_marker.id = 0
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        box_marker.pose.position.x = box_x
        box_marker.pose.position.y = box_y
        box_marker.pose.position.z = 0.15
        box_marker.scale.x = 0.3
        box_marker.scale.y = 0.3
        box_marker.scale.z = 0.3
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.9
        box_marker.lifetime.sec = 1
        marker_array.markers.append(box_marker)

        # Marker 2: Stopping Goal (Green Arrow)
        goal_marker = Marker()
        goal_marker.header.frame_id = self.target_frame
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'stopping_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose = goal_pose.pose
        goal_marker.scale.x = 0.4
        goal_marker.scale.y = 0.08
        goal_marker.scale.z = 0.08
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.2
        goal_marker.color.a = 0.9
        goal_marker.lifetime.sec = 1
        marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDepthCoordinateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
