#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

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


class TargetCoordinateNode(Node):
    def __init__(self):
        super().__init__('target_coordinate_node')

        # Parameters
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_hfov_deg', 62.2)
        self.declare_parameter('lidar_window_deg', 6.0)
        self.declare_parameter('min_valid_distance', 0.15)
        self.declare_parameter('max_valid_distance', 6.0)
        self.declare_parameter('stop_distance', 0.5)

        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_hfov = math.radians(self.get_parameter('camera_hfov_deg').value)
        self.lidar_window = math.radians(self.get_parameter('lidar_window_deg').value)
        self.min_dist = self.get_parameter('min_valid_distance').value
        self.max_dist = self.get_parameter('max_valid_distance').value
        self.stop_dist = self.get_parameter('stop_distance').value

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State storage
        self.latest_detection = None
        self.latest_scan = None

        # QoS
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
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_sensor
        )

        # Publishers
        self.target_goal_pub = self.create_publisher(PoseStamped, '/redbox/target_goal', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/redbox/marker', 10)

        # Periodic fusion timer (10 Hz)
        self.timer = self.create_timer(0.1, self.fusion_loop)

        self.get_logger().info('Target Coordinate & Sensor Fusion Node initialized.')

    def detection_callback(self, msg: PointStamped):
        self.latest_detection = msg

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def fusion_loop(self):
        if self.latest_detection is None or self.latest_scan is None:
            return

        # Check if red box is detected in vision (point.z == 1.0)
        if self.latest_detection.point.z < 0.5:
            return

        norm_x = self.latest_detection.point.x
        # Convert norm_x (-1 to +1) to azimuth angle in base_footprint / camera frame
        # In ROS coordinate frame: positive angle is to the left (counter-clockwise)
        # norm_x > 0 is to the right of image, so angle is negative
        azimuth_angle = -norm_x * (self.camera_hfov / 2.0)

        # Find LiDAR distance at azimuth_angle
        distance = self.get_lidar_distance(azimuth_angle, self.latest_scan)
        if distance is None:
            return

        # Calculate relative coordinates in base_footprint
        x_rel = distance * math.cos(azimuth_angle)
        y_rel = distance * math.sin(azimuth_angle)

        # Create PointStamped in base_frame
        rel_point = PointStamped()
        rel_point.header.stamp = self.get_clock().now().to_msg()
        rel_point.header.frame_id = self.base_frame
        rel_point.point.x = x_rel
        rel_point.point.y = y_rel
        rel_point.point.z = 0.0

        # Transform to global map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            global_point = tf2_geometry_msgs.do_transform_point(rel_point, transform)
        except Exception as e:
            self.get_logger().warn(f'TF transform from {self.base_frame} to {self.target_frame} failed: {e}', throttle_duration_sec=2.0)
            return

        box_x = global_point.point.x
        box_y = global_point.point.y

        # Robot's current position in map
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        # Vector from robot to box
        dx = box_x - robot_x
        dy = box_y - robot_y
        dist_to_box = math.sqrt(dx * dx + dy * dy)

        if dist_to_box <= 0.05:
            return

        # Target stopping position: Stop distance (e.g. 0.5m) in front of the box
        # Direction unit vector from robot to box
        ux = dx / dist_to_box
        uy = dy / dist_to_box

        # If robot is already closer than stop_dist, goal is current pos
        effective_stop = min(self.stop_dist, max(0.0, dist_to_box - 0.2))
        goal_x = box_x - ux * effective_stop
        goal_y = box_y - uy * effective_stop

        # Yaw angle pointing directly towards the red box
        goal_yaw = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, goal_yaw)

        # Construct PoseStamped
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

    def get_lidar_distance(self, target_angle, scan_msg: LaserScan):
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_inc = scan_msg.angle_increment

        min_idx = int((target_angle - self.lidar_window - angle_min) / angle_inc)
        max_idx = int((target_angle + self.lidar_window - angle_min) / angle_inc)

        num_ranges = len(scan_msg.ranges)
        min_idx = max(0, min(num_ranges - 1, min_idx))
        max_idx = max(0, min(num_ranges - 1, max_idx))

        if min_idx > max_idx:
            min_idx, max_idx = max_idx, min_idx

        valid_ranges = []
        for i in range(min_idx, max_idx + 1):
            r = scan_msg.ranges[i]
            if not math.isnan(r) and not math.isinf(r) and self.min_dist <= r <= self.max_dist:
                valid_ranges.append(r)

        if not valid_ranges:
            return None

        # Return the closest obstacle within the vision ray
        valid_ranges.sort()
        # Take the 20th percentile / median of lowest values for robustness
        idx = max(0, len(valid_ranges) // 4)
        return valid_ranges[idx]

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
        box_marker.color.a = 0.85
        box_marker.lifetime.sec = 1
        marker_array.markers.append(box_marker)

        # Marker 2: Target Stopping Goal (Green Arrow)
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
    node = TargetCoordinateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
