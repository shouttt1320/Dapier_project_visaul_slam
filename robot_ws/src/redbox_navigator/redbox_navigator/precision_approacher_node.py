#!/usr/bin/env python3
# ==============================================================================
# Precision Close-Docking Node for TurtleBot3 (5cm Parallel Docking)
# - Stage 1: Nav2 Staging Navigation & Tri-Factor Verification
# - Stage 2: OS30A 3D Depth Visual Servoing (Desk Edge & Parallelism Tracking)
# - Stage 3: Single Contact Bumper Stop (Physical 5cm Clearance Assurance)
# ==============================================================================

import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from dataclasses import dataclass

import tf2_ros
from cv_bridge import CvBridge


@dataclass
class TargetObject:
    center: np.ndarray = None          # [x, y, z] in base_footprint (m)
    docking_point: np.ndarray = None   # [x, y, z] in base_footprint (m)
    approach_normal: float = 0.0       # rad
    docking_distance: float = 0.05     # 5cm target
    valid: bool = False

DOCKING_ACTIVE_STATES = (
    'CHECK_STAGING', 'BACKUP_STANDOFF', 'BACKUP_PREPARE', 'BACKUP_ALIGN_PREPARE',
    'TURN_LATERAL', 'LATERAL_CRUISE', 'TURN_FACE_DESK',
    'CARROT_ALIGN', 'ALIGN_PARALLEL', 'CARROT_PURSUIT_ALIGN', 'VERIFY_DUAL_ALIGN',
    'COARSE_APPROACH', 'FINE_APPROACH', 'FINAL_APPROACH', 'CRAWL_CONTACT',
    'BACKUP_RETRY'
)


class PrecisionApproacherNode(Node):
    def __init__(self):
        super().__init__('precision_approacher_node')

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('staging_x', 1.10)         # 1.10m gives ~30-35cm clearance to desk front
        self.declare_parameter('staging_y', 0.40)
        self.declare_parameter('staging_yaw', 0.0)
        self.declare_parameter('target_clearance', 0.05)  # 50mm (5cm)
        self.declare_parameter('reference_x', 0.100)      # Astra S front tip
        self.declare_parameter('max_linear_speed', 0.04)   # 4cm/s max
        self.declare_parameter('crawl_linear_speed', 0.008) # 8mm/s
        self.declare_parameter('max_angular_speed', 0.20)  # 0.20 rad/s (~11.5 deg/s)
        self.declare_parameter('skip_nav2_if_close', True)
        self.declare_parameter('show_window', True)        # Live visual window
        self.declare_parameter('docking_timeout_sec', 35.0) # Abort and retreat if docking exceeds this timeout
        self.declare_parameter('autostart', False)         # If False, start in NAV2_READY idle state

        self.staging_x = self.get_parameter('staging_x').value
        self.staging_y = self.get_parameter('staging_y').value
        self.staging_yaw = self.get_parameter('staging_yaw').value
        self.target_clearance = self.get_parameter('target_clearance').value
        self.reference_x = self.get_parameter('reference_x').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.crawl_linear_speed = self.get_parameter('crawl_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.skip_nav2_if_close = self.get_parameter('skip_nav2_if_close').value
        self.show_window = self.get_parameter('show_window').value
        self.docking_timeout_sec = self.get_parameter('docking_timeout_sec').value
        self.autostart = self.get_parameter('autostart').value
        self.docking_routine_start_time = None

        # OS30A Extrinsic Configuration (from model.sdf)
        self.cam_pitch = 1.0472  # 60 degrees downward (radians)
        self.cam_x_base = 0.000  # Camera mounted at robot center X=0.0
        self.cam_z_base = 0.520  # Camera height Z=0.520m

        # Intrinsic Camera Parameters (default fallback for 320x240, updated via CameraInfo)
        self.fx = 190.7
        self.fy = 190.7
        self.cx = 160.0
        self.cy = 120.0
        self.camera_info_received = False

        # RTAB-Map Pause/Resume Clients (Freeze SLAM during docking to save CPU)
        self.rtabmap_pause_client = self.create_client(Empty, '/rtabmap/pause')
        self.rtabmap_resume_client = self.create_client(Empty, '/rtabmap/resume')

        # Camera Mode Switch Clients & Services (Time-division Single Camera Control)
        self.set_mode_docking_cli = self.create_client(Trigger, '/set_mode_docking')
        self.set_mode_nav2_cli = self.create_client(Trigger, '/set_mode_nav2')
        self.undock_srv = self.create_service(Trigger, '/undock_to_nav2', self.handle_undock_to_nav2)
        self.dock_srv = self.create_service(Trigger, '/start_docking', self.handle_start_docking)
        self.abort_srv = self.create_service(Trigger, '/abort_docking', self.handle_abort_docking)
        self.mode_sub = self.create_subscription(String, '/current_camera_mode', self.camera_mode_callback, 10)

        # Mode switch echo guard to prevent race conditions during dynamic camera switching
        self._internal_mode_request = None
        self._mode_request_time = 0.0

        # Forward stall detection in FINAL_APPROACH
        self._stall_check_dist = None
        self._stall_start_time = None

        # ----------------------------------------------------------------------
        # FSM States
        # ----------------------------------------------------------------------
        # States: INIT, WAIT_NAV2, VERIFY_STAGING, ALIGN_PARALLEL,
        #         COARSE_APPROACH, FINE_APPROACH, CRAWL_CONTACT, DOCKED, NAV2_READY, FAILSAFE
        self.state = 'INIT' if self.autostart else 'NAV2_READY'
        self.state_start_time = self.get_clock().now()
        if not self.autostart:
            self.get_logger().info(">>> Initialized in NAV2_READY mode. Ready for Nav2 navigation or GUI [/start_docking] trigger.")

        # Contact Sensor State
        self.contact_detected = False
        self.contact_duration_start = None
        self.contact_debounce_sec = 0.05  # 50ms debounce

        # 3D Desk Edge Estimate
        self.desk_distance = None      # Distance from reference plane (m)
        self.desk_yaw_error = None     # Parallelism error (rad, positive = tilted left)
        self.last_depth_time = None
        self.depth_timeout_sec = 2.00  # 2.0s depth timeout (prevents stuttering during simulation render)

        # Final Approach & Docking Safeguards
        self.final_approach_start_pose = None
        self.final_approach_start_dist = None

        # Staging & Pre-docking Alignment Parameters
        self.align_stable_count = 0
        self.align_stable_required = 4   # 4 ticks * 0.05s = 0.20s continuous stability
        self.retry_count = 0
        self.max_retries = 3

        # Target Object & Virtual Docking Pose Parameters
        self.target_obj = TargetObject()
        self.target_u_c = None
        self.target_v_c = None
        self.target_bbox = None
        self.min_staging_distance = 0.32    # Minimum distance required for turn-in (m)
        self.k_clearance = 1.5              # Clearance multiplier on lateral offset
        self.base_clearance = 0.18          # Minimum vehicle sweep clearance (m)
        self.backup_target_distance = 0.40  # Standby distance to reverse to (m)
        self.dual_align_stable_count = 0
        self.dual_align_stable_required = 4 # 4 ticks * 0.05s = 0.20s continuous stability
        self.carrot_x_b = None
        self.carrot_y_b = None

        # S-Curve In-place Turn & Lateral Cruise Variables
        self.target_lateral_offset = 0.0     # Target lateral displacement (m)
        self.turn_start_yaw = 0.0            # Heading before in-place turn
        self.target_turn_angle = 0.0         # Target rotation angle (+-pi/2)
        self.lateral_start_pose = None       # (x, y) start pose for lateral cruise
        self.lateral_traveled = 0.0          # Integrated lateral distance traveled (m)
        self.stable_yaw_count = 0            # Stable heading count for in-place turns
        self.in_place_turn_dir = 1.0         # +1.0 for CCW, -1.0 for CW

        self.bridge = CvBridge()

        # ----------------------------------------------------------------------
        # TF Buffer & Listener
        # ----------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/docking/status', 10)
        self.marker_pub = self.create_publisher(Marker, '/docking/marker', 10)
        self.vis_pub = self.create_publisher(Image, '/docking/debug_image', 10)
        self.last_color_img = None

        # OS30A Depth, Color Image & CameraInfo Subscribers (SENSOR_DATA QoS matching ros_gz_bridge)
        self.depth_sub = self.create_subscription(
            Image, '/os30a/camera/depth/image_raw', self.depth_callback, qos_profile_sensor_data
        )
        self.color_sub = self.create_subscription(
            Image, '/os30a/camera/color/image_raw', self.color_callback, qos_profile_sensor_data
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/os30a/camera/depth/camera_info', self.cam_info_callback, qos_profile_sensor_data
        )

        # Single Contact Bumper Subscriber
        self.bumper_sub = self.create_subscription(
            Bool, '/robot/bumper/contact', self.bumper_callback, 10
        )

        # Nav2 NavigateToPose Action Client
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None
        self.nav2_finished = False
        self.nav2_success = False

        # Main FSM Control Loop (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("========================================================")
        self.get_logger().info(" Precision Close-Docking Node Initialized               ")
        self.get_logger().info(f" - Staging Pose:   ({self.staging_x:.2f}, {self.staging_y:.2f}, {self.staging_yaw:.2f})")
        self.get_logger().info(f" - Target Gap:     {self.target_clearance*100:.1f} cm from Reference Plane")
        self.get_logger().info(" - Alignment:      OS30A 3D PointCloud RANSAC Line Fit ")
        self.get_logger().info(" - Contact Stop:   Single Bumper Probe (/robot/bumper/contact)")
        self.get_logger().info("========================================================")

    # ==========================================================================
    # Callbacks
    # ==========================================================================
    def cam_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0] if msg.k[0] > 0 else self.fx
            self.fy = msg.k[4] if msg.k[4] > 0 else self.fy
            self.cx = msg.k[2] if msg.k[2] > 0 else self.cx
            self.cy = msg.k[5] if msg.k[5] > 0 else self.cy
            self.camera_info_received = True

    def color_callback(self, msg: Image):
        # Time-division: skip only during far-away Nav2 or standby
        if self.state in ('INIT', 'WAIT_NAV2', 'NAV2_READY'):
            return
        try:
            self.last_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def bumper_callback(self, msg: Bool):
        now = time.time()
        if msg.data:
            if self.contact_duration_start is None:
                self.contact_duration_start = now
            elif (now - self.contact_duration_start) >= self.contact_debounce_sec:
                self.contact_detected = True
        else:
            self.contact_duration_start = None
            self.contact_detected = False

    def depth_callback(self, msg: Image):
        # 1. Time-division sensor management: skip during Nav2 or idle standby
        if self.state in ('INIT', 'WAIT_NAV2', 'NAV2_READY'):
            return

        try:
            # Keep sensor watchdog updated so timeout never false-triggers!
            self.last_depth_time = self.get_clock().now()

            # Convert depth image (support 16UC1 in mm, or 32FC1 in meters)
            if msg.encoding == '16UC1':
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_m = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # 2. Blind Odometry Maneuver Handling:
            # While turning lateral or cruising, robot is facing away from desk.
            # Keep live video HUD rendering so user sees continuous video feed,
            # but mask desk/target estimates so FSM relies purely on odometry dead-reckoning!
            if self.state in ('TURN_LATERAL', 'LATERAL_CRUISE', 'TURN_FACE_DESK'):
                self.desk_distance = None
                self.desk_yaw_error = None
                self.target_obj = TargetObject()
                self.render_and_publish_visualization(depth_m, None, None, None)
                return

            self.detect_target_object(depth_m)
            self.process_depth_for_desk_edge(depth_m)

        except Exception as e:
            self.get_logger().warn(f"Depth processing error: {e}")

    # ==========================================================================
    # Target Object Detection (Red Box HSV + 11x11 Median Depth)
    # ==========================================================================
    def detect_target_object(self, depth_m: np.ndarray):
        """Detects the Red Box in color image using dual-range HSV masking,
        extracts an 11x11 patch median depth, and deprojects to 3D base_footprint coordinates.
        """
        if self.last_color_img is None:
            return

        try:
            h, w = depth_m.shape
            color = self.last_color_img
            if color.shape[:2] != (h, w):
                color = cv2.resize(color, (w, h))

            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            # Red color wraps around 0 and 180 in HSV
            mask1 = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([12, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 90, 60]), np.array([180, 255, 255]))
            red_mask = cv2.bitwise_or(mask1, mask2)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_cnt = None
            max_area = 0.0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area and area >= 35.0:  # Minimum 35 pixels
                    max_area = area
                    best_cnt = cnt

            if best_cnt is not None:
                x_b, y_b, w_b, h_b = cv2.boundingRect(best_cnt)
                u_c = int(x_b + w_b / 2.0)
                v_c = int(y_b + h_b / 2.0)

                # 11x11 median depth window (robust against specular / NaN / dropout)
                r = 5
                u_min, u_max = max(0, u_c - r), min(w, u_c + r + 1)
                v_min, v_max = max(0, v_c - r), min(h, v_c + r + 1)
                patch = depth_m[v_min:v_max, u_min:u_max]
                valid_patch = patch[(patch > 0.05) & (patch < 1.6) & np.isfinite(patch)]

                if len(valid_patch) >= 3:
                    z_val = float(np.median(valid_patch))
                elif np.isfinite(depth_m[v_c, u_c]) and depth_m[v_c, u_c] > 0.05:
                    z_val = float(depth_m[v_c, u_c])
                else:
                    z_val = None

                if z_val is not None:
                    x_c = (u_c - self.cx) * z_val / self.fx
                    y_c = (v_c - self.cy) * z_val / self.fy

                    sin_p = math.sin(self.cam_pitch)
                    cos_p = math.cos(self.cam_pitch)
                    x_obj = self.cam_x_base + cos_p * z_val - sin_p * y_c
                    y_obj = -x_c
                    z_obj = self.cam_z_base - sin_p * z_val - cos_p * y_c

                    self.target_obj.center = np.array([x_obj, y_obj, z_obj])
                    self.target_obj.docking_point = np.array([x_obj, y_obj, z_obj])
                    self.target_obj.valid = True
                    self.target_obj.last_seen_time = time.time()
                    self.target_u_c = u_c
                    self.target_v_c = v_c
                    self.target_bbox = (x_b, y_b, w_b, h_b)
                    return

            # If not detected this frame, hold for 0.8s
            if time.time() - self.target_obj.last_seen_time > 0.8:
                self.target_obj.valid = False
                self.target_u_c = None
                self.target_v_c = None
                self.target_bbox = None

        except Exception as e:
            self.get_logger().warn(f"Target detection error: {e}", throttle_duration_sec=2.0)

    def compute_docking_errors(self):
        """Computes unified 3-DOF error vector [e_x, e_y, e_theta] relative to Virtual Docking Pose.
        - e_x: Longitudinal error to target 5cm clearance (m)
        - e_y: Lateral error between robot centerline and target object (m, +e_y = target to left)
        - e_theta: Heading error between robot heading and desk normal (rad, positive = tilted left)
        """
        if self.desk_distance is None:
            return None, None, None

        # 1. Longitudinal error e_x: distance remaining to 5cm clearance
        e_x = self.desk_distance - self.target_clearance

        # 2. Heading error e_theta: angle relative to desk normal
        e_theta = self.desk_yaw_error if self.desk_yaw_error is not None else 0.0

        # 3. Lateral error e_y: offset from robot center line to target object center
        # In base_footprint, Y is positive to the left.
        # If target is at Y_obj > 0 (to the left), robot needs to steer left (positive omega).
        if self.target_obj.valid and self.target_obj.center is not None:
            e_y = float(self.target_obj.center[1])
        else:
            e_y = 0.0

        return e_x, e_y, e_theta

    # ==========================================================================
    # 3D Desk Edge & Parallelism Extraction
    # ==========================================================================
    def process_depth_for_desk_edge(self, depth_m: np.ndarray):
        h, w = depth_m.shape
        # Downsample: step 2 for 320x240 (160x120), step 4 for 640x480
        step = 2 if w <= 320 else 4
        u_grid, v_grid = np.meshgrid(
            np.arange(0, w, step),
            np.arange(0, h, step)
        )
        z_c = depth_m[v_grid, u_grid]

        # Valid depth mask (0.15m to 1.8m)
        valid_mask = (z_c > 0.15) & (z_c < 1.8) & np.isfinite(z_c)
        min_valid = 20 if w <= 320 else 50
        if np.count_nonzero(valid_mask) < min_valid:
            self.get_logger().info(f"[EDGE] Valid points too low: {np.count_nonzero(valid_mask)}", throttle_duration_sec=2.0)
            return

        u_val = u_grid[valid_mask]
        v_val = v_grid[valid_mask]
        z_val = z_c[valid_mask]

        # 3D Optical Coordinates (X: Right, Y: Down, Z: Forward)
        x_c = (u_val - self.cx) * z_val / self.fx
        y_c = (v_val - self.cy) * z_val / self.fy

        # Transform to robot base_footprint coordinates
        # Camera optical axis is pitched 60 deg down (1.047 rad)
        sin_p = math.sin(self.cam_pitch)
        cos_p = math.cos(self.cam_pitch)

        x_b = self.cam_x_base + cos_p * z_val - sin_p * y_c
        y_b = -x_c
        z_b = self.cam_z_base - sin_p * z_val - cos_p * y_c

        # Filter for desk surface height:
        # Table top is precisely at Z=0.14~0.15m in base_footprint (ground = 0.0m).
        # We enforce a tight slice Z in [0.11, 0.17]m to strictly isolate the elevated desk surface.
        table_mask = (z_b >= 0.11) & (z_b <= 0.17) & (x_b >= 0.145) & (x_b < 1.40) & (np.abs(y_b) <= 0.40)
        table_cnt = np.count_nonzero(table_mask)
        min_table = 10 if w <= 320 else 20
        if table_cnt < min_table:
            self.get_logger().info(f"[EDGE] Table points too low: {table_cnt}, zb min={np.min(z_b):.2f}, max={np.max(z_b):.2f}", throttle_duration_sec=2.0)
            self.render_and_publish_visualization(depth_m, None, None, None)
            return

        x_table = x_b[table_mask]
        y_table = y_b[table_mask]
        z_table = z_b[table_mask]

        # Planar Slope Verification (Ground-Parallel Test):
        # A true desk surface is strictly horizontal to ground (slope_x ~ 0, slope_y ~ 0).
        # Floor points projected through tilted camera produce an inclined pseudo-plane (|slope_x| > 0.40).
        try:
            A_mat = np.column_stack([x_table, y_table, np.ones_like(x_table)])
            coeffs, _, _, _ = np.linalg.lstsq(A_mat, z_table, rcond=None)
            slope_x, slope_y = float(coeffs[0]), float(coeffs[1])
            # If surface is tilted forward/backward by more than 14 deg (|slope_x| > 0.25), reject as floor projection
            if abs(slope_x) > 0.25:
                self.get_logger().info(f"[EDGE] Rejected non-horizontal surface (slope_x={slope_x:+.2f} > 0.25). Likely floor.", throttle_duration_sec=2.0)
                self.render_and_publish_visualization(depth_m, None, None, None)
                return
        except Exception:
            pass

        # Extract front edge: For each Y bin (lateral position), find minimum X (front-most surface point)
        y_min_robust = np.percentile(y_table, 5)
        y_max_robust = np.percentile(y_table, 95)
        if y_max_robust <= y_min_robust:
            self.render_and_publish_visualization(depth_m, None, None, None)
            return

        y_bins = np.linspace(y_min_robust, y_max_robust, 15)
        edge_x = []
        edge_y = []

        for i in range(len(y_bins) - 1):
            bin_lateral = (y_b >= y_bins[i]) & (y_b < y_bins[i + 1]) & (x_b >= 0.145) & (x_b < 1.40)
            bin_table = bin_lateral & (z_b >= 0.11) & (z_b <= 0.17)
            if np.count_nonzero(bin_table) >= 2:
                # 10th percentile as candidate front edge point
                front_x = float(np.percentile(x_b[bin_table], 10))
                mid_y = 0.5 * (y_bins[i] + y_bins[i + 1])

                # Step-height (Cliff) Discontinuity Check:
                # When desk is at moderate distance, floor is visible in front (z_b < 0.06m).
                # When robot is docked very close (<= 25cm), ground leaves FOV and only table is seen.
                floor_in_front = bin_lateral & (z_b < 0.06) & (x_b < front_x + 0.05)
                front_region_mask = bin_lateral & (x_b < front_x + 0.02) & (x_b > front_x - 0.20)

                mean_table_z = float(np.mean(z_b[bin_table]))
                is_step_edge = False

                # Case 1: Ground visible in front, verify clear vertical drop (>= 0.06m)
                if np.count_nonzero(floor_in_front) >= 2:
                    mean_floor_z = float(np.mean(z_b[floor_in_front]))
                    if (mean_table_z - mean_floor_z) >= 0.06:
                        is_step_edge = True
                # Case 2: Robot in close approach (<= 25cm) where ground is below camera frame
                elif self.desk_distance is not None and self.desk_distance <= 0.25:
                    is_step_edge = True
                # Case 3: Front points exist, verify drop
                elif np.count_nonzero(front_region_mask) > 0:
                    min_front_z = float(np.min(z_b[front_region_mask]))
                    if (mean_table_z - min_front_z) >= 0.06:
                        is_step_edge = True

                if is_step_edge:
                    edge_x.append(front_x)
                    edge_y.append(mid_y)

        if len(edge_x) >= 5:
            edge_x = np.array(edge_x)
            edge_y = np.array(edge_y)

            # RANSAC Robust Line Fit to reject side edges & corner outliers when approaching skewed
            best_inliers = []
            best_p = None
            n_pts = len(edge_x)
            for _ in range(35):
                sample_idx = np.random.choice(n_pts, 2, replace=False)
                dy = edge_y[sample_idx[1]] - edge_y[sample_idx[0]]
                if abs(dy) < 0.04:
                    continue
                cand_p = np.polyfit(edge_y[sample_idx], edge_x[sample_idx], 1)
                # Perpendicular distance: |m*y - x + c| / sqrt(m^2 + 1)
                perp_dists = np.abs(cand_p[0] * edge_y - edge_x + cand_p[1]) / math.sqrt(cand_p[0]**2 + 1)
                inliers = np.where(perp_dists < 0.022)[0]  # 22mm threshold
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_p = cand_p

            if len(best_inliers) >= 4:
                p = np.polyfit(edge_y[best_inliers], edge_x[best_inliers], 1)
                inlier_x = edge_x[best_inliers]
                inlier_y = edge_y[best_inliers]
            else:
                p = np.polyfit(edge_y, edge_x, 1)
                inlier_x = edge_x
                inlier_y = edge_y

            slope_m, intercept_c = p[0], p[1]
            yaw_err = math.atan(slope_m)
            # Robust physical distance to front edge (immune to angle extrapolation errors)
            dist_from_ref = float(np.median(inlier_x)) - self.reference_x

            self.desk_distance = dist_from_ref
            self.desk_yaw_error = yaw_err
            self.get_logger().info(f"[EDGE FOUND (RANSAC)] dist={dist_from_ref*100:.1f}cm, yaw_err={math.degrees(yaw_err):.2f}° ({len(inlier_x)}/{n_pts} inliers)", throttle_duration_sec=1.0)
            self.publish_edge_marker(intercept_c, slope_m)

            edge_pts_3d = list(zip(inlier_x, inlier_y))
            self.render_and_publish_visualization(depth_m, edge_pts_3d, slope_m, intercept_c)
        else:
            self.get_logger().info(f"[EDGE] edge_x count too small: {len(edge_x)}", throttle_duration_sec=2.0)
            self.render_and_publish_visualization(depth_m, None, None, None)

    def project_point_to_image(self, x_b, y_b, z_b=0.14):
        """Projects a 3D point in robot base_footprint frame to camera image (u, v)."""
        sin_p = math.sin(self.cam_pitch)
        cos_p = math.cos(self.cam_pitch)
        dx = x_b - self.cam_x_base
        dz = z_b - self.cam_z_base
        z_c = cos_p * dx - sin_p * dz
        if z_c <= 0.05:
            return None
        y_c = -sin_p * dx - cos_p * dz
        x_c = -y_b
        u = int(self.fx * (x_c / z_c) + self.cx)
        v = int(self.fy * (y_c / z_c) + self.cy)
        return (u, v)

    def render_and_publish_visualization(self, depth_m, edge_pts_3d=None, slope_m=None, intercept_c=None):
        """Renders live visual overlay: RGB background, fitted edge line, 5cm target line, depth thumbnail, and HUD."""
        try:
            h, w = depth_m.shape
            # Base image: use real color RGB if available, otherwise colormap depth
            if self.last_color_img is not None:
                canvas = self.last_color_img.copy()
            else:
                d_vis = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
                d_norm = cv2.normalize(d_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                canvas = cv2.applyColorMap(d_norm, cv2.COLORMAP_TURBO)

            # Upscale canvas 2x to 640x480 for crisp typography and UI overlay
            vis_w, vis_h = 640, 480
            vis = cv2.resize(canvas, (vis_w, vis_h), interpolation=cv2.INTER_LINEAR)
            sx = vis_w / float(w)
            sy = vis_h / float(h)

            # 1. Draw Target Docking Reference Line (Clearance = 5cm -> x_b = 0.150m)
            # Astra front tip is at x=0.100m, target gap=0.050m -> front edge must align at x_b = 0.150m
            target_pts = []
            for y_samp in np.linspace(-0.25, 0.25, 20):
                pt = self.project_point_to_image(0.150, y_samp, z_b=0.14)
                if pt is not None:
                    u_s = int(pt[0] * sx)
                    v_s = int(pt[1] * sy)
                    if 0 <= u_s < vis_w and 0 <= v_s < vis_h:
                        target_pts.append((u_s, v_s))

            for i in range(len(target_pts) - 1):
                if i % 2 == 0:
                    cv2.line(vis, target_pts[i], target_pts[i + 1], (255, 255, 0), 2, cv2.LINE_AA)

            if target_pts:
                mid_tgt = target_pts[len(target_pts) // 2]
                cv2.putText(vis, "TARGET 5CM GOAL LINE", (max(10, mid_tgt[0] - 85), min(vis_h - 20, mid_tgt[1] + 18)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1, cv2.LINE_AA)

            # 2. Draw Detected Desk Edge Points & Fitted Line
            if slope_m is not None and intercept_c is not None:
                edge_line_pts = []
                for y_samp in np.linspace(-0.35, 0.35, 25):
                    x_val = slope_m * y_samp + intercept_c
                    pt = self.project_point_to_image(x_val, y_samp, z_b=0.14)
                    if pt is not None:
                        u_s = int(pt[0] * sx)
                        v_s = int(pt[1] * sy)
                        if 0 <= u_s < vis_w and 0 <= v_s < vis_h:
                            edge_line_pts.append((u_s, v_s))

                # Outer glow + bright green line
                for i in range(len(edge_line_pts) - 1):
                    cv2.line(vis, edge_line_pts[i], edge_line_pts[i + 1], (0, 160, 0), 6, cv2.LINE_AA)
                    cv2.line(vis, edge_line_pts[i], edge_line_pts[i + 1], (0, 255, 0), 2, cv2.LINE_AA)

                # Detected front edge points (Yellow dots)
                if edge_pts_3d:
                    for (x_p, y_p) in edge_pts_3d:
                        pt = self.project_point_to_image(x_p, y_p, z_b=0.14)
                        if pt is not None:
                            u_s = int(pt[0] * sx)
                            v_s = int(pt[1] * sy)
                            if 0 <= u_s < vis_w and 0 <= v_s < vis_h:
                                cv2.circle(vis, (u_s, v_s), 4, (0, 255, 255), -1, cv2.LINE_AA)
                                cv2.circle(vis, (u_s, v_s), 5, (0, 0, 0), 1, cv2.LINE_AA)

            # 2.5 Draw Target Object (Red Box) & Virtual Docking Axis
            if self.target_bbox is not None:
                bx, by, bw, bh = self.target_bbox
                u1 = int(bx * sx)
                v1 = int(by * sy)
                u2 = int((bx + bw) * sx)
                v2 = int((by + bh) * sy)
                cv2.rectangle(vis, (u1, v1), (u2, v2), (0, 0, 255), 2)
                cv2.circle(vis, (int((u1 + u2) / 2), int((v1 + v2) / 2)), 5, (0, 0, 255), -1)
                t_label = "TARGET RED BOX"
                if self.target_obj.valid and self.target_obj.center is not None:
                    t_label += f" ({self.target_obj.center[0]:.2f}m, {self.target_obj.center[1]*100:+.1f}cm)"
                cv2.putText(vis, t_label, (max(10, u1), max(20, v1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 60, 255), 1, cv2.LINE_AA)

            # Virtual Docking Axis (Approach centerline toward target object)
            if self.target_obj.valid and self.target_obj.center is not None:
                y_dock = float(self.target_obj.center[1])
                dock_ray_pts = []
                for x_samp in np.linspace(0.10, max(0.50, float(self.target_obj.center[0]) + 0.10), 16):
                    pt = self.project_point_to_image(x_samp, y_dock, z_b=0.14)
                    if pt is not None:
                        u_s = int(pt[0] * sx)
                        v_s = int(pt[1] * sy)
                        if 0 <= u_s < vis_w and 0 <= v_s < vis_h:
                            dock_ray_pts.append((u_s, v_s))
                for i in range(len(dock_ray_pts) - 1):
                    if i % 2 == 0:
                        cv2.line(vis, dock_ray_pts[i], dock_ray_pts[i + 1], (255, 220, 0), 2, cv2.LINE_AA)
                if dock_ray_pts:
                    mid_ray = dock_ray_pts[0]
                    cv2.putText(vis, "DOCKING AXIS", (mid_ray[0] + 6, mid_ray[1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 220, 0), 1, cv2.LINE_AA)

            # Carrot Point (Lookahead goal along docking axis)
            if self.carrot_x_b is not None and self.carrot_y_b is not None:
                pt_c = self.project_point_to_image(self.carrot_x_b, self.carrot_y_b, z_b=0.14)
                if pt_c is not None:
                    uc_s = int(pt_c[0] * sx)
                    vc_s = int(pt_c[1] * sy)
                    if 0 <= uc_s < vis_w and 0 <= vc_s < vis_h:
                        cv2.circle(vis, (uc_s, vc_s), 6, (255, 0, 255), -1, cv2.LINE_AA)
                        cv2.circle(vis, (uc_s, vc_s), 8, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.putText(vis, "CARROT", (uc_s + 8, vc_s + 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 0, 255), 1, cv2.LINE_AA)

            # 3. Mini Depth Inset (Picture-in-Picture at bottom-right)
            d_vis = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
            d_norm = cv2.normalize(d_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_thumb = cv2.applyColorMap(d_norm, cv2.COLORMAP_TURBO)
            thumb_w, thumb_h = 160, 120
            depth_thumb = cv2.resize(depth_thumb, (thumb_w, thumb_h))
            x_off = vis_w - thumb_w - 12
            y_off = vis_h - thumb_h - 12
            vis[y_off:y_off + thumb_h, x_off:x_off + thumb_w] = depth_thumb
            cv2.rectangle(vis, (x_off, y_off), (x_off + thumb_w, y_off + thumb_h), (200, 200, 200), 1)
            cv2.putText(vis, "DEPTH MAP (OS30A)", (x_off + 6, y_off + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

            # 4. Modern Glassmorphism Top HUD Dashboard
            overlay = vis.copy()
            hud_h = 92
            cv2.rectangle(overlay, (0, 0), (vis_w, hud_h), (15, 15, 18), -1)
            cv2.addWeighted(overlay, 0.82, vis, 0.18, 0, vis)
            cv2.line(vis, (0, hud_h), (vis_w, hud_h), (60, 60, 70), 1)

            # State color logic
            state_color = (0, 255, 255)
            if self.state == 'DOCKED':
                state_color = (0, 255, 0)
            elif 'ALIGN' in self.state:
                state_color = (255, 200, 0)
            elif 'CRAWL' in self.state or 'FINAL' in self.state:
                state_color = (0, 165, 255)
            elif 'ABORT' in self.state or 'UNDOCK' in self.state:
                state_color = (0, 140, 255)  # Orange for active safety retreat
            elif self.state == 'FAILSAFE':
                state_color = (0, 0, 255)

            # Row 1: State & Bumper Status
            if self.state == 'ABORT_TO_NAV2':
                elapsed_abort = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                state_text = f"STATE: ABORTING [{max(0.0, 4.0 - elapsed_abort):3.1f}s RETREAT]"
            elif self.state == 'UNDOCKING':
                elapsed_undock = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                state_text = f"STATE: UNDOCKING [{max(0.0, 3.0 - elapsed_undock):3.1f}s BACKUP]"
            else:
                state_text = f"STATE: {self.state}"
                if self.docking_routine_start_time is not None:
                    elapsed_dock = (self.get_clock().now() - self.docking_routine_start_time).nanoseconds / 1e9
                    rem_dock = max(0.0, self.docking_timeout_sec - elapsed_dock)
                    state_text += f" [{rem_dock:4.1f}s left]"

            cv2.putText(vis, state_text, (14, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2, cv2.LINE_AA)

            bumper_text = "BUMPER: [ CONTACT! ]" if self.contact_detected else "BUMPER: [ CLEAR ]"
            bumper_color = (0, 0, 255) if self.contact_detected else (180, 180, 180)
            cv2.putText(vis, bumper_text, (vis_w - 225, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, bumper_color, 2 if self.contact_detected else 1, cv2.LINE_AA)

            # Row 2 & 3: Distance, Yaw Error, Lateral Offset (ey), and Target Object Status
            e_x, e_y, e_theta = self.compute_docking_errors()
            if self.desk_distance is not None and self.desk_yaw_error is not None:
                dist_cm = self.desk_distance * 100.0
                dist_err_cm = (self.desk_distance - self.target_clearance) * 100.0
                yaw_deg = math.degrees(self.desk_yaw_error)

                dist_col = (0, 255, 0) if abs(dist_err_cm) <= 1.0 else ((0, 255, 255) if dist_cm <= 20.0 else (240, 240, 240))
                yaw_col = (0, 255, 0) if abs(yaw_deg) <= 2.0 else (0, 200, 255)

                dist_str = f"DIST: {dist_cm:4.1f}cm (Target: 5.0cm, Err: {dist_err_cm:+4.1f}cm)"
                yaw_str = f"YAW ERR: {yaw_deg:+5.1f}° (Tol: ±2.0°)"

                cv2.putText(vis, dist_str, (14, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.48, dist_col, 1, cv2.LINE_AA)
                cv2.putText(vis, yaw_str, (14, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.48, yaw_col, 1, cv2.LINE_AA)
            else:
                cv2.putText(vis, "DESK EDGE: SEARCHING...", (14, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 120, 255), 1, cv2.LINE_AA)

            # Column 2 (x = 340): Lateral Error (ey) & Target Object
            if e_y is not None:
                ey_cm = e_y * 100.0
                ey_col = (0, 255, 0) if abs(ey_cm) <= 1.2 else (0, 220, 255)
                ey_str = f"LATERAL (ey): {ey_cm:+5.1f}cm (Tol: ±1.2cm)"
                cv2.putText(vis, ey_str, (340, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.48, ey_col, 1, cv2.LINE_AA)
            else:
                cv2.putText(vis, "LATERAL (ey): --", (340, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (160, 160, 160), 1, cv2.LINE_AA)

            if self.target_obj.valid and self.target_obj.center is not None:
                t_status_str = f"TARGET: [ FOUND ] X={self.target_obj.center[0]:.2f}m"
                cv2.putText(vis, t_status_str, (340, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 120), 1, cv2.LINE_AA)
            else:
                cv2.putText(vis, "TARGET: [ SEARCHING ]", (340, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 120, 255), 1, cv2.LINE_AA)

            # 5. Publish to ROS 2 Image Topic
            out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'os30a_camera_color_optical_frame'
            self.vis_pub.publish(out_msg)

            # 6. Display OpenCV GUI window if requested (Only during active docking)
            if self.show_window and self.state in DOCKING_ACTIVE_STATES:
                cv2.imshow("OS30A Precision Docking Visualizer", vis)
                cv2.waitKey(1)
            elif self.show_window and self.state not in DOCKING_ACTIVE_STATES:
                try:
                    cv2.destroyWindow("OS30A Precision Docking Visualizer")
                except Exception:
                    pass

        except Exception as e:
            self.get_logger().warn(f"Visualization render error: {e}", throttle_duration_sec=2.0)

    def publish_edge_marker(self, c, m):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'desk_edge'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.015  # Line width
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0

        for y in [-0.25, 0.25]:
            x = m * y + c
            from geometry_msgs.msg import Point
            p = Point(x=float(x), y=float(y), z=0.14)
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_standby_hud(self):
        """Publishes a clean standby banner when in NAV2_READY so HUD doesn't freeze on old state."""
        try:
            h, w = 480, 640
            vis = np.zeros((h, w, 3), dtype=np.uint8)
            vis[:] = (20, 24, 28)
            cv2.rectangle(vis, (40, 140), (w - 40, 340), (35, 40, 48), -1)
            cv2.rectangle(vis, (40, 140), (w - 40, 340), (0, 200, 255), 2)
            cv2.putText(vis, "NAV2 AUTONOMOUS NAVIGATION ACTIVE", (70, 205),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(vis, "Astra S Camera Active  |  RTAB-Map SLAM Resumed", (85, 250),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.putText(vis, "[ Precision Docking Engine on Standby ]", (135, 295),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 255, 100), 1, cv2.LINE_AA)
            out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'os30a_camera_color_optical_frame'
            self.vis_pub.publish(out_msg)
        except Exception:
            pass

    # ==========================================================================
    # Main Finite State Machine (FSM)
    # ==========================================================================
    def control_loop(self):
        try:
            self._control_loop_impl()
        except Exception as e:
            self.get_logger().error(f"Unexpected error in control_loop: {e}", throttle_duration_sec=1.0)

    def _control_loop_impl(self):
        now = self.get_clock().now()
        twist = Twist()

        # Compute 3-DOF docking errors
        e_x, e_y, e_theta = self.compute_docking_errors()

        # Publish status string
        ey_str = f"{e_y*100:+.1f}" if e_y is not None else "0.0"
        status_msg = String()
        status_msg.data = f"STATE={self.state}, dist={self.desk_distance}, yaw_err={self.desk_yaw_error}, ey={ey_str}, target={self.target_obj.valid}, contact={self.contact_detected}"
        self.status_pub.publish(status_msg)

        # ----------------------------------------------------------------------
        # Global Docking Timeout Guard: Abort, reverse, and return to Nav2
        # ----------------------------------------------------------------------
        if self.state in DOCKING_ACTIVE_STATES:
            if self.docking_routine_start_time is not None:
                total_docking_elapsed = (now - self.docking_routine_start_time).nanoseconds / 1e9
                if total_docking_elapsed > self.docking_timeout_sec:
                    self.get_logger().warn(f">>> [DOCKING TIMEOUT] Exceeded {self.docking_timeout_sec:.1f}s without docking. Aborting, retreating, and returning to Nav2 mode...")
                    self.transition_to('ABORT_TO_NAV2')
                    return

            # Note: Target object may exit camera FOV when getting close to desk surface.
            # If target object is lost but desk edge is visible, continue docking using desk edge alignment!
            if self.state in ('CARROT_PURSUIT_ALIGN', 'VERIFY_DUAL_ALIGN'):
                if not self.target_obj.valid and self.desk_distance is not None:
                    self.get_logger().info(">>> [TARGET FOV] Object out of view close to desk. Holding centerline heading with desk edge.", throttle_duration_sec=3.0)

        # ----------------------------------------------------------------------
        # STATE: INIT
        # ----------------------------------------------------------------------
        if self.state == 'INIT':
            curr_pose = self.get_robot_pose_in_map()
            if curr_pose is None:
                self.get_logger().info(">>> [INIT] Waiting for map -> base_footprint TF...", throttle_duration_sec=2.0)
                return

            # If desk is already in view of OS30A, robot is at the staging area
            if self.desk_distance is not None and 0.20 <= self.desk_distance <= 0.55:
                self.get_logger().info(f">>> [INIT] Desk already in view ({self.desk_distance*100:.1f}cm). Transitioning directly to CHECK_STAGING.")
                self.transition_to('CHECK_STAGING')
                return

            dist_to_staging = math.hypot(curr_pose[0] - self.staging_x, curr_pose[1] - self.staging_y)
            self.get_logger().info(f">>> [INIT] Current pose: X={curr_pose[0]:.2f}, Y={curr_pose[1]:.2f}, Yaw={math.degrees(curr_pose[2]):.1f}°. Dist to staging: {dist_to_staging:.2f}m")
            if self.skip_nav2_if_close and dist_to_staging <= 0.50:
                self.get_logger().info(">>> Already near staging position. Transitioning to CHECK_STAGING.")
                self.transition_to('CHECK_STAGING')
            else:
                self.get_logger().info(">>> Far from staging position. Dispatching Nav2 Staging Goal...")
                self.send_nav2_staging_goal()
                self.transition_to('WAIT_NAV2')

        # ----------------------------------------------------------------------
        # STATE: WAIT_NAV2 (Wait for Nav2 staging navigation to reach vicinity)
        # ----------------------------------------------------------------------
        elif self.state == 'WAIT_NAV2':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            curr_pose = self.get_robot_pose_in_map()

            # 1. Check if Nav2 completed
            if self.nav2_finished:
                if self.nav2_success:
                    self.get_logger().info(">>> Nav2 reached staging pose successfully! Proceeding to CHECK_STAGING.")
                else:
                    self.get_logger().warn(">>> Nav2 ended without success. Proceeding to CHECK_STAGING directly.")
                self.transition_to('CHECK_STAGING')
                return

            # 2. Camera desk acquisition: as soon as desk is seen, cancel Nav2 and take over
            if self.desk_distance is not None and 0.20 <= self.desk_distance <= 0.55:
                self.get_logger().info(f">>> Desk acquired in view ({self.desk_distance*100:.1f}cm). Canceling Nav2 and taking over precision approach!")
                if self.nav2_goal_handle is not None:
                    try:
                        self.nav2_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self.transition_to('CHECK_STAGING')
                return

            # 3. Timeout guard: 25s
            if elapsed > 25.0:
                self.get_logger().warn(">>> Nav2 staging wait timeout (25s). Transitioning to CHECK_STAGING directly.")
                if self.nav2_goal_handle is not None:
                    try:
                        self.nav2_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self.transition_to('CHECK_STAGING')
                return

        # ----------------------------------------------------------------------
        # STATE: CHECK_STAGING (Assess clearance and lateral offset)
        # ----------------------------------------------------------------------
        elif self.state == 'CHECK_STAGING':
            if self.desk_distance is not None:
                e_x, e_y, e_theta = self.compute_docking_errors()
                e_y_val = abs(e_y) if e_y is not None else 0.0

                # 1. Need standoff clearance if too close to desk (< 35cm)
                if self.desk_distance < 0.35:
                    self.get_logger().info(
                        f">>> [CHECK_STAGING] Clearance too tight ({self.desk_distance*100:.1f}cm < 35cm). "
                        f"Backing up to 40cm optimal standoff..."
                    )
                    self.transition_to('BACKUP_STANDOFF')
                    return

                # 2. Too far (> 48cm), gently advance to 38-42cm zone
                if self.desk_distance > 0.48:
                    twist.linear.x = 0.04
                    self.get_logger().info(f">>> Approaching staging zone ({self.desk_distance*100:.1f}cm)...", throttle_duration_sec=1.5)
                    return

                # 3. Optimal standoff distance secured (35cm ~ 48cm): Evaluate Lateral Offset (ey)
                self.target_lateral_offset = e_y if e_y is not None else 0.0
                curr_pose = self.get_robot_pose_in_map()
                self.turn_start_yaw = curr_pose[2] if curr_pose is not None else 0.0

                # Branch B: Large Lateral Offset (|ey| > 4.0cm) -> In-place 90 deg Turn & Cruise
                if abs(self.target_lateral_offset) > 0.040:
                    self.get_logger().info(
                        f">>> [CASE B] Large lateral offset detected (ey={self.target_lateral_offset*100:+.1f}cm > 4.0cm). "
                        f"Initiating In-place 90° Turn toward target object..."
                    )
                    self.transition_to('TURN_LATERAL')
                    return

                # Branch A: Small Lateral Offset (1.2cm < |ey| <= 4.0cm) -> Fine Carrot Alignment
                elif abs(self.target_lateral_offset) > 0.012:
                    self.get_logger().info(
                        f">>> [CASE A] Small lateral offset detected (ey={self.target_lateral_offset*100:+.1f}cm <= 4.0cm). "
                        f"Initiating Carrot-Point Pure Pursuit alignment..."
                    )
                    self.dual_align_stable_count = 0
                    self.transition_to('CARROT_ALIGN')
                    return

                # Both aligned: |ey| <= 1.2cm -> Parallel Check or Direct Final Approach
                else:
                    if abs(e_theta) > math.radians(2.0):
                        self.get_logger().info(f">>> Centerline matched (ey={self.target_lateral_offset*100:+.1f}cm). Aligning heading parallel...")
                        self.dual_align_stable_count = 0
                        self.transition_to('ALIGN_PARALLEL')
                    else:
                        self.get_logger().info(f">>> Center and heading fully aligned! Proceeding directly to FINAL_APPROACH.")
                        self.transition_to('FINAL_APPROACH')
                    return

            else:
                # Searching for desk
                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                if elapsed < 2.0:
                    twist.linear.x = 0.0
                elif elapsed < 25.0:
                    twist.linear.x = 0.04
                    self.get_logger().info(">>> Seeking desk edge... advancing forward at 4cm/s...", throttle_duration_sec=1.5)
                else:
                    self.get_logger().warn(">>> Cannot find desk edge after 25s search. Aborting to Nav2 mode...")
                    self.transition_to('ABORT_TO_NAV2')

        # ----------------------------------------------------------------------
        # STATE: BACKUP_STANDOFF (Retreat to 40cm optimal standoff distance)
        # ----------------------------------------------------------------------
        elif self.state in ('BACKUP_STANDOFF', 'BACKUP_PREPARE', 'BACKUP_ALIGN_PREPARE'):
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            twist.linear.x = -0.05  # -5 cm/s reverse
            if (self.desk_distance is not None and self.desk_distance >= 0.40) or elapsed > 4.5:
                self.get_logger().info(">>> Optimal 40cm standoff secured. Re-assessing staging...")
                self.stop_robot()
                self.transition_to('CHECK_STAGING')
                return

        # ======================================================================
        # CASE B: Large Lateral Error Sequence (TURN_LATERAL -> LATERAL_CRUISE -> TURN_FACE_DESK)
        # ======================================================================
        # STATE: TURN_LATERAL (In-place 90 deg turn toward object direction)
        # ----------------------------------------------------------------------
        elif self.state == 'TURN_LATERAL':
            twist.linear.x = 0.0  # Strict zero linear velocity
            curr_pose = self.get_robot_pose_in_map()
            if curr_pose is not None:
                curr_yaw = curr_pose[2]
                # If target is to left (ey > 0), turn CCW (+90 deg, +pi/2)
                # If target is to right (ey < 0), turn CW (-90 deg, -pi/2)
                target_yaw = self.turn_start_yaw + (math.pi / 2.0 if self.target_lateral_offset > 0 else -math.pi / 2.0)
                yaw_err = math.atan2(math.sin(target_yaw - curr_yaw), math.cos(target_yaw - curr_yaw))

                if abs(yaw_err) <= math.radians(2.5):
                    self.stable_yaw_count += 1
                    twist.angular.z = 0.0
                    if self.stable_yaw_count >= 3:
                        self.get_logger().info(f">>> [TURN_LATERAL] 90° Turn Complete! Heading aligned with desk lateral axis.")
                        self.stop_robot()
                        self.transition_to('LATERAL_CRUISE')
                        return
                else:
                    self.stable_yaw_count = 0
                    w_z = math.copysign(min(0.25, max(0.06, 0.65 * abs(yaw_err))), yaw_err)
                    twist.angular.z = float(w_z)
            else:
                # TF fallback: time-based turn ~ 0.20 rad/s * 7.85s = 1.57 rad
                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                w_dir = 0.20 if self.target_lateral_offset > 0 else -0.20
                twist.angular.z = w_dir
                if elapsed >= 7.85:
                    self.stop_robot()
                    self.transition_to('LATERAL_CRUISE')
                    return

        # ----------------------------------------------------------------------
        # STATE: LATERAL_CRUISE (Cruise along desk to eliminate lateral offset ey)
        # ----------------------------------------------------------------------
        elif self.state == 'LATERAL_CRUISE':
            # Target translation distance = |target_lateral_offset|
            dist_target = max(0.02, abs(self.target_lateral_offset))
            curr_pose = self.get_robot_pose_in_map()

            if self.lateral_start_pose is None and curr_pose is not None:
                self.lateral_start_pose = (curr_pose[0], curr_pose[1])

            if curr_pose is not None and self.lateral_start_pose is not None:
                dx = curr_pose[0] - self.lateral_start_pose[0]
                dy = curr_pose[1] - self.lateral_start_pose[1]
                self.lateral_traveled = math.hypot(dx, dy)
            else:
                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                self.lateral_traveled = elapsed * 0.040

            self.get_logger().info(
                f">>> [LATERAL_CRUISE] Cruising toward object: traveled {self.lateral_traveled*100:.1f}cm / target {dist_target*100:.1f}cm...",
                throttle_duration_sec=1.0
            )

            # Check termination: reached target distance (with 1.0cm margin)
            if self.lateral_traveled >= (dist_target - 0.010):
                self.get_logger().info(f">>> [LATERAL_CRUISE] Target displacement reached! Preparing to face desk...")
                self.stop_robot()
                self.transition_to('TURN_FACE_DESK')
                return

            twist.linear.x = 0.040  # 4 cm/s forward along desk edge
            twist.angular.z = 0.0

        # ----------------------------------------------------------------------
        # STATE: TURN_FACE_DESK (Turn in-place 90 deg back to face desk front via Odometry)
        # ----------------------------------------------------------------------
        elif self.state == 'TURN_FACE_DESK':
            twist.linear.x = 0.0  # Strict zero linear velocity
            curr_pose = self.get_robot_pose_in_map()

            if curr_pose is not None:
                curr_yaw = curr_pose[2]
                # Target yaw is the original frontal desk orientation saved at CHECK_STAGING
                target_yaw = self.turn_start_yaw
                yaw_err = math.atan2(math.sin(target_yaw - curr_yaw), math.cos(target_yaw - curr_yaw))

                if abs(yaw_err) <= math.radians(2.5):
                    self.stable_yaw_count += 1
                    twist.angular.z = 0.0
                    if self.stable_yaw_count >= 3:
                        self.get_logger().info(
                            f">>> [TURN_FACE_DESK] 90° Reverse Turn Complete! Re-faced desk front (yaw_err={math.degrees(yaw_err):+.1f}°). "
                            f"Resuming camera perception and re-evaluating staging..."
                        )
                        self.stop_robot()
                        self.reset_docking_variables()  # Clear stale depth data
                        self.transition_to('CHECK_STAGING')
                        return
                else:
                    self.stable_yaw_count = 0
                    w_z = math.copysign(min(0.25, max(0.06, 0.65 * abs(yaw_err))), yaw_err)
                    twist.angular.z = float(w_z)
            else:
                # Time-based blind turn fallback: opposite to TURN_LATERAL
                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                w_dir = -0.20 if self.target_lateral_offset > 0 else 0.20
                twist.angular.z = w_dir
                if elapsed >= 7.85:
                    self.stop_robot()
                    self.reset_docking_variables()
                    self.transition_to('CHECK_STAGING')
                    return

        # ======================================================================
        # CASE A: Small Lateral Error Sequence (CARROT_ALIGN -> ALIGN_PARALLEL)
        # ======================================================================
        # STATE: CARROT_ALIGN (Fine Carrot-Point Pure Pursuit without heading damping)
        # ----------------------------------------------------------------------
        elif self.state in ('CARROT_ALIGN', 'CARROT_PURSUIT_ALIGN', 'COARSE_APPROACH'):
            if self.check_depth_timeout():
                return

            e_x, e_y, e_theta = self.compute_docking_errors()
            if e_x is not None:
                # Reach 12cm standoff from target (i.e. desk_dist <= 17cm)
                if e_x <= 0.12 or (e_y is not None and abs(e_y) <= 0.010):
                    ey_val_str = f"{e_y*100:+.1f}cm" if e_y is not None else "None"
                    self.get_logger().info(f">>> [CARROT_ALIGN] Standoff reached (ex={e_x*100:.1f}cm, ey={ey_val_str}). Aligning parallel...")
                    self.dual_align_stable_count = 0
                    self.stop_robot()
                    self.transition_to('ALIGN_PARALLEL')
                    return

                # Pure Pursuit toward Carrot Point along docking axis
                L = 0.20  # 20cm lookahead
                e_y_val = e_y if e_y is not None else 0.0
                y_carrot = float(e_y_val * np.clip(L / max(e_x, 0.06), 0.35, 1.0))
                x_carrot = float(L)
                alpha = math.atan2(y_carrot, x_carrot)

                v_x = 0.030  # 3 cm/s gentle advance
                w_z = float(np.clip((2.0 * v_x * math.sin(alpha)) / L, -0.22, 0.22))

                twist.linear.x = v_x
                twist.angular.z = w_z
                self.carrot_x_b = x_carrot
                self.carrot_y_b = y_carrot

        # ----------------------------------------------------------------------
        # STATE: ALIGN_PARALLEL (In-place rotation to make desk edge strictly parallel)
        # ----------------------------------------------------------------------
        elif self.state in ('ALIGN_PARALLEL', 'FINE_APPROACH', 'VERIFY_DUAL_ALIGN'):
            if self.check_depth_timeout() or self.check_contact_trigger():
                return

            twist.linear.x = 0.0  # Zero forward motion during in-place parallel alignment
            e_x, e_y, e_theta = self.compute_docking_errors()

            if e_theta is not None:
                e_y_val = abs(e_y) if e_y is not None else 0.0
                aligned = (abs(e_theta) <= math.radians(1.5)) and (e_y_val <= 0.018)

                if aligned:
                    self.dual_align_stable_count += 1
                    twist.angular.z = 0.0
                    if self.dual_align_stable_count >= self.dual_align_stable_required:
                        self.get_logger().info(
                            f">>> [ALIGN_PARALLEL] Both Center & Parallelism Verified! (ey={e_y*100:+.1f}cm, yaw={math.degrees(e_theta):+.1f}°). "
                            f"Commencing FINAL_APPROACH."
                        )
                        self.stop_robot()
                        self.transition_to('FINAL_APPROACH')
                        return
                else:
                    self.dual_align_stable_count = 0
                    w_z = -math.copysign(min(0.15, max(0.03, 0.40 * abs(e_theta))), e_theta)
                    twist.angular.z = float(w_z)

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > 8.0:
                self.get_logger().warn(">>> Parallel alignment time limit (8s). Checking ey before final approach...")
                if e_y is not None and abs(e_y) <= 0.025:
                    self.transition_to('FINAL_APPROACH')
                else:
                    # Still misaligned: re-stage
                    self.transition_to('CHECK_STAGING')
                return

        # ----------------------------------------------------------------------
        # STATE: FINAL_APPROACH (Straight crawl, heading strictly locked)
        # ----------------------------------------------------------------------
        elif self.state in ('CRAWL_CONTACT', 'FINAL_APPROACH'):
            # HARD GUARD: Do not allow final crawl if lateral offset is unacceptable (> 2.5cm)
            e_x, e_y, e_theta = self.compute_docking_errors()
            if e_y is not None and abs(e_y) > 0.025 and self.desk_distance is not None and self.desk_distance > 0.12:
                self.get_logger().warn(f">>> [GUARD] Lateral offset ey={e_y*100:+.1f}cm exceeded 2.5cm in FINAL_APPROACH! Returning to CHECK_STAGING.")
                self.stop_robot()
                self.transition_to('CHECK_STAGING')
                return

            # 1. Primary Trigger: Physical Contact Switch (Micro Bumper) - Highest Reliability
            if self.check_contact_trigger():
                return

            # 2. Secondary Trigger: Optical Clearance Fallback (5.0cm +- 2mm) with Travel Guard
            if self.desk_distance is not None and self.desk_distance <= 0.052:
                # Calculate actual forward movement since entering FINAL_APPROACH
                curr_pose = self.get_robot_pose_in_map()
                fwd_travel = 0.0
                if curr_pose is not None and self.final_approach_start_pose is not None:
                    fwd_travel = math.hypot(curr_pose[0] - self.final_approach_start_pose[0], curr_pose[1] - self.final_approach_start_pose[1])

                dist_reduced = 0.0
                if self.final_approach_start_dist is not None:
                    dist_reduced = self.final_approach_start_dist - self.desk_distance

                # Require either physical contact, actual forward progress (>=3.0cm), or confirmed approach
                if self.contact_detected or fwd_travel >= 0.030 or dist_reduced >= 0.030:
                    self.get_logger().info("========================================================")
                    self.get_logger().info(f" ★ EXACT 5CM CLEARANCE REACHED (Clearance: {self.desk_distance*100:.1f}cm, Travel: {fwd_travel*100:.1f}cm)! ")
                    self.get_logger().info(" ★ Single Bumper Flush with Desk Front!                ")
                    self.get_logger().info(" ★ DOCKED STATE ACHIEVED! Full Motor Lock Engaged.     ")
                    self.get_logger().info("========================================================")
                    self.stop_robot()
                    self.transition_to('DOCKED')
                    return
                else:
                    self.get_logger().info(
                        f">>> [FINAL_APPROACH] Optical dist={self.desk_distance*100:.1f}cm but bumper CLEAR & fwd_travel={fwd_travel*100:.1f}cm < 3cm. "
                        f"Continuing crawl for physical bumper contact...",
                        throttle_duration_sec=1.0
                    )

            # 3. Tertiary Trigger: Forward Progress Stall
            if self.desk_distance is not None and self.desk_distance <= 0.075:
                if self._stall_check_dist is None:
                    self._stall_check_dist = self.desk_distance
                    self._stall_start_time = now
                else:
                    dist_diff = abs(self.desk_distance - self._stall_check_dist)
                    if dist_diff < 0.003:
                        stall_duration = (now - self._stall_start_time).nanoseconds / 1e9
                        if stall_duration > 1.0:
                            self.get_logger().info("========================================================")
                            self.get_logger().info(f" ★ PHYSICAL CONTACT CONFIRMED BY STALL (Clearance: {self.desk_distance*100:.1f}cm)! ")
                            self.get_logger().info(" ★ Robot firmly docked against desk surface.            ")
                            self.get_logger().info(" ★ DOCKED STATE ACHIEVED! Full Motor Lock Engaged.     ")
                            self.get_logger().info("========================================================")
                            self.stop_robot()
                            self.transition_to('DOCKED')
                            return
                    else:
                        self._stall_check_dist = self.desk_distance
                        self._stall_start_time = now
            else:
                self._stall_check_dist = None
                self._stall_start_time = None

            # Straight crawl (2.0 cm/s), heading strictly locked
            twist.linear.x = 0.020
            twist.angular.z = 0.0

        # ----------------------------------------------------------------------
        # STATE: DOCKED
        # ----------------------------------------------------------------------
        elif self.state == 'DOCKED':
            # Complete Zero-Velocity Lock
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # ----------------------------------------------------------------------
        # STATE: UNDOCKING (Safely backup 15cm from desk before returning to Nav2)
        # ----------------------------------------------------------------------
        elif self.state == 'UNDOCKING':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed < 3.0:
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(">>> Safely backed up 15cm from desk. Switching camera mode to NAV2...")
                self.transition_to('NAV2_READY')
                return

        # ----------------------------------------------------------------------
        # STATE: ABORT_TO_NAV2 (Safely retreat 24cm from desk, restore Nav2)
        # ----------------------------------------------------------------------
        elif self.state == 'ABORT_TO_NAV2':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed < 4.0:
                twist.linear.x = -0.06  # Safe reverse at 6 cm/s (total 24cm)
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(">>> Abort retreat complete. Switching camera mode to NAV2...")
                self.transition_to('NAV2_READY')
                return

        # ----------------------------------------------------------------------
        # STATE: NAV2_READY (Idle state allowing user Nav2 2D goal navigation)
        # ----------------------------------------------------------------------
        elif self.state == 'NAV2_READY':
            # Precision approacher yields full control of /cmd_vel to Nav2
            return

        # ----------------------------------------------------------------------
        # STATE: FAILSAFE
        # ----------------------------------------------------------------------
        elif self.state == 'FAILSAFE':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        if self.state not in ('WAIT_NAV2', 'NAV2_READY'):
            self.cmd_vel_pub.publish(twist)

    # ==========================================================================
    # Helper Functions & Guard Conditions
    # ==========================================================================
    def reset_docking_variables(self):
        """Resets all transient docking variables for a fresh approach sequence."""
        self.desk_distance = None
        self.desk_yaw_error = None
        self.last_depth_time = None
        self.contact_detected = False
        self.contact_duration_start = None
        self.align_stable_count = 0
        self.dual_align_stable_count = 0
        self.retry_count = 0
        self.nav2_finished = False
        self.nav2_success = False
        self.docking_routine_start_time = None
        self.target_obj = TargetObject()
        self.target_u_c = None
        self.target_v_c = None
        self.target_bbox = None
        self.carrot_x_b = None
        self.carrot_y_b = None
        self._stall_check_dist = None
        self._stall_start_time = None
        self.target_lateral_offset = 0.0
        self.turn_start_yaw = 0.0
        self.target_turn_angle = 0.0
        self.lateral_start_pose = None
        self.lateral_traveled = 0.0
        self.stable_yaw_count = 0
        self.in_place_turn_dir = 1.0

    def stop_robot(self):
        """Immediately commands zero velocity to prevent robot inertia drift."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(3):
            self.cmd_vel_pub.publish(twist)

    def check_contact_trigger(self) -> bool:
        if self.contact_detected:
            self.get_logger().info("========================================================")
            self.get_logger().info(" ★ SINGLE BUMPER CONTACT CONFIRMED! (50ms debounced)   ")
            self.get_logger().info(" ★ Exactly 5cm clearance established at Desk Front!     ")
            self.get_logger().info(" ★ DOCKED STATE ACHIEVED! Full Motor Lock Engaged.     ")
            self.get_logger().info("========================================================")
            self.stop_robot()
            self.transition_to('DOCKED')
            return True
        return False

    def check_depth_timeout(self) -> bool:
        if self.last_depth_time is not None:
            age = (self.get_clock().now() - self.last_depth_time).nanoseconds / 1e9
            if age > self.depth_timeout_sec:
                self.get_logger().warn(f"Depth stream interrupted for {age:.2f}s. Pausing movement.", throttle_duration_sec=2.0)
                return True
        return False

    def transition_to(self, new_state: str):
        self.get_logger().info(f"[FSM] Transition: {self.state} -> {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now()

        # Start docking overall stopwatch when starting docking
        if new_state == 'CHECK_STAGING' and self.docking_routine_start_time is None:
            self.docking_routine_start_time = self.get_clock().now()
        elif new_state in ('NAV2_READY', 'FAILSAFE', 'DOCKED', 'ABORT_TO_NAV2'):
            self.docking_routine_start_time = None

        if new_state in ('FINAL_APPROACH', 'CRAWL_CONTACT'):
            self.final_approach_start_pose = self.get_robot_pose_in_map()
            self.final_approach_start_dist = self.desk_distance
            self._stall_check_dist = None
            self._stall_start_time = None

        # Stop robot when entering idle, docked, or fail states
        if new_state in ('NAV2_READY', 'FAILSAFE', 'DOCKED'):
            self.stop_robot()

        # Time-division sensor management:
        # Keep OS30A active during all docking phases including safe abort/undock retreat!
        # Only switch to Nav2 when retreat has fully concluded and robot enters NAV2_READY.
        if new_state in DOCKING_ACTIVE_STATES or new_state in ('ABORT_TO_NAV2', 'UNDOCKING'):
            self.call_set_mode_docking()
            self.pause_rtabmap()
        elif new_state in ('FAILSAFE', 'WAIT_NAV2', 'NAV2_READY'):
            self.call_set_mode_nav2()
            self.resume_rtabmap()
            self.reset_docking_variables()
            self.publish_standby_hud()

    def call_set_mode_docking(self):
        self._internal_mode_request = 'DOCKING_OS30A'
        self._mode_request_time = time.time()
        if not self.set_mode_docking_cli.service_is_ready():
            self.set_mode_docking_cli.wait_for_service(timeout_sec=0.5)
        if self.set_mode_docking_cli.service_is_ready():
            self.get_logger().info("[CAMERA_MODE] Requesting DOCKING mode (OS30A ON, Astra OFF)...")
            self.set_mode_docking_cli.call_async(Trigger.Request())
        else:
            self.get_logger().warn("[CAMERA_MODE] /set_mode_docking service unavailable!")

    def call_set_mode_nav2(self):
        self._internal_mode_request = 'NAV2_ASTRA'
        self._mode_request_time = time.time()
        if not self.set_mode_nav2_cli.service_is_ready():
            self.set_mode_nav2_cli.wait_for_service(timeout_sec=0.5)
        if self.set_mode_nav2_cli.service_is_ready():
            self.get_logger().info("[CAMERA_MODE] Requesting NAV2 mode (Astra ON, OS30A OFF)...")
            self.set_mode_nav2_cli.call_async(Trigger.Request())
        else:
            self.get_logger().warn("[CAMERA_MODE] /set_mode_nav2 service unavailable!")

    def handle_start_docking(self, request, response):
        self.get_logger().info(">>> [/start_docking] Triggering Precision Close-Docking Routine!")
        self.reset_docking_variables()
        self.stop_robot()

    def handle_start_docking(self, request, response):
        self.get_logger().info(">>> [/start_docking] Triggering Precision Close-Docking Routine!")
        self.reset_docking_variables()
        self.stop_robot()
        self.transition_to('CHECK_STAGING')
        response.success = True
        response.message = "Precision Close-Docking Started (OS30A active, Astra S paused)."
        return response

    def handle_undock_to_nav2(self, request, response):
        self.get_logger().info(">>> [/undock_to_nav2] Received command to undock and return to Nav2!")
        self.stop_robot()
        if self.state == 'DOCKED':
            self.transition_to('UNDOCKING')
            response.success = True
            response.message = "Initiated safe undocking backup (15cm) and switching to Nav2 mode."
        else:
            self.transition_to('NAV2_READY')
            response.success = True
            response.message = "Switched to NAV2_READY mode (Astra S Active, OS30A Inactive)."
        return response

    def handle_abort_docking(self, request, response):
        self.get_logger().warn(">>> [/abort_docking] Abort requested by user/GUI. Retreating and returning to Nav2 mode...")
        self.stop_robot()
        self.transition_to('ABORT_TO_NAV2')
        response.success = True
        response.message = "Docking aborted. Retreating 24cm and returning to Nav2 mode."
        return response

    def camera_mode_callback(self, msg: String):
        current_cam = msg.data.strip()
        self.get_logger().info(f"[CAMERA_MODE] Current active sensor: {current_cam}", throttle_duration_sec=5.0)

    def pause_rtabmap(self):
        if self.rtabmap_pause_client.service_is_ready():
            self.get_logger().info("[RESOURCE] Pausing RTAB-Map SLAM to free CPU for precision docking...")
            req = Empty.Request()
            self.rtabmap_pause_client.call_async(req)

    def resume_rtabmap(self):
        if self.rtabmap_resume_client.service_is_ready():
            self.get_logger().info("[RESOURCE] Resuming RTAB-Map SLAM...")
            req = Empty.Request()
            self.rtabmap_resume_client.call_async(req)

    def get_robot_pose_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # Compute yaw from quaternion
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception:
            return None

    def send_nav2_staging_goal(self):
        self.get_logger().info(">>> Waiting for Nav2 NavigateToPose action server...")
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 Action Server not available. Proceeding directly to Staging Check.")
            self.transition_to('CHECK_STAGING')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.staging_x
        goal_msg.pose.pose.position.y = self.staging_y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation: staging_yaw = 0.0 -> quaternion (0, 0, 0, 1)
        goal_msg.pose.pose.orientation.w = math.cos(self.staging_yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(self.staging_yaw / 2.0)

        self.get_logger().info(f">>> Sending Nav2 Goal: ({self.staging_x:.2f}, {self.staging_y:.2f})")
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(">>> Nav2 Staging Goal was REJECTED.")
            self.nav2_finished = True
            self.nav2_success = False
            return

        self.get_logger().info(">>> Nav2 Staging Goal ACCEPTED. Tracking execution...")
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        result = future.result()
        self.nav2_finished = True
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_success = True
        else:
            self.nav2_success = False


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionApproacherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        node.get_logger().error(f"Fatal unhandled exception in precision_approacher: {e}\n{traceback.format_exc()}")
    finally:
        # Zero velocity fail-safe shutdown
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        stop_twist = Twist()
        try:
            node.cmd_vel_pub.publish(stop_twist)
            node.resume_rtabmap()
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
