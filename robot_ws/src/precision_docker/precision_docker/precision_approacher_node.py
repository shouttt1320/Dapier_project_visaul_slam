#!/usr/bin/env python3
# ==============================================================================
# Precision Autonomous Docking Node for TurtleBot3
# Supports:
#   1. Visual Marker Docking ('marker'):
#      - Front-facing Astra S RGB camera + ArUco marker pose estimation (solvePnP)
#      - Carrot Pursuit along marker normal axis (simultaneous heading + lateral convergence)
#      - In-place 90° lateral step maneuver for close proximity / large offset
#      - Micro-switch contact bumper + motor stall terminal stop (5cm physical assurance)
#      - Automatic RTAB-Map SLAM pause/resume management (frees Pi 4 CPU & stops map drift)
#   2. 3D Depth Edge Docking ('depth_edge'):
#      - OS30A 3D Depth Visual Servoing & RANSAC edge extraction (legacy/simulation mode)
# ==============================================================================

import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger, SetBool
from visualization_msgs.msg import Marker

try:
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    NAV2_ACTION_AVAILABLE = True
except ImportError:
    NavigateToPose = None
    GoalStatus = None
    NAV2_ACTION_AVAILABLE = False

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
    last_seen_time: float = 0.0


DOCKING_ACTIVE_STATES = (
    'SEARCH_MARKER', 'CHECK_STAGING', 'STAGING_SETTLE', 'STAGING_AIM',
    'CARROT_ALIGN', 'STANDOFF_SETTLE', 'CRAWL_CONTACT', 'BACKUP_STANDOFF', 'BACKUP_RETRY'
)


class PrecisionApproacherNode(Node):
    def __init__(self):
        super().__init__('precision_approacher_node')

        # ----------------------------------------------------------------------
        # Parameters: General & Navigation
        # ----------------------------------------------------------------------
        self.declare_parameter('docking_mode', 'marker')    # 'marker' (Astra S ArUco)
        self.declare_parameter('staging_x', 1.10)
        self.declare_parameter('staging_y', 0.40)
        self.declare_parameter('staging_yaw', 0.0)
        self.declare_parameter('target_clearance', 0.05)   # 50mm (5cm) target clearance
        self.declare_parameter('reference_x', 0.100)       # Astra S / front bumper tip offset from base_footprint (m)
        self.declare_parameter('max_linear_speed', 0.04)   # 4cm/s max
        self.declare_parameter('crawl_linear_speed', 0.025) # 2.5cm/s crawl
        self.declare_parameter('max_angular_speed', 0.22)  # 0.22 rad/s (~12.6 deg/s)
        self.declare_parameter('skip_nav2_if_close', True)
        self.declare_parameter('show_window', False)        # Local GUI window
        self.declare_parameter('docking_timeout_sec', 40.0) # Global timeout before aborting
        self.declare_parameter('autostart', False)
        self.declare_parameter('calibration_mode', False)  # Passive mode: zero motor cmds, active perception
        self.declare_parameter('publish_debug_img', True)
        self.declare_parameter('debug_img_stride', 3)      # Publish every N frames over Wi-Fi
        self.declare_parameter('bumper_topic', '/robot/bumper/contact')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('min_crawl_speed', 0.040)

        # ----------------------------------------------------------------------
        # Parameters: ArUco Marker Docking (Astra S Front Camera)
        # ----------------------------------------------------------------------
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('enable_stamped_cmd_vel', True)
        self.declare_parameter('marker_size', 0.060)       # 60mm (0.06m)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('marker_yaw_bias_deg', 0.0) # Physical camera mount yaw bias in degrees
        self.declare_parameter('cam_info_topic', '/camera/color/camera_info')
        self.declare_parameter('carrot_lookahead', 0.22)   # 22cm nominal lookahead
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('backup_retry_distance', 0.25) # 25cm reverse upon retry

        # Astra S Extrinsic relative to base_footprint (horizontal forward Z=0.134m, X=0.08m)
        self.declare_parameter('astra_x_base', 0.080)
        self.declare_parameter('astra_y_base', 0.015)   # Lateral mounting offset along base Y-axis (+1.5cm)
        self.declare_parameter('astra_z_base', 0.134)
        self.declare_parameter('astra_pitch', 0.0)

        # Retrieve parameters
        self.docking_mode = str(self.get_parameter('docking_mode').value).lower()
        self.staging_x = float(self.get_parameter('staging_x').value)
        self.staging_y = float(self.get_parameter('staging_y').value)
        self.staging_yaw = float(self.get_parameter('staging_yaw').value)
        self.target_clearance = float(self.get_parameter('target_clearance').value)
        self.reference_x = float(self.get_parameter('reference_x').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.crawl_linear_speed = float(self.get_parameter('crawl_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.skip_nav2_if_close = bool(self.get_parameter('skip_nav2_if_close').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.docking_timeout_sec = 75.0  # Generous timeout to allow complete docking routine
        self.autostart = bool(self.get_parameter('autostart').value)
        self.calibration_mode = bool(self.get_parameter('calibration_mode').value)
        self.publish_debug_img = bool(self.get_parameter('publish_debug_img').value)
        self.debug_img_stride = int(self.get_parameter('debug_img_stride').value)
        self.bumper_topic = str(self.get_parameter('bumper_topic').value)

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.enable_stamped_cmd_vel = bool(self.get_parameter('enable_stamped_cmd_vel').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.marker_yaw_bias_rad = math.radians(float(self.get_parameter('marker_yaw_bias_deg').value))
        self.cam_info_topic = str(self.get_parameter('cam_info_topic').value)
        self.carrot_lookahead = float(self.get_parameter('carrot_lookahead').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.backup_retry_distance = float(self.get_parameter('backup_retry_distance').value)

        self.astra_x_base = float(self.get_parameter('astra_x_base').value)
        self.astra_y_base = float(self.get_parameter('astra_y_base').value)
        self.astra_z_base = float(self.get_parameter('astra_z_base').value)
        self.astra_pitch = float(self.get_parameter('astra_pitch').value)

        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.min_crawl_speed = float(self.get_parameter('min_crawl_speed').value)

        self._debug_img_counter = 0
        self.docking_routine_start_time = None

        # ----------------------------------------------------------------------
        # Camera Intrinsics
        # ----------------------------------------------------------------------
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False

        # ----------------------------------------------------------------------
        # ArUco Configuration (OpenCV 4.6.0 Compatible)
        # ----------------------------------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.04           # Reject small noise specks
        self.aruco_params.maxErroneousBitsInBorderRate = 0.25      # Strict black border required (no false positives!)
        self.aruco_params.errorCorrectionRate = 0.60               # Prevent random textures from hallucinating IDs
        self.aruco_params.polygonalApproxAccuracyRate = 0.03       # Strict quad shape validation
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.05
        # 3D Marker Corners in marker coordinate frame: Z=0, centered
        s = self.marker_size
        self.marker_3d_corners = np.array([
            [-s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0, -s / 2.0, 0.0],
            [-s / 2.0, -s / 2.0, 0.0]
        ], dtype=np.float32)

        # Marker tracking state
        self.marker_detected = False
        self.marker_last_seen = 0.0
        self.marker_timeout_sec = 1.2
        self.marker_pos_base = None    # [x, y, z] in base_footprint (m)
        self.marker_normal_base = None # [dx, dy, dz] pointing into desk in base_footprint
        self.marker_corners_img = None
        self.marker_rvec = None
        self.marker_tvec = None

        # Dual-Marker Geometric Pose & Anti-Chatter Filter State
        self.tracked_markers = {}       # mid -> {'corners': ndarray, 'rvec': rvec, 'tvec': tvec, 'last_seen': float}
        self.marker_history_sec = 0.80  # Persistent memory window for brief occlusion / packet drops
        self.docking_alignment_mode = 'NONE' # 'DUAL:XXcm' or 'SINGLE:ID' or 'NONE'
        self.yaw_history = []           # Rolling history for median filter
        self.yaw_history_len = 5
        self.filtered_yaw = None

        # Centerline (Lateral Error e_y) Anti-Chatter & Outlier Rejection Filter State
        self.ey_history = []            # Rolling history for e_y median filter
        self.ey_history_len = 5
        self.filtered_ey = None
        self.marker_offsets_to_center = {}  # Learned 3D offset from marker to box center: mid -> np.ndarray([dx, dy, dz])

        # Slew-Rate Acceleration Limiter & Motor Anti-Chatter
        self.current_cmd_vx = 0.0
        self.current_cmd_wz = 0.0
        self._last_cmd_vel_time = time.time()
        self.max_wz_accel = 0.45        # rad/s^2 (smooth steering, prevents Dynamixel backlash)
        self.max_vx_accel = 0.20        # m/s^2 (smooth forward acceleration)
        self.align_yaw_deadband = math.radians(1.5)  # ±1.5° deadband in ALIGN_PARALLEL

        # ----------------------------------------------------------------------
        # RTAB-Map Pause/Resume Clients (Freeze SLAM during docking to save CPU)
        # ----------------------------------------------------------------------
        self.rtabmap_pause_client = self.create_client(Empty, '/rtabmap/pause')
        self.rtabmap_resume_client = self.create_client(Empty, '/rtabmap/resume')

        # Camera Mode Switch Clients & Services
        self.set_mode_docking_cli = self.create_client(Trigger, '/set_mode_docking')
        self.set_mode_nav2_cli = self.create_client(Trigger, '/set_mode_nav2')
        self.undock_srv = self.create_service(Trigger, '/undock_to_nav2', self.handle_undock_to_nav2)
        self.dock_srv = self.create_service(Trigger, '/start_docking', self.handle_start_docking)
        self.abort_srv = self.create_service(Trigger, '/abort_docking', self.handle_abort_docking)
        self.estop_srv = self.create_service(Trigger, '/emergency_stop', self.handle_emergency_stop)
        self.motor_power_cli = self.create_client(SetBool, '/motor_power')
        self._motor_init_timer = self.create_timer(1.5, self._init_motor_power_callback)

        # ----------------------------------------------------------------------
        # FSM State & Variables
        # ----------------------------------------------------------------------
        if self.calibration_mode:
            self.state = 'CALIBRATION'
            self.get_logger().info(">>> Initialized in CALIBRATION mode. Motors halted, passive perception active.")
        elif self.autostart:
            self.state = 'INIT'
        else:
            self.state = 'NAV2_READY'
            self.get_logger().info(">>> Initialized in NAV2_READY mode. Waiting for Nav2 navigation or [/start_docking] trigger.")

        self.state_start_time = self.get_clock().now()

        # Contact Bumper State (Micro-switch)
        self.contact_detected = False
        self.contact_latched = False
        self.contact_duration_start = None
        self.contact_debounce_sec = 0.05  # 50ms debounce

        # Forward Stall Detection (Secondary Stop Guard)
        self._stall_check_dist = None
        self._stall_start_time = None

        # Desk & Target Distance/Angles
        self.desk_distance = None      # Distance from reference tip to target/desk (m)
        self.desk_yaw_error = None     # Heading error relative to normal (rad, positive = tilted left)
        self.locked_docking_yaw = None # Locked heading angle for blind crawl approach
        self.last_sensor_time = None
        self.target_obj = TargetObject()
        self.carrot_x_b = None
        self.carrot_y_b = None

        # Odom-Anchored Visual Memory & Blind Pursuit
        self.odom_target_center = None        # [x, y, z] in odom frame
        self.odom_target_normal = None        # Unit vector pointing outward from box in odom frame
        self.odom_target_latched = False      # True when dual markers validly latched
        self.odom_target_last_seen = 0.0      # Timestamp of last direct optical update
        self.odom_blind_start_time = None     # Timestamp when optical view was lost
        self.odom_tracking_active = False     # True when navigating blindly using odom memory
        self.dual_markers_visible = False     # True only when both markers are currently in camera FOV
        self.initial_ey = None                # Initial lateral error recorded at start of CARROT_ALIGN
        self.backup_reason = "NONE"           # Reason string explaining why robot entered BACKUP state
        self.backup_substate = 'BACK_AIM'     # Substate within BACKUP_STANDOFF: 'BACK_AIM' -> 'BACK_REVERSE' -> 'BACK_REALIGN'
        self.reverse_latched_origin_yaw = None  # Reference yaw before backup (facing marker normal axis)
        self.reverse_latched_aim_yaw = None     # Target yaw angled towards reverse virtual carrot
        self.reverse_start_odom_pose = None     # (x, y, yaw) in odom at start of BACK_REVERSE
        self.reverse_target_distance = 0.14     # Target straight reverse distance in meters
        self.settle_substate = 'FINE_ROTATE'  # Substate within STANDOFF_SETTLE: 'FINE_ROTATE' -> 'STATIC_WAIT'
        self.settle_stop_start_time = None    # Timestamp when robot completely halted in STANDOFF_SETTLE
        self.latched_aim_delta_yaw = 0.0      # Relative angle (rad) to forward carrot point latched in STAGING_SETTLE
        self.latched_aim_target_yaw = None   # Absolute target yaw in odom/map latched in STAGING_SETTLE

        # Docking Convergence & Verification
        self.target_lateral_offset = 0.0
        self.dual_align_stable_count = 0
        self.dual_align_stable_required = 3

        # Retry & Failure Recovery
        self.retry_count = 0
        self.bridge = CvBridge()

        # ----------------------------------------------------------------------
        # TF Buffer & Listener
        # ----------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------------------
        if self.enable_stamped_cmd_vel:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/docking/status', 10)
        self.marker_pub = self.create_publisher(Marker, '/docking/marker', 10)
        self.vis_pub = self.create_publisher(Image, '/docking/debug_image', 10)
        self.vis_comp_pub = self.create_publisher(CompressedImage, '/docking/debug_image/compressed', 10)

        self.last_color_img = None
        self.last_color_stamp = None
        self.last_color_frame_id = 'camera_color_optical_frame'
        self.last_depth_img = None
        self.last_depth_raw = None
        self.last_depth_is_raw16 = True
        self.last_depth_time = 0.0
        self.plane_fit_source = 'NONE'
        self.plane_fit_points = 0
        self.plane_roi_rect = None

        # Color & CameraInfo Subscribers (Reliable QoS depth 10 matching Astra driver)
        self.color_sub = self.create_subscription(
            Image, self.color_topic, self.color_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.cam_info_topic, self.cam_info_callback, 10
        )

        # Depth Subscriber (Used if docking_mode == 'depth_edge')
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )

        # Bumper Switch Subscriber
        self.bumper_sub = self.create_subscription(
            Bool, self.bumper_topic, self.bumper_callback, 10
        )

        # Nav2 NavigateToPose Action Client
        if NAV2_ACTION_AVAILABLE:
            self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self.nav2_client = None
        self.nav2_goal_handle = None
        self.nav2_finished = False
        self.nav2_success = False

        # Main Control Loop Timer (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("========================================================")
        self.get_logger().info(" Precision Autonomous Docking Node Initialized          ")
        self.get_logger().info(f" - Docking Mode:   {self.docking_mode.upper()}")
        if self.docking_mode == 'marker':
            self.get_logger().info(f" - Marker ID:      {self.marker_id} ({self.marker_size*1000:.0f}mm ArUco DICT_4X4_50)")
            self.get_logger().info(f" - Color Stream:   {self.color_topic}")
            self.get_logger().info(f" - Info Stream:    {self.cam_info_topic}")
        else:
            self.get_logger().info(f" - Depth Stream:   {self.depth_topic}")
        self.get_logger().info(f" - Target Gap:     {self.target_clearance*100:.1f} cm from Reference Tip")
        self.get_logger().info(f" - Terminal Stop:  Micro-switch Bumper ({self.bumper_topic}) + Stall")
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
            self.camera_matrix = np.array([
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            if len(msg.d) >= 4:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.camera_info_received = True

    def bumper_callback(self, msg: Bool):
        if msg.data:
            self.contact_detected = True
            self.contact_latched = True
            # Global Emergency Contact Interlock: halt immediately if in any docking approach
            if self.state in DOCKING_ACTIVE_STATES:
                self.get_logger().info("========================================================")
                self.get_logger().info(" ★ INSTANT BUMPER CONTACT CONFIRMED! Full Motor Lock!   ")
                self.get_logger().info(" ★ Robot docked firmly against desk. Entering DOCKED.   ")
                self.get_logger().info("========================================================")
                self.stop_robot(hard_brake=True)
                self.transition_to('DOCKED')
        else:
            self.contact_detected = False

    def color_callback(self, msg: Image):
        # Throttle to max ~15 FPS to eliminate Pi 4 CPU bottleneck and live stream delay
        now_time = time.time()
        if hasattr(self, '_last_img_time') and (now_time - self._last_img_time) < 0.065:
            return
        self._last_img_time = now_time

        try:
            self.last_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_color_stamp = msg.header.stamp
            self.last_color_frame_id = msg.header.frame_id or 'camera_color_optical_frame'
            self.last_sensor_time = self.get_clock().now()

            # If in marker docking mode, process ArUco detection immediately on new frame
            if self.docking_mode == 'marker':
                self.process_marker_detection(self.last_color_img, self.last_color_frame_id)
        except Exception as e:
            self.get_logger().warn(f"Color callback error: {e}", throttle_duration_sec=2.0)

    def depth_callback(self, msg: Image):
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
                self.last_depth_raw = depth_raw
                self.last_depth_img = None
                self.last_depth_is_raw16 = True
            elif msg.encoding == '32FC1':
                depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                self.last_depth_raw = depth_m
                self.last_depth_img = depth_m
                self.last_depth_is_raw16 = False
            else:
                return

            self.last_depth_time = time.time()
        except Exception as e:
            self.get_logger().warn(f"Depth callback error: {e}", throttle_duration_sec=2.0)

    def query_marker_center_depth(self, marker_corners):
        """Ultra-lightweight pinpoint sampling of depth at the marker center.
        Queries a 5x5 pixel window around the 2D center and returns the median depth in meters.
        Execution time < 0.01ms (CPU ~0%).
        """
        if self.last_depth_raw is None or (time.time() - self.last_depth_time) > 1.2:
            return None

        H, W = self.last_depth_raw.shape[:2]
        u_c = int(round(float(np.mean(marker_corners[:, 0]))))
        v_c = int(round(float(np.mean(marker_corners[:, 1]))))

        u_min = max(0, u_c - 2)
        u_max = min(W, u_c + 3)
        v_min = max(0, v_c - 2)
        v_max = min(H, v_c + 3)

        if (u_max - u_min) < 3 or (v_max - v_min) < 3:
            return None

        patch = self.last_depth_raw[v_min:v_max, u_min:u_max]
        if self.last_depth_is_raw16:
            valid = patch[(patch > 200) & (patch < 2500)]  # 200mm ~ 2500mm
            if len(valid) >= 5:
                return float(np.median(valid)) / 1000.0
        else:
            valid = patch[(patch > 0.20) & (patch < 2.50) & np.isfinite(patch)]
            if len(valid) >= 5:
                return float(np.median(valid))
        return None

    def fit_depth_plane(self, marker_corners, z_marker, n_pnp):
        """
        Fits a 3D plane using SVD on dense depth points on the box surface
        surrounding the ArUco marker.
        Returns:
            n_cam: np.ndarray (3,) unit normal vector in camera optical frame, or None if failed
            point_count: int, number of valid inlier 3D points used
        """
        if (time.time() - self.last_depth_time) > 1.5:
            self.plane_roi_rect = None
            return None, 0

        if self.last_depth_img is None:
            if self.last_depth_raw is not None and self.last_depth_is_raw16:
                self.last_depth_img = self.last_depth_raw.astype(np.float32) / 1000.0
            else:
                self.plane_roi_rect = None
                return None, 0

        H, W = self.last_depth_img.shape[:2]

        # Calculate bounding box of marker in pixels
        u_coords = marker_corners[:, 0]
        v_coords = marker_corners[:, 1]
        u_c = float(np.mean(u_coords))
        v_c = float(np.mean(v_coords))
        w_m = float(np.max(u_coords) - np.min(u_coords))
        h_m = float(np.max(v_coords) - np.min(v_coords))

        # Expand ROI to cover box face surrounding the marker (box is ~30cm wide)
        roi_half_w = max(int(w_m * 1.8), 35)
        roi_half_h = max(int(h_m * 1.2), 25)

        u_min = max(0, int(u_c - roi_half_w))
        u_max = min(W, int(u_c + roi_half_w))
        v_min = max(0, int(v_c - roi_half_h))
        v_max = min(H, int(v_c + roi_half_h))

        if (u_max - u_min) < 15 or (v_max - v_min) < 15:
            self.plane_roi_rect = None
            return None, 0

        self.plane_roi_rect = (u_min, v_min, u_max, v_max)

        # Extract depth patch
        depth_patch = self.last_depth_img[v_min:v_max, u_min:u_max]

        # Create coordinate grids for patch
        u_grid, v_grid = np.meshgrid(
            np.arange(u_min, u_max, dtype=np.float32),
            np.arange(v_min, v_max, dtype=np.float32)
        )

        # Depth gate: points on box surface must be close to marker depth z_marker (+/- 9cm)
        valid_mask = (depth_patch > 0.15) & (depth_patch < 2.50) & (np.abs(depth_patch - z_marker) < 0.09) & np.isfinite(depth_patch)

        if np.count_nonzero(valid_mask) < 35:
            return None, 0

        valid_z = depth_patch[valid_mask]
        valid_u = u_grid[valid_mask]
        valid_v = v_grid[valid_mask]

        # Downsample if too dense for ultra-fast SVD
        if len(valid_z) > 600:
            valid_z = valid_z[::2]
            valid_u = valid_u[::2]
            valid_v = valid_v[::2]

        # Project pixels to 3D camera optical coordinates
        X = (valid_u - self.cx) * valid_z / self.fx
        Y = (valid_v - self.cy) * valid_z / self.fy
        Z = valid_z
        pts = np.column_stack((X, Y, Z))

        try:
            # Stage 1: Initial SVD plane fit
            centroid1 = np.mean(pts, axis=0)
            _, _, vh1 = np.linalg.svd(pts - centroid1, full_matrices=False)
            n_init = vh1[-1, :]
            norm_len1 = np.linalg.norm(n_init)
            if norm_len1 < 1e-6:
                return None, 0
            n_init /= norm_len1

            # Stage 2: Outlier rejection (residual < 2.0 cm)
            residuals = np.abs(np.dot(pts - centroid1, n_init))
            inlier_mask = residuals < 0.020
            pts_inliers = pts[inlier_mask]

            if len(pts_inliers) >= 30:
                centroid2 = np.mean(pts_inliers, axis=0)
                _, _, vh2 = np.linalg.svd(pts_inliers - centroid2, full_matrices=False)
                normal = vh2[-1, :]
                norm_len2 = np.linalg.norm(normal)
                if norm_len2 < 1e-6:
                    return None, 0
                normal /= norm_len2
                n_used = len(pts_inliers)
            else:
                normal = n_init
                n_used = len(pts)

            # Enforce consistent orientation pointing OUT towards the camera
            if np.dot(normal, n_pnp) < 0:
                normal = -normal
            if normal[2] > 0:
                normal = -normal

            return normal, n_used
        except Exception:
            return None, 0

    # ==========================================================================
    # Visual Marker Perception & Pose Estimation (Dual-Marker & Single Fallback)
    # ==========================================================================
    def process_marker_detection(self, color_img: np.ndarray, frame_id: str):
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # 1. Detect ArUco markers using universal OpenCV 4.x API
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        now_t = time.time()
        detected = False

        if ids is not None and len(ids) > 0:
            for idx in range(len(ids)):
                m_id = int(ids.flatten()[idx])
                m_corners = corners[idx][0]  # Shape: (4, 2)

                # SolvePnP for each detected marker
                succ, rvec, tvec = cv2.solvePnP(
                    self.marker_3d_corners,
                    m_corners,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                if succ:
                    self.tracked_markers[m_id] = {
                        'corners': m_corners,
                        'rvec': rvec,
                        'tvec': tvec,
                        'last_seen': now_t
                    }

        # Prune expired markers from temporal memory
        expired_ids = [mid for mid, data in self.tracked_markers.items() if (now_t - data['last_seen']) > self.marker_history_sec]
        for mid in expired_ids:
            del self.tracked_markers[mid]

        active_ids = list(self.tracked_markers.keys())

        # 2. Geometric Pose Estimation: Dual-Marker vs Single Fallback
        if len(active_ids) >= 2:
            # Sort active markers left to right along camera X-axis
            active_ids.sort(key=lambda mid: self.tracked_markers[mid]['tvec'][0][0])
            left_id = active_ids[0]
            right_id = active_ids[-1]

            p_left = self.tracked_markers[left_id]['tvec'].flatten()
            p_right = self.tracked_markers[right_id]['tvec'].flatten()

            span = float(np.linalg.norm(p_right - p_left))

            # Validate physical span (Accommodates perspective distance variation: 6cm ~ 35cm)
            if 0.06 <= span <= 0.35:
                # Optical midpoint distance approximation
                opt_dist = float((p_left[2] + p_right[2]) / 2.0)
                curr_dist = self.desk_distance if self.desk_distance is not None else opt_dist

                use_depth_hybrid = False
                # Hybrid Switching: Use lightweight pinpoint Depth sampling when distance > 45cm (0.45m)
                # to extinguish optical PnP normal distortion at long range
                if curr_dist > 0.45 or opt_dist > 0.45:
                    z_depth_left = self.query_marker_center_depth(self.tracked_markers[left_id]['corners'])
                    z_depth_right = self.query_marker_center_depth(self.tracked_markers[right_id]['corners'])

                    if (z_depth_left is not None and z_depth_right is not None and
                            0.30 <= z_depth_left <= 1.50 and 0.30 <= z_depth_right <= 1.50 and
                            abs(z_depth_right - z_depth_left) < 0.20):
                        # Reconstruct high-precision 3D positions using physical depth sensor values
                        u_L = float(np.mean(self.tracked_markers[left_id]['corners'][:, 0]))
                        v_L = float(np.mean(self.tracked_markers[left_id]['corners'][:, 1]))
                        u_R = float(np.mean(self.tracked_markers[right_id]['corners'][:, 0]))
                        v_R = float(np.mean(self.tracked_markers[right_id]['corners'][:, 1]))

                        x_L = (u_L - self.cx) * z_depth_left / self.fx
                        y_L = (v_L - self.cy) * z_depth_left / self.fy
                        x_R = (u_R - self.cx) * z_depth_right / self.fx
                        y_R = (v_R - self.cy) * z_depth_right / self.fy

                        p_left_d = np.array([x_L, y_L, z_depth_left], dtype=np.float64)
                        p_right_d = np.array([x_R, y_R, z_depth_right], dtype=np.float64)

                        t_cam_mid = (p_left_d + p_right_d) / 2.0
                        dx = float(p_right_d[0] - p_left_d[0])
                        dy = float(p_right_d[1] - p_left_d[1])
                        dz = float(p_right_d[2] - p_left_d[2])
                        use_depth_hybrid = True
                        mode_str = f"DUAL:DEPTH({span*100:.0f}cm)"
                        source_str = "DUAL:DEP"

                if not use_depth_hybrid:
                    # Within 45cm or depth fallback: Use high-precision Close-Range RGB Optical solvePnP
                    t_cam_mid = (p_left + p_right) / 2.0
                    dx = float(p_right[0] - p_left[0])
                    dy = float(p_right[1] - p_left[1])
                    dz = float(p_right[2] - p_left[2])
                    mode_str = f"DUAL:RGB({span*100:.0f}cm)"
                    source_str = "DUAL:RGB"

                # Learn relative 3D vector from each marker to box center to prevent jump if one drops
                self.marker_offsets_to_center[left_id] = (t_cam_mid - p_left).copy()
                self.marker_offsets_to_center[right_id] = (t_cam_mid - p_right).copy()

                # Normal vector pointing from box face outward toward camera (-Z direction)
                norm_len = math.hypot(dz, dx)
                if norm_len > 1e-4:
                    n_cam = np.array([dz / norm_len, 0.0, -dx / norm_len], dtype=np.float64)
                else:
                    n_cam = np.array([0.0, 0.0, -1.0], dtype=np.float64)

                p_base, n_base = self.transform_optical_to_base(t_cam_mid, n_cam, frame_id)

                if p_base is not None and n_base is not None:
                    desk_normal = -n_base
                    raw_heading_err = math.atan2(desk_normal[1], desk_normal[0]) - self.marker_yaw_bias_rad

                    self.docking_alignment_mode = mode_str
                    self.plane_fit_source = source_str
                    self.plane_fit_points = 2
                    self.marker_corners_img = self.tracked_markers[left_id]['corners']
                    self.marker_rvec = self.tracked_markers[left_id]['rvec']
                    self.marker_tvec = t_cam_mid.reshape(3, 1)

                    detected = True
                    self.dual_markers_visible = True

                    # Transform & Latch Target Pose into ODOM frame ONLY when DUAL markers are recognized
                    try:
                        t_odom = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                        trans = t_odom.transform.translation
                        rot = t_odom.transform.rotation
                        R_odom = self.quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
                        t_vec = np.array([trans.x, trans.y, trans.z], dtype=np.float64)

                        p_odom = R_odom @ p_base + t_vec
                        n_odom = R_odom @ desk_normal
                        n_len = np.linalg.norm(n_odom)
                        if n_len > 1e-6:
                            n_odom /= n_len

                        if self.odom_target_center is None:
                            self.odom_target_center = p_odom
                            self.odom_target_normal = n_odom
                        else:
                            self.odom_target_center = 0.80 * self.odom_target_center + 0.20 * p_odom
                            self.odom_target_normal = 0.80 * self.odom_target_normal + 0.20 * n_odom
                            norm_l = np.linalg.norm(self.odom_target_normal)
                            if norm_l > 1e-6:
                                self.odom_target_normal /= norm_l

                        self.odom_target_latched = True
                        self.odom_target_last_seen = time.time()
                        self.odom_blind_start_time = None
                    except Exception:
                        pass

        if not detected and len(active_ids) >= 1:
            self.dual_markers_visible = False
            # Single-marker fallback mode
            mid = self.marker_id if self.marker_id in active_ids else active_ids[0]
            m_corners = self.tracked_markers[mid]['corners']
            rvec = self.tracked_markers[mid]['rvec']
            tvec = self.tracked_markers[mid]['tvec']
            t_cam = tvec.flatten()

            # Seamless box center compensation: apply learned offset from dual tracking to prevent lateral jump
            offset_to_center = self.marker_offsets_to_center.get(mid, np.zeros(3, dtype=np.float64))
            t_cam_box = t_cam + offset_to_center

            R_cam, _ = cv2.Rodrigues(rvec)
            n_local = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            n_cam_pnp = R_cam @ n_local

            # Depth plane fitting fusion if available
            n_cam_depth, n_pts = self.fit_depth_plane(m_corners, t_cam[2], n_cam_pnp)
            if n_cam_depth is not None:
                n_cam = n_cam_depth
                self.plane_fit_source = f"DEP:{n_pts}"
                self.plane_fit_points = n_pts
            else:
                n_cam = n_cam_pnp
                self.plane_fit_source = "PNP"
                self.plane_fit_points = 0

            p_base, n_base = self.transform_optical_to_base(t_cam_box, n_cam, frame_id)

            if p_base is not None and n_base is not None:
                desk_normal = -n_base
                raw_heading_err = math.atan2(desk_normal[1], desk_normal[0]) - self.marker_yaw_bias_rad

                self.docking_alignment_mode = f"SINGLE:ID{mid}"
                self.marker_corners_img = m_corners
                self.marker_rvec = rvec
                self.marker_tvec = tvec

                detected = True

        # 3. Filtering Pipeline (Median + Adaptive EMA) & Target State Update
        if detected:
            # Stage 1: 5-frame rolling median (spike & glitch rejection)
            self.yaw_history.append(raw_heading_err)
            if len(self.yaw_history) > self.yaw_history_len:
                self.yaw_history.pop(0)
            median_yaw = float(np.median(self.yaw_history))

            # Stage 2: Adaptive low-pass EMA filter (smooth jitter down to <= 0.2 deg)
            if self.filtered_yaw is None:
                self.filtered_yaw = median_yaw
            else:
                step_diff = math.atan2(math.sin(median_yaw - self.filtered_yaw), math.cos(median_yaw - self.filtered_yaw))
                if self.docking_alignment_mode.startswith("DUAL"):
                    alpha = 0.50 if abs(step_diff) > math.radians(4.0) else 0.20
                    self.filtered_yaw = float(self.filtered_yaw + alpha * step_diff)
                elif self.plane_fit_source.startswith("DEP"):
                    alpha = 0.60 if abs(step_diff) > math.radians(8.0) else 0.35
                    self.filtered_yaw = float(self.filtered_yaw + alpha * step_diff)
                else:
                    if abs(step_diff) > math.radians(6.0):
                        step_diff = math.copysign(math.radians(2.5), step_diff)
                    self.filtered_yaw = float(self.filtered_yaw + 0.25 * step_diff)

            self.desk_yaw_error = self.filtered_yaw

            dist_m = float(p_base[0]) - self.reference_x
            self.desk_distance = dist_m
            self.marker_pos_base = p_base
            self.marker_normal_base = desk_normal
            self.marker_detected = True
            self.marker_last_seen = time.time()

            self.target_obj.center = p_base
            self.target_obj.docking_point = p_base
            self.target_obj.valid = True
            self.target_obj.last_seen_time = time.time()

            self.publish_aruco_rviz_marker(p_base, desk_normal)

        else:
            self.dual_markers_visible = False
            if time.time() - self.marker_last_seen > self.marker_timeout_sec:
                self.marker_detected = False
                self.marker_corners_img = None
                self.marker_rvec = None
                self.marker_tvec = None
                if not self.odom_target_latched:
                    self.desk_distance = None
                    self.desk_yaw_error = None
                    self.filtered_yaw = None
                    self.yaw_history.clear()
                    self.filtered_ey = None
                    self.ey_history.clear()
                    self.marker_offsets_to_center.clear()
                    self.marker_pos_base = None
                    self.marker_normal_base = None
                    self.target_obj.valid = False
                    self.docking_alignment_mode = 'NONE'
                else:
                    self.docking_alignment_mode = 'ODOM_TRACK'

        # Render and publish visual HUD overlay
        self.render_and_publish_visualization(color_img, self.marker_corners_img, self.marker_rvec, self.marker_tvec)

    def transform_optical_to_base(self, t_cam, n_cam, frame_id):
        """Transforms 3D position and normal vector from camera optical frame to base_footprint frame."""
        # Method A: Use active TF lookup if available
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', frame_id, rclpy.time.Time()
            )
            trans = t.transform.translation
            rot = t.transform.rotation
            q = [rot.x, rot.y, rot.z, rot.w]

            # Quaternion to rotation matrix
            R_tf = self.quaternion_to_matrix(q)
            t_tf = np.array([trans.x, trans.y, trans.z], dtype=np.float64)

            p_base = R_tf @ t_cam + t_tf
            p_base[1] += self.astra_y_base
            n_base = R_tf @ n_cam
            return p_base, n_base
        except Exception:
            pass

        # Method B: Fallback Analytical Extrinsic (Astra S mounted at Z=0.134m, X=0.08m, horizontal)
        # Camera optical frame: X: Right, Y: Down, Z: Forward
        # Robot base_footprint: X: Forward, Y: Left, Z: Up
        sin_p = math.sin(self.astra_pitch)
        cos_p = math.cos(self.astra_pitch)

        x_b = self.astra_x_base + cos_p * t_cam[2] - sin_p * t_cam[1]
        y_b = self.astra_y_base - t_cam[0]
        z_b = self.astra_z_base - sin_p * t_cam[2] - cos_p * t_cam[1]
        p_base = np.array([x_b, y_b, z_b], dtype=np.float64)

        nx_b = cos_p * n_cam[2] - sin_p * n_cam[1]
        ny_b = -n_cam[0]
        nz_b = -sin_p * n_cam[2] - cos_p * n_cam[1]
        n_base = np.array([nx_b, ny_b, nz_b], dtype=np.float64)
        norm_len = np.linalg.norm(n_base)
        if norm_len > 1e-6:
            n_base /= norm_len

        return p_base, n_base

    @staticmethod
    def quaternion_to_matrix(q):
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x**2 + y**2)]
        ], dtype=np.float64)

    def publish_aruco_rviz_marker(self, p_base=None, desk_normal=None):
        if p_base is not None and desk_normal is not None:
            marker = Marker()
            marker.header.frame_id = 'base_footprint'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'aruco_docking'
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.015  # Shaft diameter
            marker.scale.y = 0.030  # Head diameter
            marker.scale.z = 0.040  # Head length
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.2
            marker.color.a = 1.0

            start_pt = Point(x=float(p_base[0]), y=float(p_base[1]), z=float(p_base[2]))
            end_pt = Point(
                x=float(p_base[0] + 0.15 * desk_normal[0]),
                y=float(p_base[1] + 0.15 * desk_normal[1]),
                z=float(p_base[2] + 0.15 * desk_normal[2])
            )
            marker.points = [start_pt, end_pt]
            self.marker_pub.publish(marker)

        # Also publish latched target in odom frame if active (Cyan Arrow)
        if self.odom_target_latched and self.odom_target_center is not None:
            m_odom = Marker()
            m_odom.header.frame_id = 'odom'
            m_odom.header.stamp = self.get_clock().now().to_msg()
            m_odom.ns = 'odom_visual_memory'
            m_odom.id = 1
            m_odom.type = Marker.ARROW
            m_odom.action = Marker.ADD
            m_odom.scale.x = 0.015
            m_odom.scale.y = 0.030
            m_odom.scale.z = 0.040
            m_odom.color.r = 0.1
            m_odom.color.g = 0.8
            m_odom.color.b = 1.0  # Cyan for odom visual memory
            m_odom.color.a = 0.9

            p_o = self.odom_target_center
            n_o = self.odom_target_normal
            s_pt = Point(x=float(p_o[0]), y=float(p_o[1]), z=float(p_o[2]))
            e_pt = Point(
                x=float(p_o[0] + 0.15 * n_o[0]),
                y=float(p_o[1] + 0.15 * n_o[1]),
                z=float(p_o[2] + 0.15 * n_o[2])
            )
            m_odom.points = [s_pt, e_pt]
            self.marker_pub.publish(m_odom)

        # Publish Virtual Carrot Point in base_footprint (Orange Sphere)
        if self.carrot_x_b is not None and self.carrot_y_b is not None:
            m_carrot = Marker()
            m_carrot.header.frame_id = 'base_footprint'
            m_carrot.header.stamp = self.get_clock().now().to_msg()
            m_carrot.ns = 'carrot_waypoint'
            m_carrot.id = 2
            m_carrot.type = Marker.SPHERE
            m_carrot.action = Marker.ADD
            m_carrot.pose.position.x = float(self.carrot_x_b)
            m_carrot.pose.position.y = float(self.carrot_y_b)
            m_carrot.pose.position.z = 0.0
            m_carrot.pose.orientation.w = 1.0
            m_carrot.scale.x = 0.05
            m_carrot.scale.y = 0.05
            m_carrot.scale.z = 0.05
            m_carrot.color.r = 1.0
            m_carrot.color.g = 0.55
            m_carrot.color.b = 0.0
            m_carrot.color.a = 0.95
            self.marker_pub.publish(m_carrot)
        else:
            m_del = Marker()
            m_del.header.frame_id = 'base_footprint'
            m_del.ns = 'carrot_waypoint'
            m_del.id = 2
            m_del.action = Marker.DELETE
            self.marker_pub.publish(m_del)

    def get_carrot_point_in_base(self, L_carrot=0.28):
        """Calculates 3D coordinates of the Virtual Carrot Waypoint in base_footprint frame.
        Carrot point is located on the docking corridor at distance L_carrot in front of the desk:
        P_carrot = P_box - L_carrot * desk_normal
        """
        if self.marker_pos_base is not None and self.marker_normal_base is not None:
            c_x = float(self.marker_pos_base[0]) - L_carrot * float(self.marker_normal_base[0])
            c_y = float(self.marker_pos_base[1]) - L_carrot * float(self.marker_normal_base[1])
            c_z = float(self.marker_pos_base[2]) - L_carrot * float(self.marker_normal_base[2])
            return np.array([c_x, c_y, c_z], dtype=np.float64)
        return None

    # ==========================================================================
    # Unified Error Computation & Control Loop
    # ==========================================================================
    def compute_docking_errors(self):
        """Returns [e_x, e_y, e_theta] relative to Target Docking Pose with consistent normal and 2-stage filtering."""
        # 1. Direct Optical Perception
        if self.marker_detected and self.desk_distance is not None:
            self.odom_tracking_active = False
            self.odom_blind_start_time = None

            e_x = self.desk_distance - self.target_clearance
            e_theta = self.desk_yaw_error if self.desk_yaw_error is not None else 0.0

            if self.target_obj.valid and self.target_obj.center is not None:
                # Fixed-Frame (Odom) Centerline Error: measures true perpendicular distance to box corridor
                # (Invariant under robot in-place rotation, completely eliminating false divergence during turns!)
                used_fixed_ey = False
                if self.odom_target_latched and self.odom_target_center is not None and self.odom_target_normal is not None:
                    try:
                        t_b_o = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                        rx = t_b_o.transform.translation.x
                        ry = t_b_o.transform.translation.y
                        bx = float(self.odom_target_center[0])
                        by = float(self.odom_target_center[1])
                        # n_odom points into desk; outward corridor is -n_odom
                        ux = -float(self.odom_target_normal[0])
                        uy = -float(self.odom_target_normal[1])
                        u_len = math.hypot(ux, uy)
                        if u_len > 1e-6:
                            ux /= u_len
                            uy /= u_len
                        # Perpendicular distance: vector (box - robot) cross corridor unit vector
                        dx = bx - rx
                        dy = by - ry
                        raw_e_y = float(dx * uy - dy * ux)
                        used_fixed_ey = True
                    except Exception:
                        pass

                if not used_fixed_ey:
                    phi = e_theta + self.marker_yaw_bias_rad
                    ux = -math.cos(phi)
                    uy = -math.sin(phi)
                    px = float(self.target_obj.center[0])
                    py = float(self.target_obj.center[1])
                    raw_e_y = float(px * uy - py * ux)

                # 2-Stage Anti-Chatter & Outlier Filter on Centerline Error (e_y)
                self.ey_history.append(raw_e_y)
                if len(self.ey_history) > self.ey_history_len:
                    self.ey_history.pop(0)
                median_ey = float(np.median(self.ey_history))

                if self.filtered_ey is None:
                    self.filtered_ey = median_ey
                else:
                    alpha_ey = 0.50 if abs(median_ey - self.filtered_ey) > 0.03 else 0.25
                    self.filtered_ey = float(self.filtered_ey + alpha_ey * (median_ey - self.filtered_ey))

                e_y = self.filtered_ey
            else:
                e_y = 0.0

            return e_x, e_y, e_theta

        # 2. Odom-Anchored Visual Memory Tracking (Blind Pursuit when markers swing out of FOV)
        elif self.odom_target_latched and self.odom_target_center is not None:
            try:
                t = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time())
                trans = t.transform.translation
                rot = t.transform.rotation
                R_b_o = self.quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
                t_b_o = np.array([trans.x, trans.y, trans.z], dtype=np.float64)

                p_base = R_b_o @ self.odom_target_center + t_b_o
                # desk_normal was saved pointing into desk in odom frame
                desk_normal = R_b_o @ self.odom_target_normal
                norm_l = np.linalg.norm(desk_normal)
                if norm_l > 1e-6:
                    desk_normal /= norm_l

                dist_m = float(p_base[0]) - self.reference_x
                self.desk_distance = dist_m
                self.marker_pos_base = p_base
                self.marker_normal_base = desk_normal
                self.target_obj.center = p_base
                self.target_obj.docking_point = p_base
                self.target_obj.valid = True

                self.desk_yaw_error = math.atan2(desk_normal[1], desk_normal[0]) - self.marker_yaw_bias_rad
                self.odom_tracking_active = True
                if self.odom_blind_start_time is None:
                    self.odom_blind_start_time = time.time()

                e_x = self.desk_distance - self.target_clearance
                e_theta = self.desk_yaw_error

                used_fixed_ey = False
                try:
                    t_o_b = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                    rx = t_o_b.transform.translation.x
                    ry = t_o_b.transform.translation.y
                    bx = float(self.odom_target_center[0])
                    by = float(self.odom_target_center[1])
                    ux = -float(self.odom_target_normal[0])
                    uy = -float(self.odom_target_normal[1])
                    u_len = math.hypot(ux, uy)
                    if u_len > 1e-6:
                        ux /= u_len
                        uy /= u_len
                    dx = bx - rx
                    dy = by - ry
                    raw_e_y = float(dx * uy - dy * ux)
                    used_fixed_ey = True
                except Exception:
                    pass

                if not used_fixed_ey:
                    phi = e_theta + self.marker_yaw_bias_rad
                    ux = -math.cos(phi)
                    uy = -math.sin(phi)
                    px = float(p_base[0])
                    py = float(p_base[1])
                    raw_e_y = float(px * uy - py * ux)

                self.ey_history.append(raw_e_y)
                if len(self.ey_history) > self.ey_history_len:
                    self.ey_history.pop(0)
                median_ey = float(np.median(self.ey_history))

                if self.filtered_ey is None:
                    self.filtered_ey = median_ey
                else:
                    alpha_ey = 0.50 if abs(median_ey - self.filtered_ey) > 0.03 else 0.25
                    self.filtered_ey = float(self.filtered_ey + alpha_ey * (median_ey - self.filtered_ey))

                e_y = self.filtered_ey
                return e_x, e_y, e_theta
            except Exception as e:
                self.get_logger().warn(f"Odom memory TF lookup failed: {e}", throttle_duration_sec=2.0)
                return None, None, None

        return None, None, None

    def control_loop(self):
        try:
            self._control_loop_impl()
        except Exception as e:
            self.get_logger().error(f"Error in control_loop: {e}", throttle_duration_sec=1.0)

    def _control_loop_impl(self):
        now = self.get_clock().now()
        twist = Twist()

        e_x, e_y, e_theta = self.compute_docking_errors()

        # Publish status string
        ey_str = f"{e_y*100:+.1f}" if e_y is not None else "0.0"
        yaw_str = f"{math.degrees(e_theta):+.1f}" if e_theta is not None else "0.0"
        dist_str = f"{self.desk_distance:.3f}" if self.desk_distance is not None else "None"
        sensor_type = "DEPTH" if "DEPTH" in self.docking_alignment_mode else ("RGB" if "RGB" in self.docking_alignment_mode else "NONE")
        status_msg = String()
        status_msg.data = (
            f"STATE={self.state}, dist={dist_str}, yaw_err={yaw_str}deg, "
            f"ey={ey_str}cm, dual={1 if self.dual_markers_visible else 0}, "
            f"sensor={sensor_type}, mode={self.docking_alignment_mode}, "
            f"latched={1 if self.odom_target_latched else 0}, odom_track={1 if self.odom_tracking_active else 0}, "
            f"target={self.target_obj.valid}, contact={self.contact_detected}, "
            f"retries={self.retry_count}/{self.max_retries}, reason={self.backup_reason}"
        )
        self.status_pub.publish(status_msg)

        # If no camera frames have arrived yet, publish diagnostic screen to /docking/debug_image
        if self.last_color_img is None:
            self.render_and_publish_diagnostic_screen()

        # Static calibration mode: keep motors halted
        if self.calibration_mode:
            self.send_cmd_vel(0.0, 0.0)
            return

        # Fully docked or bumper contact latched: enforce immediate full motor lock
        if self.state == 'DOCKED' or self.contact_latched:
            self.stop_robot(hard_brake=False)
            return

        # ----------------------------------------------------------------------
        # Global Docking Timeout Guard: Abort, reverse, and return to Nav2
        # ----------------------------------------------------------------------
        if self.state in DOCKING_ACTIVE_STATES:
            if self.docking_routine_start_time is not None:
                total_docking_elapsed = (now - self.docking_routine_start_time).nanoseconds / 1e9
                if total_docking_elapsed > self.docking_timeout_sec:
                    self.get_logger().warn(
                        f">>> [DOCKING TIMEOUT] Exceeded {self.docking_timeout_sec:.1f}s without docking. "
                        f"Aborting and retreating to Nav2 mode..."
                    )
                    self.transition_to('ABORT_TO_NAV2')
                    return

        # ======================================================================
        # FSM State Handlers
        # ======================================================================
        # ----------------------------------------------------------------------
        # STATE: INIT
        # ----------------------------------------------------------------------
        if self.state == 'INIT':
            if self.docking_mode == 'marker':
                if self.marker_detected:
                    self.get_logger().info(f">>> [INIT] Marker ID {self.marker_id} acquired ({self.desk_distance*100:.1f}cm). Proceeding to CHECK_STAGING.")
                    self.transition_to('CHECK_STAGING')
                    return
                else:
                    self.transition_to('SEARCH_MARKER')
                    return
            else:
                curr_pose = self.get_robot_pose_in_map()
                if curr_pose is None:
                    return
                if self.desk_distance is not None and 0.20 <= self.desk_distance <= 0.55:
                    self.transition_to('CHECK_STAGING')
                    return
                dist_to_staging = math.hypot(curr_pose[0] - self.staging_x, curr_pose[1] - self.staging_y)
                if self.skip_nav2_if_close and dist_to_staging <= 0.50:
                    self.transition_to('CHECK_STAGING')
                else:
                    self.send_nav2_staging_goal()
                    self.transition_to('WAIT_NAV2')

        # ----------------------------------------------------------------------
        # STATE: WAIT_NAV2
        # ----------------------------------------------------------------------
        elif self.state == 'WAIT_NAV2':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if self.nav2_finished:
                self.transition_to('CHECK_STAGING')
                return
            if self.desk_distance is not None and 0.20 <= self.desk_distance <= 0.65:
                if self.nav2_goal_handle is not None:
                    try:
                        self.nav2_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self.transition_to('CHECK_STAGING')
                return
            if elapsed > 25.0:
                self.transition_to('CHECK_STAGING')
                return

        # ----------------------------------------------------------------------
        # STATE: SEARCH_MARKER
        # ----------------------------------------------------------------------
        elif self.state == 'SEARCH_MARKER':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if self.marker_detected and self.desk_distance is not None:
                self.get_logger().info(f">>> [SEARCH_MARKER] Marker found at d={self.desk_distance*100:.1f}cm! Transitioning to CHECK_STAGING.")
                self.stop_robot()
                self.transition_to('CHECK_STAGING')
                return

            # Active visual search sweep (±30° rotation to bring marker into camera FOV)
            if elapsed < 0.8:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif elapsed < 4.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.25  # Sweep left
                self.get_logger().info(">>> [SEARCH_MARKER] Scanning left for ArUco marker...", throttle_duration_sec=1.5)
            elif elapsed < 10.0:
                twist.linear.x = 0.0
                twist.angular.z = -0.25  # Sweep right
                self.get_logger().info(">>> [SEARCH_MARKER] Scanning right for ArUco marker...", throttle_duration_sec=1.5)
            elif elapsed < 13.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.25  # Return to center
            else:
                self.get_logger().warn(">>> [SEARCH_MARKER] Marker not found after 13s scan. Aborting to Nav2 mode...")
                self.transition_to('ABORT_TO_NAV2')
                return

        # ----------------------------------------------------------------------
        # STATE: CHECK_STAGING (Assess clearance and route to STAGING_SETTLE)
        # ----------------------------------------------------------------------
        elif self.state == 'CHECK_STAGING':
            if self.desk_distance is not None:
                e_x, e_y, e_theta = self.compute_docking_errors()
                e_y_val = abs(e_y) if e_y is not None else 0.0
                e_theta_val = abs(e_theta) if e_theta is not None else 0.0

                # 1. Clearance Check: If too close (< 35cm) where marker starts clipping from FOV, back up to 52cm
                if self.desk_distance < 0.35 and (e_y_val > 0.025 or e_theta_val > math.radians(3.0)):
                    self.backup_reason = f"초기 위치 너무 가까움 ({self.desk_distance*100:.0f}cm < 35cm, 편차={e_y_val*100:.1f}cm)"
                    self.get_logger().info(
                        f">>> [CHECK_STAGING] Distance too close ({self.desk_distance*100:.1f}cm < 35cm) for camera FOV. "
                        f"Backing up to 52cm optimal long-range staging zone..."
                    )
                    self.transition_to('BACKUP_STANDOFF')
                    return

                # Route to STAGING_SETTLE to halt all motors and statically evaluate errors before aiming
                self.transition_to('STAGING_SETTLE')
                return

            else:
                # Searching for target
                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                if elapsed > 3.0:
                    self.transition_to('SEARCH_MARKER' if self.docking_mode == 'marker' else 'ABORT_TO_NAV2')
                    return

        # ----------------------------------------------------------------------
        # STATE: STAGING_SETTLE (Stationary settling >= 1.2s to extinguish vibration and collect stable errors)
        # ----------------------------------------------------------------------
        elif self.state == 'STAGING_SETTLE':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed < 1.2:
                self.get_logger().info(
                    f">>> [STAGING_SETTLE] Stationary wait for chassis/camera vibration settling ({elapsed:.1f}s / 1.2s)...",
                    throttle_duration_sec=0.3
                )
                return

            e_x, e_y, e_theta = self.compute_docking_errors()
            if e_y is not None and e_x is not None:
                # --------------------------------------------------------------
                # Calculate and Latch True Virtual Carrot Point Aiming Goal (1-Time)
                # --------------------------------------------------------------
                d_ref = max(0.10, e_x if e_x is not None else (self.desk_distance if self.desk_distance is not None else 0.45))
                L = float(np.clip(0.60 * d_ref, 0.18, 0.28))

                c_pt = self.get_carrot_point_in_base(L)
                if c_pt is not None:
                    c_x = float(c_pt[0])
                    c_y = float(c_pt[1])
                    dist_to_c = math.hypot(c_x, c_y)
                    self.latched_aim_delta_yaw = math.atan2(c_y, max(0.05, c_x))
                    self.carrot_x_b = c_x
                    self.carrot_y_b = c_y
                else:
                    dist_to_c = max(0.15, d_ref - L)
                    self.latched_aim_delta_yaw = math.atan2(e_y, dist_to_c)
                    self.carrot_x_b = dist_to_c
                    self.carrot_y_b = e_y

                curr_yaw = self.get_robot_yaw()
                if curr_yaw is not None:
                    self.latched_aim_target_yaw = math.atan2(
                        math.sin(curr_yaw + self.latched_aim_delta_yaw),
                        math.cos(curr_yaw + self.latched_aim_delta_yaw)
                    )
                else:
                    self.latched_aim_target_yaw = None

                self.get_logger().info(
                    f">>> [STAGING_SETTLE] Settling complete! Target Desk: dist={self.desk_distance*100:.1f}cm, ey={e_y*100:+.1f}cm, yaw={math.degrees(e_theta):+.1f}°. "
                    f"Virtual Carrot: dist={dist_to_c*100:.1f}cm, Aim Delta: {math.degrees(self.latched_aim_delta_yaw):+.1f}°. "
                    f"Transitioning to STAGING_AIM..."
                )
                self.transition_to('STAGING_AIM')
                return
            else:
                if elapsed > 3.5:
                    self.get_logger().warn(">>> [STAGING_SETTLE] Marker lost while settling. Transitioning to SEARCH_MARKER.")
                    self.transition_to('SEARCH_MARKER')
                    return

        # ----------------------------------------------------------------------
        # STATE: STAGING_AIM (Aim robot heading directly at CARROT POINT using IMU/Odom closed loop)
        # ----------------------------------------------------------------------
        elif self.state == 'STAGING_AIM':
            twist.linear.x = 0.0  # In-place rotation only: no forward translation while aiming!

            curr_yaw = self.get_robot_yaw()
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            # Determine remaining heading error to the LATCHED carrot point
            if self.latched_aim_target_yaw is not None and curr_yaw is not None:
                # IMU/Odom closed-loop: clean, zero-jitter relative angle
                aim_err = math.atan2(
                    math.sin(self.latched_aim_target_yaw - curr_yaw),
                    math.cos(self.latched_aim_target_yaw - curr_yaw)
                )
            else:
                # Fallback only if TF/Odom missing: compute live relative aim error to carrot
                e_x, e_y, e_theta = self.compute_docking_errors()
                if e_x is None or e_y is None:
                    twist.angular.z = 0.0
                    if elapsed > 2.5:
                        self.transition_to('SEARCH_MARKER')
                    return
                d_ref = max(0.10, e_x if e_x is not None else 0.45)
                L = float(np.clip(0.60 * d_ref, 0.18, 0.28))
                c_pt = self.get_carrot_point_in_base(L)
                if c_pt is not None:
                    aim_err = math.atan2(float(c_pt[1]), max(0.05, float(c_pt[0])))
                    self.carrot_x_b = float(c_pt[0])
                    self.carrot_y_b = float(c_pt[1])
                else:
                    aim_err = math.atan2(e_y, max(0.15, d_ref - L))
                    self.carrot_x_b = max(0.15, d_ref - L)
                    self.carrot_y_b = e_y

            # Aiming Tolerance Condition: Heading aligned to Carrot Point within 2.0 deg
            if abs(aim_err) <= math.radians(2.0) or elapsed > 4.5:
                self.stop_robot()
                self.get_logger().info(
                    f">>> [STAGING_AIM] Carrot point aiming completed! (aim_err={math.degrees(aim_err):+.1f}° <= 2.0°). "
                    f"Initiating smooth CARROT_ALIGN approach!"
                )
                self.transition_to('CARROT_ALIGN')
                return
            else:
                w_z = math.copysign(min(0.18, max(0.05, 0.70 * abs(aim_err))), aim_err)
                twist.angular.z = float(w_z)
                self.get_logger().info(
                    f">>> [STAGING_AIM] In-place aiming at latched carrot: aim_err={math.degrees(aim_err):+.1f}°, wz={w_z:+.2f}rad/s ({elapsed:.1f}s/4.5s)",
                    throttle_duration_sec=0.3
                )

        # ----------------------------------------------------------------------
        # STATE: BACKUP_STANDOFF (Virtual Reverse Carrot: BACK_AIM -> BACK_REVERSE -> BACK_REALIGN)
        # Closed-loop IMU/Wheel Odom ONLY; zero vision polling to eliminate sensor noise during maneuvers.
        # ----------------------------------------------------------------------
        elif self.state in ('BACKUP_STANDOFF', 'BACKUP_PREPARE', 'BACKUP_ALIGN_PREPARE'):
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            # Phase 1: In-place pivot towards Reverse Virtual Carrot Point (Pure IMU/Odom Closed-Loop)
            if self.backup_substate == 'BACK_AIM':
                twist.linear.x = 0.0
                curr_yaw = self.get_robot_yaw()
                if self.reverse_latched_aim_yaw is not None and curr_yaw is not None:
                    yaw_err = math.atan2(
                        math.sin(self.reverse_latched_aim_yaw - curr_yaw),
                        math.cos(self.reverse_latched_aim_yaw - curr_yaw)
                    )
                else:
                    yaw_err = 0.0

                if abs(yaw_err) <= math.radians(2.0) or elapsed > 2.5:
                    self.stop_robot()
                    odom_p = self.get_robot_pose_in_odom()
                    self.reverse_start_odom_pose = odom_p if odom_p is not None else (0.0, 0.0, 0.0)
                    self.backup_substate = 'BACK_REVERSE'
                    self.state_start_time = now
                    self.get_logger().info(
                        f">>> [BACKUP_STANDOFF: BACK_AIM] Pivot complete (aim_err={math.degrees(yaw_err):+.1f}°). "
                        f"Beginning straight reverse of {self.reverse_target_distance*100:.1f}cm..."
                    )
                    return
                else:
                    w_z = math.copysign(min(0.20, max(0.06, 0.70 * abs(yaw_err))), yaw_err)
                    twist.angular.z = float(w_z)
                    self.get_logger().info(
                        f">>> [BACKUP_STANDOFF: BACK_AIM] Pivoting to reverse carrot: yaw_err={math.degrees(yaw_err):+.1f}°, wz={w_z:+.2f}rad/s ({elapsed:.1f}s/2.5s)",
                        throttle_duration_sec=0.4
                    )

            # Phase 2: Straight reverse along aimed vector using wheel odometry (No vision polling)
            elif self.backup_substate == 'BACK_REVERSE':
                twist.linear.x = -0.065  # -6.5 cm/s straight reverse
                twist.angular.z = 0.0    # Pure straight trajectory

                curr_odom_p = self.get_robot_pose_in_odom()
                if curr_odom_p is not None and self.reverse_start_odom_pose is not None:
                    dist_traveled = math.hypot(
                        curr_odom_p[0] - self.reverse_start_odom_pose[0],
                        curr_odom_p[1] - self.reverse_start_odom_pose[1]
                    )
                else:
                    dist_traveled = 0.065 * elapsed  # Fallback dead reckoning

                # Complete reverse once target distance (~14cm) reached or safety timeout
                if dist_traveled >= self.reverse_target_distance or elapsed > 3.2:
                    self.stop_robot()
                    self.backup_substate = 'BACK_REALIGN'
                    self.state_start_time = now
                    self.get_logger().info(
                        f">>> [BACKUP_STANDOFF: BACK_REVERSE] Reverse complete (traveled={dist_traveled*100:.1f}cm). "
                        f"Commencing BACK_REALIGN to face marker normal..."
                    )
                    return
                else:
                    self.get_logger().info(
                        f">>> [BACKUP_STANDOFF: BACK_REVERSE] Reversing: traveled={dist_traveled*100:.1f}cm / {self.reverse_target_distance*100:.1f}cm ({elapsed:.1f}s)",
                        throttle_duration_sec=0.5
                    )

            # Phase 3: In-place pivot back to face marker normal axis (Pure IMU/Odom Closed-Loop)
            elif self.backup_substate == 'BACK_REALIGN':
                twist.linear.x = 0.0
                curr_yaw = self.get_robot_yaw()
                if self.reverse_latched_origin_yaw is not None and curr_yaw is not None:
                    realign_err = math.atan2(
                        math.sin(self.reverse_latched_origin_yaw - curr_yaw),
                        math.cos(self.reverse_latched_origin_yaw - curr_yaw)
                    )
                else:
                    realign_err = 0.0

                if abs(realign_err) <= math.radians(2.0) or elapsed > 2.5:
                    self.stop_robot()
                    self.get_logger().info(
                        f">>> [BACKUP_STANDOFF: BACK_REALIGN] Realign complete (err={math.degrees(realign_err):+.1f}°). "
                        f"Staging zone reached. Transitioning to STAGING_SETTLE for static evaluation..."
                    )
                    self.transition_to('STAGING_SETTLE')
                    return
                else:
                    w_z = math.copysign(min(0.20, max(0.06, 0.70 * abs(realign_err))), realign_err)
                    twist.angular.z = float(w_z)
                    self.get_logger().info(
                        f">>> [BACKUP_STANDOFF: BACK_REALIGN] Realigning to front normal: err={math.degrees(realign_err):+.1f}°, wz={w_z:+.2f}rad/s ({elapsed:.1f}s/2.5s)",
                        throttle_duration_sec=0.4
                    )

        # ======================================================================
        # STATE: CARROT_ALIGN (Continuous Adaptive Pure Pursuit & Heading Alignment)
        # ======================================================================
        elif self.state == 'CARROT_ALIGN':
            # Safety contact check: immediate stop if bumper clicks during approach
            if self.check_contact_trigger():
                return
            if self.check_sensor_timeout():
                return

            e_x, e_y, e_theta = self.compute_docking_errors()
            if e_x is not None and e_y is not None:
                # ------------------------------------------------------------------
                # Case 1 Failure Guard: Dynamic Centerline Deviation Divergence
                # - Allow 3.0s grace period for initial differential-drive pivot & transient response.
                # - Allow max(15cm, initial_ey + 6cm) buffer so natural S-curve pivot is not aborted.
                # ------------------------------------------------------------------
                if self.initial_ey is None:
                    self.initial_ey = abs(e_y)

                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                ey_diverge_threshold = max(0.180, self.initial_ey + 0.080)
                if elapsed > 4.0 and abs(e_y) > ey_diverge_threshold:
                    self.backup_reason = f"중심선 편차 발산 (현재:{e_y*100:+.1f}cm > 한계:{ey_diverge_threshold*100:.1f}cm)"
                    self.get_logger().warn(
                        f">>> [CASE 1] {self.backup_reason}. Aborting carrot alignment, retreating to standoff..."
                    )
                    self.stop_robot()
                    self.transition_to('BACKUP_STANDOFF')
                    return

                # ------------------------------------------------------------------
                # Case 2 Failure Guard: Blind Odom Pursuit Timeout (> 8.0s)
                # ------------------------------------------------------------------
                if self.odom_tracking_active and self.odom_blind_start_time is not None:
                    blind_elapsed = time.time() - self.odom_blind_start_time
                    if blind_elapsed > 8.0:
                        self.backup_reason = f"시각 메모리 블라인드 주행 시간 초과 ({blind_elapsed:.1f}s > 8.0s)"
                        self.get_logger().warn(
                            f">>> [CASE 2] {self.backup_reason}. Marker not re-acquired. Initiating SEARCH_MARKER visual sweep..."
                        )
                        self.stop_robot()
                        self.transition_to('SEARCH_MARKER')
                        return

                # ------------------------------------------------------------------
                # ------------------------------------------------------------------
                # 2. Standoff Zone Reached Gate (Elastic 33cm ~ 38cm):
                # If already well centered (|ey| <= 2.5cm and |e_theta| <= 4.0 deg) at <= 38cm, halt immediately!
                # If still converging (|ey| > 2.5cm), allow gentle crawling down to 33cm to finish S-curve!
                # ------------------------------------------------------------------
                # 2. Standoff Zone Reached Gate (<= 38cm):
                # Completely halt forward motion and enter STANDOFF_SETTLE for static evaluation & in-place facing
                # ------------------------------------------------------------------
                if self.desk_distance is not None and self.desk_distance <= 0.38:
                    self.get_logger().info(
                        f">>> [CARROT_ALIGN] Standoff zone reached ({self.desk_distance*100:.1f}cm <= 38cm). "
                        f"Halting forward motion and transitioning to STANDOFF_SETTLE for static evaluation..."
                    )
                    self.stop_robot()
                    self.dual_align_stable_count = 0
                    self.transition_to('STANDOFF_SETTLE')
                    return

                # 3. Adaptive Lookahead (True 3D Virtual Carrot Waypoint)
                d_ref = max(0.10, e_x if e_x is not None else (self.desk_distance if self.desk_distance is not None else 0.40))
                L = float(np.clip(0.60 * d_ref, 0.14, 0.28))

                c_pt = self.get_carrot_point_in_base(L)
                if c_pt is not None:
                    c_x = float(c_pt[0])
                    c_y = float(c_pt[1])
                    dist_carrot = math.hypot(c_x, c_y)
                    alpha_carrot = math.atan2(c_y, max(0.05, c_x))
                    self.carrot_x_b = c_x
                    self.carrot_y_b = c_y
                else:
                    dist_carrot = max(0.15, d_ref - L)
                    alpha_carrot = math.atan2(e_y, dist_carrot)
                    self.carrot_x_b = dist_carrot
                    self.carrot_y_b = e_y

                # 4. Decoupled Velocity & Steering Control (Pure Pursuit on Virtual Carrot)
                # A. Distance-to-Carrot Smoothstep Deceleration Profile (5.0 cm/s initial -> 2.5 cm/s near carrot)
                v_init = 0.050   # 5.0 cm/s initial cruise speed along straight corridor
                v_final = 0.025  # 2.5 cm/s gentle arrival speed as carrot point is reached
                ratio = float(np.clip((dist_carrot - 0.15) / (0.40 - 0.15), 0.0, 1.0))
                smooth_s = ratio * ratio * (3.0 - 2.0 * ratio)  # S-curve smoothstep weight
                v_x = float(v_final + (v_init - v_final) * smooth_s)

                # B. Pure Pursuit steering directly targeting the Virtual Carrot Waypoint
                omega_pp = (2.0 * v_x * math.sin(alpha_carrot)) / max(0.15, dist_carrot)
                w_z = float(np.clip(omega_pp, -0.22, 0.22))

                twist.linear.x = v_x
                twist.angular.z = w_z
                self.get_logger().info(
                    f">>> [CARROT_ALIGN] Tracking CARROT | Carrot: d={dist_carrot*100:.1f}cm, aim={math.degrees(alpha_carrot):+.1f}° | "
                    f"Target Desk: d={self.desk_distance*100:.1f}cm, ey={e_y*100:+.1f}cm, yaw={math.degrees(e_theta):+.1f}°, "
                    f"vx={v_x*100:.1f}cm/s, wz={w_z:+.2f}rad/s",
                    throttle_duration_sec=0.8
                )
            else:
                # Marker momentarily lost while in CARROT_ALIGN:
                # If already close (<= 33cm) and was well centered, commence CRAWL_CONTACT
                if self.desk_distance is not None and self.desk_distance <= 0.33 and abs(self.target_lateral_offset) <= 0.025:
                    self.get_logger().info(">>> [CARROT_ALIGN] Standoff reached (<=33cm) and centered. Locking heading and commencing CRAWL_CONTACT.")
                    self.locked_docking_yaw = self.get_robot_yaw()
                    self.transition_to('CRAWL_CONTACT')
                    return
                else:
                    elapsed = (now - self.state_start_time).nanoseconds / 1e9
                    if elapsed > 2.5:
                        self.get_logger().warn(">>> [CARROT_ALIGN] Target lost at standoff without odom latch. Transitioning to SEARCH_MARKER.")
                        self.transition_to('SEARCH_MARKER')
                        return

        # ----------------------------------------------------------------------
        # STATE: STANDOFF_SETTLE (Step 1: Fine Rotate to Marker Normal -> Step 2: Static Halt & Evaluate Errors)
        # ----------------------------------------------------------------------
        elif self.state == 'STANDOFF_SETTLE':
            if self.check_sensor_timeout() or self.check_contact_trigger():
                return

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            twist.linear.x = 0.0  # Zero forward velocity: completely halted

            e_x, e_y, e_theta = self.compute_docking_errors()
            if e_theta is None or e_y is None:
                twist.angular.z = 0.0
                if elapsed > 3.5:
                    self.backup_reason = "스탠드오프 정지 계측 중 마커 미감지"
                    self.get_logger().warn(">>> [STANDOFF_SETTLE] Marker lost while settling. Transitioning to SEARCH_MARKER.")
                    self.transition_to('SEARCH_MARKER')
                return

            # Step 1: In-place fine rotation to bring camera directly facing marker normal (|e_theta| <= 3.5 deg)
            # BEFORE evaluating lateral error, we must align heading so camera lever-arm distortion is eliminated!
            if self.settle_substate == 'FINE_ROTATE':
                rotate_elapsed = (now - self.settle_rotate_start_time).nanoseconds / 1e9 if self.settle_rotate_start_time else elapsed
                if abs(e_theta) > math.radians(3.5) and rotate_elapsed < 4.5:
                    w_z = math.copysign(min(0.18, max(0.05, 0.60 * abs(e_theta))), e_theta)
                    twist.angular.z = float(w_z)
                    self.settle_stop_start_time = None
                    self.get_logger().info(
                        f">>> [STANDOFF_SETTLE] Fine rotating heading to marker normal: yaw_err={math.degrees(e_theta):+.1f}°, wz={w_z:+.2f}rad/s ({rotate_elapsed:.1f}s/4.5s)",
                        throttle_duration_sec=0.4
                    )
                else:
                    # Heading aligned within 3.5 deg (or 4.5s budget reached) -> Full Stop and begin STATIC_WAIT
                    twist.angular.z = 0.0
                    self.stop_robot()
                    self.settle_substate = 'STATIC_WAIT'
                    self.settle_stop_start_time = now
                    self.dual_align_stable_count = 0
                    self.get_logger().info(">>> [STANDOFF_SETTLE] Heading rotation finished. Halting all motors for 1.0s vibration settling & static evaluation...")
                    return

            # Step 2: Stationary pause (>=1.0s) + Extended Static Verification Window (up to 4.5s)
            elif self.settle_substate == 'STATIC_WAIT':
                twist.angular.z = 0.0
                twist.linear.x = 0.0

                if self.settle_stop_start_time is None:
                    self.settle_stop_start_time = now
                    return

                stopped_elapsed = (now - self.settle_stop_start_time).nanoseconds / 1e9

                # Mandatory stationary pause of at least 1.0 full second for camera/chassis settling
                if stopped_elapsed < 1.0:
                    self.get_logger().info(
                        f">>> [STANDOFF_SETTLE] Stationary wait for rotation settling ({stopped_elapsed:.1f}s / 1.0s)...",
                        throttle_duration_sec=0.3
                    )
                    return

                # Statically evaluate error after >= 1.0s complete standstill with robot facing marker
                aligned_ok = (abs(e_y) <= 0.028 and abs(e_theta) <= math.radians(3.5))
                if aligned_ok:
                    self.dual_align_stable_count += 1
                    if self.dual_align_stable_count >= 3:
                        self.locked_docking_yaw = self.get_robot_yaw()
                        self.get_logger().info(
                            f">>> [STANDOFF_SETTLE] Statically verified after {stopped_elapsed:.1f}s stationary evaluation! "
                            f"(ey={e_y*100:+.1f}cm <= 2.8cm, yaw={math.degrees(e_theta):+.1f}° <= 3.5°). "
                            f"Commencing terminal CRAWL_CONTACT."
                        )
                        self.stop_robot()
                        self.transition_to('CRAWL_CONTACT')
                        return
                else:
                    self.dual_align_stable_count = 0

                # Generous evaluation window: Only abort after 4.5s standstill without passing consistency
                if stopped_elapsed > 4.5:
                    self.backup_reason = f"정지 계측(4.5초간 계측) 오차 초과 (ey={e_y*100:+.1f}cm > 2.8cm, yaw={math.degrees(e_theta):+.1f}° > 3.5°)"
                    self.get_logger().warn(f">>> [STANDOFF_SETTLE] {self.backup_reason}. Backing up to standoff...")
                    self.stop_robot()
                    self.transition_to('BACKUP_STANDOFF')
                    return

        # ----------------------------------------------------------------------
        # STATE: CRAWL_CONTACT (Terminal crawl with IMU heading lock & micro-switch bumper)
        # ----------------------------------------------------------------------
        elif self.state == 'CRAWL_CONTACT':
            # 1. Exclusive Trigger: Physical Contact Micro-Switch Bumper
            if self.check_contact_trigger():
                return

            # 2. Case 4 Failure Guard: Timeout (12.0s) without physical bumper contact -> BACKUP_RETRY
            # (Note: From 38cm standoff to 12cm bumper touch = 26cm forward travel. At 4.5cm/s, travel takes ~5.8s. 12.0s provides safe margin)
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > 12.0:
                self.backup_reason = "최종 접근 12.0초 동안 범퍼 미접촉 (타임아웃)"
                self.get_logger().warn(">>> [CASE 4] Terminal crawl timeout (12.0s) without bumper contact. Initiating BACKUP_RETRY...")
                self.transition_to('BACKUP_RETRY')
                return

            # 4. Gyro Heading Lock Straight Crawl: drive forward holding locked_docking_yaw
            twist.linear.x = max(0.045, self.min_crawl_speed)
            curr_yaw = self.get_robot_yaw()
            if curr_yaw is not None and self.locked_docking_yaw is not None:
                yaw_diff = math.atan2(math.sin(self.locked_docking_yaw - curr_yaw), math.cos(self.locked_docking_yaw - curr_yaw))
                twist.angular.z = float(np.clip(1.2 * yaw_diff, -0.12, 0.12))
            else:
                twist.angular.z = 0.0

            dist_str = f"{self.desk_distance*100:.1f}cm" if self.desk_distance is not None else "blind"
            self.get_logger().info(
                f">>> [CRAWL_CONTACT] Crawling to bumper touch (dist={dist_str}, vx={twist.linear.x:.2f}m/s, wz={twist.angular.z:.2f}rad/s)...",
                throttle_duration_sec=1.0
            )

        # ----------------------------------------------------------------------
        # STATE: BACKUP_RETRY (Reverse 12.5cm [cut in half], increment retry counter, re-approach)
        # ----------------------------------------------------------------------
        elif self.state == 'BACKUP_RETRY':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            # Reverse at -0.05m/s for 2.5s (total 12.5cm, cut in half from 25cm)
            if elapsed < 2.5:
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            else:
                self.stop_robot()
                self.retry_count += 1
                if self.retry_count <= self.max_retries:
                    self.get_logger().info(f">>> [RETRY] Half-backup complete (12.5cm). Attempting re-approach (Retry {self.retry_count}/{self.max_retries})...")
                    self.transition_to('CHECK_STAGING')
                else:
                    self.get_logger().error(f">>> [FAILED] Exceeded maximum retries ({self.max_retries}). Aborting to Nav2 mode...")
                    self.transition_to('ABORT_TO_NAV2')
                return

        # ----------------------------------------------------------------------
        # STATE: DOCKED
        # ----------------------------------------------------------------------
        elif self.state == 'DOCKED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # ----------------------------------------------------------------------
        # ----------------------------------------------------------------------
        # STATE: UNDOCKING
        # ----------------------------------------------------------------------
        elif self.state == 'UNDOCKING':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed < 2.4:
                twist.linear.x = -0.10  # Firm reverse 24cm at -0.10 m/s
                twist.angular.z = 0.0
            else:
                self.stop_robot()
                self.transition_to('NAV2_READY')
                return

        # ----------------------------------------------------------------------
        # STATE: ABORT_TO_NAV2
        # ----------------------------------------------------------------------
        elif self.state == 'ABORT_TO_NAV2':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed < 2.4:
                twist.linear.x = -0.10  # Firm reverse 24cm at -0.10 m/s
                twist.angular.z = 0.0
            else:
                self.stop_robot()
                self.transition_to('NAV2_READY')
                return

        # ----------------------------------------------------------------------
        # STATE: NAV2_READY & FAILSAFE
        # ----------------------------------------------------------------------
        elif self.state in ('NAV2_READY', 'FAILSAFE'):
            return

        if self.state not in ('WAIT_NAV2', 'NAV2_READY'):
            self.send_cmd_vel(twist)

    # ==========================================================================
    # Visual HUD Overlay & Debug Image Publisher
    # ==========================================================================
    def render_and_publish_diagnostic_screen(self):
        """Publishes an informative diagnostic screen to /docking/debug_image when camera feed is not yet active."""
        if not self.publish_debug_img:
            return
        self._debug_img_counter += 1
        # Publish at ~4Hz (every 5 ticks of 20Hz control loop)
        if (self._debug_img_counter % 5) != 0:
            return

        vis = np.zeros((480, 640, 3), dtype=np.uint8)
        vis[:] = (22, 26, 32)

        # Top Banner
        cv2.rectangle(vis, (0, 0), (640, 80), (15, 18, 22), -1)
        cv2.line(vis, (0, 80), (640, 80), (55, 65, 75), 1)

        state_col = (255, 255, 0) if self.calibration_mode else (0, 255, 255)
        calib_tag = "[PASSIVE]" if self.calibration_mode else ""
        cv2.putText(vis, f"STATE: {self.state} {calib_tag}",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, state_col, 2, cv2.LINE_AA)

        bumper_str = "BUMPER: [ CONTACT! ]" if self.contact_detected else "BUMPER: [ CLEAR ]"
        bumper_col = (0, 0, 255) if self.contact_detected else (180, 180, 180)
        cv2.putText(vis, bumper_str, (430, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.48, bumper_col, 2 if self.contact_detected else 1, cv2.LINE_AA)

        # Central Diagnostic Box
        cv2.rectangle(vis, (40, 110), (600, 390), (30, 36, 45), -1)
        cv2.rectangle(vis, (40, 110), (600, 390), (0, 180, 255), 2)

        cv2.putText(vis, "WAITING FOR CAMERA STREAM...", (85, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 220, 255), 2, cv2.LINE_AA)

        cv2.putText(vis, f"Subscribed Topic: {self.color_topic}", (65, 215),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (230, 230, 230), 1, cv2.LINE_AA)

        cv2.putText(vis, "Next Step on Raspberry Pi:", (65, 265),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 255, 150), 1, cv2.LINE_AA)
        cv2.putText(vis, "1. Open a new terminal on Pi: ssh user@192.168.0.148", (80, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(vis, "2. Launch Astra S camera driver: bash ~/run_astra.sh", (80, 335),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(vis, f"Target Marker: ArUco DICT_4X4_50 | ID: {self.marker_id} ({self.marker_size*1000:.0f}mm)",
                    (65, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 180, 190), 1, cv2.LINE_AA)

        # Bottom Bar
        cv2.putText(vis, "Precision Marker Docking System v2.0 - Active Watchdog", (15, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (110, 120, 130), 1, cv2.LINE_AA)

        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            succ, enc_img = cv2.imencode('.jpg', vis, encode_param)
            if succ:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = self.get_clock().now().to_msg()
                comp_msg.header.frame_id = 'camera_color_optical_frame'
                comp_msg.format = 'jpeg'
                comp_msg.data = enc_img.tobytes()
                self.vis_comp_pub.publish(comp_msg)

            msg_out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = 'camera_color_optical_frame'
            self.vis_pub.publish(msg_out)
        except Exception:
            pass

    def render_and_publish_visualization(self, color_img, corners_img, rvec, tvec, depth_m=None):
        try:
            if color_img is not None:
                vis = color_img.copy()
            elif depth_m is not None:
                d_vis = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
                d_norm = cv2.normalize(d_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                vis = cv2.applyColorMap(d_norm, cv2.COLORMAP_TURBO)
            else:
                vis = np.zeros((480, 640, 3), dtype=np.uint8)

            vis_h, vis_w = vis.shape[:2]

            # 1. Draw ArUco 2D Bounding Boxes & 3D Frame Axes for all tracked markers
            if hasattr(self, 'tracked_markers') and len(self.tracked_markers) > 0:
                for mid, mdata in self.tracked_markers.items():
                    c_pts = mdata['corners'].astype(np.int32)
                    cv2.polylines(vis, [c_pts], True, (0, 255, 0), 2, cv2.LINE_AA)
                    center_2d = np.mean(c_pts, axis=0).astype(int)
                    cv2.circle(vis, tuple(center_2d), 4, (0, 255, 255), -1, cv2.LINE_AA)
                    cv2.putText(vis, f"ID:{mid}", (int(c_pts[0][0]), max(14, int(c_pts[0][1]) - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1, cv2.LINE_AA)
                    if mdata['rvec'] is not None and mdata['tvec'] is not None:
                        cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs, mdata['rvec'], mdata['tvec'], 0.035)

                # If dual markers active, draw connecting baseline & box center
                if len(self.tracked_markers) >= 2:
                    active_mids = list(self.tracked_markers.keys())
                    active_mids.sort(key=lambda m: self.tracked_markers[m]['tvec'][0][0])
                    left_c = np.mean(self.tracked_markers[active_mids[0]]['corners'], axis=0).astype(int)
                    right_c = np.mean(self.tracked_markers[active_mids[-1]]['corners'], axis=0).astype(int)
                    cv2.line(vis, tuple(left_c), tuple(right_c), (0, 200, 255), 2, cv2.LINE_AA)
                    mid_c = ((left_c + right_c) // 2)
                    cv2.circle(vis, tuple(mid_c), 5, (0, 0, 255), -1, cv2.LINE_AA)
            elif corners_img is not None:
                pts = corners_img.astype(np.int32)
                cv2.polylines(vis, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
                center_2d = np.mean(pts, axis=0).astype(int)
                cv2.circle(vis, tuple(center_2d), 4, (0, 0, 255), -1, cv2.LINE_AA)
                if rvec is not None and tvec is not None:
                    cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.04)

            # Draw Depth Plane Sampling ROI if active
            if self.plane_roi_rect is not None and self.plane_fit_source.startswith("DEP"):
                u1, v1, u2, v2 = self.plane_roi_rect
                cv2.rectangle(vis, (u1, v1), (u2, v2), (0, 220, 255), 1)
                cv2.putText(vis, f"PLANE ROI ({self.plane_fit_points}pts)", (u1, max(14, v1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 220, 255), 1, cv2.LINE_AA)

            # 2. Top Dashboard Banner (Ultra-clean non-overlapping layout for 320x240 & 640x480)
            hud_h = 58
            cv2.rectangle(vis, (0, 0), (vis_w, hud_h), (16, 18, 22), -1)
            cv2.line(vis, (0, hud_h), (vis_w, hud_h), (45, 55, 65), 1)

            # State color coding
            state_color = (0, 255, 255)
            if self.calibration_mode:
                state_color = (255, 255, 0)
            elif self.state == 'NAV2_READY':
                state_color = (100, 255, 120)
            elif self.state == 'DOCKED':
                state_color = (0, 255, 0)
            elif 'CRAWL' in self.state or 'FINAL' in self.state:
                state_color = (0, 165, 255)
            elif 'ABORT' in self.state or 'RETRY' in self.state:
                state_color = (0, 100, 255)

            # --- ROW 1 (Y=16): State & Mode Badge ---
            state_str = f"STATE: {self.state}"
            if self.docking_routine_start_time is not None and self.state not in ('NAV2_READY', 'DOCKED'):
                elapsed = (self.get_clock().now() - self.docking_routine_start_time).nanoseconds / 1e9
                rem = max(0.0, self.docking_timeout_sec - elapsed)
                state_str += f" [{rem:2.0f}s]"
            cv2.putText(vis, state_str, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, state_color, 1, cv2.LINE_AA)

            # Right: Mode Badge Box (Top Right, Y=3~18)
            if "DEPTH" in self.docking_alignment_mode:
                b_txt = "ASTRA DEPTH"
                b_col = (255, 220, 0)      # Cyan in BGR
                b_bg = (45, 28, 6)
            elif "RGB" in self.docking_alignment_mode:
                b_txt = "RGB OPTICAL"
                b_col = (0, 255, 100)      # Green in BGR
                b_bg = (10, 38, 15)
            elif self.odom_tracking_active:
                b_txt = "ODOM TRACK"
                b_col = (255, 160, 50)     # Sky Blue in BGR
                b_bg = (38, 20, 10)
            else:
                b_txt = "SEARCHING"
                b_col = (180, 180, 180)
                b_bg = (30, 30, 30)

            bw = 88 if vis_w <= 360 else 105
            bx1 = vis_w - bw - 4
            cv2.rectangle(vis, (bx1, 3), (vis_w - 4, 18), b_bg, -1)
            cv2.rectangle(vis, (bx1, 3), (vis_w - 4, 18), b_col, 1)
            cv2.putText(vis, b_txt, (bx1 + 5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.32, b_col, 1, cv2.LINE_AA)

            # --- ROW 2 (Y=33): Tracking Info & Bumper ---
            if self.desk_distance is not None:
                dist_cm = self.desk_distance * 100.0
                if self.state in ('STAGING_AIM', 'CARROT_ALIGN') and self.carrot_y_b is not None:
                    aim_deg = math.degrees(math.atan2(self.carrot_y_b, max(0.05, getattr(self, 'carrot_x_b', 0.20))))
                    target_info = f"TRACK: CARROT (aim:{aim_deg:+2.0f}deg)"
                    t_col = (0, 165, 255)  # Orange
                elif self.state in ('STANDOFF_SETTLE', 'FINAL_APPROACH', 'CRAWL_CONTACT'):
                    target_info = f"TRACK: DOCK (clear:{self.target_clearance*100:.0f}cm)"
                    t_col = (0, 255, 0)    # Green
                elif 'BACKUP' in self.state:
                    target_info = f"TRACK: REV_CARROT ({self.backup_substate})"
                    t_col = (255, 100, 200)  # Pink
                else:
                    target_info = f"TRACK: TARGET ({dist_cm:4.1f}cm)"
                    t_col = (210, 215, 220)
            else:
                target_info = f"SEARCHING ARUCO ID {self.marker_id}..."
                t_col = (120, 190, 255)
            cv2.putText(vis, target_info, (6, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.36, t_col, 1, cv2.LINE_AA)

            bumper_str = "BUMP: CONTACT!" if self.contact_detected else "BUMP: CLEAR"
            bumper_col = (0, 0, 255) if self.contact_detected else (160, 160, 160)
            cv2.putText(vis, bumper_str, (vis_w - 88, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.34, bumper_col, 1, cv2.LINE_AA)

            # --- ROW 3 (Y=50): Telemetry (d, ey, yaw, try) ---
            if self.desk_distance is not None:
                e_x, e_y, e_theta = self.compute_docking_errors()
                dist_cm = self.desk_distance * 100.0
                yaw_deg = math.degrees(self.desk_yaw_error) if self.desk_yaw_error is not None else 0.0
                ey_cm = (e_y * 100.0) if e_y is not None else 0.0

                telem_str = f"d:{dist_cm:4.1f}cm  ey:{ey_cm:+4.1f}cm  yaw:{yaw_deg:+4.1f}deg"
                cv2.putText(vis, telem_str, (6, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (225, 225, 225), 1, cv2.LINE_AA)

                retry_text = f"T:{self.retry_count}/{self.max_retries}"
                cv2.putText(vis, retry_text, (vis_w - 42, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (170, 170, 170), 1, cv2.LINE_AA)
            else:
                cv2.putText(vis, "Target not detected", (6, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (140, 140, 140), 1, cv2.LINE_AA)

            # Publish image to ROS 2 topic
            if self.publish_debug_img:
                self._debug_img_counter += 1
                if (self._debug_img_counter % max(1, self.debug_img_stride)) == 0:
                    stamp = self.get_clock().now().to_msg()
                    # 1. Publish lightweight compressed JPEG over Wi-Fi
                    try:
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                        succ, enc_img = cv2.imencode('.jpg', vis, encode_param)
                        if succ:
                            comp_msg = CompressedImage()
                            comp_msg.header.stamp = stamp
                            comp_msg.header.frame_id = self.last_color_frame_id
                            comp_msg.format = 'jpeg'
                            comp_msg.data = enc_img.tobytes()
                            self.vis_comp_pub.publish(comp_msg)
                    except Exception:
                        pass

                    # 2. Publish uncompressed locally
                    try:
                        msg_out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                        msg_out.header.stamp = stamp
                        msg_out.header.frame_id = self.last_color_frame_id
                        self.vis_pub.publish(msg_out)
                    except Exception:
                        pass

            if self.show_window:
                cv2.imshow("Precision Docking Visualizer", vis)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Visualizer render error: {e}", throttle_duration_sec=2.0)

    # ==========================================================================
    # Helper Functions & Safety Guards
    # ==========================================================================
    def check_contact_trigger(self) -> bool:
        if self.contact_detected or self.contact_latched:
            self.contact_latched = True
            self.get_logger().info("========================================================")
            self.get_logger().info(" ★ INSTANT BUMPER CONTACT CONFIRMED! Full Motor Lock!   ")
            self.get_logger().info(" ★ Exactly 5cm clearance established at Desk Front!     ")
            self.get_logger().info(" ★ DOCKED STATE ACHIEVED! Full Motor Lock Engaged.     ")
            self.get_logger().info("========================================================")
            self.stop_robot(hard_brake=True)
            self.transition_to('DOCKED')
            return True
        return False

    def check_sensor_timeout(self) -> bool:
        if self.last_sensor_time is not None:
            age = (self.get_clock().now() - self.last_sensor_time).nanoseconds / 1e9
            if age > 2.0:
                self.get_logger().warn(f"Sensor stream interrupted for {age:.2f}s. Pausing movement.", throttle_duration_sec=2.0)
                return True
        return False

    def send_cmd_vel(self, twist_or_vx, wz=0.0):
        if isinstance(twist_or_vx, Twist):
            target_vx = float(twist_or_vx.linear.x)
            target_wz = float(twist_or_vx.angular.z)
        elif isinstance(twist_or_vx, (int, float)):
            target_vx = float(twist_or_vx)
            target_wz = float(wz)
        else:
            target_vx = 0.0
            target_wz = 0.0

        now_t = time.time()
        dt = now_t - self._last_cmd_vel_time
        self._last_cmd_vel_time = now_t

        if dt > 0.5:  # First call or hiatus
            dt = 0.05
        dt = max(0.005, min(dt, 0.20))

        # Slew-rate acceleration limits (prevents Dynamixel gear backlash chattering)
        max_dw = self.max_wz_accel * dt
        dw = target_wz - self.current_cmd_wz
        if abs(dw) > max_dw:
            self.current_cmd_wz += math.copysign(max_dw, dw)
        else:
            self.current_cmd_wz = target_wz

        max_dv = self.max_vx_accel * dt
        dv = target_vx - self.current_cmd_vx
        if abs(dv) > max_dv:
            self.current_cmd_vx += math.copysign(max_dv, dv)
        else:
            self.current_cmd_vx = target_vx

        # Deadband snap for stationary commands
        if abs(target_vx) < 1e-4 and abs(self.current_cmd_vx) < 0.005:
            self.current_cmd_vx = 0.0
        if abs(target_wz) < 1e-4 and abs(self.current_cmd_wz) < 0.01:
            self.current_cmd_wz = 0.0

        vx = self.current_cmd_vx
        wz = self.current_cmd_wz

        if self.enable_stamped_cmd_vel:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x = vx
            msg.twist.angular.z = wz
            self.cmd_vel_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            self.cmd_vel_pub.publish(msg)

    def stop_robot(self, hard_brake=False):
        self.current_cmd_vx = 0.0
        self.current_cmd_wz = 0.0
        repeats = 5 if hard_brake else 3
        for _ in range(repeats):
            if self.enable_stamped_cmd_vel:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_footprint'
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)

    def transition_to(self, new_state: str):
        self.get_logger().info(f"[FSM] Transition: {self.state} -> {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now()

        if new_state in ('CHECK_STAGING', 'SEARCH_MARKER') and self.docking_routine_start_time is None:
            self.docking_routine_start_time = self.get_clock().now()
        elif new_state in ('NAV2_READY', 'FAILSAFE', 'DOCKED', 'ABORT_TO_NAV2'):
            self.docking_routine_start_time = None

        if new_state == 'CRAWL_CONTACT':
            self._stall_check_dist = None
            self._stall_start_time = None

        if new_state == 'STAGING_SETTLE':
            self.latched_aim_target_yaw = None
            self.latched_aim_delta_yaw = 0.0

        if new_state == 'CARROT_ALIGN':
            self.initial_ey = None

        if new_state in ('STANDOFF_SETTLE', 'CARROT_ALIGN'):
            self.dual_align_stable_count = 0

        if new_state in ('STANDOFF_SETTLE', 'FINAL_APPROACH', 'CRAWL_CONTACT', 'BACKUP_STANDOFF', 'BACKUP_RETRY', 'NAV2_READY', 'FAILSAFE', 'DOCKED', 'ABORT_TO_NAV2'):
            self.carrot_x_b = None
            self.carrot_y_b = None

        if new_state == 'STANDOFF_SETTLE':
            self.settle_substate = 'FINE_ROTATE'
            self.settle_stop_start_time = None
            self.settle_rotate_start_time = self.get_clock().now()

        if new_state == 'BACKUP_STANDOFF':
            self.backup_substate = 'BACK_AIM'
            self.reverse_start_odom_pose = None

            # Latch reference static errors and determine Reverse Virtual Carrot aiming parameters
            e_x, e_y, e_theta = self.compute_docking_errors()
            curr_yaw = self.get_robot_yaw()
            self.reverse_latched_origin_yaw = curr_yaw

            # 1. Target Reverse Distance: retreat to ~52cm staging zone (nominal 14cm)
            if e_x is not None:
                self.reverse_target_distance = float(np.clip(0.52 - e_x, 0.12, 0.18))
            elif self.desk_distance is not None:
                self.reverse_target_distance = float(np.clip(0.52 - self.desk_distance, 0.12, 0.18))
            else:
                self.reverse_target_distance = 0.14

            # 2. Aiming Angle to Reverse Virtual Carrot:
            # Inverted sign: target reverse vector to cancel ey during straight retreat
            # Clamped to +/- 18 deg to guarantee marker stays well within camera FOV
            if e_y is not None:
                delta_yaw = -math.atan2(e_y, max(0.10, self.reverse_target_distance))
                delta_yaw = float(np.clip(delta_yaw, -math.radians(18.0), math.radians(18.0)))
            else:
                delta_yaw = 0.0

            if curr_yaw is not None:
                self.reverse_latched_aim_yaw = math.atan2(
                    math.sin(curr_yaw + delta_yaw),
                    math.cos(curr_yaw + delta_yaw)
                )
            else:
                self.reverse_latched_aim_yaw = None

            self.get_logger().info(
                f">>> [TRANSITION: BACKUP_STANDOFF] Latching Reverse Virtual Carrot: "
                f"ey={e_y*100 if e_y is not None else 0.0:+.1f}cm, "
                f"target_dist={self.reverse_target_distance*100:.1f}cm, "
                f"aim_delta={math.degrees(delta_yaw):+.1f}°, "
                f"origin_yaw={math.degrees(curr_yaw) if curr_yaw is not None else 0.0:+.1f}° -> "
                f"aim_yaw={math.degrees(self.reverse_latched_aim_yaw) if self.reverse_latched_aim_yaw is not None else 0.0:+.1f}°"
            )
        else:
            self.backup_reason = "NONE"

        if new_state in ('NAV2_READY', 'FAILSAFE', 'DOCKED'):
            self.stop_robot()

        if new_state in DOCKING_ACTIVE_STATES:
            self.pause_rtabmap()
        elif new_state in ('NAV2_READY', 'FAILSAFE'):
            self.resume_rtabmap()

    def pause_rtabmap(self):
        if self.rtabmap_pause_client.service_is_ready():
            self.get_logger().info("[RESOURCE] Pausing RTAB-Map SLAM to free Pi 4 CPU and prevent map drift...")
            self.rtabmap_pause_client.call_async(Empty.Request())

    def resume_rtabmap(self):
        if self.rtabmap_resume_client.service_is_ready():
            self.get_logger().info("[RESOURCE] Resuming RTAB-Map SLAM...")
            self.rtabmap_resume_client.call_async(Empty.Request())

    def _init_motor_power_callback(self):
        if hasattr(self, '_motor_init_timer') and self._motor_init_timer is not None:
            self._motor_init_timer.cancel()
            self._motor_init_timer = None
        self.ensure_motor_power()

    def ensure_motor_power(self):
        if self.motor_power_cli.service_is_ready():
            req = SetBool.Request()
            req.data = True
            self.motor_power_cli.call_async(req)
            self.get_logger().info("[MOTOR] Dynamixel motor torque enabled via /motor_power")
        else:
            self.get_logger().warn("[MOTOR] /motor_power service not available yet", throttle_duration_sec=5.0)

    def handle_start_docking(self, request, response):
        self.get_logger().info(">>> [/start_docking] Received request to start Precision Docking...")

        # Strict Safety Gate 1: Both ArUco markers must be actively detected in camera FOV
        if not self.dual_markers_visible:
            msg = "도킹 시작 실패: 마커 2개가 모두 카메라 시야에 인식되어야 합니다."
            self.get_logger().warn(f">>> [/start_docking] Rejected: {msg}")
            response.success = False
            response.message = msg
            return response

        # Strict Safety Gate 2: Distance to target must be within 100cm (1.00m)
        if self.desk_distance is None or self.desk_distance > 1.00:
            dist_val = f"{self.desk_distance*100:.1f}cm" if self.desk_distance is not None else "미인식"
            msg = f"도킹 시작 실패: 마커와의 거리가 100cm 이내여야 합니다 (현재: {dist_val})."
            self.get_logger().warn(f">>> [/start_docking] Rejected: {msg}")
            response.success = False
            response.message = msg
            return response

        self.get_logger().info(f">>> [/start_docking] Dual markers verified (d={self.desk_distance*100:.1f}cm <= 100cm). Starting docking routine!")
        self.contact_latched = False
        self.ensure_motor_power()
        self.retry_count = 0
        self.initial_ey = None
        self.stop_robot()
        self.transition_to('CHECK_STAGING')
        response.success = True
        response.message = f"정밀 도킹 루틴 시작 (마커 2개 확인 완료, 거리: {self.desk_distance*100:.1f}cm)."
        return response

    def handle_undock_to_nav2(self, request, response):
        self.get_logger().info(">>> [/undock_to_nav2] Received command to undock and return to Nav2!")
        self.contact_latched = False
        self.odom_target_latched = False
        self.odom_target_center = None
        self.odom_target_normal = None
        self.odom_tracking_active = False
        self.initial_ey = None
        self.stop_robot()
        if self.state == 'DOCKED':
            self.transition_to('UNDOCKING')
            response.success = True
            response.message = "Initiating safe undock retreat (15cm)."
        else:
            self.transition_to('NAV2_READY')
            response.success = True
            response.message = "Switched to NAV2_READY mode."
        return response

    def handle_abort_docking(self, request, response):
        self.get_logger().warn(">>> [/abort_docking] Abort requested. Clearing odom memory and retreating to Nav2 mode...")
        self.contact_latched = False
        self.odom_target_latched = False
        self.odom_target_center = None
        self.odom_target_normal = None
        self.odom_tracking_active = False
        self.initial_ey = None
        self.stop_robot()
        self.transition_to('ABORT_TO_NAV2')
        response.success = True
        response.message = "Docking aborted. Retreating 24cm."
        return response

    def handle_emergency_stop(self, request, response):
        self.get_logger().warn(">>> [/emergency_stop] E-STOP Triggered! Immediate Motor Halt and Docking Reset.")
        self.contact_latched = False
        self.odom_target_latched = False
        self.odom_target_center = None
        self.odom_target_normal = None
        self.odom_tracking_active = False
        self.initial_ey = None
        self.stop_robot(hard_brake=True)
        self.transition_to('NAV2_READY')
        response.success = True
        response.message = "E-STOP: Robot motors halted immediately."
        return response

    def get_robot_pose_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception:
            return None

    def get_robot_pose_in_odom(self):
        """Returns robot (x, y, yaw) in odom frame using wheel odometry."""
        for base_frame in ('base_footprint', 'base_link'):
            try:
                t = self.tf_buffer.lookup_transform('odom', base_frame, rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return (x, y, yaw)
            except Exception:
                continue
        return None

    def get_robot_yaw(self):
        """Returns robot yaw in odom or map frame using high-rate IMU/Wheel odometry."""
        for target_frame in ('odom', 'map'):
            for base_frame in ('base_footprint', 'base_link'):
                try:
                    t = self.tf_buffer.lookup_transform(target_frame, base_frame, rclpy.time.Time())
                    q = t.transform.rotation
                    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    return math.atan2(siny_cosp, cosy_cosp)
                except Exception:
                    continue
        return None

    def send_nav2_staging_goal(self):
        if not NAV2_ACTION_AVAILABLE or self.nav2_client is None:
            self.transition_to('CHECK_STAGING')
            return
        if not self.nav2_client.wait_for_server(timeout_sec=3.0):
            self.transition_to('CHECK_STAGING')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.staging_x
        goal.pose.pose.position.y = self.staging_y
        goal.pose.pose.orientation.w = math.cos(self.staging_yaw / 2.0)
        goal.pose.pose.orientation.z = math.sin(self.staging_yaw / 2.0)

        future = self.nav2_client.send_goal_async(goal)
        future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav2_finished = True
            self.nav2_success = False
            return
        self.nav2_goal_handle = goal_handle
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        res = future.result()
        self.nav2_finished = True
        self.nav2_success = (res.status == GoalStatus.STATUS_SUCCEEDED)


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
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        stop_twist = Twist()
        try:
            node.send_cmd_vel(0.0, 0.0)
            node.resume_rtabmap()
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
