#!/usr/bin/env python3
# ==============================================================================
# Precision Autonomous Docking Controller Node
# Engine: 1:1 Mirror of Verified Precision Approacher Node Algorithm
# Integrated with: Qwen GUI, TaskManager, RViz Gizmo, StationManager
# ==============================================================================

import time
import math
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Point, Quaternion, Vector3
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray

try:
    from turtlebot3_msgs.msg import SensorState
    TURTLEBOT3_MSGS_AVAILABLE = True
except ImportError:
    TURTLEBOT3_MSGS_AVAILABLE = False

from dataclasses import dataclass
import tf2_ros
from cv_bridge import CvBridge

from llm_semantic_docker.station_manager import StationManager


@dataclass
class TargetObject:
    center: Optional[np.ndarray] = None          # [x, y, z] in base_footprint (m)
    docking_point: Optional[np.ndarray] = None   # [x, y, z] in base_footprint (m)
    approach_normal: float = 0.0                 # rad
    docking_distance: float = 0.05               # 5cm target
    valid: bool = False
    last_seen_time: float = 0.0


DOCKING_ACTIVE_STATES = (
    'SEARCH_MARKER', 'CHECK_STAGING', 'STAGING_SETTLE', 'STAGING_AIM',
    'CARROT_ALIGN', 'STANDOFF_SETTLE', 'CRAWL_CONTACT', 'BACKUP_STANDOFF', 'BACKUP_RETRY'
)


class PrecisionDockingControllerNode(Node):
    """
    Precision Docking Controller Node.
    Algorithm is a 1:1 direct mirror of the verified PrecisionApproacherNode,
    with interfaces mapped for Qwen Semantic GUI and TaskManager.
    """

    def __init__(self):
        super().__init__('precision_docking_controller_node')

        # ----------------------------------------------------------------------
        # Parameters: General & Hardware
        # ----------------------------------------------------------------------
        self.declare_parameter('enable_stamped_cmd_vel', True)
        self.declare_parameter('docking_mode', 'marker')
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('marker_size', 0.060)               # 60mm (0.06m)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('cam_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('bumper_topic', '/robot/bumper/contact')
        self.declare_parameter('target_clearance', 0.05)          # 5cm target clearance
        self.declare_parameter('reference_x', 0.100)              # Bumper tip offset from base_footprint (m)
        self.declare_parameter('max_linear_speed', 0.050)         # 5.0 cm/s
        self.declare_parameter('crawl_linear_speed', 0.045)       # 4.5 cm/s crawl
        self.declare_parameter('max_angular_speed', 0.22)         # 0.22 rad/s (~12.6 deg/s)
        self.declare_parameter('docking_timeout_sec', 180.0)      # Global ceiling timeout: 180.0s (3min)
        self.declare_parameter('crawl_timeout_sec', 35.0)         # Dedicated final crawl window: 35.0s
        self.declare_parameter('stall_timeout_sec', 20.0)         # Progress stall watchdog: 20.0s
        self.declare_parameter('marker_yaw_bias_deg', 0.0)
        self.declare_parameter('carrot_lookahead', 0.22)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('backup_retry_distance', 0.18)

        # Astra S camera physical extrinsic relative to base_footprint
        self.declare_parameter('astra_x_base', 0.080)
        self.declare_parameter('astra_y_base', 0.015)
        self.declare_parameter('astra_z_base', 0.134)
        self.declare_parameter('astra_pitch', 0.0)

        # Retrieve parameters
        self.enable_stamped_cmd_vel = bool(self.get_parameter('enable_stamped_cmd_vel').value)
        self.docking_mode = str(self.get_parameter('docking_mode').value).lower()
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.cam_info_topic = str(self.get_parameter('cam_info_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.bumper_topic = str(self.get_parameter('bumper_topic').value)
        self.target_clearance = float(self.get_parameter('target_clearance').value)
        self.reference_x = float(self.get_parameter('reference_x').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.crawl_linear_speed = float(self.get_parameter('crawl_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.docking_timeout_sec = float(self.get_parameter('docking_timeout_sec').value)
        self.crawl_timeout_sec = float(self.get_parameter('crawl_timeout_sec').value)
        self.stall_timeout_sec = float(self.get_parameter('stall_timeout_sec').value)
        self.marker_yaw_bias_rad = math.radians(float(self.get_parameter('marker_yaw_bias_deg').value))
        self.carrot_lookahead = float(self.get_parameter('carrot_lookahead').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.backup_retry_distance = float(self.get_parameter('backup_retry_distance').value)

        self.astra_x_base = float(self.get_parameter('astra_x_base').value)
        self.astra_y_base = float(self.get_parameter('astra_y_base').value)
        self.astra_z_base = float(self.get_parameter('astra_z_base').value)
        self.astra_pitch = float(self.get_parameter('astra_pitch').value)

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

        # OpenCV ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.04
        self.aruco_params.maxErroneousBitsInBorderRate = 0.25
        self.aruco_params.errorCorrectionRate = 0.60
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        s = self.marker_size
        self.marker_3d_corners = np.array([
            [-s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0, -s / 2.0, 0.0],
            [-s / 2.0, -s / 2.0, 0.0]
        ], dtype=np.float32)

        # Marker Tracking & Perception State
        self.marker_detected: bool = False
        self.marker_last_seen: float = 0.0
        self.marker_timeout_sec: float = 1.2
        self.marker_pos_base: Optional[np.ndarray] = None
        self.marker_normal_base: Optional[np.ndarray] = None
        self.desk_distance: Optional[float] = None
        self.desk_yaw_error: Optional[float] = None
        self.marker_corners_img = None
        self.marker_rvec = None
        self.marker_tvec = None

        self.tracked_markers: Dict[int, Any] = {}
        self.marker_offsets_to_center: Dict[int, np.ndarray] = {}
        self.docking_alignment_mode: str = 'NONE'
        self.plane_fit_source: str = 'NONE'
        self.plane_fit_points: int = 0
        self.dual_markers_visible: bool = False

        # 2-Stage Filter State (Median + Adaptive EMA)
        self.yaw_history: List[float] = []
        self.yaw_history_len: int = 5
        self.filtered_yaw: Optional[float] = None

        self.ey_history: List[float] = []
        self.ey_history_len: int = 5
        self.filtered_ey: Optional[float] = None

        # Odom-Anchored Visual Memory Tracking (Blind Pursuit)
        self.odom_target_latched: bool = False
        self.odom_target_center: Optional[np.ndarray] = None   # [x, y, z] in odom
        self.odom_target_normal: Optional[np.ndarray] = None   # normal pointing into desk in odom
        self.odom_target_last_seen: float = 0.0
        self.odom_tracking_active: bool = False
        self.odom_blind_start_time: Optional[float] = None

        # Virtual Carrot State
        self.carrot_x_b: Optional[float] = None
        self.carrot_y_b: Optional[float] = None
        self.latched_aim_delta_yaw: float = 0.0
        self.latched_aim_target_yaw: Optional[float] = None
        self.locked_docking_yaw: Optional[float] = None
        self.target_carrot_dist: float = 0.0
        self.carrot_start_odom_pose: Optional[Tuple[float, float, float]] = None
        self.carrot_locked_yaw: Optional[float] = None

        # FSM State & Timing
        self.state: str = 'NAV2_READY'
        self.state_start_time = self.get_clock().now()
        self.docking_routine_start_time: Optional[float] = None
        self.last_progress_time: float = time.time()
        self.min_tracked_distance: float = 999.0
        self.retry_count: int = 0
        self.initial_ey: Optional[float] = None
        self.backup_reason: str = "NONE"

        # Standoff Evaluation State
        self.settle_substate: str = 'FINE_ROTATE'
        self.settle_rotate_start_time = None
        self.settle_stop_start_time = None
        self.dual_align_stable_count: int = 0

        # Backup Substate
        self.backup_substate: str = 'BACK_AIM'
        self.reverse_latched_aim_yaw: Optional[float] = None
        self.reverse_latched_origin_yaw: Optional[float] = None
        self.reverse_target_distance: float = 0.18
        self.reverse_start_odom_pose: Optional[Tuple[float, float, float]] = None

        # Contact & Bumper State
        self.contact_detected: bool = False
        self.contact_latched: bool = False

        # Sensor Images
        self.last_color_img: Optional[np.ndarray] = None
        self.last_depth_raw: Optional[np.ndarray] = None
        self.last_depth_time: float = 0.0
        self.bridge = CvBridge()

        # Station Manager & Coordinates
        self.station_mgr = StationManager()
        self.active_station_name: Optional[str] = None
        self.target_obj = TargetObject()

        # TF2 Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------------------
        if self.enable_stamped_cmd_vel:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.status_pub = self.create_publisher(String, '/docking/status', 10)
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)
        self.marker_pub = self.create_publisher(Marker, '/docking/marker_debug', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/docking/target_object_pose', 10)
        self.carrot_marker_pub = self.create_publisher(Marker, '/docking/carrot_marker', 10)
        self.gizmo_markers_pub = self.create_publisher(MarkerArray, '/docking_teacher_gizmo/markers', 10)

        # Subscriptions
        self.cam_info_sub = self.create_subscription(CameraInfo, self.cam_info_topic, self._on_cam_info, 10)
        self.color_sub = self.create_subscription(Image, self.color_topic, self._on_color_img, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self._on_depth_img, 10)
        self.bumper_sub = self.create_subscription(Bool, self.bumper_topic, self._on_bumper, 10)
        if TURTLEBOT3_MSGS_AVAILABLE:
            self.sensor_state_sub = self.create_subscription(SensorState, '/sensor_state', self._on_sensor_state, 10)
        self.software_stop_sub = self.create_subscription(Bool, '/safety/software_stop', self._on_software_stop, 10)
        self.start_station_sub = self.create_subscription(String, '/docking/start_station', self._on_start_station, 10)

        # Services
        self.create_service(Trigger, '/docking/start', self._srv_start_docking)
        self.create_service(Trigger, '/start_docking', self._srv_start_docking)
        self.create_service(Trigger, '/docking/abort', self._srv_abort_docking)
        self.create_service(Trigger, '/abort_docking', self._srv_abort_docking)
        self.create_service(Trigger, '/docking/confirm', self._srv_confirm_docking)
        self.create_service(Trigger, '/undock_to_nav2', self._srv_undock_to_nav2)
        self.create_service(Trigger, '/emergency_stop', self._srv_emergency_stop)

        # Timers
        self.control_timer = self.create_timer(0.05, self._control_loop)
        self.gizmo_timer = self.create_timer(1.0, self._publish_gizmo_markers)

        self.get_logger().info("=" * 65)
        self.get_logger().info("🚀 [PrecisionDockingController] 1:1 Approacher Engine Initialized")
        self.get_logger().info(f"   Default Marker ID: {self.marker_id} ({self.marker_size*1000:.0f}mm)")
        self.get_logger().info(f"   Registered Stations: {self.station_mgr.list_stations()}")
        self.get_logger().info("=" * 65)

    # ==========================================================================
    # GUI Logging & Notification Helper
    # ==========================================================================
    def _report_gui(self, text: str, is_info: bool = True):
        prefix = "INFO:" if is_info else "ALERT:"
        if is_info:
            self.get_logger().info(f"[GUI-LOG] {text}")
        else:
            self.get_logger().warn(f"[GUI-ALERT] {text}")
        try:
            self.task_status_pub.publish(String(data=f"{prefix} {text}"))
        except Exception:
            pass

    # ==========================================================================
    # Callbacks: Sensors & Cameras
    # ==========================================================================
    def _on_cam_info(self, msg: CameraInfo):
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

    def _on_bumper(self, msg: Bool):
        if msg.data:
            self.contact_detected = True
            self.contact_latched = True
            if self.state in DOCKING_ACTIVE_STATES:
                self.get_logger().info("💥 [BUMPER CONTACT] Confirmed via /robot/bumper/contact! Full Motor Lock!")
                self._report_gui("💥 [도킹 완료] 전면 물리 범퍼 접촉 감지! 완벽 안착되었습니다.")
                self.stop_robot()
                self.transition_to('DOCKED')
        else:
            self.contact_detected = False

    def _on_sensor_state(self, msg: 'SensorState'):
        if bool(msg.bumper & 1):
            self.contact_detected = True
            self.contact_latched = True
            if self.state in DOCKING_ACTIVE_STATES:
                self.get_logger().info("💥 [BUMPER CONTACT] Confirmed via /sensor_state! Full Motor Lock!")
                self._report_gui("💥 [도킹 완료] 전면 물리 범퍼 접촉 감지! 완벽 안착되었습니다.")
                self.stop_robot()
                self.transition_to('DOCKED')

    def _on_software_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("🚨 [EMERGENCY STOP] Received /safety/software_stop! Halting immediately.")
            self.stop_robot()
            self._report_gui("🚨 [비상 정지] 비상 정지 신호 수신. 정밀 도킹을 중단합니다 ➔ 대기(NAV2_READY) 전환", is_info=False)
            self.transition_to('NAV2_READY')

    def _on_color_img(self, msg: Image):
        # Throttle to ~15 FPS to eliminate Pi 4 CPU bottleneck
        now_t = time.time()
        if hasattr(self, '_last_img_time') and (now_t - self._last_img_time) < 0.065:
            return
        self._last_img_time = now_t

        try:
            self.last_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_id = msg.header.frame_id or 'camera_color_optical_frame'
            self._process_marker_detection(self.last_color_img, frame_id)
        except Exception:
            pass

    def _on_depth_img(self, msg: Image):
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                self.last_depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            elif msg.encoding == '32FC1':
                self.last_depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.last_depth_time = time.time()
        except Exception:
            pass

    def _query_marker_center_depth(self, marker_corners: np.ndarray) -> Optional[float]:
        if self.last_depth_raw is None or (time.time() - self.last_depth_time) > 1.2:
            return None
        H, W = self.last_depth_raw.shape[:2]
        u_c = int(round(float(np.mean(marker_corners[:, 0]))))
        v_c = int(round(float(np.mean(marker_corners[:, 1]))))
        u1, u2 = max(0, u_c - 2), min(W, u_c + 3)
        v1, v2 = max(0, v_c - 2), min(H, v_c + 3)
        patch = self.last_depth_raw[v1:v2, u1:u2]
        if self.last_depth_raw.dtype == np.uint16:
            v_pts = patch[(patch > 150) & (patch < 2500)]
            if len(v_pts) >= 4:
                return float(np.median(v_pts)) / 1000.0
        else:
            v_pts = patch[(patch > 0.15) & (patch < 2.5) & np.isfinite(patch)]
            if len(v_pts) >= 4:
                return float(np.median(v_pts))
        return None

    # ==========================================================================
    # Visual Perception & Pose Estimation (1:1 with precision_approacher_node)
    # ==========================================================================
    def _process_marker_detection(self, color_img: np.ndarray, frame_id: str):
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        now_t = time.time()
        detected = False

        if ids is not None and len(ids) > 0:
            for idx in range(len(ids)):
                m_id = int(ids.flatten()[idx])
                m_corners = corners[idx][0]
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

        # Expire old markers (> 0.8s)
        expired = [mid for mid, data in self.tracked_markers.items() if (now_t - data['last_seen']) > 0.80]
        for mid in expired:
            del self.tracked_markers[mid]

        active_ids = list(self.tracked_markers.keys())

        # 1. Dual-Marker Fusion
        if len(active_ids) >= 2:
            active_ids.sort(key=lambda mid: self.tracked_markers[mid]['tvec'][0][0])
            left_id = active_ids[0]
            right_id = active_ids[-1]

            p_left = self.tracked_markers[left_id]['tvec'].flatten()
            p_right = self.tracked_markers[right_id]['tvec'].flatten()
            span = float(np.linalg.norm(p_right - p_left))

            if 0.05 <= span <= 0.40:
                t_cam_mid = (p_left + p_right) / 2.0
                dx = float(p_right[0] - p_left[0])
                dz = float(p_right[2] - p_left[2])

                self.marker_offsets_to_center[left_id] = (t_cam_mid - p_left).copy()
                self.marker_offsets_to_center[right_id] = (t_cam_mid - p_right).copy()

                norm_len = math.hypot(dz, dx)
                if norm_len > 1e-4:
                    n_cam = np.array([dz / norm_len, 0.0, -dx / norm_len], dtype=np.float64)
                else:
                    n_cam = np.array([0.0, 0.0, -1.0], dtype=np.float64)

                p_base, n_base = self._transform_optical_to_base(t_cam_mid, n_cam, frame_id)

                if p_base is not None and n_base is not None:
                    desk_normal = -n_base
                    raw_heading_err = math.atan2(desk_normal[1], desk_normal[0]) - self.marker_yaw_bias_rad

                    self.docking_alignment_mode = f"DUAL:RGB({span*100:.0f}cm)"
                    self.marker_corners_img = self.tracked_markers[left_id]['corners']
                    self.marker_rvec = self.tracked_markers[left_id]['rvec']
                    self.marker_tvec = t_cam_mid.reshape(3, 1)

                    detected = True
                    self.dual_markers_visible = True

                    # Latch Target Pose into ODOM frame
                    try:
                        t_odom = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                        trans = t_odom.transform.translation
                        rot = t_odom.transform.rotation
                        R_odom = self._quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
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

        # 2. Single-Marker Fallback
        if not detected and len(active_ids) >= 1:
            self.dual_markers_visible = False
            mid = self.marker_id if self.marker_id in active_ids else active_ids[0]
            m_corners = self.tracked_markers[mid]['corners']
            rvec = self.tracked_markers[mid]['rvec']
            tvec = self.tracked_markers[mid]['tvec']
            t_cam = tvec.flatten()

            offset_to_center = self.marker_offsets_to_center.get(mid, np.zeros(3, dtype=np.float64))
            t_cam_box = t_cam + offset_to_center

            R_cam, _ = cv2.Rodrigues(rvec)
            n_local = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            n_cam = R_cam @ n_local

            p_base, n_base = self._transform_optical_to_base(t_cam_box, n_cam, frame_id)
            if p_base is not None and n_base is not None:
                desk_normal = -n_base
                raw_heading_err = math.atan2(desk_normal[1], desk_normal[0]) - self.marker_yaw_bias_rad
                self.docking_alignment_mode = f"SINGLE:ID{mid}"
                self.marker_corners_img = m_corners
                self.marker_rvec = rvec
                self.marker_tvec = tvec
                detected = True

        # 3. Filtering Pipeline (Median + Adaptive EMA)
        if detected:
            self.yaw_history.append(raw_heading_err)
            if len(self.yaw_history) > self.yaw_history_len:
                self.yaw_history.pop(0)
            median_yaw = float(np.median(self.yaw_history))

            if self.filtered_yaw is None:
                self.filtered_yaw = median_yaw
            else:
                step_diff = math.atan2(math.sin(median_yaw - self.filtered_yaw), math.cos(median_yaw - self.filtered_yaw))
                alpha = 0.50 if abs(step_diff) > math.radians(4.0) else 0.20
                self.filtered_yaw = float(self.filtered_yaw + alpha * step_diff)

            self.desk_yaw_error = self.filtered_yaw
            self.desk_distance = float(p_base[0]) - self.reference_x
            self.marker_pos_base = p_base
            self.marker_normal_base = desk_normal
            self.marker_detected = True
            self.marker_last_seen = time.time()

            self.target_obj.center = p_base
            self.target_obj.docking_point = p_base
            self.target_obj.valid = True
            self.target_obj.last_seen_time = time.time()

            self._publish_marker_debug(p_base, desk_normal)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_footprint'
            pose_msg.pose.position.x = float(p_base[0])
            pose_msg.pose.position.y = float(p_base[1])
            pose_msg.pose.position.z = float(p_base[2])
            yaw_m = math.atan2(desk_normal[1], desk_normal[0])
            pose_msg.pose.orientation.z = math.sin(yaw_m * 0.5)
            pose_msg.pose.orientation.w = math.cos(yaw_m * 0.5)
            self.target_pose_pub.publish(pose_msg)

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

    def _transform_optical_to_base(self, t_cam: np.ndarray, n_cam: np.ndarray, frame_id: str):
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', frame_id, rclpy.time.Time())
            trans = t.transform.translation
            rot = t.transform.rotation
            R_tf = self._quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
            t_tf = np.array([trans.x, trans.y, trans.z], dtype=np.float64)

            p_base = R_tf @ t_cam + t_tf
            p_base[1] += self.astra_y_base
            n_base = R_tf @ n_cam
            return p_base, n_base
        except Exception:
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
    def _quaternion_to_matrix(q):
        x, y, z, w = q
        return np.array([
            [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
            [2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
            [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)]
        ], dtype=np.float64)

    # ==========================================================================
    # Error Computation & Virtual Carrot (Stabilized Projection)
    # ==========================================================================
    def get_carrot_point_in_base(self, L_carrot: float = 0.28) -> Optional[np.ndarray]:
        if self.marker_pos_base is not None and self.marker_normal_base is not None:
            # Unit normal vector in base_footprint (pointing into desk)
            nx = float(self.marker_normal_base[0])
            ny = float(self.marker_normal_base[1])
            nz = float(self.marker_normal_base[2])
            n_len = math.hypot(nx, ny)
            if n_len > 1e-6:
                nx /= n_len
                ny /= n_len

            # Heading alignment factor: cos(e_theta) = nx (forward normal component)
            cos_yaw = float(np.clip(nx, 0.0, 1.0))
            # Attenuate lateral corridor projection when heading error is large
            # (Prevents violent S-curves and false opposite turns during angled approach)
            lateral_damping = cos_yaw * cos_yaw

            px = float(self.marker_pos_base[0])
            py = float(self.marker_pos_base[1])
            pz = float(self.marker_pos_base[2])

            # Adaptive lookahead length: gracefully reduces if robot is severely angled
            L_eff = L_carrot * max(0.50, cos_yaw)
            delta_x = L_eff * nx
            delta_y = L_eff * ny * lateral_damping

            # Clamp lateral offset so the carrot waypoint never deviates drastically from the marker line
            max_lateral_shift = float(np.clip(0.35 * px, 0.04, 0.10))
            delta_y = float(np.clip(delta_y, -max_lateral_shift, max_lateral_shift))

            c_x = max(0.08, px - delta_x)
            c_y = py - delta_y
            c_z = pz - L_eff * nz
            return np.array([c_x, c_y, c_z], dtype=np.float64)
        return None

    def compute_docking_errors(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        # 1. Direct Optical Perception
        if self.marker_detected and self.desk_distance is not None:
            self.odom_tracking_active = False
            self.odom_blind_start_time = None

            e_x = self.desk_distance - self.target_clearance
            e_theta = self.desk_yaw_error if self.desk_yaw_error is not None else 0.0

            if self.target_obj.valid and self.target_obj.center is not None:
                used_fixed_ey = False
                if self.odom_target_latched and self.odom_target_center is not None and self.odom_target_normal is not None:
                    try:
                        t_b_o = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                        rx = t_b_o.transform.translation.x
                        ry = t_b_o.transform.translation.y
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
                    px = float(self.target_obj.center[0])
                    py = float(self.target_obj.center[1])
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
            else:
                e_y = 0.0

            return e_x, e_y, e_theta

        # 2. Odom-Anchored Visual Memory Tracking (Blind Pursuit)
        elif self.odom_target_latched and self.odom_target_center is not None:
            try:
                t = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time())
                trans = t.transform.translation
                rot = t.transform.rotation
                R_b_o = self._quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
                t_b_o = np.array([trans.x, trans.y, trans.z], dtype=np.float64)

                p_base = R_b_o @ self.odom_target_center + t_b_o
                desk_normal = R_b_o @ self.odom_target_normal
                norm_l = np.linalg.norm(desk_normal)
                if norm_l > 1e-6:
                    desk_normal /= norm_l

                dist_m = float(p_base[0]) - self.reference_x
                self.desk_distance = dist_m
                self.marker_pos_base = p_base
                self.marker_normal_base = desk_normal

                if not self.odom_tracking_active:
                    self.odom_tracking_active = True
                    self.odom_blind_start_time = time.time()
                    self.get_logger().info(">>> [ODOM VISUAL MEMORY] Markers out of FOV. Blind pursuit engaged!")

                e_x = dist_m - self.target_clearance
                e_theta = math.atan2(desk_normal[1], desk_normal[0]) - self.marker_yaw_bias_rad

                # Fixed-frame centerline error
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
            except Exception:
                pass

        return None, None, None

    def get_robot_yaw(self) -> Optional[float]:
        for target_frame in ('odom', 'map'):
            for base_frame in ('base_footprint', 'base_link'):
                try:
                    t = self.tf_buffer.lookup_transform(target_frame, base_frame, rclpy.time.Time())
                    q = t.transform.rotation
                    siny = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    return math.atan2(siny, cosy)
                except Exception:
                    continue
        return None

    def get_robot_pose_in_odom(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return (x, y, math.atan2(siny, cosy))
        except Exception:
            return None

    # ==========================================================================
    # Main Control Loop FSM (1:1 with precision_approacher_node)
    # ==========================================================================
    def _control_loop(self):
        now = self.get_clock().now()
        twist = Twist()

        # Telemetry string for Qwen GUI & TaskManager (20Hz)
        dist_str = f"{self.desk_distance:.2f}m" if self.desk_distance is not None else "0.00m"
        yaw_str = f"{math.degrees(self.desk_yaw_error):+.1f}" if self.desk_yaw_error is not None else "0.0"
        ey_str = f"{(self.filtered_ey*100):+.1f}cm" if self.filtered_ey is not None else "+0.0cm"
        status_line = f"STATE={self.state}, ey={ey_str}, yaw_err={yaw_str}deg, dist={dist_str}"
        self.status_pub.publish(String(data=status_line))

        # Check contact trigger
        if self.contact_latched and self.state in DOCKING_ACTIVE_STATES:
            self.stop_robot()
            self.transition_to('DOCKED')
            return

        # Track continuous docking progress (Dynamic Lease Renewal)
        now_sec = time.time()
        if self.state in DOCKING_ACTIVE_STATES:
            # If distance decreased by >= 2cm, mark progress and renew watchdog
            if self.desk_distance is not None and self.desk_distance < (self.min_tracked_distance - 0.02):
                self.min_tracked_distance = self.desk_distance
                self.last_progress_time = now_sec

            # Stall watchdog: Check if robot has made ZERO progress for stall_timeout_sec
            if (now_sec - self.last_progress_time) > self.stall_timeout_sec:
                self.get_logger().warn(
                    f">>> [STALL WATCHDOG] No state change or forward progress for {self.stall_timeout_sec:.1f}s in state '{self.state}'."
                )
                self._report_gui(f"⚠️ [도킹 정체 감지] {self.stall_timeout_sec:.0f}초간 진전 없음 ➔ 안전 재정렬", is_info=False)
                self.last_progress_time = now_sec
                if self.state in ('CARROT_ALIGN', 'STAGING_AIM'):
                    self.transition_to('BACKUP_STANDOFF')
                    return
                elif self.state == 'CRAWL_CONTACT':
                    self.transition_to('BACKUP_RETRY')
                    return

        # Check global docking ceiling timeout (Hard safety limit)
        if self.docking_routine_start_time is not None and self.state in DOCKING_ACTIVE_STATES:
            if (now_sec - self.docking_routine_start_time) > self.docking_timeout_sec:
                self.get_logger().error(f">>> [DOCKING TIMEOUT] Exceeded global ceiling {self.docking_timeout_sec:.1f}s without completing docking!")
                self._report_gui(f"⏱️ [도킹 타임아웃] 전체 제한 시간({self.docking_timeout_sec:.0f}초) 초과 ➔ 대기(NAV2_READY) 전환", is_info=False)
                self.stop_robot()
                self.transition_to('NAV2_READY')
                return

        # ----------------------------------------------------------------------
        # STATE: NAV2_READY (Idle & Motor Topic Release)
        # ----------------------------------------------------------------------
        if self.state in ('NAV2_READY', 'FAILSAFE'):
            return

        # ----------------------------------------------------------------------
        # STATE: INIT
        # ----------------------------------------------------------------------
        elif self.state == 'INIT':
            if self.marker_detected:
                self.get_logger().info(f">>> [INIT] Marker ID {self.marker_id} acquired ({self.desk_distance*100:.1f}cm). Proceeding to CHECK_STAGING.")
                self.transition_to('CHECK_STAGING')
                return
            else:
                self.transition_to('SEARCH_MARKER')
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

            if elapsed < 0.8:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif elapsed < 4.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.25  # Sweep left
            elif elapsed < 10.0:
                twist.linear.x = 0.0
                twist.angular.z = -0.25 # Sweep right
            elif elapsed < 13.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.25  # Return to center
            else:
                self.get_logger().warn(">>> [SEARCH_MARKER] Marker not found after 13s. Returning to NAV2_READY.")
                self.stop_robot()
                self._report_gui("⚠️ [마커 미검출] 13초간 마커가 감지되지 않아 대기 상태로 전환합니다.", is_info=False)
                self.transition_to('NAV2_READY')
                return

        # ----------------------------------------------------------------------
        # STATE: CHECK_STAGING
        # ----------------------------------------------------------------------
        elif self.state == 'CHECK_STAGING':
            if self.desk_distance is not None:
                e_x, e_y, e_theta = self.compute_docking_errors()
                e_y_val = abs(e_y) if e_y is not None else 0.0
                e_theta_val = abs(e_theta) if e_theta is not None else 0.0

                if self.desk_distance < 0.35 and (e_y_val > 0.025 or e_theta_val > math.radians(3.0)):
                    self.backup_reason = f"초기 위치 너무 가까움 ({self.desk_distance*100:.0f}cm < 35cm)"
                    self.get_logger().info(f">>> [CHECK_STAGING] Too close for FOV. Backing up to standoff...")
                    self.transition_to('BACKUP_STANDOFF')
                    return

                self.transition_to('STAGING_SETTLE')
                return
            else:
                elapsed = (now - self.state_start_time).nanoseconds / 1e9
                if elapsed > 3.0:
                    self.transition_to('SEARCH_MARKER')
                    return

        # ----------------------------------------------------------------------
        # STATE: STAGING_SETTLE
        # ----------------------------------------------------------------------
        elif self.state == 'STAGING_SETTLE':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            if elapsed < 0.2:
                pass
            else:
                e_x, e_y, e_theta = self.compute_docking_errors()
                if e_y is not None and e_x is not None:
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

                    self.target_carrot_dist = max(0.12, dist_to_c)

                    curr_yaw = self.get_robot_yaw()
                    if curr_yaw is not None:
                        self.latched_aim_target_yaw = math.atan2(
                            math.sin(curr_yaw + self.latched_aim_delta_yaw),
                            math.cos(curr_yaw + self.latched_aim_delta_yaw)
                        )
                    else:
                        self.latched_aim_target_yaw = None

                    self.get_logger().info(
                        f">>> [STAGING_SETTLE] Settling complete! dist={self.desk_distance*100:.1f}cm, ey={e_y*100:+.1f}cm, yaw={math.degrees(e_theta):+.1f}°. "
                        f"Carrot dist: {self.target_carrot_dist*100:.1f}cm, Aim Delta: {math.degrees(self.latched_aim_delta_yaw):+.1f}°. Transitioning to STAGING_AIM..."
                    )
                    self._report_gui(
                        f"📍 [대기점 정지 완료] 마커 거리: {self.desk_distance*100:.1f}cm, 편차: {e_y*100:+.1f}cm ➔ 가상 당근점({self.target_carrot_dist*100:.0f}cm) 조준 시작"
                    )
                    self.transition_to('STAGING_AIM')
                    self._publish_carrot_marker()
                    return
                else:
                    if elapsed > 3.5:
                        self.get_logger().warn(">>> [STAGING_SETTLE] Marker lost while settling. -> SEARCH_MARKER")
                        self.transition_to('SEARCH_MARKER')
                        return

        # ----------------------------------------------------------------------
        # STATE: STAGING_AIM
        # ----------------------------------------------------------------------
        elif self.state == 'STAGING_AIM':
            twist.linear.x = 0.0
            curr_yaw = self.get_robot_yaw()
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            if self.latched_aim_target_yaw is not None and curr_yaw is not None:
                aim_err = math.atan2(
                    math.sin(self.latched_aim_target_yaw - curr_yaw),
                    math.cos(self.latched_aim_target_yaw - curr_yaw)
                )
            else:
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
                else:
                    aim_err = math.atan2(e_y, max(0.15, d_ref - L))

            if abs(aim_err) <= math.radians(2.0) or elapsed > 10.0:
                self.stop_robot()
                odom_p = self.get_robot_pose_in_odom()
                self.carrot_start_odom_pose = odom_p if odom_p is not None else (0.0, 0.0, 0.0)
                self.carrot_locked_yaw = curr_yaw if curr_yaw is not None else self.latched_aim_target_yaw
                self.get_logger().info(
                    f">>> [STAGING_AIM] Aim completed (aim_err={math.degrees(aim_err):+.1f}°). "
                    f"Latched Carrot Dist: {self.target_carrot_dist*100:.1f}cm. -> CARROT_ALIGN"
                )
                self._report_gui(f"✔ [당근점 조준 완료] 조준 오차: {math.degrees(aim_err):.1f}° ➔ 당근점 직선 주행 시작({self.target_carrot_dist*100:.0f}cm)")
                self.transition_to('CARROT_ALIGN')
                return
            else:
                w_z = math.copysign(min(0.18, max(0.05, 0.70 * abs(aim_err))), aim_err)
                twist.angular.z = float(w_z)

        # ----------------------------------------------------------------------
        # STATE: CARROT_ALIGN (Straight drive toward latched carrot point)
        # ----------------------------------------------------------------------
        elif self.state == 'CARROT_ALIGN':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            # 1. 이동 거리 계산 (출발 오도메트리 기준)
            curr_odom_p = self.get_robot_pose_in_odom()
            if curr_odom_p is not None and self.carrot_start_odom_pose is not None:
                dist_traveled = math.hypot(
                    curr_odom_p[0] - self.carrot_start_odom_pose[0],
                    curr_odom_p[1] - self.carrot_start_odom_pose[1]
                )
            else:
                dist_traveled = 0.050 * elapsed

            # 2. 도착 판정 (기억한 당근점 거리 도달 or 마커 앞 38cm 스탠드오프 존 진입)
            reached_target = (dist_traveled >= max(0.10, self.target_carrot_dist))
            reached_standoff_dist = (self.desk_distance is not None and self.desk_distance <= 0.38)

            if reached_target or reached_standoff_dist:
                self.stop_robot()
                self.dual_align_stable_count = 0
                self.get_logger().info(
                    f">>> [CARROT_ALIGN] Standoff reached (traveled={dist_traveled*100:.1f}cm/{self.target_carrot_dist*100:.1f}cm, "
                    f"desk={f'{self.desk_distance*100:.1f}cm' if self.desk_distance is not None else 'N/A'}). -> STANDOFF_SETTLE"
                )
                self._report_gui(f"✔ [당근점 직선 도착] 이동거리 {dist_traveled*100:.0f}cm ➔ 마커 정면 회전 및 정밀 오차 검증 시작")
                self.transition_to('STANDOFF_SETTLE')
                return

            # 타임아웃 안전 가드
            if elapsed > 30.0:
                self.get_logger().warn(f">>> [CARROT_ALIGN] Straight drive timeout ({elapsed:.1f}s) -> STANDOFF_SETTLE")
                self.stop_robot()
                self.dual_align_stable_count = 0
                self.transition_to('STANDOFF_SETTLE')
                return

            # 3. 속도 계산: 흔들림 없는 직선 주행 및 헤딩 락
            rem_dist = max(0.0, self.target_carrot_dist - dist_traveled)
            ratio = float(np.clip(rem_dist / max(0.10, self.target_carrot_dist), 0.0, 1.0))
            v_x = float(0.025 + 0.025 * ratio)  # 0.050 m/s -> 0.025 m/s 부드러운 감속

            # 헤딩 락: 조준해둔 당근점 방향 각도를 정밀 유지 (미세 자이로 P 제어)
            curr_yaw = self.get_robot_yaw()
            if curr_yaw is not None and self.carrot_locked_yaw is not None:
                yaw_err = math.atan2(
                    math.sin(self.carrot_locked_yaw - curr_yaw),
                    math.cos(self.carrot_locked_yaw - curr_yaw)
                )
                w_z = float(np.clip(1.2 * yaw_err, -0.10, 0.10))
            else:
                w_z = 0.0

            twist.linear.x = v_x
            twist.angular.z = w_z
            self._publish_carrot_marker()
            self.get_logger().info(
                f">>> [CARROT_ALIGN Straight] Traveled: {dist_traveled*100:.1f}/{self.target_carrot_dist*100:.1f}cm, "
                f"vx={v_x*100:.1f}cm/s, wz={w_z:+.2f}rad/s",
                throttle_duration_sec=0.8
            )

        # ----------------------------------------------------------------------
        # STATE: STANDOFF_SETTLE
        # ----------------------------------------------------------------------
        elif self.state == 'STANDOFF_SETTLE':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            twist.linear.x = 0.0

            e_x, e_y, e_theta = self.compute_docking_errors()
            if e_theta is None or e_y is None:
                twist.angular.z = 0.0
                if elapsed > 3.5:
                    self.transition_to('SEARCH_MARKER')
                return

            # Step 1: Fine rotate heading to marker normal
            if self.settle_substate == 'FINE_ROTATE':
                rotate_elapsed = (now - self.settle_rotate_start_time).nanoseconds / 1e9 if self.settle_rotate_start_time else elapsed
                if abs(e_theta) > math.radians(3.5) and rotate_elapsed < 2.5:
                    # Positive feedback alignment: e_theta < 0 -> rotate CW (-wz), e_theta > 0 -> rotate CCW (+wz)
                    w_z = math.copysign(min(0.18, max(0.05, 0.60 * abs(e_theta))), e_theta)
                    twist.angular.z = float(w_z)
                else:
                    twist.angular.z = 0.0
                    self.stop_robot()
                    self.settle_substate = 'STATIC_WAIT'
                    self.settle_stop_start_time = now
                    self.dual_align_stable_count = 0
                    return

            # Step 2: Stationary halt (>= 0.3s) & verify static error
            elif self.settle_substate == 'STATIC_WAIT':
                twist.angular.z = 0.0
                twist.linear.x = 0.0

                if self.settle_stop_start_time is None:
                    self.settle_stop_start_time = now
                    return

                stopped_elapsed = (now - self.settle_stop_start_time).nanoseconds / 1e9
                if stopped_elapsed < 0.3:
                    return

                aligned_ok = (abs(e_y) <= 0.028 and abs(e_theta) <= math.radians(3.5))
                if aligned_ok:
                    self.dual_align_stable_count += 1
                    if self.dual_align_stable_count >= 3:
                        self.locked_docking_yaw = self.get_robot_yaw()
                        self.stop_robot()
                        self._report_gui(f"✅ [정적 검증 통과] 편차: {e_y*100:+.1f}cm, 각도: {math.degrees(e_theta):+.1f}° ➔ 최종 크롤링 개시!")
                        self.transition_to('CRAWL_CONTACT')
                        return
                else:
                    self.dual_align_stable_count = 0

                if stopped_elapsed > 2.0:
                    self.backup_reason = f"정지 계측 오차 초과 (ey={e_y*100:+.1f}cm, yaw={math.degrees(e_theta):+.1f}°)"
                    self.get_logger().warn(f">>> [STANDOFF_SETTLE] {self.backup_reason} -> BACKUP_STANDOFF")
                    self.stop_robot()
                    self._report_gui(f"⚠️ [정적 검증 실패] {self.backup_reason} ➔ 안전 후진 재정렬", is_info=False)
                    self.transition_to('BACKUP_STANDOFF')
                    return

        # ----------------------------------------------------------------------
        # STATE: CRAWL_CONTACT
        # ----------------------------------------------------------------------
        elif self.state == 'CRAWL_CONTACT':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > self.crawl_timeout_sec:
                self.get_logger().warn(f">>> [CRAWL_CONTACT] {self.crawl_timeout_sec:.1f}s timeout -> BACKUP_RETRY")
                self._report_gui(f"⏱️ [크롤링 타임아웃] {self.crawl_timeout_sec:.0f}초간 범퍼 미접촉 ➔ 후진 재시도", is_info=False)
                self.transition_to('BACKUP_RETRY')
                return

            twist.linear.x = max(0.045, self.crawl_linear_speed)
            curr_yaw = self.get_robot_yaw()
            if curr_yaw is not None and self.locked_docking_yaw is not None:
                yaw_diff = math.atan2(math.sin(self.locked_docking_yaw - curr_yaw), math.cos(self.locked_docking_yaw - curr_yaw))
                twist.angular.z = float(np.clip(1.2 * yaw_diff, -0.12, 0.12))
            else:
                twist.angular.z = 0.0

        # ----------------------------------------------------------------------
        # STATE: DOCKED
        # ----------------------------------------------------------------------
        elif self.state == 'DOCKED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.stop_robot()
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > 2.0:
                # Return to NAV2_READY after 2 seconds to release cmd_vel
                self.transition_to('NAV2_READY')
            return

        # ----------------------------------------------------------------------
        # STATE: BACKUP_STANDOFF (Virtual Reverse Carrot)
        # ----------------------------------------------------------------------
        elif self.state in ('BACKUP_STANDOFF', 'BACKUP_PREPARE', 'BACKUP_ALIGN_PREPARE'):
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            if self.backup_substate == 'BACK_AIM':
                twist.linear.x = 0.0
                curr_yaw = self.get_robot_yaw()
                yaw_err = (
                    math.atan2(math.sin(self.reverse_latched_aim_yaw - curr_yaw), math.cos(self.reverse_latched_aim_yaw - curr_yaw))
                    if (self.reverse_latched_aim_yaw is not None and curr_yaw is not None)
                    else 0.0
                )

                if abs(yaw_err) <= math.radians(2.0) or elapsed > 2.5:
                    self.stop_robot()
                    odom_p = self.get_robot_pose_in_odom()
                    self.reverse_start_odom_pose = odom_p if odom_p is not None else (0.0, 0.0, 0.0)
                    self.backup_substate = 'BACK_REVERSE'
                    self.state_start_time = now
                    return
                else:
                    twist.angular.z = float(math.copysign(min(0.20, max(0.06, 0.70 * abs(yaw_err))), yaw_err))

            elif self.backup_substate == 'BACK_REVERSE':
                twist.linear.x = -0.065
                twist.angular.z = 0.0
                curr_odom_p = self.get_robot_pose_in_odom()
                dist_traveled = (
                    math.hypot(curr_odom_p[0] - self.reverse_start_odom_pose[0], curr_odom_p[1] - self.reverse_start_odom_pose[1])
                    if (curr_odom_p is not None and self.reverse_start_odom_pose is not None)
                    else 0.065 * elapsed
                )

                if dist_traveled >= self.reverse_target_distance or elapsed > 3.2:
                    self.stop_robot()
                    self.backup_substate = 'BACK_REALIGN'
                    self.state_start_time = now
                    return

            elif self.backup_substate == 'BACK_REALIGN':
                twist.linear.x = 0.0
                curr_yaw = self.get_robot_yaw()
                realign_err = (
                    math.atan2(math.sin(self.reverse_latched_origin_yaw - curr_yaw), math.cos(self.reverse_latched_origin_yaw - curr_yaw))
                    if (self.reverse_latched_origin_yaw is not None and curr_yaw is not None)
                    else 0.0
                )

                if abs(realign_err) <= math.radians(2.0) or elapsed > 2.5:
                    self.stop_robot()
                    self.transition_to('STAGING_SETTLE')
                    return
                else:
                    twist.angular.z = float(math.copysign(min(0.20, max(0.06, 0.70 * abs(realign_err))), realign_err))

        # ----------------------------------------------------------------------
        # STATE: BACKUP_RETRY
        # ----------------------------------------------------------------------
        elif self.state == 'BACKUP_RETRY':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed < 2.5:
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            else:
                self.stop_robot()
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    self.get_logger().error(f">>> [FAILED] Exceeded maximum retries ({self.max_retries}). Aborting to NAV2_READY...")
                    self._report_gui("❌ [도킹 실패] 최대 재시도 횟수 초과 ➔ 대기(NAV2_READY) 전환", is_info=False)
                    self.transition_to('NAV2_READY')
                else:
                    self.get_logger().info(f">>> [RETRY] Backup complete. Re-attempting marker search (Retry {self.retry_count}/{self.max_retries})...")
                    self.transition_to('SEARCH_MARKER')
                return

        # Publish motor velocity if in active docking state
        if self.state not in ('NAV2_READY', 'FAILSAFE', 'DOCKED'):
            self._send_cmd_vel(twist.linear.x, twist.angular.z)

    # ==========================================================================
    # State Transitions & Motor Commands
    # ==========================================================================
    def transition_to(self, new_state: str):
        old_state = self.state
        self.get_logger().info(f"[FSM] Transition: {old_state} -> {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        now_sec = time.time()
        self.last_progress_time = now_sec
        if self.desk_distance is not None:
            self.min_tracked_distance = self.desk_distance

        # State entrance triggers
        if new_state == 'INIT':
            self.docking_routine_start_time = now_sec
            self.retry_count = 0
            self.initial_ey = None
            self.contact_detected = False
            self.contact_latched = False

        elif new_state == 'CRAWL_CONTACT':
            # Reset global routine timer on final crawl to ensure full 35s execution
            self.docking_routine_start_time = now_sec
            self.last_progress_time = now_sec
            if self.desk_distance is not None:
                self.min_tracked_distance = self.desk_distance
            self.get_logger().info(
                f">>> [CRAWL_CONTACT] Direct forward crawling engaged. Reset global timer & allocated {self.crawl_timeout_sec:.1f}s dedicated window."
            )

        elif new_state == 'STANDOFF_SETTLE':
            self.settle_substate = 'FINE_ROTATE'
            self.settle_rotate_start_time = self.get_clock().now()
            self.settle_stop_start_time = None
            self.dual_align_stable_count = 0

        elif new_state in ('BACKUP_STANDOFF', 'BACKUP_PREPARE', 'BACKUP_ALIGN_PREPARE'):
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.get_logger().error(f">>> [FAILED] Exceeded maximum backup retries ({self.max_retries}).")
                self._report_gui("❌ [도킹 실패] 최대 재정렬 재시도 초과 ➔ 대기(NAV2_READY) 전환", is_info=False)
                self.stop_robot()
                self.state = 'NAV2_READY'
                return

            self.backup_substate = 'BACK_AIM'
            self.reverse_start_odom_pose = None
            e_x, e_y, e_theta = self.compute_docking_errors()
            curr_yaw = self.get_robot_yaw()
            self.reverse_latched_origin_yaw = curr_yaw if curr_yaw is not None else 0.0

            if e_y is not None:
                d_rev = float(np.clip(abs(e_y) * 1.5, 0.12, 0.22))
                self.reverse_target_distance = d_rev
                aim_delta = math.atan2(-e_y, d_rev)
                self.reverse_latched_aim_yaw = math.atan2(
                    math.sin(self.reverse_latched_origin_yaw + aim_delta),
                    math.cos(self.reverse_latched_origin_yaw + aim_delta)
                )
            else:
                self.reverse_target_distance = 0.16
                self.reverse_latched_aim_yaw = self.reverse_latched_origin_yaw

    def stop_robot(self):
        self._send_cmd_vel(0.0, 0.0)

    def _send_cmd_vel(self, vx: float, wz: float):
        if self.enable_stamped_cmd_vel:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = 'base_footprint'
            stamped.twist.linear.x = float(vx)
            stamped.twist.angular.z = float(wz)
            self.cmd_vel_pub.publish(stamped)
        else:
            t = Twist()
            t.linear.x = float(vx)
            t.angular.z = float(wz)
            self.cmd_vel_pub.publish(t)

        raw = Twist()
        raw.linear.x = float(vx)
        raw.angular.z = float(wz)
        self.cmd_vel_unstamped_pub.publish(raw)
        self.cmd_vel_nav_pub.publish(raw)

    # ==========================================================================
    # Service Handlers & Station Integration
    # ==========================================================================
    def _on_start_station(self, msg: String):
        station_name = msg.data.strip()
        self.get_logger().info(f"📥 Received station dock request for: '{station_name}'")
        self.start_station_docking(station_name)

    def _srv_start_docking(self, request, response):
        self.transition_to('INIT')
        response.success = True
        response.message = "Autonomous marker precision docking initiated"
        return response

    def _srv_abort_docking(self, request, response):
        self.get_logger().warn("🛑 Emergency Abort requested!")
        self.stop_robot()
        self._report_gui("🛑 [도킹 중단] 사용자 요청으로 정밀 도킹이 중단되었습니다 ➔ 대기(NAV2_READY) 전환", is_info=False)
        self.transition_to('NAV2_READY')
        response.success = True
        response.message = "Precision docking aborted and returned to NAV2_READY"
        return response

    def _srv_confirm_docking(self, request, response):
        response.success = True
        response.message = "Auto-confirmed"
        return response

    def _srv_undock_to_nav2(self, request, response):
        self.stop_robot()
        self.transition_to('NAV2_READY')
        response.success = True
        response.message = "Undocked to NAV2_READY"
        return response

    def _srv_emergency_stop(self, request, response):
        self.stop_robot()
        self.transition_to('NAV2_READY')
        response.success = True
        response.message = "Emergency stop executed"
        return response

    def start_station_docking(self, station_name: str) -> bool:
        self.active_station_name = station_name
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"🎯 [정밀 도킹 개시] 대상: '{station_name}' (ID:{self.marker_id})")
        self.get_logger().info("=" * 70)
        self._report_gui(f"🎯 [정밀 도킹 개시] 대상: '{station_name}' ➔ 전방 ArUco 마커(ID:{self.marker_id}) 검출 여부 확인")
        self.transition_to('INIT')
        return True

    # ==========================================================================
    # RViz Marker Visualizations
    # ==========================================================================
    def _publish_carrot_marker(self):
        if self.carrot_x_b is None or self.carrot_y_b is None:
            return
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "virtual_carrot"
        m.id = 999
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.carrot_x_b)
        m.pose.position.y = float(self.carrot_y_b)
        m.pose.position.z = 0.05
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 0.9
        self.carrot_marker_pub.publish(m)

    def _publish_marker_debug(self, p_base: np.ndarray, desk_normal: np.ndarray):
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aruco_docking'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.015
        m.scale.y = 0.030
        m.scale.z = 0.040
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.2
        m.color.a = 1.0

        start_pt = Point(x=float(p_base[0]), y=float(p_base[1]), z=float(p_base[2]))
        end_pt = Point(
            x=float(p_base[0] + 0.15 * desk_normal[0]),
            y=float(p_base[1] + 0.15 * desk_normal[1]),
            z=float(p_base[2] + 0.15 * desk_normal[2])
        )
        m.points = [start_pt, end_pt]
        self.marker_pub.publish(m)

    def _publish_gizmo_markers(self):
        if hasattr(self.station_mgr, 'get_all_stations'):
            stations = self.station_mgr.get_all_stations()
        else:
            stations = getattr(self.station_mgr, 'stations', {})
        if not stations:
            return
        ma = MarkerArray()
        st_idx = 0
        for st_name, st_data in stations.items():
            face_data = st_data.get('target_face') or st_data.get('target_coordinate') or {}
            center = face_data.get('center')
            size = face_data.get('size', [0.215, 0.36, 0.225])
            if not center:
                continue

            frame_id = st_data.get('frame', 'map')
            normal = face_data.get('normal', [-1.0, 0.0, 0.0])
            st_yaw = math.atan2(float(normal[1]), float(normal[0]))
            cy = math.cos(st_yaw * 0.5)
            sy = math.sin(st_yaw * 0.5)
            ori_q = Quaternion(x=0.0, y=0.0, z=sy, w=cy)

            # 1. 3D Bounding Box with True Taught Orientation
            s_box = Marker()
            s_box.header.frame_id = frame_id
            s_box.header.stamp = self.get_clock().now().to_msg()
            s_box.ns = "gizmo_preview"
            s_box.id = st_idx * 10
            s_box.type = Marker.CUBE
            s_box.action = Marker.ADD
            s_box.pose.position.x = float(center[0])
            s_box.pose.position.y = float(center[1])
            s_box.pose.position.z = float(center[2])
            s_box.pose.orientation = ori_q
            s_box.scale.x = float(size[0])
            s_box.scale.y = float(size[1])
            s_box.scale.z = float(size[2])

            if self.active_station_name == st_name and self.state in DOCKING_ACTIVE_STATES:
                s_box.color.r = 1.0; s_box.color.g = 0.6; s_box.color.b = 0.0; s_box.color.a = 0.75
            elif self.active_station_name == st_name and self.state == 'DOCKED':
                s_box.color.r = 0.0; s_box.color.g = 1.0; s_box.color.b = 0.2; s_box.color.a = 0.85
            else:
                s_box.color.r = 0.0; s_box.color.g = 0.8; s_box.color.b = 1.0; s_box.color.a = 0.55
            ma.markers.append(s_box)

            # 2. Front Approach Arrow Marker
            s_arrow = Marker()
            s_arrow.header.frame_id = frame_id
            s_arrow.header.stamp = self.get_clock().now().to_msg()
            s_arrow.ns = "gizmo_preview"
            s_arrow.id = st_idx * 10 + 1
            s_arrow.type = Marker.ARROW
            s_arrow.action = Marker.ADD
            s_arrow.pose.position.x = float(center[0])
            s_arrow.pose.position.y = float(center[1])
            s_arrow.pose.position.z = float(center[2])
            s_arrow.pose.orientation = ori_q
            s_arrow.scale = Vector3(x=float(size[0] / 2.0 + 0.25), y=0.04, z=0.04)
            s_arrow.color.r = 1.0; s_arrow.color.g = 0.2; s_arrow.color.b = 0.1; s_arrow.color.a = 0.85
            ma.markers.append(s_arrow)

            # 3. Clean Text Label directly centered above the box
            s_text = Marker()
            s_text.header.frame_id = frame_id
            s_text.header.stamp = self.get_clock().now().to_msg()
            s_text.ns = "gizmo_preview"
            s_text.id = st_idx * 10 + 2
            s_text.type = Marker.TEXT_VIEW_FACING
            s_text.action = Marker.ADD
            s_text.pose.position.x = float(center[0])
            s_text.pose.position.y = float(center[1])
            s_text.pose.position.z = float(center[2] + size[2] / 2.0 + 0.15)
            s_text.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            s_text.scale.z = 0.12
            s_text.text = f"📍 {st_name}"
            s_text.color.r = 0.0; s_text.color.g = 0.95; s_text.color.b = 1.0; s_text.color.a = 0.95
            ma.markers.append(s_text)
            st_idx += 1

        self.gizmo_markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionDockingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
