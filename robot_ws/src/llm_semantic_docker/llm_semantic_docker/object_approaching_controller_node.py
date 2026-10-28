#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# YOLO Dual-Engine Floor Object Approaching Controller Node
# - GPU-accelerated YOLO Detection (Custom Fine-tuned 1st, YOLO-World Open-Vocab 2nd)
# - Dynamic Model Introspection: Reads model.names automatically (Zero-code expansion)
# - Declarative YAML registry (floor_target_objects.yaml)
# - 20Hz Real-Time Closed-Loop Servoing (Latency: 7ms vs Qwen's 2500ms)
# - Odometry Spatial Memory: 1.5s Dead-Reckoning against far-distance flickering
# - Hybrid Depth Fusion: Floor Pinhole Geometry (>1.2m) + Astra S IR 3D Depth (<1.0m)
# - Post-Approach: Safely halts motors & releases bus for robotic arm manipulation
# ==============================================================================

import os
import sys
import time
import math
import json
import yaml
from typing import Optional, Tuple, List, Dict, Any
import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO, YOLOWorld
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


APPROACH_ACTIVE_STATES = (
    'SEARCH_ALIGN', 'APPROACH_OBJECT', 'FINAL_SETTLE'
)

CUSTOM_WEIGHTS_PATH = "/home/jjj/Documents/Dapier/Project/custom_floor_detector_best.pt"
YOLO_WORLD_WEIGHTS = "/home/jjj/Documents/Dapier/Project/yolov8s-worldv2.pt"
TARGET_CONFIG_PATH = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/floor_target_objects.yaml"
CALIB_CONFIG_PATH = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/camera_pitch_calib.yaml"


class ObjectApproachingControllerNode(Node):
    def __init__(self):
        super().__init__('object_approaching_controller_node')

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('enable_stamped_cmd_vel', True)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('cam_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('bumper_topic', '/robot/bumper/contact')

        self.declare_parameter('target_standoff_m', 0.25)     # Default standoff distance (25cm)
        self.declare_parameter('default_query', 'red_cube')
        self.declare_parameter('conf_threshold', 0.20)

        # Astra S Extrinsics relative to base_footprint
        self.declare_parameter('astra_x_base', 0.080)
        self.declare_parameter('astra_y_base', 0.015)         # +1.5cm left offset
        self.declare_parameter('astra_z_base', 0.147)         # 14.7cm lens height from floor
        self.declare_parameter('astra_pitch', 0.0)

        self.declare_parameter('max_linear_speed', 0.16)
        self.declare_parameter('crawl_linear_speed', 0.040)
        self.declare_parameter('max_angular_speed', 0.22)
        self.declare_parameter('approach_timeout_sec', 90.0)

        self.enable_stamped_cmd_vel = bool(self.get_parameter('enable_stamped_cmd_vel').value)
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.cam_info_topic = str(self.get_parameter('cam_info_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.bumper_topic = str(self.get_parameter('bumper_topic').value)

        self.target_standoff_m = float(self.get_parameter('target_standoff_m').value)
        self.target_query = str(self.get_parameter('default_query').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.astra_x_base = float(self.get_parameter('astra_x_base').value)
        self.astra_y_base = float(self.get_parameter('astra_y_base').value)
        self.astra_z_base = float(self.get_parameter('astra_z_base').value)
        self.astra_pitch = float(self.get_parameter('astra_pitch').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.crawl_linear_speed = float(self.get_parameter('crawl_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.approach_timeout_sec = float(self.get_parameter('approach_timeout_sec').value)

        # Camera Intrinsics (Defaults tailored for Astra S 320x240 resolution)
        self.fx = 285.17
        self.fy = 285.17
        self.cx = 159.5
        self.cy = 119.5
        self.camera_info_received = False

        # Live Image Buffers
        self.bridge = CvBridge()
        self.last_color_img: Optional[np.ndarray] = None
        self.last_color_time: float = 0.0
        self.last_depth_raw: Optional[np.ndarray] = None
        self.last_depth_time: float = 0.0
        self.color_source: str = "NONE"

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # Target Registry & YAML Configuration
        # ----------------------------------------------------------------------
        self.target_profiles: Dict[str, Any] = {}
        self._load_target_profiles()
        self._load_pitch_calibration()

        # ----------------------------------------------------------------------
        # YOLO Dual-Engine Initialization
        # ----------------------------------------------------------------------
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.device_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'

        self.custom_model: Optional[YOLO] = None
        self.custom_classes: List[str] = []
        self.world_model: Optional[YOLOWorld] = None
        self.active_engine_name: str = "NONE"

        self._init_yolo_engines()

        # Tracking & Spatial Memory State
        self.obj_detected: bool = False
        self.obj_last_seen: float = 0.0
        self.tracked_bbox: Optional[Tuple[int, int, int, int]] = None
        self.detected_class_name: str = ""
        self.detected_conf: float = 0.0

        self.lens_distance: Optional[float] = None            # Current distance to lens (m)
        self.lateral_error: Optional[float] = None            # Lateral error relative to heading (m)
        self.heading_error: Optional[float] = None            # Yaw error (rad)
        self.depth_source_mode: str = "NONE"                  # 'IR 3D' vs 'GEOM'

        # Odom Spatial Memory (Latched coordinate in odom frame against flickering)
        self.odom_target_latched: bool = False
        self.odom_target_pos: Optional[np.ndarray] = None     # [x, y, z] in odom frame
        self.odom_target_time: float = 0.0

        # FSM State
        self.state: str = 'NAV2_READY'
        self.state_start_time = self.get_clock().now()
        self.routine_start_time: Optional[float] = None
        self.settle_stable_count: int = 0
        self.target_lock_count: int = 0        # 연속 감지 신뢰도 카운터 (최소 3프레임 락온)
        self.align_stable_count: int = 0       # 조향 정렬 안정화 카운터 (최소 2프레임 확인)

        # Stats
        self.fps: float = 0.0
        self.yolo_latency_ms: float = 0.0
        self._frame_count: int = 0
        self._fps_start_time: float = time.time()

        # ----------------------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------------------
        if self.enable_stamped_cmd_vel:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.status_pub = self.create_publisher(String, '/object_approach/status', 10)
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/docking/target_object_pose', 10)
        self.debug_comp_pub = self.create_publisher(CompressedImage, '/docking/object_debug_image/compressed', 10)
        self.detected_event_pub = self.create_publisher(String, '/floor_object/detected_event', 10)

        # On-the-fly detection event tracking
        self._consecutive_detect_count: int = 0
        self._last_event_pub_time: float = 0.0

        # QoS for sensor streams
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(CameraInfo, self.cam_info_topic, self._on_cam_info, 10)
        self.create_subscription(Image, '/camera/color/image_decompressed', self._on_color_decomp, sensor_qos)
        self.create_subscription(Image, self.color_topic, self._on_color_raw, sensor_qos)
        self.create_subscription(CompressedImage, f"{self.color_topic}/compressed", self._on_color_comp, sensor_qos)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._on_color_comp, sensor_qos)

        self.create_subscription(Image, '/camera/depth/image_decompressed', self._on_depth_raw, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._on_depth_raw, sensor_qos)
        self.create_subscription(CompressedImage, f"{self.depth_topic}/compressedDepth", self._on_depth_comp, sensor_qos)
        self.create_subscription(CompressedImage, '/camera/depth/image_raw/compressedDepth', self._on_depth_comp, sensor_qos)

        self.create_subscription(String, '/object_approach/target_query', self._on_target_query_cfg, 10)
        self.create_subscription(String, '/object_approach/target_standoff', self._on_standoff_cfg, 10)
        self.create_subscription(Bool, self.bumper_topic, self._on_bumper, 10)
        self.create_subscription(Bool, '/safety/software_stop', self._on_software_stop, 10)

        # Services
        self.create_service(Trigger, '/object_approach/start', self._srv_start)
        self.create_service(Trigger, '/object_approach/abort', self._srv_abort)

        # 20Hz Main Control Loop (50ms interval)
        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info("=" * 70)
        self.get_logger().info("🚀 [ObjectApproachingController] YOLO 3D Real-Time Servoing Ready!")
        self.get_logger().info(f"   Active Target: '{self.target_query}', Standoff: {self.target_standoff_m*100:.1f}cm")
        self.get_logger().info(f"   Custom Model Classes: {self.custom_classes}")
        self.get_logger().info("=" * 70)

    # ==========================================================================
    # Initialization Helpers
    # ==========================================================================
    def _load_target_profiles(self):
        """Loads floor_target_objects.yaml registry if present"""
        if os.path.exists(TARGET_CONFIG_PATH):
            try:
                with open(TARGET_CONFIG_PATH, "r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f) or {}
                    self.target_profiles = cfg.get("targets", {})
                    self.get_logger().info(f"📄 Target profiles loaded ({len(self.target_profiles)} targets configured)")
            except Exception as e:
                self.get_logger().warn(f"Failed to load target config: {e}")

    def _load_pitch_calibration(self):
        """Loads camera_pitch_calib.yaml if available to compensate for physical camera tilt and height"""
        if os.path.exists(CALIB_CONFIG_PATH):
            try:
                with open(CALIB_CONFIG_PATH, "r", encoding="utf-8") as f:
                    calib_data = yaml.safe_load(f) or {}
                    calib_pitch = calib_data.get("astra_pitch_deg")
                    if calib_pitch is not None:
                        self.astra_pitch = float(calib_pitch)
                        self.get_logger().info(
                            f"📐 [Pitch 캘리브레이션 적용] astra_pitch: {self.astra_pitch:+.2f}° from {CALIB_CONFIG_PATH}"
                        )
                    calib_z = calib_data.get("astra_z_base_m")
                    if calib_z is not None:
                        self.astra_z_base = float(calib_z)
                        self.get_logger().info(
                            f"📏 [카메라 높이 적용] astra_z_base: {self.astra_z_base*100:.1f}cm from {CALIB_CONFIG_PATH}"
                        )
            except Exception as e:
                self.get_logger().warn(f"Failed to load pitch calib: {e}")

    def _init_yolo_engines(self):
        """Initializes Custom YOLO and YOLO-World with GPU warmup"""
        if not YOLO_AVAILABLE:
            self.get_logger().error("Ultralytics YOLO not installed!")
            return

        # 1. Custom fine-tuned model (Priority 1)
        if os.path.exists(CUSTOM_WEIGHTS_PATH):
            try:
                t0 = time.time()
                self.custom_model = YOLO(CUSTOM_WEIGHTS_PATH)
                self.custom_classes = list(self.custom_model.names.values())
                # Warmup
                dummy = np.zeros((240, 320, 3), dtype=np.uint8)
                self.custom_model.predict(dummy, device=self.device, verbose=False)
                self.get_logger().info(f"✅ Custom YOLO Warmup Complete in {(time.time()-t0)*1000:.1f}ms: {self.custom_classes}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load custom YOLO model: {e}")

        # 2. YOLO-World model (Priority 2, Open-Vocabulary Fallback)
        if os.path.exists(YOLO_WORLD_WEIGHTS):
            try:
                t0 = time.time()
                self.world_model = YOLOWorld(YOLO_WORLD_WEIGHTS)
                self.world_model.set_classes(["box", "cube", "cup", "bottle", "ball"])
                dummy = np.zeros((240, 320, 3), dtype=np.uint8)
                self.world_model.predict(dummy, device=self.device, verbose=False)
                self.get_logger().info(f"✅ YOLO-World Warmup Complete in {(time.time()-t0)*1000:.1f}ms")
            except Exception as e:
                self.get_logger().warn(f"Failed to load YOLO-World model: {e}")

    # ==========================================================================
    # Callbacks
    # ==========================================================================
    def _report_gui(self, text: str, is_info: bool = True):
        prefix = "INFO:" if is_info else "ALERT:"
        self.get_logger().info(f"[OBJ-APPROACH] {text}")
        self.task_status_pub.publish(String(data=f"{prefix} {text}"))

    def _on_target_query_cfg(self, msg: String):
        q = msg.data.strip()
        if q:
            self.target_query = q
            self._apply_target_profile(q)

    def _apply_target_profile(self, query: str):
        # Resolve aliases from YAML
        matched_id = query
        for tgt_id, info in self.target_profiles.items():
            aliases = info.get("aliases", [])
            if query.lower() == tgt_id.lower() or query.lower() in [a.lower() for a in aliases]:
                matched_id = tgt_id
                standoff = info.get("standoff_m")
                if standoff:
                    self.target_standoff_m = float(standoff)
                break
        self.get_logger().info(f"🎯 Target set to '{matched_id}' (Query: '{query}', Standoff: {self.target_standoff_m*100:.1f}cm)")

    def _on_standoff_cfg(self, msg: String):
        try:
            val_m = float(msg.data.strip())
            if 0.15 <= val_m <= 0.60:
                self.target_standoff_m = val_m
                self.get_logger().info(f"Updated target_standoff_m to {self.target_standoff_m * 100:.1f}cm")
        except Exception:
            pass

    def _on_cam_info(self, msg: CameraInfo):
        if not self.camera_info_received and msg.k[0] > 0:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
            self.camera_info_received = True

    def _on_bumper(self, msg: Bool):
        if msg.data and self.state in APPROACH_ACTIVE_STATES:
            self.stop_robot()
            self.transition_to('APPROACHED')

    def _on_software_stop(self, msg: Bool):
        if msg.data:
            self.stop_robot()
            self.transition_to('NAV2_READY')

    def _on_color_raw(self, msg: Image):
        try:
            self.last_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_color_time = time.time()
            self.color_source = "RAW"
        except Exception:
            pass

    def _on_color_decomp(self, msg: Image):
        try:
            self.last_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_color_time = time.time()
            self.color_source = "DECOMPRESSED"
        except Exception:
            pass

    def _on_color_comp(self, msg: CompressedImage):
        if time.time() - self.last_color_time > 0.35:
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                im = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if im is not None:
                    self.last_color_img = im
                    self.last_color_time = time.time()
                    self.color_source = "COMPRESSED"
            except Exception:
                pass

    def _on_depth_raw(self, msg: Image):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth_img is not None:
                d = depth_img.astype(np.float32)
                if depth_img.dtype == np.uint16 or np.nanmax(d) > 10.0:
                    d /= 1000.0  # mm -> m
                self.last_depth_raw = d
                self.last_depth_time = time.time()
        except Exception:
            pass

    def _on_depth_comp(self, msg: CompressedImage):
        try:
            raw_bytes = bytes(msg.data)
            if raw_bytes.startswith(b'\x89PNG'):
                np_arr = np.frombuffer(raw_bytes, np.uint8)
            elif len(raw_bytes) > 12 and raw_bytes[12:].startswith(b'\x89PNG'):
                np_arr = np.frombuffer(raw_bytes[12:], np.uint8)
            else:
                np_arr = np.frombuffer(raw_bytes, np.uint8)

            depth_png = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth_png is not None:
                self.last_depth_raw = depth_png.astype(np.float32) / 1000.0
                self.last_depth_time = time.time()
        except Exception:
            pass

    # ==========================================================================
    # Real-Time YOLO Dual-Engine Inference & 3D Tracking (20Hz)
    # ==========================================================================
    def _run_yolo_perception(self) -> Optional[Dict[str, Any]]:
        if self.last_color_img is None or (time.time() - self.last_color_time) > 1.5:
            return None

        bgr = self.last_color_img.copy()
        H, W = bgr.shape[:2]

        # Dynamic Camera Intrinsics Auto-guard:
        # If CameraInfo is not yet received or resolution mismatches, adapt to current frame shape
        if not self.camera_info_received or abs(self.cx * 2 - W) > 5.0 or abs(self.cy * 2 - H) > 5.0:
            scale_x = W / 320.0
            scale_y = H / 240.0
            self.cx = W / 2.0
            self.cy = H / 2.0
            self.fx = 285.17 * scale_x
            self.fy = 285.17 * scale_y

        t0 = time.time()

        # Decide whether to use Custom YOLO or YOLO-World
        # During IDLE (patrol/standby): use Custom YOLO to detect ANY trained floor object!
        target_norm = self.target_query.lower().strip().replace(" ", "_")
        if self.state == "IDLE":
            use_custom = (self.custom_model is not None)
        else:
            use_custom = False
            for cls_name in self.custom_classes:
                if target_norm in cls_name.lower() or cls_name.lower() in target_norm:
                    use_custom = True
                    break

        dets = []
        if use_custom and self.custom_model is not None:
            self.active_engine_name = "CUSTOM"
            res = self.custom_model.predict(bgr, conf=self.conf_threshold, device=self.device, verbose=False)
            if res and len(res[0].boxes) > 0:
                for b in res[0].boxes:
                    cls_id = int(b.cls[0].cpu().numpy())
                    c_name = self.custom_classes[cls_id] if cls_id < len(self.custom_classes) else f"cls_{cls_id}"
                    conf = float(b.conf[0].cpu().numpy())
                    xyxy = b.xyxy[0].cpu().numpy().astype(int)
                    dets.append((c_name, conf, xyxy))
        elif self.world_model is not None:
            self.active_engine_name = "WORLD"
            # Set open vocabulary dynamically (cached to prevent 20Hz text embedding overhead)
            world_classes = [self.target_query, "box", "cube", "ball", "cup", "bottle"]
            if getattr(self, '_current_world_classes', None) != world_classes:
                try:
                    self.world_model.set_classes(world_classes)
                    self._current_world_classes = list(world_classes)
                except Exception as e:
                    self.get_logger().warn(f"Failed to set YOLO-World classes: {e}")
            res = self.world_model.predict(bgr, conf=self.conf_threshold, device=self.device, verbose=False)
            if res and len(res[0].boxes) > 0:
                for b in res[0].boxes:
                    cls_id = int(b.cls[0].cpu().numpy())
                    c_name = world_classes[cls_id] if cls_id < len(world_classes) else f"cls_{cls_id}"
                    conf = float(b.conf[0].cpu().numpy())
                    xyxy = b.xyxy[0].cpu().numpy().astype(int)
                    dets.append((c_name, conf, xyxy))

        self.yolo_latency_ms = (time.time() - t0) * 1000.0

        if not dets:
            return None

        # Filter candidates:
        # During IDLE: ALL classes in custom_classes and target_profiles are valid candidates for patrol discovery!
        if self.state == "IDLE":
            valid_target_names = {c.lower() for c in self.custom_classes}
            for t_k, p_info in self.target_profiles.items():
                valid_target_names.add(t_k.lower())
                valid_target_names.add(p_info.get("canonical_id", "").lower())
                for al in p_info.get("aliases", []):
                    valid_target_names.add(al.lower())
        else:
            valid_target_names = {target_norm, self.target_query.lower()}
            if self.target_query in self.target_profiles:
                p_info = self.target_profiles[self.target_query]
                valid_target_names.add(p_info.get("canonical_id", "").lower())
                for al in p_info.get("aliases", []):
                    valid_target_names.add(al.lower())

        matched = []
        for d in dets:
            c_lower = d[0].lower().replace(" ", "_")
            if any(vt in c_lower or c_lower in vt for vt in valid_target_names if vt):
                matched.append(d)

        if not matched:
            return None

        best_cand = max(matched, key=lambda d: d[1])

        c_name, conf, xyxy = best_cand
        x1, y1, x2, y2 = xyxy
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(W, x2), min(H, y2)
        bw, bh = x2 - x1, y2 - y1

        if bw <= 4 or bh <= 4:
            return None

        u_c = (x1 + x2) * 0.5
        v_c = (y1 + y2) * 0.5
        v_bottom = y2

        # 3D Depth Extraction (IR 3D vs Floor Pinhole vs Apparent Width Fallback)
        dist_z, is_pinhole = self._extract_depth(x1, y1, x2, y2, v_bottom, H, c_name)

        if dist_z is None:
            return None

        # Pinhole projection: Optical frame (X_cam: right, Y_cam: down, Z_cam: forward)
        x_cam = (u_c - self.cx) * dist_z / self.fx
        y_cam = (v_c - self.cy) * dist_z / self.fy

        # Robot base_footprint frame (X_base: forward, Y_base: left)
        x_base = self.astra_x_base + dist_z
        y_base = self.astra_y_base - x_cam
        heading_err = math.atan2(y_base, x_base)

        return {
            "cls_name": c_name,
            "conf": conf,
            "bbox": (x1, y1, bw, bh),
            "dist_z": dist_z,
            "x_cam": x_cam,
            "y_cam": y_cam,
            "x_base": x_base,
            "y_base": y_base,
            "heading_err": heading_err,
            "is_pinhole": is_pinhole
        }

    def _extract_depth(self, x1: int, y1: int, x2: int, y2: int, v_bottom: int, H: int, cls_name: str = "") -> Tuple[Optional[float], bool]:
        """
        Hybrid Depth Estimator:
        1. If Astra S IR depth has valid points (>0.35m, <4.5m) in central ROI -> IR 3D Depth
        2. Fallback to floor pinhole contact geometry with camera pitch compensation
        3. Fallback to apparent width geometric pinhole projection from floor_target_objects.yaml
        """
        # When bottom touches lower image boundary, ground contact point is clipped (Blind Zone)
        is_bottom_clipped = (v_bottom >= (H - 6))

        # 1. IR Depth
        if not is_bottom_clipped and self.last_depth_raw is not None and (time.time() - self.last_depth_time) < 1.0:
            depth = self.last_depth_raw
            dh, dw = depth.shape[:2]
            cx1 = max(0, int(x1 + (x2 - x1) * 0.25))
            cy1 = max(0, int(y1 + (y2 - y1) * 0.25))
            cx2 = min(dw, int(x1 + (x2 - x1) * 0.75))
            cy2 = min(dh, int(y1 + (y2 - y1) * 0.75))

            patch = depth[cy1:cy2, cx1:cx2]
            valid_pts = patch[(patch > 0.35) & (patch < 4.50) & np.isfinite(patch)]
            if valid_pts.size >= 4:
                return float(np.median(valid_pts)), False

        # 2. Fallback to Pinhole ground-contact geometry with pitch compensation
        if not is_bottom_clipped:
            pitch_rad = math.radians(self.astra_pitch)
            beta = math.atan2(v_bottom - self.cy, self.fy)
            alpha = beta + pitch_rad
            if alpha > 0.02:  # Ray dips downward towards floor
                z_pinhole = self.astra_z_base / math.tan(alpha)
                if 0.15 <= z_pinhole <= 4.0:
                    return float(z_pinhole), True

        # 3. Fallback to Apparent Bounding Box Width
        bw = max(1, x2 - x1)
        expected_w = 0.045
        if cls_name in self.target_profiles:
            expected_w = float(self.target_profiles[cls_name].get("expected_width_m", 0.045))
        z_apparent = (self.fx * expected_w) / float(bw)
        if 0.15 <= z_apparent <= 4.0:
            return float(z_apparent), True

        return None, False

    # ==========================================================================
    # Spatial Memory (Odom Dead-Reckoning)
    # ==========================================================================
    def _update_odom_memory(self, x_base: float, y_base: float):
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            # Transform target from base_footprint to odom
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)

            ox = tx + math.cos(yaw) * x_base - math.sin(yaw) * y_base
            oy = ty + math.sin(yaw) * x_base + math.cos(yaw) * y_base

            self.odom_target_pos = np.array([ox, oy, 0.0])
            self.odom_target_latched = True
            self.odom_target_time = time.time()
        except Exception:
            pass

    def _get_odom_distance(self) -> Optional[Tuple[float, float, float]]:
        """Returns (dist_z, x_base, y_base) from latched odom target"""
        if not self.odom_target_latched or self.odom_target_pos is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', 'odom',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)

            ox, oy = self.odom_target_pos[0], self.odom_target_pos[1]
            bx = tx + math.cos(yaw) * ox - math.sin(yaw) * oy
            by = ty + math.sin(yaw) * ox + math.cos(yaw) * oy

            # Lens distance
            dist_z = max(0.00, bx - self.astra_x_base)
            return dist_z, bx, by
        except Exception:
            return None

    # ==========================================================================
    # 20Hz Main Control Loop
    # ==========================================================================
    def _control_loop(self):
        now_ros = self.get_clock().now()
        now_sec = time.time()
        elapsed = (now_ros - self.state_start_time).nanoseconds / 1e9

        # Update FPS
        self._frame_count += 1
        el_fps = now_sec - self._fps_start_time
        if el_fps >= 0.5:
            self.fps = self._frame_count / el_fps
            self._frame_count = 0
            self._fps_start_time = now_sec

        # 1. Perception
        detection = self._run_yolo_perception()

        if detection is not None:
            self.obj_detected = True
            self.obj_last_seen = now_sec
            self.detected_class_name = detection["cls_name"]
            self.detected_conf = detection["conf"]
            self.tracked_bbox = detection["bbox"]
            self.lens_distance = detection["dist_z"]
            self.lateral_error = detection["y_base"]
            self.heading_error = detection["heading_err"]
            self.depth_source_mode = "[GEOM]" if detection["is_pinhole"] else "[IR 3D]"

            # Update spatial memory in odom
            self._update_odom_memory(detection["x_base"], detection["y_base"])

            # Publish Target Pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now_ros.to_msg()
            pose_msg.header.frame_id = 'base_footprint'
            pose_msg.pose.position.x = float(detection["x_base"])
            pose_msg.pose.position.y = float(detection["y_base"])
            pose_msg.pose.position.z = 0.0
            self.target_pose_pub.publish(pose_msg)

            # Check for patrol detection interrupt event (Valid range: 0.45m ~ 1.20m, conf >= 0.65)
            if self.lens_distance is not None and 0.45 <= self.lens_distance <= 1.20 and self.detected_conf >= 0.65:
                self._consecutive_detect_count += 1
                if self._consecutive_detect_count >= 3 and (now_sec - self._last_event_pub_time) > 2.5:
                    self._last_event_pub_time = now_sec
                    disp_name = self.target_profiles.get(self.detected_class_name, {}).get("display_name", self.detected_class_name)
                    evt = {
                        "class_name": self.detected_class_name,
                        "display_name": disp_name,
                        "distance_m": round(float(self.lens_distance), 2),
                        "confidence": round(float(self.detected_conf), 2)
                    }
                    self.detected_event_pub.publish(String(data=json.dumps(evt)))
                    self.get_logger().info(f"📢 [FLOOR-EVENT] Broadcasted floor object detection: {evt}")
            else:
                self._consecutive_detect_count = max(0, self._consecutive_detect_count - 1)
        else:
            self._consecutive_detect_count = 0
            # Spatial memory fallback:
            # If the object was recently close (< 0.40m), it has entered the camera floor blind zone.
            # Allow extended dead-reckoning (up to 8.0s) to finish the crawl to 25cm standoff!
            is_close_blind_zone = (self.lens_distance is not None and self.lens_distance < 0.40)
            mem_timeout = 8.0 if is_close_blind_zone else 3.0

            if (now_sec - self.obj_last_seen) < mem_timeout and self.odom_target_latched:
                odom_res = self._get_odom_distance()
                if odom_res is not None:
                    d_z, bx, by = odom_res
                    self.obj_detected = True
                    self.lens_distance = d_z
                    self.lateral_error = by
                    self.heading_error = math.atan2(by, max(0.05, bx))
                    self.depth_source_mode = "[DEAD-RECKONING]" if is_close_blind_zone else "[ODOM MEM]"
                else:
                    self.obj_detected = False
            else:
                self.obj_detected = False

        # 2. Publish Debug Compressed Viewport to GUI
        self._publish_debug_viewport(detection)

        # 3. Publish Periodic Telemetry Status (every 0.25s)
        if int(now_sec * 4) != getattr(self, '_last_status_tick', 0):
            self._last_status_tick = int(now_sec * 4)
            dist_val = float(self.lens_distance) if self.lens_distance is not None else -1.0
            ey_cm = (self.lateral_error * 100.0) if self.lateral_error is not None else 0.0
            yaw_deg = math.degrees(self.heading_error) if self.heading_error is not None else 0.0
            stat_obj = {
                "active": self.state in APPROACH_ACTIVE_STATES,
                "state": self.state,
                "target": self.target_query,
                "detected": self.obj_detected,
                "class": self.detected_class_name,
                "distance": round(dist_val, 3),
                "engine": self.active_engine_name,
                "mode": self.depth_source_mode
            }
            status_line = f"STATE={self.state}, ey={ey_cm:+.1f}cm, yaw_err={yaw_deg:+.1f}deg, dist={dist_val:.2f}m, json={json.dumps(stat_obj)}"
            self.status_pub.publish(String(data=status_line))

        # 4. FSM Execution
        twist = Twist()

        # NAV2_READY / APPROACHED: IDLE state. Completely releases motors!
        if self.state in ('NAV2_READY', 'APPROACHED'):
            if self.state == 'APPROACHED' and elapsed > 1.5:
                self.transition_to('NAV2_READY')
            return

        # Check Global Timeout
        if self.routine_start_time and (now_sec - self.routine_start_time) > self.approach_timeout_sec:
            self._report_gui("⚠️ 접근 시간 초과(Timeout)로 중단합니다.", is_info=False)
            self.stop_robot()
            self.status_pub.publish(String(data="STATE=ABORTED, reason=Approach Timeout"))
            self.transition_to('NAV2_READY')
            return

        # FSM: SEARCH_ALIGN
        if self.state == 'SEARCH_ALIGN':
            if not self.obj_detected:
                # Target not seen yet: reset counters and stop
                self.target_lock_count = 0
                self.align_stable_count = 0
                self.stop_robot()
                if elapsed > 4.5:
                    self._report_gui(f"⚠️ 전방에서 '{self.target_query}' 물체를 찾지 못했습니다.", is_info=False)
                    self.status_pub.publish(String(data=f"STATE=ABORTED, reason=Target '{self.target_query}' not found in front"))
                    self.transition_to('NAV2_READY')
                return

            # 연속 N회 이상 감지 신뢰도 확보 (최소 3프레임 연속 락온)
            self.target_lock_count += 1
            if self.target_lock_count < 3:
                self.stop_robot()
                return

            yaw_err = self.heading_error if self.heading_error is not None else 0.0
            ey_err = self.lateral_error if self.lateral_error is not None else 0.0

            # Aligned within 2.5 degrees and lateral error < 2cm
            if abs(yaw_err) <= math.radians(2.5) and abs(ey_err) <= 0.020:
                self.align_stable_count += 1
                if self.align_stable_count >= 2:
                    self.stop_robot()
                    self._report_gui(f"🎯 [{self.detected_class_name} 조향 정렬 완료] Z={self.lens_distance:.2f}m ➔ 정밀 접근 시작")
                    self.align_stable_count = 0
                    self.target_lock_count = 0
                    self.transition_to('APPROACH_OBJECT')
                    return
                else:
                    self.stop_robot()
            else:
                self.align_stable_count = 0
                w_z = math.copysign(min(0.20, max(0.06, 0.90 * abs(yaw_err))), yaw_err)
                twist.angular.z = float(w_z)

        # FSM: APPROACH_OBJECT
        elif self.state == 'APPROACH_OBJECT':
            if not self.obj_detected or self.lens_distance is None:
                # Momentary flicker: stop and allow spatial memory to recover
                self.stop_robot()
                if elapsed > 3.0:
                    self.transition_to('SEARCH_ALIGN')
                return

            curr_dist = self.lens_distance
            dist_err = curr_dist - self.target_standoff_m

            # Standoff reached (within 8mm)
            if dist_err <= 0.008:
                self.stop_robot()
                self.settle_stable_count = 0
                self.transition_to('FINAL_SETTLE')
                return

            ey = self.lateral_error if self.lateral_error is not None else 0.0
            yaw_corr = math.atan2(ey, max(0.15, curr_dist + self.astra_x_base))

            # 2-Stage Velocity: Cruise (>0.8m) -> Crawl (<0.8m)
            # In Dead-Reckoning blind zone, lock to crawl speed with minimal angular correction
            if self.depth_source_mode == "[DEAD-RECKONING]":
                v_target = self.crawl_linear_speed
                w_target = np.clip(0.60 * yaw_corr, -0.08, 0.08)  # Minimal yaw perturbations
            elif dist_err > 0.55:
                v_target = np.clip(0.50 * dist_err, self.crawl_linear_speed, self.max_linear_speed)
                w_target = np.clip(1.15 * yaw_corr, -0.18, 0.18)
            else:
                v_target = np.clip(0.32 * dist_err, self.crawl_linear_speed, self.max_linear_speed * 0.70)
                w_target = np.clip(1.15 * yaw_corr, -0.18, 0.18)

            twist.linear.x = float(v_target)
            twist.angular.z = float(w_target)

        # FSM: FINAL_SETTLE (25cm fine yaw settlement)
        elif self.state == 'FINAL_SETTLE':
            twist.linear.x = 0.0
            ey = self.lateral_error if (self.obj_detected and self.lateral_error is not None) else 0.0

            # If in dead-reckoning, don't attempt optical yaw adjustment; just verify zero velocity
            if self.depth_source_mode != "[DEAD-RECKONING]" and self.obj_detected and abs(ey) > 0.012 and elapsed < 1.8:
                yaw_err = math.atan2(ey, self.target_standoff_m + self.astra_x_base)
                twist.angular.z = float(math.copysign(min(0.12, max(0.045, 0.75 * abs(yaw_err))), yaw_err))
            else:
                self.stop_robot()
                self.settle_stable_count += 1
                if self.settle_stable_count >= 5:
                    final_dist_cm = (self.lens_distance * 100.0) if self.lens_distance is not None else (self.target_standoff_m * 100.0)
                    self._report_gui(
                        f"🎉 [접근 완료] {self.detected_class_name} 렌즈 전방 {final_dist_cm:.1f}cm 정지 완료! (로봇팔 작업 준비 완료)"
                    )
                    self.transition_to('APPROACHED')
                    return

        # Send command velocity
        self._send_cmd_vel(twist.linear.x, twist.angular.z)

    # ==========================================================================
    # Utilities & Debug Visualization
    # ==========================================================================
    def transition_to(self, new_state: str):
        self.get_logger().info(f"[OBJ-FSM] {self.state} -> {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def stop_robot(self):
        self._send_cmd_vel(0.0, 0.0)

    def _send_cmd_vel(self, vx: float, wz: float):
        if self.enable_stamped_cmd_vel:
            st = TwistStamped()
            st.header.stamp = self.get_clock().now().to_msg()
            st.header.frame_id = 'base_footprint'
            st.twist.linear.x, st.twist.angular.z = float(vx), float(wz)
            self.cmd_vel_pub.publish(st)
        else:
            t = Twist()
            t.linear.x, t.angular.z = float(vx), float(wz)
            self.cmd_vel_pub.publish(t)

    def _srv_start(self, request, response):
        self.tracked_bbox = None
        self.odom_target_latched = False
        self.odom_target_pos = None
        self.target_lock_count = 0
        self.align_stable_count = 0
        self.routine_start_time = time.time()
        self.transition_to('SEARCH_ALIGN')
        self._report_gui(f"🚀 [YOLO 접근 가동] 대상: '{self.target_query}' (목표 거리: {self.target_standoff_m*100:.0f}cm)")
        response.success = True
        response.message = f"YOLO approach started for '{self.target_query}'"
        return response

    def _srv_abort(self, request, response):
        self.stop_robot()
        self.transition_to('NAV2_READY')
        self._report_gui("🛑 접근 작업이 취소되었습니다.")
        response.success = True
        response.message = "Object approach aborted"
        return response

    def _publish_debug_viewport(self, detection: Optional[Dict[str, Any]]):
        if self.last_color_img is None:
            return

        vis = self.last_color_img.copy()
        H, W = vis.shape[:2]

        # 1. Detection Bounding Box
        if detection is not None:
            x, y, bw, bh = detection["bbox"]
            cls_name = detection["cls_name"]
            conf = detection["conf"]
            dist_z = detection["dist_z"]
            ey = detection["y_base"]
            mode_str = self.depth_source_mode

            if "red" in cls_name:
                col = (0, 0, 255)       # Red
            elif "blue" in cls_name:
                col = (255, 120, 0)     # Blue / Cyan
            elif "black" in cls_name or "rod" in cls_name:
                col = (230, 230, 230)   # White / Silver
            else:
                col = (0, 255, 0)       # Green
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), col, 2)
            cv2.circle(vis, (int(x + bw * 0.5), y + bh), 4, (0, 255, 255), -1)

            tag = f"{cls_name} {conf:.2f} | Z={dist_z:.2f}m {mode_str}"
            sub_tag = f"Ey={ey*100:+.1f}cm"
            cv2.rectangle(vis, (x, max(0, y - 30)), (x + len(tag) * 7 + 8, y), (20, 20, 20), -1)
            cv2.putText(vis, tag, (x + 3, max(12, y - 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)
            cv2.putText(vis, sub_tag, (x + 3, max(24, y - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1)
        elif self.depth_source_mode == "[DEAD-RECKONING]":
            # Blind zone banner when optical target is under camera hood
            cv2.putText(vis, f"BLIND-ZONE DEAD-RECKONING (Z={self.lens_distance:.2f}m)", (10, H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 220, 255), 1)

        # 2. Top Telemetry Banner (Clean 2-line layout without text collision)
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (W, 30), (15, 20, 30), -1)
        vis = cv2.addWeighted(overlay, 0.75, vis, 0.25, 0)

        status_text = f"[{self.state}] Pitch:{self.astra_pitch:+.1f}deg"
        cv2.putText(vis, status_text, (8, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

        fps_text = f"{self.fps:.1f}FPS"
        cv2.putText(vis, fps_text, (W - 65, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

        if self.target_query:
            tgt_text = f"Tgt: '{self.target_query}'"
            cv2.putText(vis, tgt_text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (200, 230, 255), 1)

        # Compress & Publish
        success, enc = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if success:
            comp_msg = CompressedImage()
            comp_msg.header.stamp = self.get_clock().now().to_msg()
            comp_msg.header.frame_id = 'base_footprint'
            comp_msg.format = 'jpeg'
            comp_msg.data = enc.tobytes()
            self.debug_comp_pub.publish(comp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectApproachingControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()