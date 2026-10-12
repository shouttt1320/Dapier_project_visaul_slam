#!/usr/bin/env python3
"""
[Object Detector 3D Node v3.5] Hybrid YOLO-World & 2-Stage Plane Estimator
- Astra S / RGB-D Camera 기반 비마커 3D 도킹 타겟 및 기준 평면 추정기
- Stage 1:
  * Primary: YOLO-World Open-Vocabulary Detector (Shape + Color 복합 추론)
  * Exemplar Matching: target_images/red_box.jpg 참조 사진과 시각적 유사도 대조
  * Fallback: HSV Multi-color preset segmentation
- Stage 2:
  * Desk Structure Plane: 앵커 하단 대면적 책상 전면 평면 RANSAC + SVD 수직 법선 추출
  * Direct Dominant Plane: 앵커 없이 책상 자체만 도킹하는 모드 지원
- 시간축 지수이동평균(EMA, alpha=0.25) 및 Persistence Latch
"""

import sys
import os
import time
import math
import json
from typing import Optional, Tuple, List, Dict, Any

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import tf2_ros


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
        [2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
        [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)]
    ], dtype=np.float64)


class TermColor:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"


class ObjectDetector3DNode(Node):
    def __init__(self, enable_terminal_ui: bool = True):
        super().__init__('object_detector_3d_node')
        self.enable_terminal_ui = enable_terminal_ui
        self.bridge = CvBridge()

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('cam_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detector_type', 'qwen_vlm')         # 'qwen_vlm' (default), 'yolo_world', 'hsv'
        self.declare_parameter('target_color', 'red')               # red, blue, green, yellow, none
        self.declare_parameter('detection_mode', 'anchor_guided')   # 'anchor_guided' or 'dominant_plane'
        self.declare_parameter('min_contour_area', 100.0)          # Reduced min area for distant anchors
        self.declare_parameter('min_valid_depth', 0.25)            # Astra S min distance (m)
        self.declare_parameter('max_valid_depth', 2.50)            # Astra S max distance (m)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.cam_info_topic = str(self.get_parameter('cam_info_topic').value)
        self.detector_type = str(self.get_parameter('detector_type').value).lower()
        self.target_color = str(self.get_parameter('target_color').value).lower()
        self.detection_mode = str(self.get_parameter('detection_mode').value)
        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.min_valid_depth = float(self.get_parameter('min_valid_depth').value)
        self.max_valid_depth = float(self.get_parameter('max_valid_depth').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # ----------------------------------------------------------------------
        # Load Color Profiles & Target Exemplar Image (Search Multiple Paths)
        # ----------------------------------------------------------------------
        colors_file = None
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('llm_semantic_docker')
            colors_file = os.path.join(share_dir, "config", "color_profiles.yaml")
        except Exception:
            pass
        if not colors_file or not os.path.exists(colors_file):
            colors_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "color_profiles.yaml"))
        self.color_profiles = self._load_color_profiles(colors_file)

        # Candidate paths for target images: Project root or Package dir
        pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        project_root = os.path.abspath(os.path.join(pkg_dir, "..", ".."))
        self.candidate_img_paths = [
            os.path.join(project_root, "target_images", "red_box.jpg"),
            os.path.join(project_root, "target_images", "red_box.png"),
            os.path.join(pkg_dir, "target_images", "red_box.jpg"),
            os.path.join(pkg_dir, "target_images", "red_box.png"),
        ]
        self.target_img_path = self.candidate_img_paths[0]
        self.ref_hist: Optional[np.ndarray] = None
        self._load_reference_image()

        # ----------------------------------------------------------------------
        # Initialize YOLO-World Detector
        # ----------------------------------------------------------------------
        self.yolo_model = None
        if self.detector_type == 'yolo_world':
            self._init_yolo_world()

        # ----------------------------------------------------------------------
        # Camera Intrinsics (Default Astra S 320x240 bootstrap)
        # ----------------------------------------------------------------------
        self.fx = 285.0
        self.fy = 285.0
        self.cx = 160.0
        self.cy = 120.0
        self.camera_info_received = False

        # Image storage
        self.last_color_img: Optional[np.ndarray] = None
        self.last_depth_raw: Optional[np.ndarray] = None
        self.last_depth_stamp = None
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0

        # Temporal Smoothing State (EMA Filter & Persistence Latch)
        self.ema_alpha = 0.25
        self.smoothed_center: Optional[np.ndarray] = None
        self.smoothed_normal: Optional[np.ndarray] = None
        self.last_detected_time = 0.0
        self.persistence_timeout_sec = 0.5
        self.is_target_locked = False
        self.last_confidence = 0.0

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------------------------------------
        # ROS 2 Interfaces
        # ----------------------------------------------------------------------
        self.pose_pub = self.create_publisher(PoseStamped, '/docking/target_object_pose', 10)
        self.status_pub = self.create_publisher(String, '/docking/object_detection_status', 10)
        # Dedicated Qwen Vision HUD Topics (Decoupled from legacy /docking/debug_image)
        self.debug_img_pub = self.create_publisher(Image, '/qwen/debug_image', 10)
        self.debug_comp_pub = self.create_publisher(CompressedImage, '/qwen/debug_image/compressed', 10)
        self.obj_debug_pub = self.create_publisher(Image, '/docking/object_debug_image', 10)
        self.rviz_marker_pub = self.create_publisher(MarkerArray, '/redbox/marker', 10)
        self.plane_marker_pub = self.create_publisher(MarkerArray, '/docking/plane_debug_markers', 10)

        # TF2 Setup for gravity-aligned verticality checks
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/docking/set_target_color', self._set_color_cb, 10)
        self.create_subscription(String, '/qwen/target_bbox', self._qwen_bbox_cb, 10)
        self.latest_qwen_bbox: Optional[Tuple[int, int, int, int, Tuple[int, int], float]] = None
        self.latest_qwen_time: float = 0.0
        self.latest_qwen_label: str = "Target"
        self._last_marker_clear_time: float = 0.0

        # Image Subscribers
        self.create_subscription(Image, self.color_topic, self._color_cb, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(CameraInfo, self.cam_info_topic, self._cam_info_cb, 10)

        # Compressed Fallbacks
        self.create_subscription(CompressedImage, f"{self.color_topic}/compressed", self._color_comp_cb, sensor_qos)
        self.create_subscription(CompressedImage, f"{self.depth_topic}/compressedDepth", self._depth_comp_cb, sensor_qos)

        # Processing Timer
        timer_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(timer_period, self._process_cycle)

        self._print_banner()

    def _init_yolo_world(self):
        try:
            from ultralytics import YOLOWorld
            self.get_logger().info("Loading YOLO-World (yolov8s-worldv2.pt)...")
            self.yolo_model = YOLOWorld('yolov8s-worldv2.pt')
            self.yolo_classes = ['red box', 'small red box', 'red cube', 'red container', 'box']
            self.yolo_model.set_classes(self.yolo_classes)
            # Warm up
            dummy = np.zeros((240, 320, 3), dtype=np.uint8)
            self.yolo_model.predict(dummy, verbose=False)
            self.get_logger().info("YOLO-World initialized successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize YOLO-World: {e}. Falling back to HSV.")
            self.detector_type = 'hsv'

    def _load_reference_image(self):
        for path in self.candidate_img_paths:
            if os.path.exists(path):
                img = cv2.imread(path)
                if img is not None:
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
                    cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
                    self.ref_hist = hist
                    self.target_img_path = path
                    self.get_logger().info(f"Loaded reference target image from: {path}")
                    return
        self.get_logger().info("No reference image found yet. (Optional: place red_box.jpg in target_images/)")

    def _load_color_profiles(self, filepath: str) -> Dict[str, Any]:
        default_profiles = {
            "red": {
                "lower1": [0, 70, 50], "upper1": [15, 255, 255],
                "lower2": [160, 70, 50], "upper2": [180, 255, 255]
            },
            "blue": {
                "lower1": [100, 80, 50], "upper1": [135, 255, 255],
                "lower2": None, "upper2": None
            },
            "yellow": {
                "lower1": [18, 80, 80], "upper1": [38, 255, 255],
                "lower2": None, "upper2": None
            }
        }
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                    return data.get("colors", default_profiles)
            except Exception:
                pass
        return default_profiles

    def _print_banner(self):
        if not self.enable_terminal_ui:
            return
        banner = f"""
{TermColor.CYAN}╔══════════════════════════════════════════════════════════════════════════╗
║     🎯 HYBRID YOLO-WORLD & 2-STAGE STRUCTURE PLANE DOCKING DETECTOR      ║
║  Detector: {self.detector_type.upper():<10} | Mode: {self.detection_mode:<16} | Target: {self.target_color.upper():<8} ║
║  Strategy: Shape+Color Visual Prompting -> Desk Structure Plane Normal   ║
╚══════════════════════════════════════════════════════════════════════════╝{TermColor.RESET}
"""
        print(banner)

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------
    def _set_color_cb(self, msg: String):
        val = msg.data.strip().lower()
        if val in ["none", "null", "direct"]:
            self.target_color = "none"
            self.detection_mode = "dominant_plane"
            self.get_logger().info("Switched to [DIRECT DOMINANT PLANE] mode.")
        elif val in self.color_profiles or val in ["red", "blue", "yellow"]:
            self.target_color = val
            self.detection_mode = "anchor_guided"
            self.get_logger().info(f"Switched anchor color to [{val}].")

    def _qwen_bbox_cb(self, msg: String):
        """Qwen2.5-VL 멀티모달 노드에서 발행한 2D BBox 수신"""
        try:
            data = json.loads(msg.data)
            if data.get("target_found", False) and data.get("pixel_box"):
                xmin, ymin, xmax, ymax = data["pixel_box"]
                w = max(1, xmax - xmin)
                h = max(1, ymax - ymin)
                cx = int(xmin + w / 2)
                cy = int(ymin + h / 2)
                conf = float(data.get("confidence", 0.90))
                self.latest_qwen_bbox = (xmin, ymin, w, h, (cx, cy), conf)
                self.latest_qwen_time = time.time()
                self.latest_qwen_label = data.get("label", "Target")
                self.detector_type = "qwen_vlm"
        except Exception:
            pass

    def _cam_info_cb(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True

    def _color_cb(self, msg: Image):
        try:
            self.last_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _color_comp_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.last_color_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            pass

    def _depth_cb(self, msg: Image):
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.last_depth_raw = depth_raw.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                self.last_depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.last_depth_stamp = msg.header.stamp
        except Exception:
            pass

    def _depth_comp_cb(self, msg: CompressedImage):
        try:
            raw_bytes = bytes(msg.data)
            if raw_bytes.startswith(b'\x89PNG'):
                # Pure PNG byte stream from Raspberry Pi custom compressor
                np_arr = np.frombuffer(raw_bytes, np.uint8)
            elif len(raw_bytes) > 12 and raw_bytes[12:].startswith(b'\x89PNG'):
                # Standard ROS image_transport compressedDepth header (12 bytes)
                np_arr = np.frombuffer(raw_bytes[12:], np.uint8)
            else:
                np_arr = np.frombuffer(raw_bytes, np.uint8)

            depth_png = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth_png is not None:
                self.last_depth_raw = depth_png.astype(np.float32) / 1000.0
                self.last_depth_stamp = msg.header.stamp
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # Stage 1: Visual Anchor Detection (YOLO-World with Reference Match + HSV Fallback)
    # --------------------------------------------------------------------------
    def detect_anchor_yolo_world(self, bgr_img: np.ndarray) -> Optional[Tuple[int, int, int, int, Tuple[int, int], float]]:
        """
        Detects target box using YOLO-World Open-Vocabulary inference.
        Returns: (x, y, w, h, (cx, cy), conf)
        """
        if self.yolo_model is None:
            return None

        h, w = bgr_img.shape[:2]
        try:
            results = self.yolo_model.predict(bgr_img, conf=0.18, verbose=False)
            if not results or len(results[0].boxes) == 0:
                return None

            boxes = results[0].boxes
            candidates = []

            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                bw = x2 - x1
                bh = y2 - y1

                # Filter out absurd aspect ratios
                if bw < 12 or bh < 12:
                    continue

                cx = x1 + bw // 2
                cy = y1 + bh // 2

                # If reference exemplar image exists, score similarity
                sim_score = conf
                if self.ref_hist is not None:
                    crop = bgr_img[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
                    if crop.size > 0:
                        crop_hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
                        crop_hist = cv2.calcHist([crop_hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
                        cv2.normalize(crop_hist, crop_hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
                        # Correlation comparison: higher is better (-1 to 1)
                        corr = cv2.compareHist(self.ref_hist, crop_hist, cv2.HISTCMP_CORREL)
                        sim_score = conf * 0.5 + max(0.0, corr) * 0.5

                candidates.append((x1, y1, bw, bh, (cx, cy), conf, sim_score))

            if not candidates:
                return None

            # Pick highest scoring candidate
            best = max(candidates, key=lambda c: c[6])
            return (best[0], best[1], best[2], best[3], best[4], best[5])

        except Exception as e:
            return None

    def detect_anchor_hsv(self, bgr_img: np.ndarray, color_name: str) -> Optional[Tuple[int, int, int, int, Tuple[int, int], float]]:
        """Fallback HSV color segmentation."""
        if color_name not in self.color_profiles:
            return None

        cfg = self.color_profiles[color_name]
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        l1 = np.array(cfg.get("lower1", [0, 70, 50]))
        u1 = np.array(cfg.get("upper1", [15, 255, 255]))
        mask = cv2.inRange(hsv, l1, u1)

        l2 = cfg.get("lower2")
        u2 = cfg.get("upper2")
        if l2 is not None and u2 is not None:
            mask2 = cv2.inRange(hsv, np.array(l2), np.array(u2))
            mask = cv2.bitwise_or(mask, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.min_contour_area:
            return None

        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] > 0 else y + h // 2

        return (x, y, w, h, (cx, cy), 0.85)

    def detect_anchor(self, bgr_img: np.ndarray) -> Optional[Tuple[int, int, int, int, Tuple[int, int], float]]:
        """Unified Anchor Detection: Qwen2.5-VL VLM Fresh BBox -> Realtime HSV Continuous Tracking -> YOLO-World -> Latched Qwen."""
        # 1. Qwen2.5-VL Fresh BBox (Highest priority right after user command / VLM detection, 4s window)
        # Prevents background ambient colors from overriding the user-queried object
        if self.latest_qwen_bbox is not None and (time.time() - self.latest_qwen_time) < 4.0:
            return self.latest_qwen_bbox

        # 2. Real-time HSV Color Filter Tracking (Tracks object continuously as robot translates & approaches)
        if self.target_color != "none":
            hsv_res = self.detect_anchor_hsv(bgr_img, self.target_color)
            if hsv_res is not None:
                return hsv_res

        # 3. YOLO-World Detector (if enabled and available)
        if (self.detector_type == 'yolo_world' or getattr(self, 'yolo_model', None) is not None) and self.yolo_model is not None:
            res = self.detect_anchor_yolo_world(bgr_img)
            if res is not None:
                return res

        # 4. Latched Qwen BBox Fallback (Up to 180s persistence during docking)
        if self.latest_qwen_bbox is not None and (time.time() - self.latest_qwen_time) < 180.0:
            return self.latest_qwen_bbox

        return None

    # --------------------------------------------------------------------------
    # Stage 2: Robust Structure Plane Fitting with Geometric Scoring (GPT Multi-Candidate Model)
    # --------------------------------------------------------------------------
    def _get_camera_to_base_rotation(self) -> Optional[np.ndarray]:
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', self.frame_id, rclpy.time.Time())
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            self._cached_R_b_c = quaternion_to_matrix(q)
            return self._cached_R_b_c
        except Exception:
            return getattr(self, '_cached_R_b_c', None)

    def fit_plane_geometric_score(
        self, points_3d: np.ndarray, max_iterations: int = 180, residual_thresh: float = 0.025
    ) -> Optional[Tuple[np.ndarray, np.ndarray, int, np.ndarray]]:
        """
        Multi-Candidate RANSAC with Geometric Scoring:
          1. Base-frame verticality (|n_base.z| ~ 0, rejects horizontal floor/ceiling)
          2. Facing camera orientation (dot product with camera ray > 0)
          3. Inlier ratio
        Returns (normal_camera, p_mean, inlier_count, inlier_points).
        """
        num_points = points_3d.shape[0]
        if num_points < 35:
            return None

        R_b_c = self._get_camera_to_base_rotation()

        # Camera optical center is (0,0,0); ray from target to camera is -p_center
        p_center_approx = np.median(points_3d, axis=0)
        view_dir = -p_center_approx
        view_norm = np.linalg.norm(view_dir)
        if view_norm > 1e-4:
            view_dir /= view_norm
        else:
            view_dir = np.array([0.0, 0.0, -1.0])

        best_score = -1.0
        best_inliers = None
        best_normal = None
        best_count = 0

        for _ in range(max_iterations):
            idx = np.random.choice(num_points, 3, replace=False)
            p1, p2, p3 = points_3d[idx]

            v1 = p2 - p1
            v2 = p3 - p1
            n = np.cross(v1, v2)
            n_norm = np.linalg.norm(n)
            if n_norm < 1e-6:
                continue
            n = n / n_norm

            # 1. Unify normal sign: MUST point outward towards camera
            if np.dot(n, view_dir) < 0:
                n = -n

            # 2. Check verticality in base_footprint frame (eliminates floor/ceiling candidates)
            if R_b_c is not None:
                n_base = R_b_c @ n
                vert_z = abs(n_base[2])  # 0.0 = pure vertical wall, 1.0 = pure horizontal floor
                if vert_z > 0.35:        # Reject if tilted more than ~20 deg from vertical
                    continue
                vertical_score = 1.0 - vert_z
            else:
                # Optical frame fallback (Y is down)
                vert_y = abs(n[1])
                if vert_y > 0.45:
                    continue
                vertical_score = 1.0 - vert_y

            # 3. Calculate inliers
            d = -np.dot(n, p1)
            dist = np.abs(np.dot(points_3d, n) + d)
            inliers = dist < residual_thresh
            count = np.sum(inliers)
            if count < 25:
                continue

            inlier_ratio = count / float(num_points)

            # 4. Facing score (how directly plane faces camera ray)
            facing_score = max(0.0, float(np.dot(n, view_dir)))

            # 5. Composite Geometric Score
            score = 0.40 * inlier_ratio + 0.35 * vertical_score + 0.25 * facing_score

            if score > best_score:
                best_score = score
                best_count = count
                best_inliers = inliers
                best_normal = n

        if best_count < 25 or best_inliers is None:
            return None

        # SVD Refinement on optimal inliers
        inlier_points = points_3d[best_inliers]
        p_mean = np.mean(inlier_points, axis=0)
        shifted = inlier_points - p_mean
        _, _, vh = np.linalg.svd(shifted, full_matrices=False)
        normal = vh[2, :]

        # Guarantee normal points outward towards camera
        if np.dot(normal, view_dir) < 0:
            normal = -normal
        normal = normal / (np.linalg.norm(normal) + 1e-8)

        return (normal, p_mean, best_count, inlier_points)

    def _publish_plane_debug_markers(self, inlier_points: np.ndarray, center: np.ndarray, normal: np.ndarray):
        """Publishes RANSAC inlier point cloud and plane normal arrow to RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. Inlier Point Cloud (Bright Green dots)
        pts_m = Marker()
        pts_m.header.stamp = now
        pts_m.header.frame_id = self.frame_id
        pts_m.ns = "plane_inliers"
        pts_m.id = 100
        pts_m.type = Marker.POINTS
        pts_m.action = Marker.ADD
        pts_m.scale.x = 0.015  # point size
        pts_m.scale.y = 0.015
        pts_m.color.r = 0.10
        pts_m.color.g = 1.00
        pts_m.color.b = 0.20
        pts_m.color.a = 0.90
        pts_m.lifetime.sec = 1

        stride = max(1, inlier_points.shape[0] // 150)
        pts_m.points = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            for p in inlier_points[::stride]
        ]
        ma.markers.append(pts_m)

        # 2. Surface Normal Arrow from center outwards (Bright Yellow Arrow, 30cm)
        arrow_m = Marker()
        arrow_m.header.stamp = now
        arrow_m.header.frame_id = self.frame_id
        arrow_m.ns = "plane_normal"
        arrow_m.id = 101
        arrow_m.type = Marker.ARROW
        arrow_m.action = Marker.ADD
        arrow_m.points = [
            Point(x=float(center[0]), y=float(center[1]), z=float(center[2])),
            Point(x=float(center[0] + 0.30 * normal[0]),
                  y=float(center[1] + 0.30 * normal[1]),
                  z=float(center[2] + 0.30 * normal[2]))
        ]
        arrow_m.scale.x = 0.03  # shaft diameter
        arrow_m.scale.y = 0.06  # head diameter
        arrow_m.scale.z = 0.06  # head length
        arrow_m.color.r = 1.0
        arrow_m.color.g = 0.95
        arrow_m.color.b = 0.0
        arrow_m.color.a = 0.95
        arrow_m.lifetime.sec = 1
        ma.markers.append(arrow_m)

        self.plane_marker_pub.publish(ma)

    def fit_plane_ransac_svd(self, points_3d: np.ndarray, max_iterations: int = 35,
                             residual_thresh: float = 0.020) -> Optional[Tuple[np.ndarray, np.ndarray, int]]:
        res = self.fit_plane_geometric_score(points_3d, max_iterations=max_iterations, residual_thresh=residual_thresh)
        if res is not None:
            return (res[0], res[1], res[2])
        return None

    def estimate_structure_plane_anchor_guided(
        self, depth_map: np.ndarray, anchor_bbox: Tuple[int, int, int, int], centroid: Tuple[int, int]
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, Tuple[int, int, int, int], int]]:
        ax, ay, aw, ah = anchor_bbox
        acx, acy = centroid
        h_img, w_img = depth_map.shape[:2]

        # Horizontal 3.0x expansion
        half_w = int(aw * 1.5)
        rx1 = max(0, acx - half_w)
        rx2 = min(w_img - 1, acx + half_w)

        # Vertical band 10px below anchor
        ry1 = min(h_img - 15, ay + ah + 10)
        ry2 = min(h_img - 1, ry1 + 40)

        if rx2 - rx1 < 20 or ry2 - ry1 < 8:
            return None

        desk_roi = (rx1, ry1, rx2 - rx1, ry2 - ry1)
        roi_depth = depth_map[ry1:ry2, rx1:rx2]

        valid_mask = (roi_depth >= self.min_valid_depth) & (roi_depth <= self.max_valid_depth) & ~np.isnan(roi_depth)
        if np.sum(valid_mask) < 30:
            return None

        v_idx, u_idx = np.where(valid_mask)
        u_global = u_idx + rx1
        v_global = v_idx + ry1
        z_vals = roi_depth[valid_mask]

        pts_x = (u_global - self.cx) * z_vals / self.fx
        pts_y = (v_global - self.cy) * z_vals / self.fy
        pts_z = z_vals
        points_3d = np.stack([pts_x, pts_y, pts_z], axis=1)

        fit_res = self.fit_plane_ransac_svd(points_3d, max_iterations=30, residual_thresh=0.018)
        if fit_res is None:
            return None

        normal, p_mean, inlier_cnt = fit_res

        # Depth at anchor
        anchor_sample = depth_map[max(0, acy-5):min(h_img, acy+5), max(0, acx-5):min(w_img, acx+5)]
        anchor_valid = anchor_sample[anchor_sample > self.min_valid_depth]
        z_anchor = float(np.median(anchor_valid)) if len(anchor_valid) > 0 else float(p_mean[2])

        target_x = float((acx - self.cx) * z_anchor / self.fx)
        target_y = float(p_mean[1])
        target_z = float(p_mean[2])

        target_center = np.array([target_x, target_y, target_z], dtype=np.float32)
        yaw_rad = math.atan2(normal[0], -normal[2])

        return (target_center, normal, yaw_rad, desk_roi, inlier_cnt)

    def estimate_structure_plane_from_object_body(
        self, depth_map: np.ndarray, anchor_bbox: Tuple[int, int, int, int], centroid: Tuple[int, int]
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, Tuple[int, int, int, int], int]]:
        """
        단독 물체(상자, 카트, 충전기 등)의 전면(Front Face) 3D 평면 추정.
        물체 BBox 내부(중앙 70% 영역)의 유효 Depth 포인트를 추출하여 RANSAC 평면 피팅 수행.
        """
        ax, ay, aw, ah = anchor_bbox
        acx, acy = centroid
        h_img, w_img = depth_map.shape[:2]

        # BBox 경계부(배경/가장자리 잡음)를 제외한 중앙 70% 영역 추출
        margin_w = max(2, int(aw * 0.15))
        margin_h = max(2, int(ah * 0.15))
        rx1 = max(0, ax + margin_w)
        rx2 = min(w_img - 1, ax + aw - margin_w)
        ry1 = max(0, ay + margin_h)
        ry2 = min(h_img - 1, ay + ah - margin_h)

        if rx2 - rx1 < 10 or ry2 - ry1 < 10:
            return None

        body_roi = (rx1, ry1, rx2 - rx1, ry2 - ry1)
        roi_depth = depth_map[ry1:ry2, rx1:rx2]

        valid_mask = (roi_depth >= self.min_valid_depth) & (roi_depth <= self.max_valid_depth) & ~np.isnan(roi_depth)
        if np.sum(valid_mask) < 25:
            return None

        v_idx, u_idx = np.where(valid_mask)
        u_global = u_idx + rx1
        v_global = v_idx + ry1
        z_vals = roi_depth[valid_mask]

        pts_x = (u_global - self.cx) * z_vals / self.fx
        pts_y = (v_global - self.cy) * z_vals / self.fy
        pts_z = z_vals
        points_3d = np.stack([pts_x, pts_y, pts_z], axis=1)

        fit_res = self.fit_plane_geometric_score(points_3d, max_iterations=180, residual_thresh=0.025)
        if fit_res is None:
            return None

        normal, p_mean, inlier_cnt, inlier_pts = fit_res

        # 법선 벡터가 카메라/로봇 전방(Optical Z축 반대 방향)을 향하도록 정규화
        if normal[2] > 0:
            normal = -normal

        # 중심점 (X, Y, Z): 실제 피팅된 전면 평면 인라이어들의 3D 무게중심(p_mean) 채택
        target_center = np.array([float(p_mean[0]), float(p_mean[1]), float(p_mean[2])], dtype=np.float32)
        yaw_rad = math.atan2(normal[0], -normal[2])

        # RViz 디버그 마커 발행 (인라이어 점군 + 3D 법선 화살표)
        self._publish_plane_debug_markers(inlier_pts, target_center, normal)

        return (target_center, normal, yaw_rad, body_roi, inlier_cnt)

    def estimate_structure_plane_direct(
        self, depth_map: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, Tuple[int, int, int, int], int]]:
        h_img, w_img = depth_map.shape[:2]
        rx1, rx2 = int(w_img * 0.10), int(w_img * 0.90)
        ry1, ry2 = int(h_img * 0.20), int(h_img * 0.85)

        sub_depth = depth_map[ry1:ry2, rx1:rx2]
        valid_mask = (sub_depth >= 0.40) & (sub_depth <= 2.20) & ~np.isnan(sub_depth)
        if np.sum(valid_mask) < 150:
            return None

        v_idx, u_idx = np.where(valid_mask)
        pts_x = (u_idx + rx1 - self.cx) * sub_depth[valid_mask] / self.fx
        pts_y = (v_idx + ry1 - self.cy) * sub_depth[valid_mask] / self.fy
        pts_z = sub_depth[valid_mask]
        points_3d = np.stack([pts_x, pts_y, pts_z], axis=1)

        non_ground = points_3d[:, 1] < 0.20
        points_3d = points_3d[non_ground]
        if points_3d.shape[0] < 100:
            return None

        fit_res = self.fit_plane_ransac_svd(points_3d, max_iterations=35, residual_thresh=0.020)
        if fit_res is None:
            return None

        normal, p_mean, inlier_cnt = fit_res
        target_center = p_mean.astype(np.float32)
        yaw_rad = math.atan2(normal[0], -normal[2])
        desk_roi = (rx1, ry1, rx2 - rx1, ry2 - ry1)

        return (target_center, normal, yaw_rad, desk_roi, inlier_cnt)

    # --------------------------------------------------------------------------
    # Main Processing Cycle
    # --------------------------------------------------------------------------
    def _process_cycle(self):
        self.fps_counter += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter / (now - self.last_fps_time)
            self.fps_counter = 0
            self.last_fps_time = now

        if self.last_color_img is None or self.last_depth_raw is None:
            return

        bgr = self.last_color_img.copy()
        depth = self.last_depth_raw.copy()

        detected_pose = None
        anchor_bbox = None
        desk_roi = None
        inliers = 0
        conf = 0.0

        # Qwen VLM BBox가 있거나 anchor_guided 모드일 때
        has_recent_qwen = (self.latest_qwen_bbox is not None and (time.time() - self.latest_qwen_time) < 180.0)
        if has_recent_qwen or (self.target_color != "none" and self.detection_mode == "anchor_guided"):
            # 1. Detect Anchor via Qwen VLM / YOLO-World / HSV
            anchor_res = self.detect_anchor(bgr)
            if anchor_res is not None:
                anchor_bbox = (anchor_res[0], anchor_res[1], anchor_res[2], anchor_res[3])
                centroid = anchor_res[4]
                conf = anchor_res[5]
                self.last_confidence = conf

                # 2. 상자(box)나 단독 구조물인 경우 물체 자체 전면(Front Face) 우선 평면 추정
                label_str = str(self.latest_qwen_label).lower() if self.latest_qwen_label else "box"
                is_standalone_box = any(kw in label_str for kw in ["box", "상자", "charger", "station", "cabinet", "target"]) or (self.detection_mode == "anchor_guided")

                plane_res = None
                if is_standalone_box:
                    plane_res = self.estimate_structure_plane_from_object_body(depth, anchor_bbox, centroid)
                else:
                    plane_res = self.estimate_structure_plane_anchor_guided(depth, anchor_bbox, centroid)
                    if plane_res is None:
                        plane_res = self.estimate_structure_plane_from_object_body(depth, anchor_bbox, centroid)

                if plane_res is not None:
                    center_3d, normal, yaw_rad, desk_roi, inliers = plane_res
                    detected_pose = (center_3d, normal, yaw_rad)
        else:
            # Direct Desk Plane Mode
            plane_res = self.estimate_structure_plane_direct(depth)
            if plane_res is not None:
                center_3d, normal, yaw_rad, desk_roi, inliers = plane_res
                detected_pose = (center_3d, normal, yaw_rad)
                self.last_confidence = 1.0

        # Temporal Smoothing & Persistence Latch
        current_time = time.time()
        if detected_pose is not None:
            raw_center, raw_normal, raw_yaw = detected_pose
            if self.smoothed_center is None:
                self.smoothed_center = raw_center
                self.smoothed_normal = raw_normal
            else:
                self.smoothed_center = (1.0 - self.ema_alpha) * self.smoothed_center + self.ema_alpha * raw_center
                blend_normal = (1.0 - self.ema_alpha) * self.smoothed_normal + self.ema_alpha * raw_normal
                self.smoothed_normal = blend_normal / (np.linalg.norm(blend_normal) + 1e-8)

            self.last_detected_time = current_time
            self.is_target_locked = True
        else:
            if (current_time - self.last_detected_time) < self.persistence_timeout_sec and self.smoothed_center is not None:
                self.is_target_locked = True
            else:
                self.is_target_locked = False
                self.smoothed_center = None
                self.smoothed_normal = None

        # Publish and Log
        if self.is_target_locked and self.smoothed_center is not None and self.smoothed_normal is not None:
            yaw_rad = math.atan2(self.smoothed_normal[0], -self.smoothed_normal[2])
            self._publish_pose(self.smoothed_center, self.smoothed_normal, yaw_rad)
            self._publish_status(locked=True, center=self.smoothed_center, yaw=yaw_rad, inliers=inliers)
            if self.enable_terminal_ui:
                self._log_terminal_telemetry(self.smoothed_center, self.smoothed_normal, yaw_rad, inliers)
            self._render_and_publish_debug(bgr, anchor_bbox, desk_roi, self.smoothed_center, self.smoothed_normal, inliers)
        else:
            self._publish_status(locked=False)
            self._render_and_publish_debug(bgr, anchor_bbox, desk_roi, None, None, 0)

    # --------------------------------------------------------------------------
    # Telemetry & Terminal UI
    # --------------------------------------------------------------------------
    def _log_terminal_telemetry(self, center: np.ndarray, normal: np.ndarray, yaw_rad: float, inliers: int):
        yaw_deg = math.degrees(yaw_rad)
        det_tag = f"[{self.detector_type.upper()}:{self.last_confidence:.2f}]"
        status_line = (
            f"\r{TermColor.WHITE}[HYBRID VISION] {TermColor.GREEN}🎯 LOCKED {det_tag}{TermColor.RESET} | "
            f"{TermColor.BOLD}X:{center[0]:+0.3f}m Y:{center[1]:+0.3f}m Z:{center[2]:0.3f}m{TermColor.RESET} | "
            f"Yaw:{yaw_deg:+05.1f}° | "
            f"Normal:[{normal[0]:+0.2f},{normal[1]:+0.2f},{normal[2]:+0.2f}] | "
            f"Inliers:{inliers:<4} | "
            f"{self.current_fps:4.1f} FPS "
        )
        sys.stdout.write(status_line)
        sys.stdout.flush()

    def _publish_pose(self, center: np.ndarray, normal: np.ndarray, yaw_rad: float):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id

        pose_msg.pose.position.x = float(center[0])
        pose_msg.pose.position.y = float(center[1])
        pose_msg.pose.position.z = float(center[2])

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        pose_msg.pose.orientation = Quaternion(x=0.0, y=sy, z=0.0, w=cy)

        self.pose_pub.publish(pose_msg)
        self._publish_rviz_markers(center, normal, yaw_rad)

    def _publish_rviz_markers(self, center: np.ndarray, normal: np.ndarray, yaw_rad: float):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        label = getattr(self, 'latest_qwen_label', 'Target')

        # 1. 3D Bounding Cube
        cube = Marker()
        cube.header.stamp = now
        cube.header.frame_id = self.frame_id
        cube.ns = "red_box"
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = float(center[0])
        cube.pose.position.y = float(center[1])
        cube.pose.position.z = float(center[2])
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cube.pose.orientation = Quaternion(x=0.0, y=sy, z=0.0, w=cy)
        cube.scale = Vector3(x=0.25, y=0.25, z=0.20)
        cube.color.r = 0.0
        cube.color.g = 1.0
        cube.color.b = 0.5
        cube.color.a = 0.65
        cube.lifetime.sec = 1
        ma.markers.append(cube)

        # 2. Normal Vector Arrow
        arrow = Marker()
        arrow.header.stamp = now
        arrow.header.frame_id = self.frame_id
        arrow.ns = "red_box"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        p_start = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
        p_end = Point(
            x=float(center[0] + normal[0] * 0.35),
            y=float(center[1] + normal[1] * 0.35),
            z=float(center[2] + normal[2] * 0.35)
        )
        arrow.points = [p_start, p_end]
        arrow.scale = Vector3(x=0.03, y=0.06, z=0.08)
        arrow.color.r = 1.0
        arrow.color.g = 0.9
        arrow.color.b = 0.0
        arrow.color.a = 0.9
        arrow.lifetime.sec = 1
        ma.markers.append(arrow)

        # 3. 3D Text Label
        text = Marker()
        text.header.stamp = now
        text.header.frame_id = self.frame_id
        text.ns = "red_box"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(center[0])
        text.pose.position.y = float(center[1])
        text.pose.position.z = float(center[2] - 0.15)
        text.scale.z = 0.08
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        dist = float(np.linalg.norm(center))
        text.text = f"{label} (D:{dist:.2f}m)"
        text.lifetime.sec = 1
        ma.markers.append(text)

        self.rviz_marker_pub.publish(ma)

    def _publish_status(self, locked: bool, center: Optional[np.ndarray] = None, yaw: float = 0.0, inliers: int = 0):
        status_msg = String()
        if locked and center is not None:
            yaw_deg = math.degrees(yaw)
            status_msg.data = (
                f"LOCKED=1, detector={self.detector_type}, color={self.target_color}, "
                f"conf={self.last_confidence:.2f}, x={center[0]:.3f}, y={center[1]:.3f}, "
                f"z={center[2]:.3f}, yaw_deg={yaw_deg:.1f}, inliers={inliers}, fps={self.current_fps:.1f}"
            )
        else:
            status_msg.data = f"LOCKED=0, detector={self.detector_type}, fps={self.current_fps:.1f}"
        self.status_pub.publish(status_msg)

    def _render_and_publish_debug(self, bgr: np.ndarray, anchor_bbox: Optional[Tuple[int, int, int, int]],
                                  desk_roi: Optional[Tuple[int, int, int, int]],
                                  center_3d: Optional[np.ndarray], normal: Optional[np.ndarray], inliers: int):
        vis = bgr.copy()
        h, w = vis.shape[:2]

        if anchor_bbox is not None:
            ax, ay, aw, ah = anchor_bbox
            cv2.rectangle(vis, (ax, ay), (ax + aw, ay + ah), (0, 220, 255), 2)
            raw_lbl = str(self.latest_qwen_label or "TARGET")
            if any(k in raw_lbl.lower() for k in ["box", "상자"]):
                lbl = "BOX"
            elif any(k in raw_lbl.lower() for k in ["desk", "책상"]):
                lbl = "DESK"
            else:
                lbl = "".join(c for c in raw_lbl if c.isascii() and c.isalnum()) or "TARGET"
            label_text = f"Qwen2.5-VL [{lbl}]" if self.latest_qwen_bbox is not None else f"Anchor [{self.last_confidence:.2f}]"
            cv2.putText(vis, label_text, (ax, max(15, ay - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 2)

        if desk_roi is not None:
            rx, ry, rw, rh = desk_roi
            cv2.rectangle(vis, (rx, ry), (rx + rw, ry + rh), (255, 255, 0), 2)
            cv2.putText(vis, f"DOCK PLANE ({inliers} pts)", (rx, max(15, ry - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

        if center_3d is not None and normal is not None:
            cx = int(self.cx + (center_3d[0] * self.fx / center_3d[2]))
            cy = int(self.cy + (center_3d[1] * self.fy / center_3d[2]))
            cv2.circle(vis, (cx, cy), 6, (0, 255, 0), -1)

            ray_end_x = int(cx + normal[0] * 60)
            ray_end_y = int(cy + normal[1] * 60)
            cv2.arrowedLine(vis, (cx, cy), (ray_end_x, ray_end_y), (0, 255, 255), 2, tipLength=0.3)

            yaw_deg = math.degrees(math.atan2(normal[0], -normal[2]))
            t1 = f"TARGET: X:{center_3d[0]:+.2f}m, Y:{center_3d[1]:+.2f}m, Z:{center_3d[2]:.3f}m"
            t2 = f"Normal:[{normal[0]:+.2f}, {normal[1]:+.2f}, {normal[2]:+.2f}] | Yaw:{yaw_deg:+.1f} deg"
            cv2.putText(vis, t1, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(vis, t2, (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        else:
            cv2.putText(vis, "SEARCHING TARGET (Qwen2.5-VL VLM)...", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        cv2.putText(vis, f"{self.current_fps:.1f} FPS", (w - 100, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            self.debug_img_pub.publish(img_msg)
            self.obj_debug_pub.publish(img_msg)

            success, encoded = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if success:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = self.get_clock().now().to_msg()
                comp_msg.format = 'jpeg'
                comp_msg.data = encoded.tobytes()
                self.debug_comp_pub.publish(comp_msg)
                self.obj_comp_pub.publish(comp_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector3DNode(enable_terminal_ui=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Hybrid YOLO-World Object Detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
