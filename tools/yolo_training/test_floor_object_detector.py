#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Qwen2.5-VL Floor Object Perception Test & Diagnostic Tool (Zero-Motion Safe)
# - Verifies identical detection pipeline as object_approaching_controller_node.py:
#   1. Semantic Lock-On via Qwen2.5-VL (Locates target ignoring desk legs & clutter)
#   2. 20Hz Local ROI Visual Tracker (Color-difference tracking in +/-65px ROI)
#   3. Pinhole Ground-Contact Geometry + Depth Fusion (Works down to 18cm)
# - Zero-Motion: Completely safe to run, no cmd_vel published
# - Standalone: Can query Ollama directly even if llm_agent_node is not running!
# ==============================================================================

import os
import sys
import time
import math
import json
import base64
import urllib.request
import urllib.error
import threading
import subprocess
import argparse
from typing import Optional, Tuple, List, Dict, Any
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge


OLLAMA_API_URL = "http://127.0.0.1:11434/api/generate"
MODEL_NAME = "qwen2.5:3b"
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def ensure_camera_ready() -> Optional[subprocess.Popen]:
    """Astra S 카메라 노드가 실행 중인지 확인하고 미실행 시 rclpy.init() 전에 백그라운드 자동 기동"""
    try:
        pg = subprocess.run(["pgrep", "-f", "install/astra_camera/lib/astra_camera/astra_camera_node"], stdout=subprocess.PIPE, text=True)
        if pg.stdout.strip():
            print(f"📷 [Astra S] astra_camera_node가 이미 실행 중입니다 (PID: {pg.stdout.strip()}).")
            return None
    except Exception:
        pass

    try:
        ls = subprocess.run(["lsusb"], stdout=subprocess.PIPE, text=True)
        if "2bc5:" not in ls.stdout:
            print("⚠️ [Astra S] USB 포트에 Astra S 카메라가 감지되지 않습니다. 케이블을 확인해주세요.")
            return None
    except Exception:
        return None

    print("📷 [Astra S] 잔여 메모리 정리 및 카메라 드라이버를 백그라운드에서 구동합니다...")
    subprocess.run("pkill -9 -f astra_camera_node 2>/dev/null || true", shell=True)
    subprocess.run("rm -f /dev/shm/orbbec* /dev/shm/fastrtps* /dev/shm/sem.* 2>/dev/null || true", shell=True)
    time.sleep(0.5)

    home_dir = os.path.expanduser("~")
    extra_ld = f"{home_dir}/.local/lib:{PROJECT_ROOT}/robot_ws/install/astra_camera_msgs/lib"
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"{extra_ld}:{env.get('LD_LIBRARY_PATH', '')}"

    ws_setup = os.path.join(PROJECT_ROOT, "robot_ws", "install", "setup.bash")
    launch_cmd = (
        f"export LD_LIBRARY_PATH={extra_ld}:$LD_LIBRARY_PATH && "
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {ws_setup} && "
        f"ros2 launch astra_camera astra.launch.xml "
        f"depth_width:=320 depth_height:=240 color_width:=320 color_height:=240 "
        f"enable_point_cloud:=false depth_registration:=true"
    )
    try:
        cam_process = subprocess.Popen(["bash", "-c", launch_cmd], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("⏳ [Astra S] 카메라 하드웨어 초기화 중 (약 4~5초 소요)...")
        time.sleep(4.5)
        print("✅ [Astra S] 카메라 스트림 준비 완료 (320x240 RGB-D)")
        return cam_process
    except Exception as e:
        print(f"⚠️ 카메라 구동 실패: {e}")
        return None


class QwenFloorDetectorTestNode(Node):
    def __init__(self, target_query: str = "small box on the floor", target_standoff_m: float = 0.25):
        super().__init__('qwen_floor_detector_test_node')

        self.bridge = CvBridge()
        self.target_query = target_query
        self.target_standoff_m = float(target_standoff_m)

        # ----------------------------------------------------------------------
        # Camera & Robot Extrinsics (Identical to object_approaching_controller_node)
        # ----------------------------------------------------------------------
        self.astra_x_base = 0.080         # Lens front X from base_footprint (m)
        self.astra_y_base = 0.015         # Lens Y offset (+1.5cm left)
        self.astra_z_base = 0.147         # Lens Z height from floor (14.7cm calibrated)
        self.astra_pitch = 0.0

        # Camera Intrinsics
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0
        self.has_cam_info = False

        # Live Image Buffers
        self.last_color_img: Optional[np.ndarray] = None
        self.last_color_time: float = 0.0
        self.last_depth_raw: Optional[np.ndarray] = None
        self.last_depth_time: float = 0.0
        self.color_source: str = "NONE"
        self.depth_source: str = "NONE"

        # Tracking State (Identical to controller node)
        self.qwen_waiting: bool = False
        self.qwen_received_time: float = 0.0
        self.qwen_latency: float = 0.0
        self.tracked_bbox: Optional[Tuple[int, int, int, int]] = None
        self.locked_lab_color: Optional[np.ndarray] = None
        self.locked_obj_phys_width: float = 0.045
        self.obj_detected: bool = False
        self.obj_last_seen: float = 0.0
        self.active_mask: Optional[np.ndarray] = None

        # Distance & Lateral Error
        self.lens_distance: Optional[float] = None
        self.lateral_error: Optional[float] = None
        self.heading_error: Optional[float] = None
        self.dist_history: List[float] = []
        self.ey_history: List[float] = []

        # Stats
        self.fps: float = 0.0
        self._frame_count: int = 0
        self._fps_start_time: float = time.time()
        self._last_log_time: float = 0.0

        # Ensure Ollama daemon is running in background
        self._ensure_ollama_daemon()

        # Camera process reference (managed by ensure_camera_ready before rclpy.init)
        self.cam_process: Optional[subprocess.Popen] = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------------------------------------
        # ROS 2 Subscriptions & Publishers
        # ----------------------------------------------------------------------
        self.qwen_query_pub = self.create_publisher(String, '/qwen/detect_query', 10)
        self.create_subscription(String, '/qwen/target_bbox', self._on_ros_qwen_bbox, 10)

        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_cam_info, 10)

        # Color topics fallback
        self.create_subscription(Image, '/camera/color/image_decompressed', self._on_color_raw, sensor_qos)
        self.create_subscription(Image, '/camera/color/image_raw', self._on_color_raw, sensor_qos)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._on_color_comp, sensor_qos)

        # Depth topics fallback
        self.create_subscription(Image, '/camera/depth/image_decompressed', self._on_depth_raw, sensor_qos)
        self.create_subscription(Image, '/camera/depth/image_raw', self._on_depth_raw, sensor_qos)
        self.create_subscription(CompressedImage, '/camera/depth/image_raw/compressedDepth', self._on_depth_comp, sensor_qos)

        self.get_logger().info("=" * 70)
        self.get_logger().info("🔍 [Qwen Floor Object Diagnostic] Running (Zero-Motion Safe)")
        self.get_logger().info(f"   Target Query: '{self.target_query}'")
        self.get_logger().info(f"   Target Standoff: {self.target_standoff_m * 100:.1f} cm")
        self.get_logger().info("   Shortcuts: [SPACE] Lock-on Qwen  |  [R] Reset  |  [1/2/3] Change Query")
        self.get_logger().info("=" * 70)

    def _ensure_ollama_daemon(self):
        """Ollama 데몬이 실행 중인지 확인하고 미실행 시 백그라운드 자동 기동"""
        try:
            req = urllib.request.Request("http://127.0.0.1:11434/api/tags")
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                if resp.status == 200:
                    return
        except Exception:
            pass

        print(">> [Ollama] 데몬이 꺼져 있어 백그라운드 기동을 시도합니다...")
        try:
            subprocess.Popen(["ollama", "serve"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.5)
        except Exception as e:
            print(f"⚠️ Ollama 기동 확인 필요: {e}")

    def cleanup_camera(self):
        """종료 시 백그라운드로 띄운 카메라 프로세스 정리"""
        if self.cam_process is not None:
            print("🛑 자동 기동된 카메라 프로세스를 종료합니다...")
            self.cam_process.terminate()
            try:
                self.cam_process.wait(timeout=2.0)
            except Exception:
                self.cam_process.kill()
            subprocess.run("pkill -9 -f astra_camera_node 2>/dev/null || true", shell=True)
            subprocess.run("rm -f /dev/shm/orbbec* /dev/shm/fastrtps* /dev/shm/sem.* 2>/dev/null || true", shell=True)

    # ==========================================================================
    # Sensor Callbacks
    # ==========================================================================
    def _on_cam_info(self, msg: CameraInfo):
        if not self.has_cam_info and msg.k[0] > 1.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            self.has_cam_info = True

    def _on_color_raw(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_img is not None:
                self.last_color_img = cv_img
                self.last_color_time = time.time()
                self.color_source = "RAW_IMAGE"
        except Exception:
            pass

    def _on_color_comp(self, msg: CompressedImage):
        if time.time() - self.last_color_time > 0.4:
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_img is not None:
                    self.last_color_img = cv_img
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
                    d /= 1000.0
                self.last_depth_raw = d
                self.last_depth_time = time.time()
                self.depth_source = "RAW_DEPTH"
        except Exception:
            pass

    def _on_depth_comp(self, msg: CompressedImage):
        if time.time() - self.last_depth_time > 0.4:
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
                    self.depth_source = "COMPRESSED_DEPTH"
            except Exception:
                pass

    # ==========================================================================
    # Qwen2.5-VL Visual Grounding (Direct HTTP + ROS 2 Dual Support)
    # ==========================================================================
    def trigger_qwen_request(self, custom_query: Optional[str] = None):
        """Qwen2.5-VL 시각 검출 요청 (백그라운드 스레드로 비동기 처리하여 화면 멈춤 방지)"""
        if self.qwen_waiting:
            print(">> ⏳ 이전 Qwen 요청 처리 중입니다...")
            return

        if self.last_color_img is None:
            print(">> ⚠️ 카메라 영상이 아직 수신되지 않았습니다.")
            return

        if custom_query:
            self.target_query = custom_query

        self.qwen_waiting = True
        self.tracked_bbox = None
        self.locked_lab_color = None
        self.obj_detected = False

        # 1. Also publish on ROS 2 topic for integrated system
        self.qwen_query_pub.publish(String(data=self.target_query))

        # 2. Direct Ollama API async call
        threading.Thread(target=self._query_ollama_worker, args=(self.last_color_img.copy(), self.target_query), daemon=True).start()

    def _query_ollama_worker(self, img_bgr: np.ndarray, query: str):
        print(f"\n>> 🧠 [Qwen2.5-VL 요청 전송] '{query}' 탐색 중...")
        t0 = time.time()

        h, w = img_bgr.shape[:2]
        success, buffer = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            self.qwen_waiting = False
            return
        b64_img = base64.b64encode(buffer).decode('utf-8')

        # Updated prompt with floor target constraint and furniture rejection
        prompt = f"""Locate the foreground target object '{query}' resting on the floor in the lower half of the image.
Ignore background furniture, desk legs, chairs, walls, and cabinets.
Respond strictly in valid JSON format:
{{
  "target_found": true or false,
  "label": "detected object name",
  "bbox_2d": [xmin, ymin, xmax, ymax]
}}
Note: bbox_2d is [xmin, ymin, xmax, ymax] normalized integers [0, 1000] where (xmin, ymin) is top-left and (xmax, ymax) is bottom-right.
If not found, set target_found to false and bbox_2d to null.
"""

        payload = {
            "model": MODEL_NAME,
            "prompt": prompt,
            "images": [b64_img],
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1, "num_predict": 64},
            "keep_alive": "5m"
        }

        try:
            req = urllib.request.Request(
                OLLAMA_API_URL,
                data=json.dumps(payload).encode('utf-8'),
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=8.0) as resp:
                body = json.loads(resp.read().decode('utf-8'))
                raw_json = body.get("response", "{}")
                result = json.loads(raw_json)

            self.qwen_latency = time.time() - t0

            if result.get("target_found", False) and result.get("bbox_2d"):
                xmin_n, ymin_n, xmax_n, ymax_n = result["bbox_2d"]
                scale = max(w, h) / 1000.0
                xmin = max(0, min(w - 1, int(xmin_n * scale)))
                ymin = max(0, min(h - 1, int(ymin_n * scale)))
                xmax = max(0, min(w, int(xmax_n * scale)))
                ymax = max(0, min(h, int(ymax_n * scale)))
                if xmin > xmax: xmin, xmax = xmax, xmin
                if ymin > ymax: ymin, ymax = ymax, ymin

                self._apply_qwen_bbox([xmin, ymin, xmax, ymax], label=result.get("label", query))
            else:
                print(f">> ❌ [Qwen 응답] 물체를 찾지 못했습니다 ({self.qwen_latency:.2f}s).")
                self.qwen_waiting = False

        except Exception as e:
            print(f">> ⚠️ Ollama 통신 오류: {e}")
            self.qwen_waiting = False

    def _on_ros_qwen_bbox(self, msg: String):
        """llm_agent_node에서 전송하는 /qwen/target_bbox 수신 지원"""
        try:
            data = json.loads(msg.data)
            if data.get("target_found", False) and data.get("pixel_box"):
                self._apply_qwen_bbox(data["pixel_box"], label=data.get("label", "target"))
        except Exception:
            pass

    def _apply_qwen_bbox(self, pixel_box: List[int], label: str = ""):
        xmin, ymin, xmax, ymax = pixel_box
        w = max(8, xmax - xmin)
        h = max(8, ymax - ymin)
        self.tracked_bbox = (xmin, ymin, w, h)
        self.qwen_received_time = time.time()
        self.qwen_waiting = False

        # Extract Lab color signature from BBox center
        if self.last_color_img is not None:
            H, W = self.last_color_img.shape[:2]
            cx1 = max(0, int(xmin + w * 0.25))
            cy1 = max(0, int(ymin + h * 0.25))
            cx2 = min(W, int(xmin + w * 0.75))
            cy2 = min(H, int(ymin + h * 0.75))
            patch = self.last_color_img[cy1:cy2, cx1:cx2]
            if patch.size > 0:
                lab_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).astype(np.float32)
                self.locked_lab_color = np.median(lab_patch.reshape(-1, 3), axis=0)

            self._update_metrics_from_bbox(self.tracked_bbox, is_qwen_lock=True)
            print(f">> ✅ [Qwen 락온 성공!] Box=[{xmin},{ymin},{xmax},{ymax}] -> "
                  f"Dist={self.lens_distance * 100:.1f}cm, LatErr={self.lateral_error * 100:+.1f}cm ({self.qwen_latency:.2f}s)")

    # ==========================================================================
    # 20Hz Local ROI Visual Tracker (Color-contrast in +/-65px pad)
    # ==========================================================================
    def _track_local_roi(self, bgr_img: np.ndarray):
        H, W = bgr_img.shape[:2]
        x, y, w, h = self.tracked_bbox
        pad = 65

        rx1 = max(0, x - pad)
        ry1 = max(int(self.cy - 20), y - pad)
        rx2 = min(W, x + w + pad)
        ry2 = min(H, y + h + pad)

        roi = bgr_img[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return

        lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
        d_l = np.abs(lab_roi[:, :, 0] - self.locked_lab_color[0]) * 0.4
        d_a = np.abs(lab_roi[:, :, 1] - self.locked_lab_color[1]) * 1.4
        d_b = np.abs(lab_roi[:, :, 2] - self.locked_lab_color[2]) * 1.4
        diff = d_l + d_a + d_b

        mask = (diff < 36.0).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # Store for diagnostic visualization
        full_mask = np.zeros((H, W), dtype=np.uint8)
        full_mask[ry1:ry2, rx1:rx2] = mask
        self.active_mask = full_mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            if (time.time() - self.obj_last_seen) > 0.8:
                self.obj_detected = False
            return

        prev_cx = (x + w * 0.5) - rx1
        prev_cy = (y + h * 0.5) - ry1
        best_cnt = None
        best_score = 9999.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 35:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            dist_px = math.hypot((bx + bw * 0.5) - prev_cx, (by + bh * 0.5) - prev_cy)
            score = dist_px - 0.05 * area
            if score < best_score:
                best_score = score
                best_cnt = (bx + rx1, by + ry1, bw, bh)

        if best_cnt is not None:
            self.tracked_bbox = best_cnt
            self._update_metrics_from_bbox(best_cnt, is_qwen_lock=False)

    # ==========================================================================
    # Pinhole Ground-Contact Geometry & Depth Fusion
    # ==========================================================================
    def _update_metrics_from_bbox(self, bbox: Tuple[int, int, int, int], is_qwen_lock: bool = False):
        x, y, w, h = bbox
        u_center = x + w * 0.5
        v_bottom = y + h
        H_img = self.last_color_img.shape[0] if self.last_color_img is not None else 480

        # Ground-contact pinhole geometry
        if v_bottom < (H_img - 5):
            z_ground = (self.fy * self.astra_z_base) / max(v_bottom - self.cy, 12.0)
        else:
            z_ground = (self.fx * self.locked_obj_phys_width) / max(w, 12.0)

        # Depth fusion
        z_depth = self._query_bbox_depth(bbox)
        if z_depth is not None and abs(z_depth - z_ground) < 0.18:
            raw_z = 0.5 * z_ground + 0.5 * z_depth
        else:
            raw_z = z_ground

        if is_qwen_lock and v_bottom < (H_img - 10):
            self.locked_obj_phys_width = float(np.clip((w * raw_z) / self.fx, 0.025, 0.15))

        x_cam = (u_center - self.cx) * raw_z / self.fx
        x_base = self.astra_x_base + raw_z
        y_base = self.astra_y_base - x_cam

        self.dist_history.append(raw_z)
        if len(self.dist_history) > 4:
            self.dist_history.pop(0)
        self.lens_distance = float(np.median(self.dist_history))

        self.ey_history.append(y_base)
        if len(self.ey_history) > 4:
            self.ey_history.pop(0)
        self.lateral_error = float(np.median(self.ey_history))

        self.heading_error = math.atan2(self.lateral_error, max(0.10, self.lens_distance + self.astra_x_base))
        self.obj_detected = True
        self.obj_last_seen = time.time()

    def _query_bbox_depth(self, bbox: Tuple[int, int, int, int]) -> Optional[float]:
        if self.last_depth_raw is None or (time.time() - self.last_depth_time) > 1.0:
            return None
        x, y, w, h = bbox
        H, W = self.last_depth_raw.shape[:2]
        x1, x2 = max(0, int(x + w * 0.25)), min(W, int(x + w * 0.75))
        y1, y2 = max(0, int(y + h * 0.25)), min(H, int(y + h * 0.75))
        patch = self.last_depth_raw[y1:y2, x1:x2]
        if patch.size == 0:
            return None
        if self.last_depth_raw.dtype == np.uint16:
            valid = patch[(patch > 350) & (patch < 1500)]
            return (float(np.median(valid)) / 1000.0) if valid.size >= 6 else None
        else:
            valid = patch[(patch > 0.35) & (patch < 1.50) & np.isfinite(patch)]
            return float(np.median(valid)) if valid.size >= 6 else None

    # ==========================================================================
    # Rendering Frames
    # ==========================================================================
    def process_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self.last_color_img is None:
            return None, None

        bgr_img = self.last_color_img.copy()
        H, W = bgr_img.shape[:2]

        if not self.has_cam_info:
            self.cx = W / 2.0
            self.cy = H / 2.0

        if self.tracked_bbox is not None and self.locked_lab_color is not None:
            self._track_local_roi(bgr_img)

        # FPS
        self._frame_count += 1
        elapsed = time.time() - self._fps_start_time
        if elapsed >= 0.5:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start_time = time.time()

        # Terminal log every 1.0s
        now_t = time.time()
        if (now_t - self._last_log_time) >= 1.0:
            self._last_log_time = now_t
            if self.obj_detected and self.lens_distance is not None:
                print(f"[TEST] Tracked: YES | Dist: {self.lens_distance * 100:5.1f}cm (Target: {self.target_standoff_m * 100:.0f}cm) | "
                      f"ey: {self.lateral_error * 100:+5.1f}cm | FPS: {self.fps:4.1f}")
            elif self.qwen_waiting:
                print(f"[TEST] 🧠 Qwen2.5-VL VLM 추론 중... | FPS: {self.fps:4.1f}")
            else:
                print(f"[TEST] ⏸️ 대기 중 (스페이스바를 누르면 락온 시작) | FPS: {self.fps:4.1f}")

        # ----------------------------------------------------------------------
        # Render Main Live HUD
        # ----------------------------------------------------------------------
        vis = bgr_img.copy()

        # Robot True Centerline (Compensating Astra Y-offset +1.5cm)
        d_ref = self.lens_distance if (self.lens_distance and self.lens_distance > 0.15) else 0.25
        u_robot_center = int(self.cx + (self.fx * self.astra_y_base / d_ref))
        cv2.line(vis, (u_robot_center, int(self.cy)), (u_robot_center, H), (255, 255, 0), 2)
        cv2.putText(vis, "ROBOT CENTER", (max(10, u_robot_center - 45), H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # Target 25cm stop guideline
        v_target_25cm = int(self.cy + (self.fy * self.astra_z_base / max(self.target_standoff_m, 0.05)))
        if 0 <= v_target_25cm < H:
            cv2.line(vis, (0, v_target_25cm), (W, v_target_25cm), (0, 165, 255), 1, cv2.LINE_AA)
            cv2.putText(vis, f"--- Target Stop Line ({self.target_standoff_m * 100:.0f}cm) ---",
                        (20, v_target_25cm - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

        # Bounding Box & Local Search Window
        if self.tracked_bbox is not None and self.obj_detected:
            x, y, w, h = self.tracked_bbox
            # Local ROI search window (cyan/orange rectangle)
            cv2.rectangle(vis, (max(0, x - 65), max(0, y - 65)),
                          (min(W, x + w + 65), min(H, y + h + 65)), (0, 200, 255), 1)
            # Target object box (green)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(vis, (int(x + w * 0.5), int(y + h * 0.5)), 4, (0, 0, 255), -1)

            dist_cm = self.lens_distance * 100
            ey_cm = self.lateral_error * 100
            label = f"QWEN+ROI | d={dist_cm:.1f}cm | ey={ey_cm:+.1f}cm"
            cv2.putText(vis, label, (max(5, x - 15), max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        # Top Banner
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (W, 46), (15, 20, 30), -1)
        vis = cv2.addWeighted(overlay, 0.75, vis, 0.25, 0)

        status_text = "TRACKING LOCKED (20Hz)" if self.obj_detected else ("QWEN INFERENCE..." if self.qwen_waiting else "STANDBY (PRESS SPACE)")
        status_col = (0, 255, 0) if self.obj_detected else ((0, 200, 255) if self.qwen_waiting else (180, 180, 180))
        cv2.putText(vis, status_text, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_col, 2)
        cv2.putText(vis, f"Query: '{self.target_query}'", (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

        if self.obj_detected and self.lens_distance is not None:
            d_val = f"DIST: {self.lens_distance * 100:.1f}cm"
            ey_val = f"LAT_ERR: {self.lateral_error * 100:+.1f}cm"
            dist_col = (0, 255, 0) if abs(self.lens_distance - self.target_standoff_m) <= 0.03 else (255, 220, 100)
            cv2.putText(vis, d_val, (W // 2 - 40, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, dist_col, 2)
            cv2.putText(vis, ey_val, (W // 2 - 40, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

        cv2.putText(vis, f"FPS: {self.fps:.1f}", (W - 80, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 240, 255), 1)

        # Bottom Instructions
        cv2.putText(vis, "[SPACE] Lock Qwen | [R] Reset | [1/2/3] Query | [S] Snap | [Q] Quit", (10, H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1)

        # ----------------------------------------------------------------------
        # Render Diagnostic Mask & Depth View
        # ----------------------------------------------------------------------
        diag_vis = np.zeros((H, W, 3), dtype=np.uint8)

        # Active ROI color mask
        if self.active_mask is not None and self.obj_detected:
            diag_vis[:, :, 1] = self.active_mask  # Green channel

        # Depth Colormap Thumbnail
        if self.last_depth_raw is not None:
            depth_crop = self.last_depth_raw.copy()
            depth_valid = depth_crop[(depth_crop > 0.2) & (depth_crop < 2.0)]
            if depth_valid.size > 0:
                d_min, d_max = 0.2, 1.5
                d_norm = np.clip((depth_crop - d_min) / (d_max - d_min), 0.0, 1.0)
                d_color = cv2.applyColorMap((d_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                d_thumb = cv2.resize(d_color, (W // 2, H // 2))
                diag_vis[H - H // 2:H, W - W // 2:W] = d_thumb
                cv2.rectangle(diag_vis, (W - W // 2, H - H // 2), (W, H), (255, 255, 255), 1)
                cv2.putText(diag_vis, "Live Depth Colormap", (W - W // 2 + 5, H - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        cv2.putText(diag_vis, "[ROI Color Segment Mask]", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(diag_vis, "Green: Active 20Hz Target Region", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)

        return vis, diag_vis


def main():
    parser = argparse.ArgumentParser(description="Qwen Floor Object Diagnostic (Zero-Motion Safe)")
    parser.add_argument('--query', type=str, default="small box on the floor", help="Target object query for Qwen2.5-VL")
    parser.add_argument('--standoff', type=float, default=0.25, help="Target standoff distance in meters (default: 0.25)")
    args, unknown = parser.parse_known_args()

    # rclpy.init() 전에 카메라 및 공유 메모리 청소 선행 실행
    cam_proc = ensure_camera_ready()

    rclpy.init()
    node = QwenFloorDetectorTestNode(target_query=args.query, target_standoff_m=args.standoff)
    node.cam_process = cam_proc

    win_main = "TurtleBot3 - Qwen Floor Object Live HUD"
    win_diag = "Tracker Diagnostic Masks (ROI Color & Depth Map)"
    cv2.namedWindow(win_main, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(win_diag, cv2.WINDOW_AUTOSIZE)

    print("\n" + "=" * 70)
    print(" 🚀 [Qwen 바닥 물체 진단 시작] 카메라 스트림 대기 중...")
    print(f"   - 기본 타겟 질의어: '{node.target_query}'")
    print("   - [SPACEBAR] : Qwen2.5-VL 시각 검출 요청 및 20Hz 로컬 ROI 추적 락온")
    print("   - [1] : 'small box on the floor' (상자 탐색)")
    print("   - [2] : 'cube on the floor' (큐브 탐색)")
    print("   - [3] : 'ball on the floor' (공 탐색)")
    print("   - [R] : 락온 해제 및 리셋")
    print("   - [S] : 스냅샷 저장")
    print("   - [Q] 또는 [ESC] : 종료")
    print("=" * 70 + "\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            hud_frame, diag_frame = node.process_frames()
            if hud_frame is not None and diag_frame is not None:
                cv2.imshow(win_main, hud_frame)
                cv2.imshow(win_diag, diag_frame)
            else:
                # 카메라 초기 연결 대기 화면
                wait_hud = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(wait_hud, "Astra Camera Connecting...", (25, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
                cv2.putText(wait_hud, "Waiting for ROS2 image stream...", (25, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)
                cv2.imshow(win_main, wait_hud)

            key = cv2.waitKey(20) & 0xFF
            if key == ord('q') or key == 27:
                print("\n>> 테스트를 종료합니다.")
                break
            elif key == 32:  # SPACE BAR
                node.trigger_qwen_request()
            elif key == ord('1'):
                print("\n>> [질의 변경] 'small box on the floor'로 변경 후 즉시 락온합니다.")
                node.trigger_qwen_request("small box on the floor")
            elif key == ord('2'):
                print("\n>> [질의 변경] 'cube on the floor'로 변경 후 즉시 락온합니다.")
                node.trigger_qwen_request("cube on the floor")
            elif key == ord('3'):
                print("\n>> [질의 변경] 'ball on the floor'로 변경 후 즉시 락온합니다.")
                node.trigger_qwen_request("ball on the floor")
            elif key == ord('r'):
                node.tracked_bbox = None
                node.locked_lab_color = None
                node.obj_detected = False
                node.active_mask = None
                print("🔄 [리셋] 타겟 락온을 해제하고 대기 상태로 전환했습니다.")
            elif key == ord('s'):
                snap_dir = os.path.join(PROJECT_ROOT, "target_images")
                os.makedirs(snap_dir, exist_ok=True)
                t_str = time.strftime("%Y%m%d_%H%M%S")
                snap_hud = os.path.join(snap_dir, f"snap_qwen_hud_{t_str}.png")
                snap_diag = os.path.join(snap_dir, f"snap_qwen_diag_{t_str}.png")
                if hud_frame is not None and diag_frame is not None:
                    cv2.imwrite(snap_hud, hud_frame)
                    cv2.imwrite(snap_diag, diag_frame)
                    print(f"📸 [스냅샷 저장] {snap_hud}")

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.cleanup_camera()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
