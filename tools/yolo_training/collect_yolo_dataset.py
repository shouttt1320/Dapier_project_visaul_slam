#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# YOLO Fine-Tuning Multi-Class Dataset Collector & Video Slicer
# - Real-time ROS 2 Camera Stream (Raw / Compressed / Decompressed)
# - Interactive Label Switching, Manual Snap, Interval Auto-Capture, Video Recording
# - Automatic Video Slicing with Blurry Frame Filter (Laplacian variance)
# - Generates YOLO-ready directory structure & data.yaml
# ==============================================================================

import os
import sys
import time
import json
import argparse
import subprocess
from typing import Optional, Tuple, List, Dict, Any
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_DATASET_DIR = os.path.join(PROJECT_ROOT, "dataset_capture")


class DatasetCollectorNode(Node):
    def __init__(self, output_dir: str = DEFAULT_DATASET_DIR, initial_label: str = "red_cube", auto_start_camera: bool = True):
        super().__init__('dataset_collector_node')

        self.bridge = CvBridge()
        self.output_dir = os.path.abspath(output_dir)
        self.current_label = initial_label.strip()
        self.cam_process: Optional[subprocess.Popen] = None

        # Directories
        self.img_dir = os.path.join(self.output_dir, "images")
        self.video_dir = os.path.join(self.output_dir, "raw_videos")
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.video_dir, exist_ok=True)

        # Label counts
        self.label_counts: Dict[str, int] = {}
        self._scan_existing_images()

        # Capture states
        self.is_recording = False
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.current_video_path = ""
        self.rec_start_time = 0.0
        self.rec_frame_count = 0

        self.auto_interval_enabled = False
        self.auto_interval_sec = 0.8
        self.last_auto_capture_time = 0.0

        # Live Image Buffers
        self.last_color_img: Optional[np.ndarray] = None
        self.last_color_time: float = 0.0
        self.last_depth_raw: Optional[np.ndarray] = None
        self.color_source: str = "NONE"
        self.img_w: int = 320
        self.img_h: int = 240

        # Camera & Robot Geometry
        self.astra_z_base = 0.147  # Lens height from floor (14.7cm)
        self.fx = 285.17
        self.fy = 285.17
        self.cx = 159.5
        self.cy = 240.0
        self.has_cam_info = False

        # Stats
        self.fps: float = 0.0
        self._frame_count: int = 0
        self._fps_start_time: float = time.time()

        # Auto-launch Astra S camera if plugged in via USB and driver not yet running
        if auto_start_camera:
            self._ensure_camera_running()

        # ROS 2 Subscriptions
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_cam_info, 10)
        self.create_subscription(Image, '/camera/color/image_decompressed', self._on_color_raw, sensor_qos)
        self.create_subscription(Image, '/camera/color/image_raw', self._on_color_raw, sensor_qos)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._on_color_comp, sensor_qos)
        self.create_subscription(Image, '/camera/depth/image_decompressed', self._on_depth_raw, sensor_qos)
        self.create_subscription(Image, '/camera/depth/image_raw', self._on_depth_raw, sensor_qos)
        self.create_subscription(CompressedImage, '/camera/depth/image_raw/compressedDepth', self._on_depth_comp, sensor_qos)

        self.get_logger().info(f"📁 Dataset Output Directory: {self.output_dir}")
        self.get_logger().info(f"🏷️ Current Active Class: '{self.current_label}'")

    def _scan_existing_images(self):
        """Scans output directory to resume counts"""
        for entry in os.listdir(self.img_dir):
            label_folder = os.path.join(self.img_dir, entry)
            if os.path.isdir(label_folder):
                imgs = [f for f in os.listdir(label_folder) if f.lower().endswith(('.jpg', '.png'))]
                self.label_counts[entry] = len(imgs)

    def _ensure_camera_running(self):
        try:
            my_pid = str(os.getpid())
            pg = subprocess.run(["pgrep", "-f", "astra_camera_node"], stdout=subprocess.PIPE, text=True)
            pids = [p.strip() for p in pg.stdout.strip().split() if p.strip() and p.strip() != my_pid]
            if pids:
                self.get_logger().info(f"📷 astra_camera_node가 이미 실행 중입니다 (PID: {pids}).")
                return
        except Exception:
            pass

        try:
            ls = subprocess.run(["lsusb"], stdout=subprocess.PIPE, text=True)
            if "2bc5:" not in ls.stdout:
                return
        except Exception:
            return

        self.get_logger().info("📷 [Astra S USB 감지] 카메라 드라이버를 백그라운드에서 자동 구동합니다...")
        env = os.environ.copy()
        extra_ld = "/home/jjj/.local/lib:/home/jjj/Documents/Dapier/Project/robot_ws/install/astra_camera_msgs/lib"
        env["LD_LIBRARY_PATH"] = f"{extra_ld}:{env.get('LD_LIBRARY_PATH', '')}"

        ws_setup = "/home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash"
        launch_cmd = (
            f"export LD_LIBRARY_PATH={extra_ld}:$LD_LIBRARY_PATH && "
            f"source /opt/ros/jazzy/setup.bash && "
            f"source {ws_setup} && "
            f"ros2 launch astra_camera astra.launch.xml "
            f"depth_width:=320 depth_height:=240 color_width:=320 color_height:=240 "
            f"enable_point_cloud:=false depth_registration:=true"
        )
        try:
            self.cam_process = subprocess.Popen(["bash", "-c", launch_cmd], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(3.5)
            self.get_logger().info("✅ 카메라 드라이버 백그라운드 구동 완료 (320x240 RGB-D)")
        except Exception as e:
            self.get_logger().warn(f"카메라 구동 실패: {e}")

    def cleanup_camera(self):
        if self.is_recording:
            self.stop_recording()
        if self.cam_process is not None:
            self.get_logger().info("🛑 자동 기동된 카메라 프로세스를 종료합니다...")
            self.cam_process.terminate()
            try:
                self.cam_process.wait(timeout=2.0)
            except Exception:
                self.cam_process.kill()
            subprocess.run(["pkill", "-9", "-f", "astra_camera_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # --------------------------------------------------------------------------
    # Subscriptions
    # --------------------------------------------------------------------------
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
                self.color_source = "RAW"
                self.img_h, self.img_w = cv_img.shape[:2]
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
                    self.img_h, self.img_w = cv_img.shape[:2]
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
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # Capture & Recording Operations
    # --------------------------------------------------------------------------
    def set_label(self, new_label: str):
        lbl = new_label.strip().replace(" ", "_")
        if lbl:
            self.current_label = lbl
            print(f"\n>> 🏷️ [현재 라벨 변경] '{self.current_label}' (현재 수집된 장수: {self.get_current_count()}장)")

    def get_current_count(self) -> int:
        return self.label_counts.get(self.current_label, 0)

    def save_single_frame(self, frame: np.ndarray, note: str = "manual") -> Optional[str]:
        """Saves a frame to disk under the active class folder"""
        label_dir = os.path.join(self.img_dir, self.current_label)
        os.makedirs(label_dir, exist_ok=True)

        current_cnt = self.label_counts.get(self.current_label, 0) + 1
        t_str = time.strftime("%Y%m%d_%H%M%S")
        ms_str = f"{int((time.time() % 1) * 1000):03d}"
        filename = f"{self.current_label}_{t_str}_{ms_str}.jpg"
        save_path = os.path.join(label_dir, filename)

        cv2.imwrite(save_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        self.label_counts[self.current_label] = current_cnt

        # Log metadata
        meta_path = os.path.join(self.output_dir, "metadata.jsonl")
        meta = {
            "file": os.path.relpath(save_path, self.output_dir),
            "label": self.current_label,
            "timestamp": time.time(),
            "note": note,
            "resolution": [frame.shape[1], frame.shape[0]]
        }
        with open(meta_path, "a") as f:
            f.write(json.dumps(meta) + "\n")

        print(f"📸 [저장 완료 #{current_cnt}] {self.current_label}/{filename} ({note})")
        return save_path

    def start_recording(self):
        if self.is_recording or self.last_color_img is None:
            return
        t_str = time.strftime("%Y%m%d_%H%M%S")
        video_name = f"rec_{self.current_label}_{t_str}.mp4"
        self.current_video_path = os.path.join(self.video_dir, video_name)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        H, W = self.last_color_img.shape[:2]
        fps = 20.0
        self.video_writer = cv2.VideoWriter(self.current_video_path, fourcc, fps, (W, H))
        self.is_recording = True
        self.rec_start_time = time.time()
        self.rec_frame_count = 0
        print(f"\n🔴 [영상 녹화 시작] '{self.current_label}' -> {video_name}")
        print("   (다양한 각도, 높이, 0.4m ~ 3.0m 거리를 오가며 물체를 비추세요!)")

    def stop_recording(self):
        if not self.is_recording:
            return
        self.is_recording = False
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        dur = time.time() - self.rec_start_time
        print(f"\n⏹️ [영상 녹화 종료] 저장됨: {self.current_video_path}")
        print(f"   (총 {self.rec_frame_count} 프레임, {dur:.1f}초)")

        # Auto slice inquiry
        print(f">> [슬라이스 추천] 'python3 diagnostics/collect_yolo_dataset.py --slice {self.current_video_path}'")

    def update_recording(self, frame: np.ndarray):
        if self.is_recording and self.video_writer is not None:
            self.video_writer.write(frame)
            self.rec_frame_count += 1

    # --------------------------------------------------------------------------
    # HUD Rendering
    # --------------------------------------------------------------------------
    def render_hud(self) -> Optional[np.ndarray]:
        now = time.time()
        SCALE = 2.0  # 320x240 -> 640x480 for clear and comfortable viewing

        if self.last_color_img is None:
            # Show a helpful splash screen immediately so the OpenCV window opens on launch!
            splash_w, splash_h = int(320 * SCALE), int(240 * SCALE)
            splash = np.zeros((splash_h, splash_w, 3), dtype=np.uint8)
            splash[:] = (25, 20, 15)  # Dark navy

            dots = "." * (int(now * 3) % 4)
            cv2.putText(splash, "YOLO DATASET COLLECTOR", (40, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (56, 189, 248), 2)
            cv2.putText(splash, f"Connecting to Astra S Camera{dots}", (40, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1)
            cv2.putText(splash, f"Active Class: [{self.current_label}]", (40, 255),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (74, 222, 128), 2)
            cv2.putText(splash, "Waiting for image topics (/camera/color/image_raw)...", (40, 310),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (148, 163, 184), 1)
            cv2.putText(splash, "Press [Q] or ESC to exit", (40, 340),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 116, 139), 1)
            return splash

        raw_bgr = self.last_color_img.copy()
        H, W = raw_bgr.shape[:2]

        # Check sharpness (Laplacian variance)
        gray = cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2GRAY)
        blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
        is_blurry = blur_score < 60.0

        # Auto-interval capture check
        if self.auto_interval_enabled and (now - self.last_auto_capture_time) >= self.auto_interval_sec:
            self.last_auto_capture_time = now
            if not is_blurry:
                self.save_single_frame(raw_bgr, note="auto_interval")

        # Record raw frame to video if recording
        if self.is_recording:
            self.update_recording(raw_bgr)

        # FPS
        self._frame_count += 1
        elapsed = now - self._fps_start_time
        if elapsed >= 0.5:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start_time = now

        # Upscale for HUD visualization
        up_w = int(W * SCALE)
        up_h = int(H * SCALE)
        vis = cv2.resize(raw_bgr, (up_w, up_h), interpolation=cv2.INTER_LINEAR)

        # 1. Distance Horizon Reference Lines (0.5m, 1.0m, 2.0m, 3.0m)
        for dist_m, col in [(0.5, (0, 200, 255)), (1.0, (0, 255, 0)), (2.0, (255, 200, 0)), (3.0, (255, 100, 100))]:
            v_line = int((self.cy + (self.fy * self.astra_z_base / dist_m)) * SCALE)
            if 0 <= v_line < up_h:
                cv2.line(vis, (0, v_line), (up_w, v_line), col, 1, cv2.LINE_AA)
                cv2.putText(vis, f"~{dist_m:.1f}m Floor Guide", (10, v_line - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1)

        # 2. Top Banner
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (up_w, 54), (20, 25, 35), -1)
        vis = cv2.addWeighted(overlay, 0.75, vis, 0.25, 0)

        # Active Class & Count
        cnt = self.get_current_count()
        cv2.putText(vis, f"CLASS: [{self.current_label.upper()}]  ({cnt} frames)",
                    (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)

        # Stream & Sharpness
        blur_txt = "BLURRY" if is_blurry else "SHARP"
        blur_col = (0, 100, 255) if is_blurry else (0, 255, 100)
        cv2.putText(vis, f"Stream: {W}x{H} ({self.color_source}) | Focus: {blur_txt} ({blur_score:.0f})",
                    (12, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40, blur_col, 1)

        # Recording Status
        if self.is_recording:
            dur = now - self.rec_start_time
            if int(now * 2) % 2 == 0:
                cv2.circle(vis, (up_w - 150, 26), 7, (0, 0, 255), -1)
            cv2.putText(vis, f"REC {dur:.1f}s ({self.rec_frame_count}f)",
                        (up_w - 136, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 0, 255), 2)
        elif self.auto_interval_enabled:
            cv2.putText(vis, "AUTO-SNAP: ON", (up_w - 160, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 0), 2)
        else:
            cv2.putText(vis, f"FPS: {self.fps:.1f}", (up_w - 100, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)

        # 3. Bottom Key Guide
        cv2.rectangle(vis, (0, up_h - 26), (up_w, up_h), (15, 20, 30), -1)
        cv2.putText(vis, "[SPACE] Snap | [R] Record Video | [A] Auto-Snap | [L] Change Class | [Q] Exit",
                    (10, up_h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)

        return vis


# ==============================================================================
# Standalone Video Slicer / Frame Extractor
# ==============================================================================
def slice_video_to_images(video_path: str, output_dir: str, target_label: str, frame_interval: int = 5, blur_thresh: float = 50.0):
    """
    Extracts sharp frames from a recorded video clip into the label image folder.
    Avoids duplicate and blurry frames.
    """
    if not os.path.exists(video_path):
        print(f"❌ 동영상 파일을 찾을 수 없습니다: {video_path}")
        return

    label_dir = os.path.join(output_dir, "images", target_label)
    os.makedirs(label_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"\n✂️ [동영상 이미지 슬라이싱 시작]")
    print(f"   - 입력 영상: {video_path}")
    print(f"   - 총 프레임: {total_frames} (FPS: {fps:.1f})")
    print(f"   - 추출 간격: 매 {frame_interval}번째 프레임 추출")
    print(f"   - 흐림(Blur) 필터 임계값: >= {blur_thresh:.1f}")
    print(f"   - 저장 위치: {label_dir}")

    extracted_cnt = 0
    skipped_blur = 0
    frame_idx = 0
    t_base = time.strftime("%Y%m%d_%H%M%S")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        if frame_idx % frame_interval != 0:
            continue

        # Check sharpness
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        score = cv2.Laplacian(gray, cv2.CV_64F).var()
        if score < blur_thresh:
            skipped_blur += 1
            continue

        extracted_cnt += 1
        out_name = f"{target_label}_{t_base}_slice_{extracted_cnt:04d}.jpg"
        out_path = os.path.join(label_dir, out_name)
        cv2.imwrite(out_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

    cap.release()
    print(f"✅ 슬라이싱 완료! 총 {extracted_cnt}장 유효 이미지 저장 (흐림으로 제외된 프레임: {skipped_blur}장)\n")


# ==============================================================================
# Generate Initial YOLO data.yaml Template
# ==============================================================================
def generate_data_yaml(output_dir: str):
    img_dir = os.path.join(output_dir, "images")
    classes = [d for d in sorted(os.listdir(img_dir)) if os.path.isdir(os.path.join(img_dir, d))]
    if not classes:
        classes = ["red_cube", "blue_ball"]

    yaml_content = f"""# YOLOv8 / YOLO-World Fine-Tuning Dataset Configuration
path: {output_dir}
train: images
val: images

names:
"""
    for idx, c in enumerate(classes):
        yaml_content += f"  {idx}: {c}\n"

    yaml_path = os.path.join(output_dir, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"📄 [YOLO data.yaml 템플릿 생성] {yaml_path}")
    print(f"   - 등록된 클래스: {classes}")


# ==============================================================================
# Main GUI Loop
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="YOLO Fine-Tuning Dataset Collector")
    parser.add_argument('--output', type=str, default=DEFAULT_DATASET_DIR,
                        help=f"Dataset output directory (default: {DEFAULT_DATASET_DIR})")
    parser.add_argument('--label', type=str, default="red_cube",
                        help="Initial class label (e.g. red_cube, blue_ball)")
    parser.add_argument('--no-cam', action='store_true',
                        help="Do not auto-start camera driver")
    parser.add_argument('--slice', type=str, default="",
                        help="Path to an mp4 video file to slice into images")
    parser.add_argument('--slice-label', type=str, default="",
                        help="Class label for sliced images (default: matches --label)")
    parser.add_argument('--interval', type=int, default=5,
                        help="Frame extraction interval for video slicing (default: 5)")
    args, unknown = parser.parse_known_args()

    # If standalone video slicing requested
    if args.slice:
        lbl = args.slice_label if args.slice_label else args.label
        slice_video_to_images(args.slice, args.output, lbl, frame_interval=args.interval)
        generate_data_yaml(args.output)
        return

    rclpy.init()
    node = DatasetCollectorNode(
        output_dir=args.output,
        initial_label=args.label,
        auto_start_camera=not args.no_cam
    )

    win_name = "YOLO Dataset Collector - Live Viewport"
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)

    print("\n" + "=" * 75)
    print(" 📸 [YOLO 파인튜닝 데이터 수집 도구 시작]")
    print(f"   - 저장 디렉토리 : {node.output_dir}")
    print(f"   - 시작 라벨     : {node.current_label}")
    print("   - [SPACE]       : 단일 고화질 스냅샷 즉시 저장")
    print("   - [R]           : 고속 동영상(MP4) 녹화 시작 / 정지")
    print("   - [A]           : 0.8초 주기 자동 캡처(Auto-Snap) ON / OFF")
    print("   - [L]           : 활성 클래스 라벨 변경 (터미널 입력)")
    print("   - [Q] / ESC     : 프로그램 종료 및 data.yaml 자동 생성")
    print("=" * 75 + "\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            vis_frame = node.render_hud()
            if vis_frame is not None:
                cv2.imshow(win_name, vis_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("\n>> 데이터 수집을 종료합니다.")
                break
            elif key == 32:  # SPACE
                if node.last_color_img is not None:
                    node.save_single_frame(node.last_color_img, note="manual_space")
            elif key == ord('r'):
                if not node.is_recording:
                    node.start_recording()
                else:
                    node.stop_recording()
            elif key == ord('a'):
                node.auto_interval_enabled = not node.auto_interval_enabled
                state_str = "ON (0.8초마다 자동 저장)" if node.auto_interval_enabled else "OFF"
                print(f"\n>> ⏱️ [자동 연속 캡처] {state_str}")
            elif key == ord('l'):
                print("\n>> 변경할 클래스 이름을 입력하세요 (예: red_cube, blue_ball, snack_box):")
                try:
                    new_lbl = input(">> 새 라벨: ").strip()
                    if new_lbl:
                        node.set_label(new_lbl)
                except Exception:
                    pass

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        generate_data_yaml(node.output_dir)
        node.cleanup_camera()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
