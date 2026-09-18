#!/usr/bin/env python3
"""
================================================================================
RedBox TurtleBot3 Mode Switcher & Live HUD Dashboard (PyQt5 + ROS 2 Jazzy)
================================================================================
Features:
  1. Ultra-Low-Latency Compressed Video Stream Viewport (/docking/debug_image/compressed)
  2. One-Click Mode Switcher:
     - Nav2 Autonomous Navigation Mode (Astra S + RTAB-Map SLAM Active)
     - Precision ArUco Docking Mode (Visual Servoing + Bumper Contact)
  3. Real-Time Telemetry: Distance, Yaw Error, Lateral Offset (ey), Bumper State
  4. Safety Controls: Abort & 24cm Safe Reverse, Instant E-STOP (TwistStamped)
================================================================================
"""

import sys
import math
import time
from datetime import datetime

import cv2
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QTextEdit, QFrame, QGroupBox,
    QGridLayout, QSizePolicy, QMessageBox
)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import CompressedImage, Image


class RosSignalBridge(QObject):
    """Bridge for thread-safe UI updates from ROS callbacks."""
    new_frame = pyqtSignal(np.ndarray)
    log_msg = pyqtSignal(str)


class DockingGuiROSNode(Node):
    """ROS 2 Bridge Node for the GUI Dashboard."""

    def __init__(self, bridge: RosSignalBridge):
        super().__init__('docking_gui_node')
        self.bridge = bridge

        # State Variables
        self.current_mode = "UNKNOWN"
        self.fsm_state = "NAV2_READY"
        self.desk_dist = None
        self.desk_yaw = None
        self.lateral_err = None
        self.target_valid = False
        self.dual_markers_visible = False
        self.odom_target_latched = False
        self.odom_tracking_active = False
        self.contact_detected = False
        self.sensor_mode = "NONE"
        self.backup_reason = "NONE"
        self._last_emitted_reason = ""
        self.docking_timeout_sec = 35.0
        self.docking_start_time = None
        self.last_frame_time = None
        self.fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()

        # 1. Subscriptions
        self.sub_mode = self.create_subscription(
            String, '/current_camera_mode', self._mode_callback, 10
        )
        self.sub_status = self.create_subscription(
            String, '/docking/status', self._status_callback, 10
        )

        # Compressed Video Stream (Zero-lag over Wi-Fi)
        self.sub_img_comp = self.create_subscription(
            CompressedImage,
            '/docking/debug_image/compressed',
            self._compressed_image_callback,
            qos_profile_sensor_data
        )

        # Fallback Raw Video Stream
        self.sub_img_raw = self.create_subscription(
            Image,
            '/docking/debug_image',
            self._raw_image_callback,
            qos_profile_sensor_data
        )

        # 2. Publishers (geometry_msgs/msg/TwistStamped for OpenCR turtlebot3_ros in ROS 2 Jazzy)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # 3. Service Clients
        self.cli_start_docking = self.create_client(Trigger, '/start_docking')
        self.cli_undock = self.create_client(Trigger, '/undock_to_nav2')
        self.cli_abort = self.create_client(Trigger, '/abort_docking')
        self.cli_estop = self.create_client(Trigger, '/emergency_stop')

        self.get_logger().info("DockingGuiROSNode initialized successfully.")

    def _mode_callback(self, msg: String):
        self.current_mode = msg.data.strip()

    def _status_callback(self, msg: String):
        # Format: STATE=..., dist=..., yaw_err=..., ey=..., target=..., contact=...
        try:
            tokens = msg.data.split(',')
            for token in tokens:
                kv = token.strip().split('=')
                if len(kv) == 2:
                    k, v = kv[0].strip(), kv[1].strip()
                    if k == 'STATE':
                        old_state = self.fsm_state
                        self.fsm_state = v
                        if old_state != v:
                            STATE_DESC = {
                                'NAV2_READY': '대기 모드 (자율주행 대기)',
                                'CHECK_STAGING': '접근 시작 및 스테이징 거리/편차 평가',
                                'STAGING_SETTLE': '후진 후 1초 정지 대기 (진동 소멸 및 마커 정적 계측)',
                                'STAGING_AIM': '당근선(Carrot Vector) 방향 제자리 조향 조준',
                                'CARROT_ALIGN': '당근점(Carrot) 기반 중심선 수렴 주행',
                                'STANDOFF_SETTLE': '스탠드오프 정지 계측 (마커 방향 정렬 후 정적 판별)',
                                'ALIGN_PARALLEL': '책상 정면 헤딩 평행 정렬',
                                'CRAWL_CONTACT': '최종 접근 (범퍼 접촉 대기 직진 크롤)',
                                'DOCKED': '도킹 완료 (모터 정지 및 락)',
                                'BACKUP_STANDOFF': '안전 거리 확보 후진 (52cm 스테이징)',
                                'BACKUP_RETRY': '재시도 후진 (12.5cm)',
                                'SEARCH_MARKER': '마커 탐색 스위프 회전',
                                'ABORT_TO_NAV2': '도킹 중단 및 24cm 안전 후진'
                            }
                            desc = STATE_DESC.get(v, v)
                            self.bridge.log_msg.emit(f"🔄 [단계 전환] {old_state} ➜ {v} ({desc})")

                        if v in ('CHECK_STAGING', 'STAGING_SETTLE', 'STAGING_AIM', 'BACKUP_STANDOFF', 'BACKUP_PREPARE',
                                 'CARROT_ALIGN', 'STANDOFF_SETTLE', 'BACKUP_RETRY', 'CRAWL_CONTACT'):
                            if self.docking_start_time is None:
                                self.docking_start_time = time.time()
                        elif v in ('DOCKED', 'NAV2_READY', 'FAILSAFE', 'ABORT_TO_NAV2', 'INIT'):
                            self.docking_start_time = None
                    elif k == 'dist':
                        v_c = v.replace('m', '').replace('cm', '').strip()
                        self.desk_dist = float(v_c) if v_c not in ('None', '') else None
                    elif k == 'yaw_err':
                        v_c = v.replace('deg', '').replace('rad', '').strip()
                        self.desk_yaw = float(v_c) if v_c not in ('None', '') else None
                    elif k == 'ey':
                        v_c = v.replace('cm', '').replace('m', '').strip()
                        self.lateral_err = float(v_c) if v_c not in ('None', '') else None
                    elif k == 'target':
                        self.target_valid = (v == 'True')
                    elif k == 'dual':
                        self.dual_markers_visible = (v in ('1', 'True'))
                    elif k == 'latched':
                        self.odom_target_latched = (v in ('1', 'True'))
                    elif k == 'odom_track':
                        self.odom_tracking_active = (v in ('1', 'True'))
                    elif k == 'contact':
                        self.contact_detected = (v == 'True')
                    elif k == 'sensor':
                        self.sensor_mode = v.strip()
                    elif k == 'reason':
                        self.backup_reason = v.strip()
                        if self.backup_reason and self.backup_reason not in ('NONE', ''):
                            if self.fsm_state in ('BACKUP_STANDOFF', 'BACKUP_RETRY', 'BACKUP_PREPARE'):
                                if self._last_emitted_reason != self.backup_reason:
                                    self._last_emitted_reason = self.backup_reason
                                    self.bridge.log_msg.emit(f"⚠️ [후진 원인] {self.backup_reason}")
                            else:
                                self._last_emitted_reason = ""
        except Exception:
            pass

    def _compressed_image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                self._update_fps()
                self.bridge.new_frame.emit(img)
        except Exception:
            pass

    def _raw_image_callback(self, msg: Image):
        # Only use raw if compressed hasn't arrived recently (> 1.5s)
        if self.last_frame_time is not None and (time.time() - self.last_frame_time) < 1.0:
            return
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ('bgr8', 'rgb8'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self._update_fps()
                self.bridge.new_frame.emit(img)
        except Exception:
            pass

    def _update_fps(self):
        self.last_frame_time = time.time()
        self._frame_count += 1
        now = time.time()
        dt = now - self._fps_time
        if dt >= 1.0:
            self.fps = self._frame_count / dt
            self._frame_count = 0
            self._fps_time = now

    def call_start_docking(self, callback):
        if self.cli_start_docking.service_is_ready():
            req = Trigger.Request()
            future = self.cli_start_docking.call_async(req)
            future.add_done_callback(callback)
            return True
        return False

    def call_undock_to_nav2(self, callback):
        if self.cli_undock.service_is_ready():
            req = Trigger.Request()
            future = self.cli_undock.call_async(req)
            future.add_done_callback(callback)
            return True
        return False

    def call_abort_docking(self, callback):
        if self.cli_abort.service_is_ready():
            req = Trigger.Request()
            future = self.cli_abort.call_async(req)
            future.add_done_callback(callback)
            return True
        return False

    def emergency_stop(self):
        # 1. Stop precision docking state machine if active
        if self.cli_estop.service_is_ready():
            self.cli_estop.call_async(Trigger.Request())
        elif self.cli_abort.service_is_ready():
            self.cli_abort.call_async(Trigger.Request())

        # 2. Publish 0-velocity directly to OpenCR /cmd_vel (TwistStamped)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)


class MainWindow(QMainWindow):
    """Main PyQt5 Mission Control Application Window."""

    def __init__(self, ros_node: DockingGuiROSNode, bridge: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.bridge = bridge

        self.setWindowTitle("RedBox TurtleBot3 Mission Control & Mode Switcher")
        self.setMinimumSize(920, 780)
        self.resize(1020, 840)

        self._setup_ui()
        self._apply_dark_theme()

        # Connect signals
        self.bridge.new_frame.connect(self._on_new_frame)
        self.bridge.log_msg.connect(self.log)

        # Qt Timer to spin ROS 2 callbacks at 30Hz
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._ros_spin_tick)
        self.ros_timer.start(33)

        # UI Update Timer at 10Hz
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_ui_state)
        self.ui_timer.start(100)

        self.log("Mission Control GUI launched. Real-time compressed video stream listening...")

    def _setup_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(12)

        # ----------------------------------------------------------------------
        # Header Section
        # ----------------------------------------------------------------------
        header_frame = QFrame()
        header_frame.setObjectName("headerFrame")
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(16, 10, 16, 10)

        title_vbox = QVBoxLayout()
        title_vbox.setSpacing(3)
        title_label = QLabel("🤖 REDBOX TURTLEBOT3 MISSION CONTROL")
        title_label.setObjectName("mainTitle")
        subtitle_label = QLabel("Visual SLAM Autonomous Navigation ⇄ Precision ArUco Docking Dashboard")
        subtitle_label.setObjectName("subTitle")
        title_vbox.addWidget(title_label)
        title_vbox.addWidget(subtitle_label)
        header_layout.addLayout(title_vbox)

        header_layout.addStretch()

        # Connection & Stream Badge
        self.lbl_stream_badge = QLabel("⚡ WI-FI COMPRESSED STREAM: ACTIVE")
        self.lbl_stream_badge.setObjectName("badgeStream")
        header_layout.addWidget(self.lbl_stream_badge)

        main_layout.addWidget(header_frame)

        # ----------------------------------------------------------------------
        # Upper Section: Mode Selector & Live Video Stream Viewport
        # ----------------------------------------------------------------------
        upper_layout = QHBoxLayout()
        upper_layout.setSpacing(14)

        # Left Column: Mode Selection Cards & Action Buttons
        left_col = QVBoxLayout()
        left_col.setSpacing(10)

        mode_box = QGroupBox("운전 모드 원클릭 전환 (MODE SELECTOR)")
        mode_box.setObjectName("modeBox")
        mode_layout = QVBoxLayout(mode_box)
        mode_layout.setSpacing(10)
        mode_layout.setContentsMargins(12, 14, 12, 12)

        # 1. Nav2 Mode Button Card
        self.btn_nav2 = QPushButton()
        self.btn_nav2.setObjectName("btnNav2")
        self.btn_nav2.setCursor(Qt.PointingHandCursor)
        self.btn_nav2.setMinimumHeight(95)
        btn_nav2_layout = QVBoxLayout(self.btn_nav2)
        btn_nav2_layout.setContentsMargins(12, 8, 12, 8)
        btn_nav2_layout.setSpacing(3)

        self.lbl_nav2_title = QLabel("🌐 Nav2 자율주행 모드")
        self.lbl_nav2_title.setObjectName("cardTitle")
        self.lbl_nav2_desc = QLabel("• Astra S (RGB-D) 3D SLAM 위치추정 활성화\n• RViz 2D Goal 자율주행 대기\n• 도킹 노드 실시간 마커 모니터링")
        self.lbl_nav2_desc.setObjectName("cardDesc")
        self.lbl_nav2_badge = QLabel("● ACTIVE (자율주행 대기)")
        self.lbl_nav2_badge.setObjectName("badgeNav2")
        self.lbl_nav2_badge.setAlignment(Qt.AlignCenter)

        btn_nav2_layout.addWidget(self.lbl_nav2_title)
        btn_nav2_layout.addWidget(self.lbl_nav2_desc)
        btn_nav2_layout.addWidget(self.lbl_nav2_badge)
        self.btn_nav2.clicked.connect(self._on_click_nav2_mode)
        mode_layout.addWidget(self.btn_nav2)

        # 2. Docking Mode Button Card
        self.btn_docking = QPushButton()
        self.btn_docking.setObjectName("btnDocking")
        self.btn_docking.setCursor(Qt.PointingHandCursor)
        self.btn_docking.setMinimumHeight(95)
        btn_dock_layout = QVBoxLayout(self.btn_docking)
        btn_dock_layout.setContentsMargins(12, 8, 12, 8)
        btn_dock_layout.setSpacing(3)

        self.lbl_dock_title = QLabel("🎯 ArUco 마커 정밀 도킹 모드")
        self.lbl_dock_title.setObjectName("cardTitle")
        self.lbl_dock_desc = QLabel("• ArUco DICT_4X4_50 (ID:1) 시각 서보잉\n• RTAB-Map SLAM 일시중지 (CPU 부하 동결)\n• 5cm 정밀 도킹 및 접촉 센서 2차 안전")
        self.lbl_dock_desc.setObjectName("cardDesc")
        self.lbl_dock_badge = QLabel("● STANDBY")
        self.lbl_dock_badge.setObjectName("badgeDocking")
        self.lbl_dock_badge.setAlignment(Qt.AlignCenter)

        btn_dock_layout.addWidget(self.lbl_dock_title)
        btn_dock_layout.addWidget(self.lbl_dock_desc)
        btn_dock_layout.addWidget(self.lbl_dock_badge)
        self.btn_docking.clicked.connect(self._on_click_docking_mode)
        mode_layout.addWidget(self.btn_docking)

        left_col.addWidget(mode_box)

        # Control & Safety Action Buttons
        action_layout = QHBoxLayout()
        action_layout.setSpacing(10)

        self.btn_abort = QPushButton("↩ 도킹 중단 & 안전 후진")
        self.btn_abort.setObjectName("btnAbort")
        self.btn_abort.setCursor(Qt.PointingHandCursor)
        self.btn_abort.setMinimumHeight(44)
        self.btn_abort.clicked.connect(self._on_click_abort)
        action_layout.addWidget(self.btn_abort)

        self.btn_estop = QPushButton("⛔ 긴급 정지 (E-STOP)")
        self.btn_estop.setObjectName("btnEstop")
        self.btn_estop.setCursor(Qt.PointingHandCursor)
        self.btn_estop.setMinimumHeight(44)
        self.btn_estop.clicked.connect(self._on_click_estop)
        action_layout.addWidget(self.btn_estop)

        left_col.addLayout(action_layout)
        upper_layout.addLayout(left_col, 1)

        # Right Column: Embedded Real-Time Compressed Video Viewport
        video_box = QGroupBox("📹 실시간 로봇 카메라 & 도킹 HUD (ULTRA-LOW LATENCY)")
        video_box.setObjectName("videoBox")
        video_layout = QVBoxLayout(video_box)
        video_layout.setContentsMargins(10, 12, 10, 10)
        video_layout.setSpacing(6)

        self.lbl_video = QLabel("카메라 영상 수신 대기 중 (/docking/debug_image/compressed)...")
        self.lbl_video.setObjectName("videoViewport")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setMinimumSize(420, 260)
        self.lbl_video.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_layout.addWidget(self.lbl_video)

        # Video Status Bar (FPS / Resolution / Latency indicator)
        video_status_bar = QHBoxLayout()
        self.lbl_video_info = QLabel("FPS: -- Hz | Stream: Compressed JPEG (~12KB/f)")
        self.lbl_video_info.setObjectName("videoInfo")
        video_status_bar.addWidget(self.lbl_video_info)
        video_status_bar.addStretch()
        self.lbl_marker_info = QLabel("Target: ArUco DICT_4X4_50 [ID: 1]")
        self.lbl_marker_info.setObjectName("videoInfoHighlight")
        video_status_bar.addWidget(self.lbl_marker_info)
        video_layout.addLayout(video_status_bar)

        upper_layout.addWidget(video_box, 1)
        main_layout.addLayout(upper_layout)

        # ----------------------------------------------------------------------
        # Middle Section: Live Telemetry & Status Card
        # ----------------------------------------------------------------------
        telemetry_box = QGroupBox("실시간 로봇 계측 및 도킹 상태 모니터링 (LIVE TELEMETRY)")
        telemetry_box.setObjectName("telemetryBox")
        telem_layout = QGridLayout(telemetry_box)
        telem_layout.setContentsMargins(14, 14, 14, 12)
        telem_layout.setHorizontalSpacing(16)
        telem_layout.setVerticalSpacing(8)

        # Row 0: State & Bumper
        lbl_fsm_hdr = QLabel("현재 상태 단계:")
        lbl_fsm_hdr.setObjectName("fieldLabel")
        self.val_fsm_state = QLabel("NAV2_READY")
        self.val_fsm_state.setObjectName("statePill")

        lbl_bmp_hdr = QLabel("접촉 범퍼 센서:")
        lbl_bmp_hdr.setObjectName("fieldLabel")
        self.val_bumper = QLabel("[ CLEAR 미접촉 ]")
        self.val_bumper.setObjectName("bumperClear")

        telem_layout.addWidget(lbl_fsm_hdr, 0, 0)
        telem_layout.addWidget(self.val_fsm_state, 0, 1)
        telem_layout.addWidget(lbl_bmp_hdr, 0, 2)
        telem_layout.addWidget(self.val_bumper, 0, 3)

        # Row 1: Distance & Yaw Error
        lbl_dist_hdr = QLabel("마커와의 거리:")
        lbl_dist_hdr.setObjectName("fieldLabel")
        self.val_dist = QLabel("-- cm (목표: 5.0cm)")
        self.val_dist.setObjectName("fieldValue")

        lbl_yaw_hdr = QLabel("헤딩 정렬 오차:")
        lbl_yaw_hdr.setObjectName("fieldLabel")
        self.val_yaw = QLabel("-- deg (허용: ±2.0deg)")
        self.val_yaw.setObjectName("fieldValue")

        telem_layout.addWidget(lbl_dist_hdr, 1, 0)
        telem_layout.addWidget(self.val_dist, 1, 1)
        telem_layout.addWidget(lbl_yaw_hdr, 1, 2)
        telem_layout.addWidget(self.val_yaw, 1, 3)

        # Row 2: Lateral Offset (ey) & Target Recognition
        lbl_ey_hdr = QLabel("중심축 편차 (ey):")
        lbl_ey_hdr.setObjectName("fieldLabel")
        self.val_ey = QLabel("-- cm (허용: ±2.5cm)")
        self.val_ey.setObjectName("fieldValue")

        lbl_tgt_hdr = QLabel("타깃 마커 인식:")
        lbl_tgt_hdr.setObjectName("fieldLabel")
        self.val_tgt = QLabel("○ 탐색 중...")
        self.val_tgt.setObjectName("fieldValuePill")

        telem_layout.addWidget(lbl_ey_hdr, 2, 0)
        telem_layout.addWidget(self.val_ey, 2, 1)
        telem_layout.addWidget(lbl_tgt_hdr, 2, 2)
        telem_layout.addWidget(self.val_tgt, 2, 3)

        # Row 3: Timeout Progress Bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(100)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setObjectName("timeoutProgress")
        telem_layout.addWidget(self.progress_bar, 3, 0, 1, 4)

        main_layout.addWidget(telemetry_box)

        # ----------------------------------------------------------------------
        # Bottom Section: Event Console Log
        # ----------------------------------------------------------------------
        log_box = QGroupBox("시스템 동작 이벤트 로그 (EVENT LOG)")
        log_box.setObjectName("logBox")
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(10, 10, 10, 8)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setObjectName("logConsole")
        self.log_text.setMaximumHeight(110)
        log_layout.addWidget(self.log_text)

        main_layout.addWidget(log_box)

    def _apply_dark_theme(self):
        style = """
        QWidget {
            background-color: #0F1117;
            color: #E2E8F0;
            font-family: 'Pretendard', 'Segoe UI', 'Ubuntu', sans-serif;
            font-size: 13px;
        }

        #headerFrame {
            background-color: #161922;
            border: 1px solid #232838;
            border-radius: 8px;
        }

        #mainTitle {
            font-size: 17px;
            font-weight: bold;
            color: #38BDF8;
            letter-spacing: 0.5px;
        }

        #subTitle {
            font-size: 11px;
            color: #94A3B8;
        }

        #badgeStream {
            background-color: #064E3B;
            color: #34D399;
            font-size: 11px;
            font-weight: bold;
            padding: 4px 10px;
            border-radius: 6px;
            border: 1px solid #059669;
        }

        QGroupBox {
            border: 1px solid #232838;
            border-radius: 8px;
            margin-top: 10px;
            font-size: 12px;
            font-weight: bold;
            color: #94A3B8;
            padding-top: 8px;
        }

        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 12px;
            padding: 0 4px;
        }

        #cardTitle {
            font-size: 14px;
            font-weight: bold;
            color: #F8FAFC;
        }

        #cardDesc {
            font-size: 11px;
            color: #CBD5E1;
            line-height: 135%;
        }

        /* Nav2 Button Card */
        #btnNav2 {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #161922, stop:1 #0F1117);
            border: 2px solid #232838;
            border-radius: 8px;
            text-align: left;
            padding: 8px;
        }
        #btnNav2:hover {
            border-color: #38BDF8;
            background: #1A1F2C;
        }

        /* Docking Button Card */
        #btnDocking {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #161922, stop:1 #0F1117);
            border: 2px solid #232838;
            border-radius: 8px;
            text-align: left;
            padding: 8px;
        }
        #btnDocking:hover {
            border-color: #F59E0B;
            background: #1A1F2C;
        }

        #badgeNav2, #badgeDocking {
            font-size: 11px;
            font-weight: bold;
            border-radius: 4px;
            padding: 2px 6px;
            background-color: #232838;
            color: #94A3B8;
        }

        #videoViewport {
            background-color: #060709;
            border: 1px solid #232838;
            border-radius: 6px;
            color: #64748B;
        }

        #videoInfo {
            font-size: 11px;
            color: #64748B;
        }

        #videoInfoHighlight {
            font-size: 11px;
            color: #38BDF8;
            font-weight: bold;
        }

        #fieldLabel {
            color: #94A3B8;
            font-size: 12px;
        }

        #fieldValue {
            color: #F1F5F9;
            font-size: 13px;
            font-weight: bold;
        }

        #fieldValuePill {
            background-color: #161922;
            color: #38BDF8;
            font-size: 12px;
            font-weight: bold;
            padding: 3px 8px;
            border-radius: 4px;
            border: 1px solid #232838;
        }

        #statePill {
            background-color: #161922;
            color: #FCD34D;
            font-size: 12px;
            font-weight: bold;
            padding: 3px 8px;
            border-radius: 4px;
            border: 1px solid #3B4252;
        }

        #bumperClear {
            background-color: #064E3B;
            color: #34D399;
            font-size: 11px;
            font-weight: bold;
            padding: 3px 8px;
            border-radius: 4px;
            text-align: center;
        }

        #bumperContact {
            background-color: #7F1D1D;
            color: #F87171;
            font-size: 11px;
            font-weight: bold;
            padding: 3px 8px;
            border-radius: 4px;
            text-align: center;
        }

        #timeoutProgress {
            background-color: #161922;
            border: 1px solid #232838;
            border-radius: 4px;
            height: 6px;
        }
        #timeoutProgress::chunk {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #10B981, stop:1 #F59E0B);
            border-radius: 3px;
        }

        #btnAbort {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B45309, stop:1 #78350F);
            color: #FEF3C7;
            border: 1px solid #D97706;
            border-radius: 6px;
            font-weight: bold;
            font-size: 12px;
        }
        #btnAbort:hover {
            background: #D97706;
        }

        #btnEstop {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B91C1C, stop:1 #7F1D1D);
            color: #FEE2E2;
            border: 1px solid #EF4444;
            border-radius: 6px;
            font-weight: bold;
            font-size: 12px;
        }
        #btnEstop:hover {
            background: #DC2626;
        }

        #logConsole {
            background-color: #090A0E;
            border: 1px solid #1C202C;
            border-radius: 6px;
            color: #94A3B8;
            font-family: 'Consolas', 'DejaVu Sans Mono', monospace;
            font-size: 11px;
            padding: 4px;
        }
        """
        self.setStyleSheet(style)

    def log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {text}")
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _ros_spin_tick(self):
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception:
            pass

    def _on_new_frame(self, cv_bgr: np.ndarray):
        """Renders incoming BGR frame directly to Qt Video Viewport."""
        try:
            rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
            rgb = np.ascontiguousarray(rgb)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888).copy()
            
            # Scale smoothly into QLabel viewport
            vp_w = max(320, self.lbl_video.width())
            vp_h = max(240, self.lbl_video.height())
            pixmap = QtGui.QPixmap.fromImage(qimg).scaled(
                vp_w, vp_h, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.lbl_video.setPixmap(pixmap)

            # Update FPS bar
            self.lbl_video_info.setText(f"FPS: {self.ros_node.fps:4.1f} Hz | Res: {w}x{h} | Stream: Compressed JPEG")
        except Exception:
            pass

    def _update_ui_state(self):
        fsm = self.ros_node.fsm_state

        # Check Stream Watchdog (if no frame arrived in >2.5s)
        now = time.time()
        if self.ros_node.last_frame_time is None or (now - self.ros_node.last_frame_time) > 2.5:
            self.lbl_stream_badge.setText("⚠ WI-FI STREAM WAITING...")
            self.lbl_stream_badge.setStyleSheet("background-color: #78350F; color: #FCD34D; font-weight: bold; border-radius: 6px; padding: 4px 10px; border: 1px solid #B45309;")
        else:
            self.lbl_stream_badge.setText(f"⚡ WI-FI COMPRESSED: {self.ros_node.fps:.0f} FPS")
            self.lbl_stream_badge.setStyleSheet("background-color: #064E3B; color: #34D399; font-weight: bold; border-radius: 6px; padding: 4px 10px; border: 1px solid #059669;")

        # Update Mode Highlights based on FSM State
        is_nav2 = (fsm in ('NAV2_READY', 'INIT', 'WAIT_NAV2'))
        if is_nav2:
            self.btn_nav2.setStyleSheet("background: #0F172A; border: 2px solid #0284C7; border-radius: 8px;")
            self.lbl_nav2_badge.setText("● ACTIVE (자율주행 대기)")
            self.lbl_nav2_badge.setStyleSheet("background-color: #0284C7; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 3px 6px;")

            self.btn_docking.setStyleSheet("background: #161922; border: 2px solid #232838; border-radius: 8px;")
            self.lbl_dock_badge.setText("● STANDBY")
            self.lbl_dock_badge.setStyleSheet("background-color: #232838; color: #94A3B8; font-weight: bold; border-radius: 4px; padding: 3px 6px;")
        else:
            self.btn_docking.setStyleSheet("background: #1C1917; border: 2px solid #D97706; border-radius: 8px;")
            if fsm == 'DOCKED':
                self.lbl_dock_badge.setText("✔ DOCKED (도킹 완료)")
                self.lbl_dock_badge.setStyleSheet("background-color: #059669; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 3px 6px;")
            else:
                self.lbl_dock_badge.setText(f"● DOCKING IN PROGRESS ({fsm})")
                self.lbl_dock_badge.setStyleSheet("background-color: #D97706; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 3px 6px;")

            self.btn_nav2.setStyleSheet("background: #161922; border: 2px solid #232838; border-radius: 8px;")
            self.lbl_nav2_badge.setText("● STANDBY")
            self.lbl_nav2_badge.setStyleSheet("background-color: #232838; color: #94A3B8; font-weight: bold; border-radius: 4px; padding: 3px 6px;")

        # Update FSM State Pill
        self.val_fsm_state.setText(fsm)
        if fsm == 'DOCKED':
            self.val_fsm_state.setStyleSheet("background-color: #065F46; color: #34D399; border: 1px solid #10B981;")
        elif 'TURN' in fsm or 'CRUISE' in fsm:
            self.val_fsm_state.setStyleSheet("background-color: #581C87; color: #E9D5FF; border: 1px solid #A855F7;")
        elif 'ALIGN' in fsm:
            self.val_fsm_state.setStyleSheet("background-color: #78350F; color: #FCD34D; border: 1px solid #F59E0B;")
        elif 'CRAWL' in fsm or 'APPROACH' in fsm:
            self.val_fsm_state.setStyleSheet("background-color: #1E3A8A; color: #60A5FA; border: 1px solid #3B82F6;")
        elif fsm in ('ABORT_TO_NAV2', 'FAILSAFE'):
            self.val_fsm_state.setStyleSheet("background-color: #7F1D1D; color: #FCA5A5; border: 1px solid #EF4444;")
        else:
            self.val_fsm_state.setStyleSheet("background-color: #161922; color: #CBD5E1; border: 1px solid #3B4252;")

        # Update Distance & Yaw Error
        if self.ros_node.desk_dist is not None:
            dist_cm = self.ros_node.desk_dist * 100.0
            col = "#34D399" if abs(dist_cm - 5.0) <= 0.8 else ("#FBBF24" if dist_cm <= 18.0 else "#F1F5F9")
            self.val_dist.setText(f"{dist_cm:5.1f} cm (목표: 5.0cm)")
            self.val_dist.setStyleSheet(f"color: {col}; font-size: 13px; font-weight: bold;")
        else:
            self.val_dist.setText("탐색 중 / 미감지")
            self.val_dist.setStyleSheet("color: #94A3B8; font-size: 13px;")

        if self.ros_node.desk_yaw is not None:
            yaw_deg = self.ros_node.desk_yaw
            col = "#34D399" if abs(yaw_deg) <= 2.0 else "#F87171"
            self.val_yaw.setText(f"{yaw_deg:+5.1f} deg (허용: ±2.0deg)")
            self.val_yaw.setStyleSheet(f"color: {col}; font-size: 13px; font-weight: bold;")
        else:
            self.val_yaw.setText("-- deg")
            self.val_yaw.setStyleSheet("color: #94A3B8; font-size: 13px;")

        # Update Lateral Error (ey) & Target Recognition
        if self.ros_node.lateral_err is not None:
            ey_cm = self.ros_node.lateral_err
            col = "#34D399" if abs(ey_cm) <= 2.5 else ("#FBBF24" if abs(ey_cm) <= 4.0 else "#F87171")
            self.val_ey.setText(f"{ey_cm:+5.1f} cm (허용: ±2.5cm)")
            self.val_ey.setStyleSheet(f"color: {col}; font-size: 13px; font-weight: bold;")
        else:
            self.val_ey.setText("-- cm")
            self.val_ey.setStyleSheet("color: #94A3B8; font-size: 13px;")

        if self.ros_node.dual_markers_visible:
            if getattr(self.ros_node, 'sensor_mode', '') == 'DEPTH':
                self.val_tgt.setText("● DUAL [DEPTH 센서] (원거리 정밀)")
                self.val_tgt.setStyleSheet("background-color: #0C4A6E; color: #38BDF8; font-weight: bold; border-radius: 4px; padding: 3px 8px; border: 1px solid #0284C7;")
            else:
                self.val_tgt.setText("● DUAL [RGB 광학] (근거리 정밀)")
                self.val_tgt.setStyleSheet("background-color: #064E3B; color: #34D399; font-weight: bold; border-radius: 4px; padding: 3px 8px; border: 1px solid #059669;")
        elif self.ros_node.odom_tracking_active:
            self.val_tgt.setText("● ODOM 시각메모리 추종")
            self.val_tgt.setStyleSheet("background-color: #082F49; color: #38BDF8; font-weight: bold; border-radius: 4px; padding: 3px 8px; border: 1px solid #0284C7;")
        elif self.ros_node.target_valid:
            self.val_tgt.setText("▲ SINGLE 1개 인식 (2개 필요)")
            self.val_tgt.setStyleSheet("background-color: #78350F; color: #FCD34D; font-weight: bold; border-radius: 4px; padding: 3px 8px; border: 1px solid #D97706;")
        else:
            self.val_tgt.setText("○ 탐색 중...")
            self.val_tgt.setStyleSheet("background-color: #161922; color: #94A3B8; font-size: 12px; font-weight: bold; padding: 3px 8px; border-radius: 4px; border: 1px solid #232838;")

        # Update Bumper
        if self.ros_node.contact_detected:
            self.val_bumper.setText("[ CONTACT! 접촉됨 ]")
            self.val_bumper.setStyleSheet("background-color: #991B1B; color: #FEE2E2; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        else:
            self.val_bumper.setText("[ CLEAR 미접촉 ]")
            self.val_bumper.setStyleSheet("background-color: #064E3B; color: #A7F3D0; font-weight: bold; border-radius: 4px; padding: 3px 8px;")

        # Update Timeout Progress Bar
        if self.ros_node.docking_start_time is not None:
            elapsed = time.time() - self.ros_node.docking_start_time
            rem = max(0.0, self.ros_node.docking_timeout_sec - elapsed)
            ratio = max(0.0, min(1.0, rem / self.ros_node.docking_timeout_sec))
            self.progress_bar.setValue(int(ratio * 100))
        else:
            self.progress_bar.setValue(100)

    def _on_click_nav2_mode(self):
        self.log(">>> [COMMAND] 'Nav2 자율주행 모드' 전환 요청 (/undock_to_nav2 호출 중...)")
        self.btn_nav2.setEnabled(False)

        def _cb(future):
            self.btn_nav2.setEnabled(True)
            try:
                res = future.result()
                if res.success:
                    self.log(f"✔ Nav2 모드 전환 성공: {res.message}")
                else:
                    self.log(f"⚠ Nav2 모드 응답: {res.message}")
            except Exception as e:
                self.log(f"✖ Nav2 전환 오류: {e}")

        success = self.ros_node.call_undock_to_nav2(_cb)
        if not success:
            self.btn_nav2.setEnabled(True)
            self.log("✖ Nav2 모드 전환 서비스(/undock_to_nav2) 미연결.")

    def _on_click_docking_mode(self):
        # 1. Dual-Marker & Distance Pre-flight Safety Gate Check
        dual_ok = self.ros_node.dual_markers_visible
        dist_val = self.ros_node.desk_dist
        dist_ok = (dist_val is not None and dist_val <= 1.00)

        if not (dual_ok and dist_ok):
            reasons = []
            if not dual_ok:
                reasons.append("• 마커 2개가 카메라 화면에 동시에 인식되지 않았습니다.")
            if not dist_ok:
                curr_dist_str = f"{dist_val*100:.1f} cm" if dist_val is not None else "거리 측정 불가 (마커 미감지)"
                reasons.append(f"• 도킹 목표점과의 거리가 100cm 이내가 아닙니다 (현재: {curr_dist_str}).")

            msg = (
                "정밀 도킹 모드로 전환할 수 없습니다.\n\n"
                "【도킹 시작 필수 안전 조건】\n"
                + "\n".join(reasons) + "\n\n"
                "조치 방법:\n"
                "로봇을 수동 주행하거나 Nav2로 목표 마커 전방 100cm 이내에 위치시키고,\n"
                "카메라 시야각에 두 마커가 모두 들어오도록 한 뒤 다시 시도해 주세요."
            )
            self.log(f"⚠ [경고] 정밀 도킹 시작 불가: {'; '.join(reasons)}")
            QMessageBox.warning(self, "정밀 도킹 시작 불가", msg)
            return

        self.log(f">>> [COMMAND] '정밀 도킹 모드' 시작 요청 (마커 2개 확인 완료, 거리 {dist_val*100:.1f}cm <= 80cm, /start_docking 호출 중...)")
        self.btn_docking.setEnabled(False)

        def _cb(future):
            self.btn_docking.setEnabled(True)
            try:
                res = future.result()
                if res.success:
                    self.log(f"✔ 정밀 도킹 루틴 시작 성공: {res.message}")
                else:
                    self.log(f"⚠ 도킹 모드 전환 거절: {res.message}")
                    QMessageBox.warning(
                        self,
                        "도킹 모드 전환 거부",
                        f"로봇 정밀 도킹 노드에서 시작을 거절했습니다:\n\n{res.message}"
                    )
            except Exception as e:
                self.log(f"✖ 도킹 시작 오류: {e}")
                QMessageBox.critical(self, "도킹 서비스 통신 오류", f"서비스 호출 중 오류가 발생했습니다:\n{e}")

        success = self.ros_node.call_start_docking(_cb)
        if not success:
            self.btn_docking.setEnabled(True)
            self.log("✖ 정밀 도킹 서비스(/start_docking) 미연결.")
            QMessageBox.critical(
                self,
                "도킹 서비스 미연결",
                "정밀 도킹 제어 서비스(/start_docking)가 현재 실행 중이지 않거나 응답하지 않습니다.\n"
                "precision_approacher_node가 정상 구동 중인지 확인해 주세요."
            )

    def _on_click_abort(self):
        self.log(">>> [SAFETY] '도킹 중단 & 24cm 안전 후진' 요청됨...")

        def _cb(future):
            try:
                res = future.result()
                self.log(f"✔ 도킹 중단 완료: {res.message}")
            except Exception as e:
                self.log(f"✖ 중단 오류: {e}")

        success = self.ros_node.call_abort_docking(_cb)
        if not success:
            self.log("⚠ 중단 서비스 미연결. 즉시 E-STOP 모터 정지 실행.")
            self.ros_node.emergency_stop()

    def _on_click_estop(self):
        self.log(">>> [E-STOP] ⛔ 긴급 정지 실행! 모터 0-속도 전송!")
        self.ros_node.emergency_stop()


def main(args=None):
    rclpy.init(args=args)
    bridge = RosSignalBridge()
    ros_node = DockingGuiROSNode(bridge)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MainWindow(ros_node, bridge)
    window.show()

    try:
        ret = app.exec_()
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
