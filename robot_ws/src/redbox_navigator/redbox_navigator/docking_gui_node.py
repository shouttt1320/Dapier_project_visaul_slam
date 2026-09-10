#!/usr/bin/env python3
"""
================================================================================
RedBox TurtleBot3 Mode Switcher GUI (PyQt5 + ROS 2 Jazzy)
================================================================================
Enables simple one-click switching between:
  1. Nav2 Autonomous Navigation Mode (Orbbec Astra S ON, RTAB-Map SLAM ON)
  2. Precision Desk Docking Mode (YDLIDAR OS30A 105° ON, SLAM Paused, 5cm Target)

Features:
  - Responsive Dark-themed Desktop Dashboard
  - Live Status & Telemetry (FSM State, Distance, Yaw Error, Contact Switch)
  - Real-time Docking Timeout Progress Bar
  - Safe Abort & 24cm Retreat Action
  - Emergency Zero-Velocity Motor Stop
================================================================================
"""

import sys
import math
import time
from datetime import datetime

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QTextEdit, QFrame, QGroupBox,
    QGridLayout, QSizePolicy
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class DockingGuiROSNode(Node):
    """ROS 2 Bridge Node for the GUI Dashboard."""

    def __init__(self):
        super().__init__('docking_gui_node')

        # Subscriptions
        self.current_mode = "UNKNOWN"
        self.fsm_state = "UNKNOWN"
        self.desk_dist = None
        self.desk_yaw = None
        self.lateral_err = None
        self.target_valid = False
        self.contact_detected = False
        self.docking_timeout_sec = 35.0
        self.docking_start_time = None

        self.sub_mode = self.create_subscription(
            String, '/current_camera_mode', self._mode_callback, 10
        )
        self.sub_status = self.create_subscription(
            String, '/docking/status', self._status_callback, 10
        )
        self.sub_status_legacy = self.create_subscription(
            String, '/precision_approacher/status', self._status_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service Clients
        self.cli_start_docking = self.create_client(Trigger, '/start_docking')
        self.cli_undock = self.create_client(Trigger, '/undock_to_nav2')
        self.cli_abort = self.create_client(Trigger, '/abort_docking')
        self.cli_set_nav2 = self.create_client(Trigger, '/set_mode_nav2')
        self.cli_set_docking = self.create_client(Trigger, '/set_mode_docking')

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
                        # Track docking routine time
                        if v in ('CHECK_STAGING', 'BACKUP_STANDOFF', 'BACKUP_PREPARE', 'BACKUP_ALIGN_PREPARE',
                                 'TURN_LATERAL', 'LATERAL_CRUISE', 'TURN_FACE_DESK',
                                 'CARROT_ALIGN', 'ALIGN_PARALLEL', 'CARROT_PURSUIT_ALIGN', 'VERIFY_DUAL_ALIGN',
                                 'BACKUP_RETRY', 'COARSE_APPROACH', 'FINE_APPROACH', 'FINAL_APPROACH', 'CRAWL_CONTACT'):
                            if self.docking_start_time is None:
                                self.docking_start_time = time.time()
                        elif v in ('DOCKED', 'NAV2_READY', 'FAILSAFE', 'ABORT_TO_NAV2', 'INIT'):
                            self.docking_start_time = None
                    elif k == 'dist':
                        self.desk_dist = float(v) if v != 'None' else None
                    elif k == 'yaw_err':
                        self.desk_yaw = float(v) if v != 'None' else None
                    elif k == 'ey':
                        self.lateral_err = float(v) if v != 'None' else None
                    elif k == 'target':
                        self.target_valid = (v == 'True')
                    elif k == 'contact':
                        self.contact_detected = (v == 'True')
        except Exception:
            pass

    def call_start_docking(self, callback):
        if self.cli_start_docking.service_is_ready():
            req = Trigger.Request()
            future = self.cli_start_docking.call_async(req)
            future.add_done_callback(callback)
            return True
        elif self.cli_set_docking.service_is_ready():
            req = Trigger.Request()
            future = self.cli_set_docking.call_async(req)
            future.add_done_callback(callback)
            return True
        return False

    def call_undock_to_nav2(self, callback):
        if self.cli_undock.service_is_ready():
            req = Trigger.Request()
            future = self.cli_undock.call_async(req)
            future.add_done_callback(callback)
            return True
        elif self.cli_set_nav2.service_is_ready():
            req = Trigger.Request()
            future = self.cli_set_nav2.call_async(req)
            future.add_done_callback(callback)
            return True
        return False

    def call_abort_docking(self, callback):
        if self.cli_abort.service_is_ready():
            req = Trigger.Request()
            future = self.cli_abort.call_async(req)
            future.add_done_callback(callback)
            return True
        elif self.cli_set_nav2.service_is_ready():
            req = Trigger.Request()
            future = self.cli_set_nav2.call_async(req)
            future.add_done_callback(callback)
            return True
        return False

    def emergency_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)


class MainWindow(QMainWindow):
    """Main PyQt5 Dashboard Application Window."""

    def __init__(self, ros_node: DockingGuiROSNode):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("RedBox TurtleBot3 Mode Controller")
        self.setMinimumSize(620, 720)
        self.resize(640, 760)

        self._setup_ui()
        self._apply_dark_theme()

        # Qt Timer to spin ROS 2 callbacks at 25Hz
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._ros_spin_tick)
        self.ros_timer.start(40)  # 40ms = 25Hz

        # UI Update Timer at 10Hz
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_ui_state)
        self.ui_timer.start(100)

        self.log("System GUI launched. Ready for user commands.")

    def _setup_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(14)

        # ----------------------------------------------------------------------
        # Header Section
        # ----------------------------------------------------------------------
        header_frame = QFrame()
        header_frame.setObjectName("headerFrame")
        header_layout = QVBoxLayout(header_frame)
        header_layout.setContentsMargins(12, 10, 12, 10)
        header_layout.setSpacing(4)

        title_label = QLabel("🤖 REDBOX TURTLEBOT3 CONTROLLER")
        title_label.setObjectName("mainTitle")
        subtitle_label = QLabel("Autonomous Nav2 Navigation ⇄ Precision Desk Docking Mode Switcher")
        subtitle_label.setObjectName("subTitle")

        header_layout.addWidget(title_label)
        header_layout.addWidget(subtitle_label)
        main_layout.addWidget(header_frame)

        # ----------------------------------------------------------------------
        # Mode Selection Cards (Nav2 Mode vs Docking Mode)
        # ----------------------------------------------------------------------
        mode_box = QGroupBox("운전 모드 원클릭 전환 (MODE SELECTOR)")
        mode_box.setObjectName("modeBox")
        mode_layout = QHBoxLayout(mode_box)
        mode_layout.setSpacing(14)
        mode_layout.setContentsMargins(12, 16, 12, 14)

        # 1. Nav2 Mode Button Card
        self.btn_nav2 = QPushButton()
        self.btn_nav2.setObjectName("btnNav2")
        self.btn_nav2.setCursor(Qt.PointingHandCursor)
        self.btn_nav2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.btn_nav2.setMinimumHeight(110)
        btn_nav2_layout = QVBoxLayout(self.btn_nav2)
        btn_nav2_layout.setContentsMargins(10, 8, 10, 8)
        btn_nav2_layout.setSpacing(4)

        self.lbl_nav2_title = QLabel("🌐 Nav2 주행 모드")
        self.lbl_nav2_title.setObjectName("cardTitle")
        self.lbl_nav2_desc = QLabel("• Astra S (RGB-D) 활성화\n• RTAB-Map SLAM 재개\n• RViz 2D Goal 주행 대기")
        self.lbl_nav2_desc.setObjectName("cardDesc")
        self.lbl_nav2_badge = QLabel("● STANDBY")
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
        self.btn_docking.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.btn_docking.setMinimumHeight(110)
        btn_dock_layout = QVBoxLayout(self.btn_docking)
        btn_dock_layout.setContentsMargins(10, 8, 10, 8)
        btn_dock_layout.setSpacing(4)

        self.lbl_dock_title = QLabel("🎯 정밀 도킹 모드")
        self.lbl_dock_title.setObjectName("cardTitle")
        self.lbl_dock_desc = QLabel("• OS30A (105°하향) 활성화\n• RTAB-Map CPU 부하 동결\n• 5cm 책상 블록 정밀 도킹")
        self.lbl_dock_desc.setObjectName("cardDesc")
        self.lbl_dock_badge = QLabel("● STANDBY")
        self.lbl_dock_badge.setObjectName("badgeDocking")
        self.lbl_dock_badge.setAlignment(Qt.AlignCenter)

        btn_dock_layout.addWidget(self.lbl_dock_title)
        btn_dock_layout.addWidget(self.lbl_dock_desc)
        btn_dock_layout.addWidget(self.lbl_dock_badge)
        self.btn_docking.clicked.connect(self._on_click_docking_mode)
        mode_layout.addWidget(self.btn_docking)

        main_layout.addWidget(mode_box)

        # ----------------------------------------------------------------------
        # Live Telemetry & Status Card
        # ----------------------------------------------------------------------
        telemetry_box = QGroupBox("실시간 로봇 상태 모니터링 (LIVE TELEMETRY)")
        telemetry_box.setObjectName("telemetryBox")
        telem_layout = QGridLayout(telemetry_box)
        telem_layout.setContentsMargins(14, 16, 14, 14)
        telem_layout.setHorizontalSpacing(16)
        telem_layout.setVerticalSpacing(10)

        # Row 0: Active Camera & FSM State
        lbl_cam_hdr = QLabel("카메라 센서 모드:")
        lbl_cam_hdr.setObjectName("fieldLabel")
        self.val_cam_mode = QLabel("INITIALIZING...")
        self.val_cam_mode.setObjectName("fieldValuePill")

        lbl_fsm_hdr = QLabel("도킹 FSM 단계:")
        lbl_fsm_hdr.setObjectName("fieldLabel")
        self.val_fsm_state = QLabel("STANDBY")
        self.val_fsm_state.setObjectName("statePill")

        telem_layout.addWidget(lbl_cam_hdr, 0, 0)
        telem_layout.addWidget(self.val_cam_mode, 0, 1)
        telem_layout.addWidget(lbl_fsm_hdr, 0, 2)
        telem_layout.addWidget(self.val_fsm_state, 0, 3)

        # Row 1: Distance & Yaw Error
        lbl_dist_hdr = QLabel("책상과의 거리:")
        lbl_dist_hdr.setObjectName("fieldLabel")
        self.val_dist = QLabel("-- cm (목표: 5.0cm)")
        self.val_dist.setObjectName("fieldValue")

        lbl_yaw_hdr = QLabel("평행 정렬 오차:")
        lbl_yaw_hdr.setObjectName("fieldLabel")
        self.val_yaw = QLabel("-- ° (허용: ±2.0°)")
        self.val_yaw.setObjectName("fieldValue")

        telem_layout.addWidget(lbl_dist_hdr, 1, 0)
        telem_layout.addWidget(self.val_dist, 1, 1)
        telem_layout.addWidget(lbl_yaw_hdr, 1, 2)
        telem_layout.addWidget(self.val_yaw, 1, 3)

        # Row 2: Lateral Offset (ey) & Target Recognition
        lbl_ey_hdr = QLabel("중심축 편차 (ey):")
        lbl_ey_hdr.setObjectName("fieldLabel")
        self.val_ey = QLabel("-- cm (허용: ±1.2cm)")
        self.val_ey.setObjectName("fieldValue")

        lbl_tgt_hdr = QLabel("작업물체 인식:")
        lbl_tgt_hdr.setObjectName("fieldLabel")
        self.val_tgt = QLabel("○ 탐색 중...")
        self.val_tgt.setObjectName("fieldValuePill")

        telem_layout.addWidget(lbl_ey_hdr, 2, 0)
        telem_layout.addWidget(self.val_ey, 2, 1)
        telem_layout.addWidget(lbl_tgt_hdr, 2, 2)
        telem_layout.addWidget(self.val_tgt, 2, 3)

        # Row 3: Bumper Contact & Timeout Bar
        lbl_bmp_hdr = QLabel("접촉 범퍼 센서:")
        lbl_bmp_hdr.setObjectName("fieldLabel")
        self.val_bumper = QLabel("[ CLEAR ]")
        self.val_bumper.setObjectName("bumperClear")

        lbl_time_hdr = QLabel("도킹 타임아웃:")
        lbl_time_hdr.setObjectName("fieldLabel")
        self.val_timeout = QLabel("35.0s")
        self.val_timeout.setObjectName("fieldValue")

        telem_layout.addWidget(lbl_bmp_hdr, 3, 0)
        telem_layout.addWidget(self.val_bumper, 3, 1)
        telem_layout.addWidget(lbl_time_hdr, 3, 2)
        telem_layout.addWidget(self.val_timeout, 3, 3)

        # Row 4: Progress Bar for Timeout
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(100)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setObjectName("timeoutProgress")
        telem_layout.addWidget(self.progress_bar, 4, 0, 1, 4)

        main_layout.addWidget(telemetry_box)

        # ----------------------------------------------------------------------
        # Control & Safety Action Buttons
        # ----------------------------------------------------------------------
        action_layout = QHBoxLayout()
        action_layout.setSpacing(12)

        self.btn_abort = QPushButton("↩ 도킹 중단 및 24cm 안전 후진 복귀")
        self.btn_abort.setObjectName("btnAbort")
        self.btn_abort.setCursor(Qt.PointingHandCursor)
        self.btn_abort.setMinimumHeight(42)
        self.btn_abort.clicked.connect(self._on_click_abort)
        action_layout.addWidget(self.btn_abort)

        self.btn_estop = QPushButton("⛔ 긴급 정지 (E-STOP)")
        self.btn_estop.setObjectName("btnEstop")
        self.btn_estop.setCursor(Qt.PointingHandCursor)
        self.btn_estop.setMinimumHeight(42)
        self.btn_estop.clicked.connect(self._on_click_estop)
        action_layout.addWidget(self.btn_estop)

        main_layout.addLayout(action_layout)

        # ----------------------------------------------------------------------
        # Event Console Log
        # ----------------------------------------------------------------------
        log_box = QGroupBox("시스템 동작 로그 (EVENT LOG)")
        log_box.setObjectName("logBox")
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(10, 12, 10, 10)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setObjectName("logConsole")
        self.log_text.setMaximumHeight(140)
        log_layout.addWidget(self.log_text)

        main_layout.addWidget(log_box)

    def _apply_dark_theme(self):
        style = """
        QWidget {
            background-color: #121316;
            color: #E2E8F0;
            font-family: 'Segoe UI', 'Pretendard', 'Ubuntu', 'Helvetica Neue', sans-serif;
            font-size: 13px;
        }

        #headerFrame {
            background-color: #1A1C23;
            border: 1px solid #2D3139;
            border-radius: 8px;
        }

        #mainTitle {
            font-size: 18px;
            font-weight: bold;
            color: #38BDF8;
            letter-spacing: 0.5px;
        }

        #subTitle {
            font-size: 12px;
            color: #94A3B8;
        }

        QGroupBox {
            border: 1px solid #2B2F3A;
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
            font-size: 15px;
            font-weight: bold;
            color: #F8FAFC;
        }

        #cardDesc {
            font-size: 11px;
            color: #CBD5E1;
            line-height: 140%;
        }

        /* Nav2 Button Card */
        #btnNav2 {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1E293B, stop:1 #0F172A);
            border: 2px solid #334155;
            border-radius: 10px;
            text-align: left;
            padding: 10px;
        }
        #btnNav2:hover {
            border-color: #38BDF8;
            background: #1E293B;
        }

        /* Docking Button Card */
        #btnDocking {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1E293B, stop:1 #0F172A);
            border: 2px solid #334155;
            border-radius: 10px;
            text-align: left;
            padding: 10px;
        }
        #btnDocking:hover {
            border-color: #F59E0B;
            background: #1E293B;
        }

        /* Badges */
        #badgeNav2, #badgeDocking {
            font-size: 11px;
            font-weight: bold;
            border-radius: 4px;
            padding: 3px 6px;
            background-color: #334155;
            color: #94A3B8;
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
            background-color: #1E293B;
            color: #38BDF8;
            font-size: 12px;
            font-weight: bold;
            padding: 3px 8px;
            border-radius: 4px;
            border: 1px solid #334155;
        }

        #statePill {
            background-color: #1E293B;
            color: #FCD34D;
            font-size: 12px;
            font-weight: bold;
            padding: 3px 8px;
            border-radius: 4px;
            border: 1px solid #475569;
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
            background-color: #1E293B;
            border: 1px solid #334155;
            border-radius: 4px;
            height: 8px;
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
            font-size: 13px;
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
            font-size: 13px;
        }
        #btnEstop:hover {
            background: #DC2626;
        }

        #logConsole {
            background-color: #0B0D11;
            border: 1px solid #232731;
            border-radius: 6px;
            color: #A1A1AA;
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

    def _update_ui_state(self):
        mode = self.ros_node.current_mode
        fsm = self.ros_node.fsm_state

        # Update Mode Display & Buttons Highlight
        if mode == 'NAV2_ASTRA':
            self.val_cam_mode.setText("NAV2_ASTRA (Astra S Active)")
            self.val_cam_mode.setStyleSheet("background-color: #0369A1; color: #E0F2FE; border: 1px solid #38BDF8;")
            self.btn_nav2.setStyleSheet("background: #0F172A; border: 2px solid #0284C7; border-radius: 10px;")
            self.lbl_nav2_badge.setText("● ACTIVE (주행 대기)")
            self.lbl_nav2_badge.setStyleSheet("background-color: #0284C7; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 3px 6px;")

            self.btn_docking.setStyleSheet("background: #1E293B; border: 2px solid #334155; border-radius: 10px;")
            self.lbl_dock_badge.setText("● STANDBY")
            self.lbl_dock_badge.setStyleSheet("background-color: #334155; color: #94A3B8; font-weight: bold; border-radius: 4px; padding: 3px 6px;")
        elif mode == 'DOCKING_OS30A':
            self.val_cam_mode.setText("DOCKING_OS30A (OS30A Active)")
            self.val_cam_mode.setStyleSheet("background-color: #B45309; color: #FEF3C7; border: 1px solid #F59E0B;")
            self.btn_docking.setStyleSheet("background: #1C1917; border: 2px solid #D97706; border-radius: 10px;")
            self.lbl_dock_badge.setText("● DOCKING ACTIVE (도킹 진행 중)")
            self.lbl_dock_badge.setStyleSheet("background-color: #D97706; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 3px 6px;")

            self.btn_nav2.setStyleSheet("background: #1E293B; border: 2px solid #334155; border-radius: 10px;")
            self.lbl_nav2_badge.setText("● STANDBY")
            self.lbl_nav2_badge.setStyleSheet("background-color: #334155; color: #94A3B8; font-weight: bold; border-radius: 4px; padding: 3px 6px;")
        else:
            self.val_cam_mode.setText(f"{mode}")
            self.val_cam_mode.setStyleSheet("background-color: #1E293B; color: #94A3B8;")

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
            self.val_fsm_state.setStyleSheet("background-color: #1E293B; color: #CBD5E1; border: 1px solid #475569;")

        # Update Distance & Yaw Error
        is_nav2 = (fsm in ('NAV2_READY', 'INIT', 'WAIT_NAV2'))
        if self.ros_node.desk_dist is not None and not is_nav2:
            dist_cm = self.ros_node.desk_dist * 100.0
            col = "#34D399" if abs(dist_cm - 5.0) <= 0.8 else ("#FBBF24" if dist_cm <= 18.0 else "#F1F5F9")
            self.val_dist.setText(f"{dist_cm:5.1f} cm (목표: 5.0cm)")
            self.val_dist.setStyleSheet(f"color: {col}; font-size: 13px; font-weight: bold;")
        else:
            self.val_dist.setText("대기 중 (Nav2 모드)" if is_nav2 else "탐색 중 / 미감지")
            self.val_dist.setStyleSheet("color: #94A3B8; font-size: 13px;")

        if self.ros_node.desk_yaw is not None and not is_nav2:
            yaw_deg = math.degrees(self.ros_node.desk_yaw)
            col = "#34D399" if abs(yaw_deg) <= 2.0 else "#F87171"
            self.val_yaw.setText(f"{yaw_deg:+5.1f}° (허용: ±2.0°)")
            self.val_yaw.setStyleSheet(f"color: {col}; font-size: 13px; font-weight: bold;")
        else:
            self.val_yaw.setText("-- ° (대기 중)" if is_nav2 else "-- ° (미감지)")
            self.val_yaw.setStyleSheet("color: #94A3B8; font-size: 13px;")

        # Update Lateral Error (ey) & Target Object
        if self.ros_node.lateral_err is not None and not is_nav2:
            ey_cm = self.ros_node.lateral_err
            col = "#34D399" if abs(ey_cm) <= 1.2 else ("#FBBF24" if abs(ey_cm) <= 4.0 else "#F87171")
            self.val_ey.setText(f"{ey_cm:+5.1f} cm (허용: ±1.2cm)")
            self.val_ey.setStyleSheet(f"color: {col}; font-size: 13px; font-weight: bold;")
        else:
            self.val_ey.setText("-- cm (대기 중)" if is_nav2 else "-- cm")
            self.val_ey.setStyleSheet("color: #94A3B8; font-size: 13px;")

        if self.ros_node.target_valid and not is_nav2:
            self.val_tgt.setText("● RED BOX 감지됨")
            self.val_tgt.setStyleSheet("background-color: #064E3B; color: #34D399; font-weight: bold; border-radius: 4px; padding: 3px 8px; border: 1px solid #059669;")
        else:
            self.val_tgt.setText("○ 대기 중 (Nav2 모드)" if is_nav2 else "○ 탐색 중...")
            self.val_tgt.setStyleSheet("background-color: #1E293B; color: #94A3B8; font-size: 12px; font-weight: bold; padding: 3px 8px; border-radius: 4px; border: 1px solid #334155;")

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
            self.val_timeout.setText(f"{rem:4.1f}s / {self.ros_node.docking_timeout_sec:.0f}s")
            self.val_timeout.setStyleSheet("color: #FCD34D; font-weight: bold;")
        else:
            self.progress_bar.setValue(100)
            self.val_timeout.setText("대기 중 (35.0s)")
            self.val_timeout.setStyleSheet("color: #94A3B8;")

    def _on_click_nav2_mode(self):
        self.log(">>> [COMMAND] 'Nav2 주행 모드' 요청됨 (/undock_to_nav2 호출 중...)")
        self.btn_nav2.setEnabled(False)

        def _cb(future):
            self.btn_nav2.setEnabled(True)
            try:
                res = future.result()
                if res.success:
                    self.log(f"✔ Nav2 모드 전환 완료: {res.message}")
                else:
                    self.log(f"⚠ Nav2 모드 응답: {res.message}")
            except Exception as e:
                self.log(f"✖ Nav2 전환 서비스 오류: {e}")

        success = self.ros_node.call_undock_to_nav2(_cb)
        if not success:
            self.btn_nav2.setEnabled(True)
            self.log("✖ Nav2 모드 전환 서비스가 준비되지 않았습니다.")

    def _on_click_docking_mode(self):
        self.log(">>> [COMMAND] '정밀 도킹 모드' 요청됨 (/start_docking 호출 중...)")
        self.btn_docking.setEnabled(False)

        def _cb(future):
            self.btn_docking.setEnabled(True)
            try:
                res = future.result()
                if res.success:
                    self.log(f"✔ 정밀 도킹 시작됨: {res.message}")
                else:
                    self.log(f"⚠ 도킹 모드 응답: {res.message}")
            except Exception as e:
                self.log(f"✖ 도킹 시작 서비스 오류: {e}")

        success = self.ros_node.call_start_docking(_cb)
        if not success:
            self.btn_docking.setEnabled(True)
            self.log("✖ 정밀 도킹 서비스(/start_docking)가 준비되지 않았습니다. precision_docking.launch.py를 먼저 실행하세요.")

    def _on_click_abort(self):
        self.log(">>> [SAFETY] '도킹 중단 & 24cm 안전 후진' 요청됨...")

        def _cb(future):
            try:
                res = future.result()
                self.log(f"✔ 도킹 중단 완료: {res.message}")
            except Exception as e:
                self.log(f"✖ 중단 서비스 오류: {e}")

        success = self.ros_node.call_abort_docking(_cb)
        if not success:
            self.log("⚠ 중단 서비스 미연결. 수동 정지 및 Nav2 모드 복구 시도.")
            self.ros_node.emergency_stop()

    def _on_click_estop(self):
        self.log(">>> [E-STOP] ⛔ 긴급 정지 실행! 모터 0-속도 잠금!")
        self.ros_node.emergency_stop()


def main(args=None):
    rclpy.init(args=args)
    ros_node = DockingGuiROSNode()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MainWindow(ros_node)
    window.show()

    try:
        ret = app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
