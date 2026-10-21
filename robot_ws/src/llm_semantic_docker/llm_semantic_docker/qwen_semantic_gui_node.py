#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Qwen2.5-VL Semantic Mission Control & Vision HUD GUI
===================================================
A modern, dark-themed robotic cockpit GUI for interacting with:
- Qwen2.5-VL Multimodal Vision-Language Agent
- 3D Visual Grounding & Plane Fitting Pipeline
- 8-State Semantic Task Manager FSM
- Closed-Loop Autonomous Docking & Safety Intercept
"""

import sys
import os
import json
import math
import time
import re
import threading
import subprocess
import urllib.request
import yaml
from typing import Optional, Tuple, List, Dict, Any
import numpy as np
import cv2

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QFrame, QGridLayout,
    QSizePolicy, QSplitter, QProgressBar, QScrollArea, QComboBox, QDialog
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QSize
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor, QPalette, QIcon

GIZMO_STATIONS_PATH = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/docking_stations.yaml"
WAYPOINTS_PATH = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/semantic_waypoints.yaml"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, BatteryState
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class PickConfirmationDialog(QDialog):
    """Modern dark-themed popup dialog for confirming floor object pickup."""
    def __init__(self, data: dict, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setFixedSize(500, 270)

        self.data = data
        self.remaining_sec = int(data.get("timeout_sec", 15))
        self.result_action = "REJECT_PICK"

        # Outer card frame
        card = QFrame(self)
        card.setGeometry(0, 0, 500, 270)
        card.setStyleSheet("""
            QFrame {
                background-color: #0f172a;
                border: 2px solid #38bdf8;
                border-radius: 14px;
            }
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)

        # Header Title
        title_box = QHBoxLayout()
        icon_lbl = QLabel("📦")
        icon_lbl.setStyleSheet("font-size: 24px; border: none;")
        title_lbl = QLabel("바닥 물체 발견 - 수거(집기) 승인 요청")
        title_lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #38bdf8; border: none;")
        title_box.addWidget(icon_lbl)
        title_box.addWidget(title_lbl)
        title_box.addStretch()
        layout.addLayout(title_box)

        # Body Message
        disp_name = data.get("display_name", "물체")
        dist_m = float(data.get("distance_m", 1.0))
        conf = int(float(data.get("confidence", 0.8)) * 100)

        body_lbl = QLabel(f"전방 <b>{dist_m:.2f}m</b>에서 <b>'{disp_name}'</b> (신뢰도 {conf}%)을(를) 발견했습니다!<br>"
                          f"자율주행을 멈추고 <b>로봇팔 수거(25cm 정밀 접근)</b>를 진행할까요?")
        body_lbl.setStyleSheet("font-size: 13px; color: #e2e8f0; line-height: 1.4; border: none;")
        body_lbl.setWordWrap(True)
        layout.addWidget(body_lbl)

        # Timer Countdown label
        self.timer_lbl = QLabel(f"⏳ 남은 시간: {self.remaining_sec}초 (미응답 시 자동 계속 주행)")
        self.timer_lbl.setStyleSheet("font-size: 11px; color: #94a3b8; border: none;")
        layout.addWidget(self.timer_lbl)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        self.btn_reject = QPushButton("⚪ 무시하고 계속 (Esc)")
        self.btn_reject.setStyleSheet("""
            QPushButton {
                background-color: #334155;
                color: #cbd5e1;
                font-size: 12px;
                font-weight: bold;
                border: 1px solid #475569;
                border-radius: 8px;
                padding: 10px 16px;
            }
            QPushButton:hover {
                background-color: #475569;
                color: #ffffff;
            }
        """)
        self.btn_reject.clicked.connect(self._on_reject)
        btn_layout.addWidget(self.btn_reject)

        self.btn_approve = QPushButton("🟢 지금 잡기 (Enter)")
        self.btn_approve.setStyleSheet("""
            QPushButton {
                background-color: #0284c7;
                color: #ffffff;
                font-size: 12px;
                font-weight: bold;
                border: 1px solid #38bdf8;
                border-radius: 8px;
                padding: 10px 20px;
            }
            QPushButton:hover {
                background-color: #0369a1;
            }
        """)
        self.btn_approve.clicked.connect(self._on_approve)
        btn_layout.addWidget(self.btn_approve)

        layout.addLayout(btn_layout)

        # Countdown Timer (1s tick)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(1000)

    def _on_tick(self):
        self.remaining_sec -= 1
        if self.remaining_sec <= 0:
            self.timer.stop()
            self._on_reject()
        else:
            self.timer_lbl.setText(f"⏳ 남은 시간: {self.remaining_sec}초 (미응답 시 자동 계속 주행)")

    def _on_approve(self):
        self.timer.stop()
        self.result_action = "APPROVE_PICK"
        self.accept()

    def _on_reject(self):
        self.timer.stop()
        self.result_action = "REJECT_PICK"
        self.reject()

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self._on_approve()
        elif event.key() == Qt.Key_Escape:
            self._on_reject()
        else:
            super().keyPressEvent(event)


class RosWorkerThread(QThread):
    """Background ROS 2 spin thread with PyQt signals for thread-safe UI updates."""
    image_signal = pyqtSignal(np.ndarray)
    fsm_status_signal = pyqtSignal(str)
    agent_response_signal = pyqtSignal(str)
    target_pose_signal = pyqtSignal(float, float, float, float)  # x, y, z, normal_yaw_rad
    bumper_signal = pyqtSignal(bool)
    velocity_signal = pyqtSignal(float, float)  # vx, wz
    
    # New Telemetry Signals
    battery_signal = pyqtSignal(float, float, str)  # voltage, percentage, status_str
    odom_pose_signal = pyqtSignal(float, float, float, float)  # x, y, yaw_deg, total_dist
    topic_health_signal = pyqtSignal(dict)  # topic stats
    connectivity_signal = pyqtSignal(dict)  # network & AI backend status
    user_prompt_signal = pyqtSignal(str)   # Modal confirmation prompt for floor object pick

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self._running = True

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Topic tracking dictionaries
        self.monitored_topics = [
            ("Camera", "/camera/color/image_decompressed"),
            ("Odom", "/odom"),
            ("Map", "/map"),
            ("CmdVel", "/cmd_vel"),
            ("TargetPose", "/docking/target_object_pose"),
            ("Bumper", "/robot/bumper/contact"),
            ("Battery", "/battery_state"),
            ("TaskStatus", "/task/status"),
            ("DockingStatus", "/docking/status"),
        ]
        self._topic_last_time = {name: 0.0 for name, _ in self.monitored_topics}
        self._topic_counts = {name: 0 for name, _ in self.monitored_topics}
        self._topic_hz = {name: 0.0 for name, _ in self.monitored_topics}
        self._last_stats_calc_time = time.time()

        # Odom tracking
        self._last_odom_x = None
        self._last_odom_y = None
        self._total_odom_dist = 0.0

        # 1. Vision Stream Subscriptions (Qwen HUD primary, fallbacks included)
        self.node.create_subscription(
            CompressedImage, '/qwen/debug_image/compressed', self._compressed_img_cb, sensor_qos)
        self.node.create_subscription(
            CompressedImage, '/docking/object_debug_image/compressed', self._compressed_img_cb, sensor_qos)
        self.node.create_subscription(
            CompressedImage, '/camera/color/image_raw/compressed', self._raw_comp_img_cb, sensor_qos)
        self.node.create_subscription(
            Image, '/camera/color/image_decompressed', self._raw_decomp_img_cb, sensor_qos)
        self.node.create_subscription(
            Image, '/camera/color/image_raw', self._raw_decomp_img_cb, sensor_qos)
        self.node.create_subscription(
            Image, '/object_approach/debug_image', self._obj_approach_img_cb, sensor_qos)

        # 2. System Status Subscriptions
        self.node.create_subscription(String, '/task/status', self._task_status_cb, 10)
        self.node.create_subscription(String, '/task/user_prompt', self._user_prompt_cb, 10)
        self.node.create_subscription(String, '/docking/status', self._docking_status_cb, 10)
        self.node.create_subscription(String, '/object_approach/status', self._obj_approach_status_cb, 10)
        self.node.create_subscription(String, '/llm/agent_response', self._agent_response_cb, 10)
        self.node.create_subscription(PoseStamped, '/docking/target_object_pose', self._target_pose_cb, 10)
        self.node.create_subscription(Bool, '/robot/bumper/contact', self._bumper_cb, 10)
        self.node.create_subscription(TwistStamped, '/cmd_vel', self._cmd_vel_stamped_cb, 10)
        self.node.create_subscription(Twist, '/cmd_vel_unstamped', self._cmd_vel_cb, 10)

        # 3. Telemetry Subscriptions (Battery, Odom, Map)
        self.node.create_subscription(BatteryState, '/battery_state', self._battery_cb, 10)
        self.node.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.node.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        # 4. Control Publishers & Detection Topics
        self.user_voice_pub = self.node.create_publisher(String, '/user_voice_command', 10)
        self.cmd_pub = self.node.create_publisher(String, '/llm/voice_command', 10)
        self.direct_task_pub = self.node.create_publisher(String, '/task/command', 10)
        self.stop_pub = self.node.create_publisher(Bool, '/safety/software_stop', 10)
        self.detect_query_pub = self.node.create_publisher(String, '/qwen/detect_query', 10)
        self.confirm_dock_pub = self.node.create_publisher(String, '/docking/confirm_cmd', 10)
        self.dock_confirm_cli = self.node.create_client(Trigger, '/docking/confirm')
        self.dock_abort_cli = self.node.create_client(Trigger, '/docking/abort')
        
        # Hard Brake Zero-Velocity Publishers
        self.cmd_vel_stamped_pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.cmd_vel_raw_pub = self.node.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.cmd_vel_nav_pub = self.node.create_publisher(Twist, '/cmd_vel_nav', 10)

        self._has_hud_image = False
        self._last_hud_img_time = 0.0
        self._last_network_check = 0.0

    def _mark_topic(self, topic_key: str):
        now = time.time()
        self._topic_counts[topic_key] = self._topic_counts.get(topic_key, 0) + 1
        self._topic_last_time[topic_key] = now

    def _compressed_img_cb(self, msg: CompressedImage):
        self._mark_topic("Camera")
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is not None:
                self._last_hud_img_time = time.time()
                self.image_signal.emit(cv_img)
        except Exception:
            pass

    def _raw_comp_img_cb(self, msg: CompressedImage):
        self._mark_topic("Camera")
        # 최근 0.5초 이내에 별도 HUD 이미지가 오지 않은 경우 raw 스트림 계속 표시
        if (time.time() - self._last_hud_img_time) > 0.5:
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_img is not None:
                    self.image_signal.emit(cv_img)
            except Exception:
                pass

    def _raw_decomp_img_cb(self, msg: Image):
        self._mark_topic("Camera")
        # 최근 0.5초 이내에 별도 HUD 이미지가 오지 않은 경우 raw 스트림 계속 표시
        if (time.time() - self._last_hud_img_time) > 0.5:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if cv_img is not None:
                    self.image_signal.emit(cv_img)
            except Exception:
                pass

    def _task_status_cb(self, msg: String):
        self._mark_topic("TaskStatus")
        self.fsm_status_signal.emit(f"SOURCE=TASK,{msg.data}")

    def _user_prompt_cb(self, msg: String):
        self.user_prompt_signal.emit(msg.data)

    def _docking_status_cb(self, msg: String):
        self._mark_topic("DockingStatus")
        self.fsm_status_signal.emit(f"SOURCE=DOCK,{msg.data}")

    def _obj_approach_img_cb(self, msg: Image):
        self._mark_topic("Camera")
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_img is not None:
                self._last_hud_img_time = time.time()
                self.image_signal.emit(cv_img)
        except Exception:
            pass

    def _obj_approach_status_cb(self, msg: String):
        self._mark_topic("DockingStatus")
        self.fsm_status_signal.emit(f"SOURCE=APPROACH,{msg.data}")

    def _agent_response_cb(self, msg: String):
        self.agent_response_signal.emit(msg.data)

    def _target_pose_cb(self, msg: PoseStamped):
        self._mark_topic("TargetPose")
        pos = msg.pose.position
        ori = msg.pose.orientation
        normal_yaw = 2.0 * math.atan2(ori.z, ori.w)
        self.target_pose_signal.emit(pos.x, pos.y, pos.z, normal_yaw)

    def _bumper_cb(self, msg: Bool):
        self._mark_topic("Bumper")
        self.bumper_signal.emit(bool(msg.data))

    def _cmd_vel_stamped_cb(self, msg: TwistStamped):
        self._mark_topic("CmdVel")
        self.velocity_signal.emit(msg.twist.linear.x, msg.twist.angular.z)

    def _cmd_vel_cb(self, msg: Twist):
        self._mark_topic("CmdVel")
        self.velocity_signal.emit(msg.linear.x, msg.angular.z)

    def _battery_cb(self, msg: BatteryState):
        self._mark_topic("Battery")
        voltage = float(msg.voltage) if not math.isnan(msg.voltage) else 0.0
        pct = float(msg.percentage) if not math.isnan(msg.percentage) else -1.0
        
        # Calculate percentage for 3S LiPo if needed
        if pct < 0.0 or pct > 1.0:
            if pct > 1.0 and pct <= 100.0:
                pct_val = pct
            elif voltage > 9.0:
                pct_val = max(0.0, min(100.0, (voltage - 11.0) / (12.6 - 11.0) * 100.0))
            else:
                pct_val = 0.0
        else:
            pct_val = pct * 100.0

        status_str = "DISCHARGING"
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            status_str = "CHARGING"
        elif msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL:
            status_str = "FULL"

        self.battery_signal.emit(voltage, pct_val, status_str)

    def _odom_cb(self, msg: Odometry):
        self._mark_topic("Odom")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        if self._last_odom_x is not None and self._last_odom_y is not None:
            dist = math.hypot(x - self._last_odom_x, y - self._last_odom_y)
            if dist < 1.0:
                self._total_odom_dist += dist
        self._last_odom_x = x
        self._last_odom_y = y

        self.odom_pose_signal.emit(x, y, yaw_deg, self._total_odom_dist)

    def _map_cb(self, msg: OccupancyGrid):
        self._mark_topic("Map")

    def _compute_and_emit_health(self):
        now = time.time()
        dt = max(0.1, now - self._last_stats_calc_time)
        self._last_stats_calc_time = now

        stats = {}
        for key, topic_path in self.monitored_topics:
            count = self._topic_counts[key]
            self._topic_counts[key] = 0
            hz = count / dt
            self._topic_hz[key] = hz

            last_seen = self._topic_last_time[key]
            elapsed = now - last_seen if last_seen > 0.0 else 999.0

            if elapsed < 2.0:
                status = "OK"
            elif elapsed < 6.0:
                status = "DELAY"
            else:
                status = "OFFLINE"

            stats[key] = {
                "topic": topic_path,
                "hz": hz,
                "elapsed": elapsed,
                "status": status
            }

        self.topic_health_signal.emit(stats)

    def _check_connectivity(self):
        # Ping Raspberry Pi (192.168.0.148)
        ping_ms = None
        try:
            res = subprocess.run(
                ["ping", "-c", "1", "-W", "1", "192.168.0.148"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1.2
            )
            if res.returncode == 0:
                m = re.search(r"time=([\d\.]+)", res.stdout)
                if m:
                    ping_ms = float(m.group(1))
                else:
                    ping_ms = 1.0
        except Exception:
            ping_ms = None

        # Check Ollama HTTP
        ollama_ok = False
        try:
            req = urllib.request.Request("http://127.0.0.1:11434/api/tags", method="GET")
            with urllib.request.urlopen(req, timeout=0.8) as resp:
                ollama_ok = (resp.status == 200)
        except Exception:
            ollama_ok = False

        self.connectivity_signal.emit({
            "pi_ping_ms": ping_ms,
            "ollama_ok": ollama_ok
        })

    def send_voice_command(self, text: str):
        msg = String()
        msg.data = text
        self.user_voice_pub.publish(msg)
        self.cmd_pub.publish(msg)

    def send_direct_task(self, task_dict: dict):
        msg = String()
        msg.data = json.dumps(task_dict)
        self.direct_task_pub.publish(msg)

    def send_detect_query(self, query_text: str):
        msg = String()
        msg.data = query_text
        self.detect_query_pub.publish(msg)

    def send_dock_confirm(self):
        msg = String()
        msg.data = "confirm"
        self.confirm_dock_pub.publish(msg)
        if self.dock_confirm_cli.service_is_ready():
            self.dock_confirm_cli.call_async(Trigger.Request())

    def send_emergency_stop(self):
        # 1. 즉각 소프트웨어 정지 플래그 발행
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)

        # 2. 음성 명령 및 Task Manager에 STOP 작업 지시
        self.send_voice_command("STOP")
        stop_cmd = String()
        stop_cmd.data = json.dumps({"action": "STOP", "reason": "GUI Emergency Stop Button"})
        self.direct_task_pub.publish(stop_cmd)

        # 3. 도킹 Abort 서비스 즉시 비동기 호출
        if self.dock_abort_cli.service_is_ready():
            self.dock_abort_cli.call_async(Trigger.Request())

        # 4. 모든 모터 토픽에 하드 정지 연사 (Zero Velocity Flooding: 20회 연속 발행)
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0

        zero_stamped = TwistStamped()
        zero_stamped.header.stamp = self.node.get_clock().now().to_msg()
        zero_stamped.header.frame_id = "base_footprint"
        zero_stamped.twist.linear.x = 0.0
        zero_stamped.twist.angular.z = 0.0

        for _ in range(20):
            self.cmd_vel_raw_pub.publish(zero_twist)
            self.cmd_vel_nav_pub.publish(zero_twist)
            self.cmd_vel_stamped_pub.publish(zero_stamped)

    def _network_check_worker(self):
        """Asynchronously monitor robot ping and Ollama status without blocking ROS spin."""
        while self._running:
            self._check_connectivity()
            # Sleep in short slices to respond quickly to shutdown
            for _ in range(25):
                if not self._running:
                    break
                time.sleep(0.1)

    def run(self):
        last_health_tick = time.time()
        # Offload all blocking networking calls to background worker thread
        net_thread = threading.Thread(target=self._network_check_worker, daemon=True)
        net_thread.start()

        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.03)
            now = time.time()
            if now - last_health_tick >= 1.0:
                self._compute_and_emit_health()
                last_health_tick = now

    def stop(self):
        self._running = False
        self.quit()
        self.wait(1000)


class QwenMissionControlGUI(QMainWindow):
    """Premium Dark-Themed Cockpit UI for Qwen2.5 (3B) + YOLO 3D Visual SLAM & Semantic Docker."""

    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("🚀 Qwen2.5 (3B) + YOLO 3D Mission Control & Semantic Docker")
        self.resize(1320, 920)
        self.setMinimumSize(1100, 750)

        # Gizmo & Registered Waypoints State
        self.gizmo_locations = {}
        self._load_gizmo_locations()

        # Telemetry State (Populated in Precision Docking Mode)
        self.target_dist = None
        self.target_yaw_deg = None
        self.target_lateral_err_cm = None
        self.target_time = 0.0
        self.target_label = "마커 (Dock Station)"
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.current_state = "IDLE"
        self.bumper_engaged = False
        self.latest_task_action = "NONE"
        self.task_fsm_state = "IDLE"
        self.docking_fsm_state = "IDLE"
        self.terminal_state = None
        self.terminal_cooldown_until = 0.0

        # OpenCV ArUco detector for lightweight client-side HUD overlay
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Start ROS Worker Thread before UI initialization
        self.worker = RosWorkerThread(self.ros_node)

        self._setup_styling()
        self._init_ui()

        # Connect signals
        self.worker.image_signal.connect(self._update_image)
        self.worker.fsm_status_signal.connect(self._update_fsm_status)
        self.worker.agent_response_signal.connect(self._append_agent_response)
        self.worker.target_pose_signal.connect(self._update_target_pose)
        self.worker.bumper_signal.connect(self._update_bumper)
        self.worker.velocity_signal.connect(self._update_velocity)
        
        # Connect Dashboard Signals
        self.worker.battery_signal.connect(self._update_battery)
        self.worker.odom_pose_signal.connect(self._update_odom_pose)
        self.worker.topic_health_signal.connect(self._update_topic_health)
        self.worker.connectivity_signal.connect(self._update_connectivity)
        self.worker.user_prompt_signal.connect(self._on_user_prompt_received)

        self.worker.start()

    def _on_user_prompt_received(self, json_str: str):
        """Displays modal confirmation dialog when robot discovers floor object on patrol."""
        try:
            data = json.loads(json_str)
            if data.get("type") == "PICK_CONFIRMATION":
                dialog = PickConfirmationDialog(data, self)
                # Center dialog over main window
                dialog.move(self.geometry().center() - dialog.rect().center())
                dialog.exec_()

                # Send user decision action to /task/command
                cmd = {"action": dialog.result_action}
                self.worker.send_direct_task(cmd)
                self.chat_history.append(
                    f"<div style='margin: 4px 0; color: #38bdf8;'><b>[수거 응답]</b> "
                    f"'{dialog.result_action}' 전송 완료.</div>"
                )
        except Exception as e:
            print(f"Failed to display prompt dialog: {e}")

    # --------------------------------------------------------------------------
    # Gizmo & Registered Waypoints Location Management
    # --------------------------------------------------------------------------
    def _load_gizmo_locations(self):
        """Loads stations from docking_stations.yaml (taught via RViz Gizmo) and waypoints from semantic_waypoints.yaml"""
        self.gizmo_locations.clear()

        # 1. Docking Stations (RViz Gizmo로 등록된 정밀 도킹 스테이션)
        if os.path.exists(GIZMO_STATIONS_PATH):
            try:
                with open(GIZMO_STATIONS_PATH, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                    stations = data.get("stations", {})
                    for name, s_info in stations.items():
                        standoff = s_info.get("standoff", {})
                        x = float(standoff.get("x", 0.0))
                        y = float(standoff.get("y", 0.0))
                        yaw = float(standoff.get("yaw_deg", 0.0))
                        desc = s_info.get("description", "RViz 기즈모 등록 도킹 스테이션")
                        label = f"🎯 [기즈모 도킹] {name}"
                        self.gizmo_locations[label] = {
                            "type": "station",
                            "name": name,
                            "x": x, "y": y, "yaw": yaw,
                            "desc": desc
                        }
            except Exception as e:
                print(f"[GUI] Failed to load docking_stations.yaml: {e}")

        # 2. Semantic Waypoints (사전 정의/티칭된 웨이포인트 목록)
        if os.path.exists(WAYPOINTS_PATH):
            try:
                with open(WAYPOINTS_PATH, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                    wps = data.get("waypoints", {})
                    for name, w_info in wps.items():
                        x = float(w_info.get("x", 0.0))
                        y = float(w_info.get("y", 0.0))
                        yaw = float(w_info.get("yaw_deg", 0.0))
                        desc = w_info.get("description", "")
                        label = f"📍 [웨이포인트] {name}"
                        self.gizmo_locations[label] = {
                            "type": "waypoint",
                            "name": name,
                            "x": x, "y": y, "yaw": yaw,
                            "desc": desc
                        }
            except Exception as e:
                print(f"[GUI] Failed to load semantic_waypoints.yaml: {e}")

    def _refresh_gizmo_list(self):
        """Reloads YAML files and refreshes the UI combo box"""
        self._load_gizmo_locations()
        if hasattr(self, 'gizmo_combo'):
            self.gizmo_combo.blockSignals(True)
            self.gizmo_combo.clear()
            for label in self.gizmo_locations.keys():
                self.gizmo_combo.addItem(label)
            self.gizmo_combo.blockSignals(False)
            self._on_gizmo_selected()
        self.dialog_box.append("🔄 [좌표 목록 새로고침] 기즈모 스테이션 및 웨이포인트 목록을 갱신했습니다.")

    def _on_gizmo_selected(self):
        """Displays coordinates and details for selected gizmo/waypoint"""
        if not hasattr(self, 'gizmo_combo') or not hasattr(self, 'gizmo_coord_lbl'):
            return
        curr_label = self.gizmo_combo.currentText()
        info = self.gizmo_locations.get(curr_label)
        if info:
            self.gizmo_coord_lbl.setText(
                f"📍 X: {info['x']:+.2f}m | Y: {info['y']:+.2f}m | Yaw: {info['yaw']:+.1f}°\n({info['desc']})"
            )
        else:
            self.gizmo_coord_lbl.setText("선택된 좌표: ---")

    def _handle_go_gizmo_click(self):
        """Dispatches navigation/docking command for currently selected gizmo/waypoint"""
        if not hasattr(self, 'gizmo_combo'):
            return
        curr_label = self.gizmo_combo.currentText()
        info = self.gizmo_locations.get(curr_label)
        if not info:
            return
        name = info["name"]
        if info["type"] == "station":
            prompt = f"{name}로 도킹해"
        else:
            prompt = f"{name}으로 이동해줘"
        self.dialog_box.append(f"\n🚀 [기즈모/웨이포인트 이동]: '{prompt}' 실행")
        self._submit_prompt(prompt)

    def _setup_styling(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0b0f17;
            }
            QWidget {
                color: #e2e8f0;
                font-family: 'Noto Sans CJK KR', 'Noto Sans KR', 'Malgun Gothic', 'Ubuntu', 'DejaVu Sans', sans-serif;
            }
            QFrame.panel {
                background-color: #131a29;
                border: 1px solid #1e293b;
                border-radius: 8px;
            }
            QFrame.sub-panel {
                background-color: #0c121e;
                border: 1px solid #1e293b;
                border-radius: 6px;
                padding: 6px;
            }
            QLabel.title {
                color: #00f0ff;
                font-size: 14px;
                font-weight: bold;
                letter-spacing: 0.5px;
            }
            QLabel.metric-val {
                border: none;
                background: transparent;
                padding: 0px;
                font-size: 17px;
                font-weight: bold;
                color: #38bdf8;
            }
            QLabel.metric-lbl {
                border: none;
                background: transparent;
                padding: 0px;
                font-size: 11px;
                color: #94a3b8;
                text-transform: uppercase;
            }
            QLineEdit {
                background-color: #1a2234;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 10px 14px;
                font-size: 13px;
                color: #f8fafc;
            }
            QLineEdit:focus {
                border: 1px solid #00f0ff;
                background-color: #1e283d;
            }
            QPushButton.primary-btn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #0ea5e9);
                border: none;
                border-radius: 6px;
                padding: 10px 18px;
                font-size: 13px;
                font-weight: bold;
                color: #ffffff;
            }
            QPushButton.primary-btn:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0369a1, stop:1 #0284c7);
            }
            QPushButton.action-btn {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 7px 10px;
                font-size: 11px;
                color: #e2e8f0;
            }
            QPushButton.action-btn:hover {
                background-color: #334155;
                border-color: #00f0ff;
            }
            QPushButton.stop-btn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #dc2626, stop:1 #ef4444);
                border: none;
                border-radius: 6px;
                padding: 10px 20px;
                font-size: 13px;
                font-weight: bold;
                color: #ffffff;
            }
            QPushButton.stop-btn:hover {
                background: #b91c1c;
            }
            QPushButton.drawer-toggle {
                background-color: #162032;
                border: 1px solid #1e293b;
                border-radius: 6px;
                padding: 6px 14px;
                font-size: 12px;
                font-weight: bold;
                color: #00f0ff;
                text-align: left;
            }
            QPushButton.drawer-toggle:hover {
                background-color: #1e293b;
                border-color: #00f0ff;
            }
            QTextEdit {
                background-color: #0c121e;
                border: 1px solid #1e293b;
                border-radius: 6px;
                color: #cbd5e1;
                font-family: 'JetBrains Mono', 'Ubuntu Mono', monospace;
                font-size: 11px;
                padding: 6px;
            }
            QProgressBar {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 4px;
                text-align: center;
                color: #ffffff;
                font-size: 11px;
                font-weight: bold;
                height: 16px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #10b981, stop:1 #34d399);
                border-radius: 3px;
            }
        """)

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 10, 12, 10)
        main_layout.setSpacing(8)

        # ----------------------------------------------------------------------
        # 1. Top Header Bar
        # ----------------------------------------------------------------------
        header = QFrame()
        header.setObjectName("header")
        header.setFixedHeight(48)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(10, 0, 10, 0)

        title_lbl = QLabel("🧠 QWEN2.5 (3B) + YOLO 3D MISSION CONTROL & DOCKER")
        title_lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #00f0ff; letter-spacing: 0.8px;")
        header_layout.addWidget(title_lbl)

        header_layout.addStretch()

        mode_tag = QLabel("MODE: QWEN2.5 + YOLO 3D SERVOING (ACTIVE)")
        mode_tag.setStyleSheet("""
            background-color: rgba(0, 240, 255, 0.12);
            color: #00f0ff;
            border: 1px solid #00f0ff;
            border-radius: 12px;
            padding: 3px 10px;
            font-size: 11px;
            font-weight: bold;
        """)
        header_layout.addWidget(mode_tag)

        self.state_badge = QLabel("FSM: IDLE")
        self.state_badge.setStyleSheet("""
            background-color: #1e293b;
            color: #94a3b8;
            border: 1px solid #475569;
            border-radius: 12px;
            padding: 3px 12px;
            font-size: 11px;
            font-weight: bold;
        """)
        header_layout.addWidget(self.state_badge)

        main_layout.addWidget(header)

        # ----------------------------------------------------------------------
        # 2. Main Content Splitter (Left: Vision Stream, Right: Dialog & Telemetry)
        # ----------------------------------------------------------------------
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(6)

        # Left: Live Vision HUD Panel
        vision_panel = QFrame()
        vision_panel.setProperty("class", "panel")
        vision_layout = QVBoxLayout(vision_panel)
        vision_layout.setContentsMargins(8, 8, 8, 8)

        vision_header = QHBoxLayout()
        v_title = QLabel("LIVE CAMERA & VLM GROUNDING HUD")
        v_title.setProperty("class", "title")
        vision_header.addWidget(v_title)
        vision_header.addStretch()
        self.fps_lbl = QLabel("30.0 FPS")
        self.fps_lbl.setStyleSheet("color: #64748b; font-size: 11px; font-weight: bold;")
        vision_header.addWidget(self.fps_lbl)
        vision_layout.addLayout(vision_header)

        self.video_viewport = QLabel("WAITING FOR CAMERA STREAM (/qwen/debug_image)...")
        self.video_viewport.setAlignment(Qt.AlignCenter)
        self.video_viewport.setStyleSheet("""
            background-color: #06090e;
            border: 1px solid #1e293b;
            border-radius: 6px;
            color: #475569;
            font-size: 13px;
        """)
        self.video_viewport.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vision_layout.addWidget(self.video_viewport)

        splitter.addWidget(vision_panel)

        # Right: Dialog, Telemetry, and Presets Panel
        right_panel = QFrame()
        right_panel.setProperty("class", "panel")
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(10, 10, 10, 10)
        right_layout.setSpacing(8)

        # 2.1 Telemetry Cards Grid
        telemetry_title = QLabel("REAL-TIME DOCKING TELEMETRY")
        telemetry_title.setProperty("class", "title")
        right_layout.addWidget(telemetry_title)

        grid = QGridLayout()
        grid.setSpacing(6)

        self.dist_card = self._create_metric_card("TARGET DIST (목표 거리)", "--- m", "#38bdf8")
        grid.addWidget(self.dist_card, 0, 0)

        self.yaw_card = self._create_metric_card("ALIGN YAW (법선 정렬각)", "--- °", "#a78bfa")
        grid.addWidget(self.yaw_card, 0, 1)

        self.axis_card = self._create_metric_card("CENTERLINE ERR (중심 편차)", "--- cm", "#f59e0b")
        grid.addWidget(self.axis_card, 1, 0)

        self.bumper_card = self._create_metric_card("BUMPER (범퍼접촉)", "SAFE", "#10b981")
        grid.addWidget(self.bumper_card, 1, 1)

        self.vel_card = self._create_metric_card("ROBOT SPEED (Vx / Wz)", "0.00 / 0.00", "#34d399")
        grid.addWidget(self.vel_card, 2, 0, 1, 2)

        right_layout.addLayout(grid)

        # 2.2 Agent Thought & Dialog Box
        dialog_title = QLabel("QWEN2.5 AGENT & YOLO 3D LOG")
        dialog_title.setProperty("class", "title")
        right_layout.addWidget(dialog_title)

        self.dialog_box = QTextEdit()
        self.dialog_box.setReadOnly(True)
        self.dialog_box.setMinimumHeight(65)
        self.dialog_box.setMaximumHeight(100)
        self.dialog_box.append("🤖 [Qwen2.5 Agent Ready] Qwen2.5 (3B) LLM + Real-time YOLO 3D active.")
        right_layout.addWidget(self.dialog_box)

        # 2.3 Registered Gizmo Stations & Semantic Waypoints
        gizmo_panel_title = QLabel("📍 GIZMO & REGISTERED WAYPOINTS")
        gizmo_panel_title.setProperty("class", "title")
        right_layout.addWidget(gizmo_panel_title)

        gizmo_box = QFrame()
        gizmo_box.setStyleSheet("background-color: #131d2e; border: 1px solid #1e293b; border-radius: 8px; padding: 6px;")
        gizmo_layout = QVBoxLayout(gizmo_box)
        gizmo_layout.setContentsMargins(6, 6, 6, 6)
        gizmo_layout.setSpacing(5)

        combo_row = QHBoxLayout()
        self.gizmo_combo = QComboBox()
        self.gizmo_combo.setStyleSheet("""
            QComboBox {
                background-color: #0f172a;
                color: #38bdf8;
                border: 1px solid #334155;
                border-radius: 4px;
                padding: 5px 8px;
                font-weight: bold;
                font-size: 11px;
            }
            QComboBox QAbstractItemView {
                background-color: #0f172a;
                color: #f8fafc;
                selection-background-color: #2563eb;
            }
        """)
        for label in self.gizmo_locations.keys():
            self.gizmo_combo.addItem(label)
        self.gizmo_combo.currentIndexChanged.connect(self._on_gizmo_selected)
        combo_row.addWidget(self.gizmo_combo, 1)

        btn_refresh_gizmo = QPushButton("🔄")
        btn_refresh_gizmo.setToolTip("기즈모 및 웨이포인트 목록 새로고침")
        btn_refresh_gizmo.setStyleSheet("background-color: #1e293b; color: #f8fafc; font-weight: bold; padding: 5px 10px; border: 1px solid #334155; border-radius: 4px;")
        btn_refresh_gizmo.clicked.connect(self._refresh_gizmo_list)
        combo_row.addWidget(btn_refresh_gizmo)
        gizmo_layout.addLayout(combo_row)

        self.gizmo_coord_lbl = QLabel("선택된 좌표: ---")
        self.gizmo_coord_lbl.setStyleSheet("color: #a78bfa; font-family: monospace; font-size: 11px; font-weight: bold; padding: 2px 4px;")
        self._on_gizmo_selected()
        gizmo_layout.addWidget(self.gizmo_coord_lbl)

        btn_go_gizmo = QPushButton("🚀 선택한 기즈모 위치로 이동 / 도킹")
        btn_go_gizmo.setStyleSheet("background-color: rgba(59, 130, 246, 0.25); border: 1px solid #3b82f6; color: #60a5fa; font-weight: bold; padding: 7px; border-radius: 6px; font-size: 12px;")
        btn_go_gizmo.clicked.connect(self._handle_go_gizmo_click)
        gizmo_layout.addWidget(btn_go_gizmo)

        right_layout.addWidget(gizmo_box)

        # 2.4 Quick Relative Navigation & Target Servoing
        presets_title = QLabel("QUICK ACTION PRESETS")
        presets_title.setProperty("class", "title")
        right_layout.addWidget(presets_title)

        quick_grid = QGridLayout()
        quick_grid.setSpacing(5)

        btn_fwd10 = QPushButton("▲ 10cm 전진")
        btn_fwd10.setProperty("class", "action-btn")
        btn_fwd10.clicked.connect(lambda: self._send_relative_nav(0.10, 0.0, 0.0))
        quick_grid.addWidget(btn_fwd10, 0, 0)

        btn_fwd20 = QPushButton("▲ 20cm 전진")
        btn_fwd20.setProperty("class", "action-btn")
        btn_fwd20.clicked.connect(lambda: self._send_relative_nav(0.20, 0.0, 0.0))
        quick_grid.addWidget(btn_fwd20, 0, 1)

        btn_fwd30 = QPushButton("▲ 30cm 전진")
        btn_fwd30.setProperty("class", "action-btn")
        btn_fwd30.clicked.connect(lambda: self._send_relative_nav(0.30, 0.0, 0.0))
        quick_grid.addWidget(btn_fwd30, 0, 2)

        btn_back10 = QPushButton("▼ 10cm 후진")
        btn_back10.setProperty("class", "action-btn")
        btn_back10.clicked.connect(lambda: self._send_relative_nav(-0.10, 0.0, 0.0))
        quick_grid.addWidget(btn_back10, 1, 0)

        btn_left15 = QPushButton("◀ 좌회전 15°")
        btn_left15.setProperty("class", "action-btn")
        btn_left15.clicked.connect(lambda: self._send_relative_nav(0.0, 0.0, 15.0))
        quick_grid.addWidget(btn_left15, 1, 1)

        btn_right15 = QPushButton("▶ 우회전 15°")
        btn_right15.setProperty("class", "action-btn")
        btn_right15.clicked.connect(lambda: self._send_relative_nav(0.0, 0.0, -15.0))
        quick_grid.addWidget(btn_right15, 1, 2)

        # 3열: 타겟 시각 접근 (빨간 큐브, 파란 공, 마커)
        btn_yolo_red = QPushButton("🔴 빨간 큐브 접근")
        btn_yolo_red.setStyleSheet("background-color: rgba(239, 68, 68, 0.20); border: 1px solid #ef4444; color: #f87171; font-weight: bold; padding: 6px;")
        btn_yolo_red.clicked.connect(lambda: self._submit_prompt("빨간 큐브 앞으로 접근해"))
        quick_grid.addWidget(btn_yolo_red, 2, 0)

        btn_yolo_blue = QPushButton("🔵 파란 공 접근")
        btn_yolo_blue.setStyleSheet("background-color: rgba(59, 130, 246, 0.20); border: 1px solid #3b82f6; color: #60a5fa; font-weight: bold; padding: 6px;")
        btn_yolo_blue.clicked.connect(lambda: self._submit_prompt("파란 공 앞으로 가줘"))
        quick_grid.addWidget(btn_yolo_blue, 2, 1)

        btn_dock_marker = QPushButton("🎯 마커 도킹")
        btn_dock_marker.setStyleSheet("background-color: rgba(16, 185, 129, 0.20); border: 1px solid #10b981; color: #34d399; font-weight: bold; padding: 6px;")
        btn_dock_marker.clicked.connect(lambda: self._submit_prompt("마커 보고 도킹해"))
        quick_grid.addWidget(btn_dock_marker, 2, 2)

        # 4열: 바닥 물체 수거 핸드셰이크 (로봇팔 수거 완료 / 건너뛰기)
        btn_pick_resume = QPushButton("🦾 수거 완료 (주행 복귀)")
        btn_pick_resume.setStyleSheet("background-color: rgba(168, 85, 247, 0.25); border: 1px solid #c084fc; color: #e9d5ff; font-weight: bold; padding: 6px;")
        btn_pick_resume.setToolTip("로봇팔 수거 작업 완료 신호를 보내고 원래 자율주행 목적지로 주행을 복귀합니다.")
        btn_pick_resume.clicked.connect(lambda: self.worker.send_direct_task({"action": "PICK_COMPLETED"}))
        quick_grid.addWidget(btn_pick_resume, 3, 0, 1, 2)

        btn_pick_skip = QPushButton("⏭️ 건너뛰기")
        btn_pick_skip.setStyleSheet("background-color: rgba(148, 163, 184, 0.20); border: 1px solid #64748b; color: #cbd5e1; font-weight: bold; padding: 6px;")
        btn_pick_skip.setToolTip("물체 수거를 건너뛰고 원래 목적지로 계속 주행합니다.")
        btn_pick_skip.clicked.connect(lambda: self.worker.send_direct_task({"action": "REJECT_PICK"}))
        quick_grid.addWidget(btn_pick_skip, 3, 2)

        right_layout.addLayout(quick_grid)

        # Wrap right panel in a sleek scroll area so telemetry cards are NEVER crushed vertically
        right_scroll = QScrollArea()
        right_scroll.setWidget(right_panel)
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_scroll.setStyleSheet("""
            QScrollArea {
                background: transparent;
                border: none;
            }
            QScrollBar:vertical {
                background: #0b111e;
                width: 6px;
                border-radius: 3px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #334155;
                min-height: 25px;
                border-radius: 3px;
            }
            QScrollBar::handle:vertical:hover {
                background: #00f0ff;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        splitter.addWidget(right_scroll)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter)

        # ----------------------------------------------------------------------
        # 3. Bottom Command Bar & Emergency STOP Controls
        # ----------------------------------------------------------------------
        bottom_bar = QFrame()
        bottom_bar.setProperty("class", "panel")
        bottom_layout = QHBoxLayout(bottom_bar)
        bottom_layout.setContentsMargins(10, 8, 10, 8)
        bottom_layout.setSpacing(8)

        cmd_icon = QLabel("💬")
        cmd_icon.setStyleSheet("font-size: 16px;")
        bottom_layout.addWidget(cmd_icon)

        self.input_edit = QLineEdit()
        self.input_edit.setPlaceholderText("명령 입력 (예: '앞으로 20cm', '우회전 15도', '?상자' 시각검색, '정지')...")
        self.input_edit.returnPressed.connect(self._handle_send_command)
        bottom_layout.addWidget(self.input_edit)

        send_btn = QPushButton("명령 전송")
        send_btn.setProperty("class", "primary-btn")
        send_btn.clicked.connect(self._handle_send_command)
        bottom_layout.addWidget(send_btn)

        stop_btn = QPushButton("⛔ 긴급 정지 (STOP)")
        stop_btn.setProperty("class", "stop-btn")
        stop_btn.clicked.connect(self._handle_emergency_stop)
        bottom_layout.addWidget(stop_btn)

        main_layout.addWidget(bottom_bar)

        # ----------------------------------------------------------------------
        # 4. Collapsible System Telemetry & Health Dashboard Drawer
        # ----------------------------------------------------------------------
        self._init_collapsible_dashboard(main_layout)

    def _init_collapsible_dashboard(self, parent_layout: QVBoxLayout):
        """Initializes the collapsible bottom dashboard with 4 health sections."""
        drawer_wrapper = QFrame()
        drawer_wrapper.setProperty("class", "panel")
        drawer_layout = QVBoxLayout(drawer_wrapper)
        drawer_layout.setContentsMargins(8, 6, 8, 8)
        drawer_layout.setSpacing(6)

        # 4.1 Drawer Header / Toggle Bar
        header_bar = QHBoxLayout()
        header_bar.setContentsMargins(0, 0, 0, 0)

        self.drawer_toggle_btn = QPushButton("▼ SYSTEM TELEMETRY & HEALTH DASHBOARD (클릭하여 접기/펼치기)")
        self.drawer_toggle_btn.setProperty("class", "drawer-toggle")
        self.drawer_toggle_btn.clicked.connect(self._toggle_dashboard)
        header_bar.addWidget(self.drawer_toggle_btn)

        # Quick Status Indicators in Header (Visible even when collapsed)
        header_bar.addSpacing(10)
        
        self.hdr_topic_badge = QLabel("🟢 TOPICS: 0/8 OK")
        self.hdr_topic_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")
        header_bar.addWidget(self.hdr_topic_badge)

        self.hdr_bat_badge = QLabel("🔋 BAT: ---V (---%)")
        self.hdr_bat_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #38bdf8; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")
        header_bar.addWidget(self.hdr_bat_badge)

        self.hdr_net_badge = QLabel("📡 ROBOT: PING ---ms")
        self.hdr_net_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")
        header_bar.addWidget(self.hdr_net_badge)

        self.hdr_ollama_badge = QLabel("🧠 VLM: CHECKING")
        self.hdr_ollama_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #a78bfa; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")
        header_bar.addWidget(self.hdr_ollama_badge)

        drawer_layout.addLayout(header_bar)

        # 4.2 Dashboard Main Body Container
        self.dashboard_body = QFrame()
        body_layout = QGridLayout(self.dashboard_body)
        body_layout.setContentsMargins(4, 4, 4, 4)
        body_layout.setSpacing(8)

        # Section A: Topic Health Monitor (Grid Table)
        topic_box = QFrame()
        topic_box.setProperty("class", "sub-panel")
        t_layout = QVBoxLayout(topic_box)
        t_layout.setContentsMargins(8, 6, 8, 6)
        t_title = QLabel("📡 ROS 2 TOPIC HEALTH MONITOR")
        t_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #00f0ff;")
        t_layout.addWidget(t_title)

        self.topic_labels = {}
        t_grid = QGridLayout()
        t_grid.setSpacing(4)
        
        row = 0
        for name, path in self.worker.monitored_topics:
            name_lbl = QLabel(f"• {name}")
            name_lbl.setStyleSheet("font-size: 11px; color: #cbd5e1; font-weight: 500;")
            
            rate_lbl = QLabel("0.0 Hz")
            rate_lbl.setStyleSheet("font-size: 10px; color: #94a3b8; font-family: monospace;")
            
            status_dot = QLabel("⚪ WAIT")
            status_dot.setStyleSheet("font-size: 10px; font-weight: bold; color: #64748b; background: #1e293b; border-radius: 3px; padding: 1px 6px;")
            
            t_grid.addWidget(name_lbl, row, 0)
            t_grid.addWidget(rate_lbl, row, 1)
            t_grid.addWidget(status_dot, row, 2)
            
            self.topic_labels[name] = (rate_lbl, status_dot)
            row += 1

        t_layout.addLayout(t_grid)
        body_layout.addWidget(topic_box, 0, 0)

        # Section B: Battery & Power Telemetry
        bat_box = QFrame()
        bat_box.setProperty("class", "sub-panel")
        b_layout = QVBoxLayout(bat_box)
        b_layout.setContentsMargins(8, 6, 8, 6)
        b_title = QLabel("🔋 BATTERY & POWER MANAGEMENT")
        b_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #38bdf8;")
        b_layout.addWidget(b_title)

        self.bat_voltage_lbl = QLabel("VOLTAGE: --- V (3S LiPo)")
        self.bat_voltage_lbl.setStyleSheet("font-size: 13px; font-weight: bold; color: #f8fafc;")
        b_layout.addWidget(self.bat_voltage_lbl)

        self.bat_progress = QProgressBar()
        self.bat_progress.setRange(0, 100)
        self.bat_progress.setValue(0)
        b_layout.addWidget(self.bat_progress)

        self.bat_status_lbl = QLabel("STATUS: NORMAL / WAITING TELEMETRY")
        self.bat_status_lbl.setStyleSheet("font-size: 11px; color: #94a3b8;")
        b_layout.addWidget(self.bat_status_lbl)

        self.bat_warning_lbl = QLabel("● VOLTAGE NORMAL (11.0V+)")
        self.bat_warning_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981;")
        b_layout.addWidget(self.bat_warning_lbl)
        b_layout.addStretch()

        body_layout.addWidget(bat_box, 0, 1)

        # Section C: Robot Pose & Localization
        pose_box = QFrame()
        pose_box.setProperty("class", "sub-panel")
        p_layout = QVBoxLayout(pose_box)
        p_layout.setContentsMargins(8, 6, 8, 6)
        p_title = QLabel("🧭 ROBOT POSE & LOCALIZATION")
        p_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #34d399;")
        p_layout.addWidget(p_title)

        self.pose_coord_lbl = QLabel("X: +0.000 m  |  Y: +0.000 m")
        self.pose_coord_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #f8fafc; font-family: monospace;")
        p_layout.addWidget(self.pose_coord_lbl)

        self.pose_yaw_lbl = QLabel("HEADING (YAW): +0.0 °")
        self.pose_yaw_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #38bdf8; font-family: monospace;")
        p_layout.addWidget(self.pose_yaw_lbl)

        self.odom_dist_lbl = QLabel("ODOM DISTANCE: 0.00 m")
        self.odom_dist_lbl.setStyleSheet("font-size: 11px; color: #94a3b8; font-family: monospace;")
        p_layout.addWidget(self.odom_dist_lbl)

        self.rtab_status_lbl = QLabel("● RTAB-MAP: READY (DB LOADED)")
        self.rtab_status_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #34d399;")
        p_layout.addWidget(self.rtab_status_lbl)
        p_layout.addStretch()

        body_layout.addWidget(pose_box, 0, 2)

        # Section D: Connectivity & AI Backend Status
        conn_box = QFrame()
        conn_box.setProperty("class", "sub-panel")
        c_layout = QVBoxLayout(conn_box)
        c_layout.setContentsMargins(8, 6, 8, 6)
        c_title = QLabel("🌐 CONNECTIVITY & AI SERVICE")
        c_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #a78bfa;")
        c_layout.addWidget(c_title)

        self.net_ping_lbl = QLabel("ROBOT (192.168.0.148): PING ...")
        self.net_ping_lbl.setStyleSheet("font-size: 11px; color: #cbd5e1;")
        c_layout.addWidget(self.net_ping_lbl)

        self.vlm_status_lbl = QLabel("OLLAMA VLM: CHECKING...")
        self.vlm_status_lbl.setStyleSheet("font-size: 11px; color: #cbd5e1;")
        c_layout.addWidget(self.vlm_status_lbl)

        self.safety_lbl = QLabel("SAFETY E-STOP: ARMED (NORMAL)")
        self.safety_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981;")
        c_layout.addWidget(self.safety_lbl)

        self.sync_clock_lbl = QLabel("TIME SYNC: PC ⇄ PI MATCHED")
        self.sync_clock_lbl.setStyleSheet("font-size: 11px; color: #94a3b8;")
        c_layout.addWidget(self.sync_clock_lbl)
        c_layout.addStretch()

        body_layout.addWidget(conn_box, 0, 3)

        body_layout.setColumnStretch(0, 2)
        body_layout.setColumnStretch(1, 2)
        body_layout.setColumnStretch(2, 2)
        body_layout.setColumnStretch(3, 2)

        drawer_layout.addWidget(self.dashboard_body)
        parent_layout.addWidget(drawer_wrapper)

        # Set default state: Expanded
        self.dashboard_is_expanded = True

    def _toggle_dashboard(self):
        """Collapses or expands the dashboard body."""
        self.dashboard_is_expanded = not self.dashboard_is_expanded
        self.dashboard_body.setVisible(self.dashboard_is_expanded)
        if self.dashboard_is_expanded:
            self.drawer_toggle_btn.setText("▼ SYSTEM TELEMETRY & HEALTH DASHBOARD (클릭하여 접기)")
        else:
            self.drawer_toggle_btn.setText("▶ SYSTEM TELEMETRY & HEALTH DASHBOARD (클릭하여 펼치기)")
        self.adjustSize()

    def _create_metric_card(self, label: str, default_val: str, color_hex: str) -> QFrame:
        card = QFrame()
        card.setObjectName("metricCard")
        card.setMinimumHeight(56)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        card.setStyleSheet("""
            QFrame#metricCard {
                background-color: #0d1422;
                border: 1px solid #1e293b;
                border-radius: 6px;
            }
        """)
        v = QVBoxLayout(card)
        v.setContentsMargins(10, 6, 10, 6)
        v.setSpacing(2)

        lbl = QLabel(label)
        lbl.setStyleSheet("border: none; background: transparent; color: #94a3b8; font-size: 11px; font-weight: 600; text-transform: uppercase;")
        v.addWidget(lbl)

        val = QLabel(default_val)
        val.setStyleSheet(f"border: none; background: transparent; color: {color_hex}; font-size: 18px; font-weight: bold; font-family: 'Ubuntu Mono', 'DejaVu Sans Mono', monospace;")
        card.value_lbl = val
        card.default_color_hex = color_hex
        v.addWidget(val)

        return card

    def _set_card_value(self, card: QFrame, text: str, color_hex: Optional[str] = None):
        card.value_lbl.setText(text)
        col = color_hex if color_hex else getattr(card, 'default_color_hex', '#38bdf8')
        card.value_lbl.setStyleSheet(f"border: none; background: transparent; color: {col}; font-size: 18px; font-weight: bold; font-family: 'Ubuntu Mono', 'DejaVu Sans Mono', monospace;")

    # --------------------------------------------------------------------------
    # UI Event Handlers & Signal Slots
    # --------------------------------------------------------------------------
    def _handle_send_command(self):
        text = self.input_edit.text().strip()
        if not text:
            return

        # 1. 비상 정지 키워드 즉시 처리 (LLM 지연 없이 즉시 모터 락)
        if text in ['정지', '멈춰', 'stop', '비상정지', 'STOP', '스톱']:
            self._handle_emergency_stop()
            self.input_edit.clear()
            return

        # 1.1 당근점 이동 승인 키워드 인터셉트
        if text.strip().lower() in ['진행', '이동', 'yes', 'y', 'confirm', 'proceed', '도킹 진행', '출발', '승인']:
            self._handle_confirm_dock_click()
            self.input_edit.clear()
            return

        # 2. '?' 접두사 시각 질의 분기 (로봇 주행하지 않고 순수 시각 검증만 수행)
        if text.startswith("?"):
            query = text[1:].strip()
            if query:
                self.dialog_box.append(f"\n🔎 [시각 질의]: '{query}' 시각 검색 요청...")
                self.worker.send_detect_query(query)
            self.input_edit.clear()
            return

        # 3. 자연어 로봇 작업 명령 전송
        self.dialog_box.append(f"\n👤 [User]: {text}")
        self.worker.send_voice_command(text)
        self.input_edit.clear()

    def _handle_confirm_dock_click(self):
        self.dialog_box.append("\n🚀 [이동 승인]: 당근점 주행을 승인합니다 (/docking/confirm).")
        self.worker.send_dock_confirm()

    def _submit_prompt(self, text: str):
        self.input_edit.setText(text)
        self._handle_send_command()

    def _handle_emergency_stop(self):
        self.dialog_box.append("\n🚨 [EMERGENCY STOP TRIGGERED] Motors halted immediately!")
        self.worker.send_emergency_stop()
        self.safety_lbl.setText("SAFETY E-STOP: 💥 TRIGGERED (HALT)")
        self.safety_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #ef4444;")

    def _send_relative_nav(self, dx: float, dy: float, dyaw: float):
        cmd_text = f"상대 이동 dx={dx:.2f}m, dy={dy:.2f}m, yaw={dyaw:.1f}°"
        self.dialog_box.append(f"\n⚡ [Quick Action]: {cmd_text}")
        task_dict = {
            "action": "NAVIGATE_RELATIVE",
            "relative_x": dx,
            "relative_y": dy,
            "relative_yaw": dyaw
        }
        self.worker.send_direct_task(task_dict)

    def _draw_hud_overlay(self, img: np.ndarray):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Subtle Centerline & Crosshair (Robot Forward Axis)
        cv2.line(img, (cx, 0), (cx, h), (60, 60, 60), 1, cv2.LINE_AA)
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (60, 60, 60), 1, cv2.LINE_AA)

        # Only display precision docking HUD when precision docking mode is active
        docking_active_states = (
            "STAGE1_APPROACH", "STAGE1_VISUAL_SERVO", "STAGE2_STANDOFF_SETTLE", "STAGE3_BLIND_CRAWL", "BACKUP_RETRY",
            "STANDOFF_SETTLE", "STANDOFF_AIM_CARROT", "STANDOFF_DRIVE_CARROT", "STANDOFF_ALIGN_BOX", "STATION_VERIFY", "STATION_CRAWL",
            "SEARCH_MARKER", "STAGING_SETTLE", "STAGING_AIM", "CARROT_ALIGN", "CRAWL_CONTACT", "BACKUP_STANDOFF",
            "INIT", "CHECK_STAGING"
        )
        is_docking = (self.docking_fsm_state in docking_active_states) or (
            time.time() - self.target_time < 3.0 and self.target_dist is not None and self.target_dist > 0.05
        )

        if not is_docking:
            # Standby during normal navigation: Clean camera feed without error cards or search text
            return

        # 2. ArUco Marker Detection & Visual HUD Overlay during precision docking
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(img, corners, ids, borderColor=(0, 255, 128))
                c = corners[0][0]
                mcx = int(np.mean(c[:, 0]))
                mcy = int(np.mean(c[:, 1]))
                # Marker center crosshair
                cv2.drawMarker(img, (mcx, mcy), (0, 255, 128), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
                # Offset guideline to robot centerline
                cv2.line(img, (cx, mcy), (mcx, mcy), (0, 165, 255), 2, cv2.LINE_AA)
                cv2.circle(img, (cx, mcy), 3, (0, 165, 255), -1)
                cv2.putText(img, f"MARKER ID:{ids[0][0]}", (mcx - 30, max(15, mcy - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 128), 1, cv2.LINE_AA)
        except Exception:
            pass

        # 3. HUD Telemetry Overlay Card (Top-Left)
        overlay = img.copy()
        card_w, card_h = 210, 76
        cv2.rectangle(overlay, (8, 8), (8 + card_w, 8 + card_h), (15, 23, 42), -1)
        cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)
        cv2.rectangle(img, (8, 8), (8 + card_w, 8 + card_h), (0, 240, 255), 1)

        dist_str = f"{self.target_dist:.2f} m" if self.target_dist is not None else "SEARCHING"
        yaw_str = f"{self.target_yaw_deg:+.1f} deg" if self.target_yaw_deg is not None else "---"
        lat_str = f"{self.target_lateral_err_cm:+.1f} cm" if self.target_lateral_err_cm is not None else "---"

        cv2.putText(img, f"DOCK MODE: {self.docking_fsm_state}", (14, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 240, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"MARKER DIST: {dist_str}", (14, 39),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (56, 189, 248), 1, cv2.LINE_AA)
        cv2.putText(img, f"ALIGN YAW:   {yaw_str}", (14, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (167, 139, 250), 1, cv2.LINE_AA)
        cv2.putText(img, f"CENTER ERR:  {lat_str}", (14, 69),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (245, 158, 11), 1, cv2.LINE_AA)

    def _update_image(self, cv_img: np.ndarray):
        # Draw Real-Time HUD Target Overlay on frame
        try:
            self._draw_hud_overlay(cv_img)
        except Exception:
            pass

        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        q_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)

        scaled_pixmap = pixmap.scaled(
            self.video_viewport.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.video_viewport.setPixmap(scaled_pixmap)

    def _update_fsm_status(self, raw_status: str):
        source = "TASK"
        status_str = raw_status
        if raw_status.startswith("SOURCE=TASK,"):
            source = "TASK"
            status_str = raw_status.replace("SOURCE=TASK,", "")
        elif raw_status.startswith("SOURCE=DOCK,"):
            source = "DOCK"
            status_str = raw_status.replace("SOURCE=DOCK,", "")
        elif raw_status.startswith("SOURCE=APPROACH,"):
            source = "APPROACH"
            status_str = raw_status.replace("SOURCE=APPROACH,", "")

        # 1. Alert / Info message handling
        if status_str.startswith("ALERT:") or "[주행 불가]" in status_str or "[도킹 불가]" in status_str or "[주행 실패]" in status_str:
            clean_msg = status_str.replace("ALERT:", "").strip()
            self.dialog_box.append(f"<span style='color: #f87171; font-weight: bold;'>⚠️ {clean_msg}</span>")
            sb = self.dialog_box.verticalScrollBar()
            sb.setValue(sb.maximum())
            return
        elif status_str.startswith("INFO:") or "[주행 시작]" in status_str or "[도킹 승인]" in status_str or "[주행 완료]" in status_str or "[이동 완료]" in status_str:
            clean_msg = status_str.replace("INFO:", "").strip()
            # Context-sensitive styling for precision docking & visual marker / carrot pursuit
            if "마커 검출" in clean_msg or "ArUco" in clean_msg or "당근점" in clean_msg or "3D 점군 ICP 일치" in clean_msg:
                color_hex = "#2dd4bf" # Bright Cyan/Teal
                icon = "🏷️"
            elif "도킹 완료" in clean_msg or "도킹 성공" in clean_msg:
                color_hex = "#4ade80" # Vivid Green
                icon = "🎉"
            elif "크롤링 전진 중" in clean_msg or "크롤링 도킹" in clean_msg:
                color_hex = "#fb923c" # Warm Orange
                icon = "🐢"
            elif "조준" in clean_msg or "정렬" in clean_msg or "피봇" in clean_msg or "정밀 도킹 개시" in clean_msg or "안착" in clean_msg:
                color_hex = "#38bdf8" # Sky Blue
                icon = "🎯"
            else:
                color_hex = "#4ade80" # Default Green
                icon = "ℹ️"

            self.dialog_box.append(f"<span style='color: {color_hex}; font-weight: bold;'>{icon} {clean_msg}</span>")
            sb = self.dialog_box.verticalScrollBar()
            sb.setValue(sb.maximum())
            return

        # 2. Extract Task Action (NAVIGATE_RELATIVE, DOCK, NAVIGATE, etc.)
        if "task=" in status_str:
            task_part = status_str.split("task=")[1].split(",")[0].strip()
            if task_part and task_part != "NONE":
                self.latest_task_action = task_part

        state_val = status_str.split("STATE=")[1].split(",")[0].strip() if "STATE=" in status_str else ""
        substate_val = status_str.split("substate=")[1].split(",")[0].strip() if "substate=" in status_str else ""

        approach_active_states = (
            "REQUEST_QWEN", "WAIT_QWEN", "ALIGN_CENTER",
            "APPROACH_OBJECT", "FINAL_SETTLE", "APPROACHED"
        )

        # 2.5 Extract precision docking / floor approach telemetry (ey, yaw_err, dist)
        docking_active = (state_val in (
            "INIT", "CHECK_STAGING", "STAGING_SETTLE", "STAGING_AIM",
            "CARROT_ALIGN", "STANDOFF_SETTLE", "CRAWL_CONTACT",
            "BACKUP_STANDOFF", "BACKUP_RETRY", "SEARCH_MARKER", "DOCKED"
        )) or (self.docking_fsm_state in (
            "INIT", "CHECK_STAGING", "STAGING_SETTLE", "STAGING_AIM",
            "CARROT_ALIGN", "STANDOFF_SETTLE", "CRAWL_CONTACT",
            "BACKUP_STANDOFF", "BACKUP_RETRY", "SEARCH_MARKER", "DOCKED"
        )) or (source == "APPROACH" and state_val in approach_active_states)

        if (source in ("DOCK", "APPROACH")) and docking_active:
            try:
                if "ey=" in status_str:
                    ey_str = status_str.split("ey=")[1].split(",")[0].replace("cm", "").strip()
                    ey_cm = float(ey_str)
                    self.target_lateral_err_cm = ey_cm
                    col = "#10b981" if abs(ey_cm) <= 2.8 else "#f59e0b"
                    self._set_card_value(self.axis_card, f"{ey_cm:+.1f} cm", col)

                if "yaw_err=" in status_str:
                    yaw_str = status_str.split("yaw_err=")[1].split(",")[0].replace("°", "").replace("deg", "").strip()
                    yaw_deg = float(yaw_str)
                    self.target_yaw_deg = yaw_deg
                    col = "#10b981" if abs(yaw_deg) <= 3.5 else "#f59e0b"
                    self._set_card_value(self.yaw_card, f"{yaw_deg:+.1f} °", col)

                if "dist=" in status_str:
                    dist_str = status_str.split("dist=")[1].split(",")[0].replace("m", "").strip()
                    dist_m = float(dist_str)
                    self.target_dist = dist_m
                    self._set_card_value(self.dist_card, f"{dist_m:.2f} m", "#38bdf8")
                self.target_time = time.time()
            except Exception:
                pass
        elif not docking_active and self.docking_fsm_state in ("IDLE", "NAV2_READY", "FAILSAFE"):
            # Standby mode during autonomous navigation
            self._set_card_value(self.dist_card, "--- m", "#38bdf8")
            self._set_card_value(self.yaw_card, "--- °", "#a78bfa")
            self._set_card_value(self.axis_card, "--- cm", "#f59e0b")
            self.target_dist = None
            self.target_yaw_deg = None
            self.target_lateral_err_cm = None

        if not state_val:
            return

        # 3. Conflict Resolution between Task Manager (10Hz) and Precision Docker (20Hz)
        docking_active_states = (
            "STAGE1_APPROACH", "STAGE1_VISUAL_SERVO", "STAGE2_STANDOFF_SETTLE", "STAGE3_BLIND_CRAWL", "BACKUP_RETRY",
            "STANDOFF_SETTLE", "STANDOFF_AIM_CARROT", "STANDOFF_DRIVE_CARROT", "STANDOFF_ALIGN_BOX", "STATION_VERIFY", "STATION_CRAWL",
            "SEARCH_MARKER", "STAGING_SETTLE", "STAGING_AIM", "CARROT_ALIGN", "CRAWL_CONTACT", "BACKUP_STANDOFF",
            "REQUEST_QWEN", "WAIT_QWEN", "APPROACH_OBJECT", "FINAL_SETTLE"
        )
        task_active_states = ("NAVIGATING", "ACQUIRING", "ALIGNING", "APPROACHING", "DOCKING", "AWAITING_PICK_CONFIRMATION", "WAITING_ARM_PICK")

        now = time.time()

        if source in ("DOCK", "APPROACH"):
            self.docking_fsm_state = state_val
        elif source == "TASK":
            self.task_fsm_state = state_val

        if state_val in ("SUCCESS", "ABORTED"):
            self.terminal_state = state_val
            self.terminal_cooldown_until = now + 3.0

        # Determine effective state to display (Priority: Active Motion > Cooldown Terminal > IDLE)
        if self.docking_fsm_state in docking_active_states:
            effective_state = self.docking_fsm_state
            self.terminal_state = None
        elif self.task_fsm_state in task_active_states:
            effective_state = self.task_fsm_state
            self.terminal_state = None
        elif self.terminal_state is not None and now < self.terminal_cooldown_until:
            effective_state = self.terminal_state
        else:
            self.terminal_state = None
            effective_state = "IDLE"

        prev_state = self.current_state
        self.current_state = effective_state

        is_docking_task = (self.latest_task_action == "DOCK")

        substate_kr_map = {
            "LATCH_STANDSTILL": "좌표 산출 중",
            "WAIT_USER_CONFIRM": "승인 대기",
            "AIM_CARROT": "당근점 정렬 회전",
            "PAUSE_AFTER_AIM": "자세 감쇠 대기",
            "DRIVE_CARROT": "당근점 직진 주행",
            "PAUSE_AFTER_DRIVE": "주행 후 진동 감쇠",
            "ALIGN_NORMAL": "상자 정면 정렬",
            "EVALUATE_STANDOFF": "카메라 오차 갱신 및 검증"
        }
        sub_display = substate_kr_map.get(substate_val, substate_val)

        label_map = {
            "SEARCH_MARKER": "DOCK: 0. 마커 검출 확인",
            "STAGING_SETTLE": "DOCK: 1. 대기점 1초 정지/오차 래치",
            "STAGING_AIM": "DOCK: 2. 당근점 방향 조준",
            "CARROT_ALIGN": "DOCK: 3. 가상 당근점 추종 주행",
            "STANDOFF_SETTLE": "DOCK: 4. 스탠드오프 정적 검증",
            "CRAWL_CONTACT": "DOCK: 5. 3cm/s 크롤링 안착",
            "BACKUP_STANDOFF": "DOCK: 오차 초과 안전 후진",
            "STANDOFF_AIM_CARROT": "DOCK: 2a. 대기점 방향 정렬",
            "STANDOFF_DRIVE_CARROT": "DOCK: 2b. 대기점 직진 전진",
            "STANDOFF_ALIGN_BOX": "DOCK: 3. 상자 정면 조준/락온",
            "STATION_VERIFY": "DOCK: 4. 마커 형상 검증",
            "STATION_CRAWL": "DOCK: 5. 3cm/s 크롤링 도킹",
            "STAGE1_APPROACH": f"DOCK: {sub_display}" if sub_display else "DOCK: 1. 당근점 접근",
            "STAGE1_VISUAL_SERVO": f"DOCK: {sub_display}" if sub_display else "DOCK: 1. 당근점 접근",
            "STAGE2_STANDOFF_SETTLE": "DOCK: 2. 스탠드오프 정지/검증",
            "STAGE3_BLIND_CRAWL": "DOCK: 3. 최종 크롤링",
            "BACKUP_RETRY": "DOCK: 후진 재진입(BACKUP)",
            "DOCKING": "DOCK: 정밀 도킹 진행중",
            "REQUEST_QWEN": "APPROACH: Qwen2.5-VL 시각 질의 요청",
            "WAIT_QWEN": "APPROACH: Qwen 시각 락온 대기",
            "ALIGN_CENTER": "APPROACH: 로봇 정중앙 정렬",
            "APPROACH_OBJECT": "APPROACH: 25cm 정밀 접근 주행",
            "FINAL_SETTLE": "APPROACH: 25cm 정지/미세 보정",
            "APPROACHING": "APPROACH: 바닥 물체 접근중",
            "APPROACHED": "APPROACH: 물체 25cm 안착 완료",
            "AWAITING_PICK_CONFIRMATION": "PICK: 🛑 물체 발견! 수거 승인 대기",
            "WAITING_ARM_PICK": "PICK: 🤖 로봇팔 수거 대기중",
            "NAVIGATING": "NAV: 이동 주행중",
            "SUCCESS": "SUCCESS (도킹/접근 완료)" if is_docking_task else "SUCCESS (이동 완료)",
            "ABORTED": "ABORTED (중단)"
        }
        display_text = label_map.get(effective_state, f"FSM: {effective_state}")
        self.state_badge.setText(display_text)

        color_map = {
            "IDLE": ("#1e293b", "#94a3b8"),
            "AWAITING_PICK_CONFIRMATION": ("rgba(245, 158, 11, 0.35)", "#fbbf24"),
            "WAITING_ARM_PICK": ("rgba(168, 85, 247, 0.35)", "#c084fc"),
            "APPROACHING": ("#1e3a8a", "#60a5fa"),
            "APPROACHED": ("#065f46", "#34d399"),
            "REQUEST_QWEN": ("rgba(234, 179, 8, 0.35)", "#facc15"),
            "WAIT_QWEN": ("rgba(168, 85, 247, 0.35)", "#c084fc"),
            "ALIGN_CENTER": ("rgba(2, 132, 199, 0.35)", "#38bdf8"),
            "APPROACH_OBJECT": ("rgba(59, 130, 246, 0.35)", "#60a5fa"),
            "FINAL_SETTLE": ("rgba(249, 115, 22, 0.35)", "#fb923c"),
            "NAVIGATING": ("rgba(2, 132, 199, 0.25)", "#38bdf8"),
            "ACQUIRING": ("rgba(234, 179, 8, 0.25)", "#facc15"),
            "ALIGNING": ("rgba(168, 85, 247, 0.25)", "#c084fc"),
            "DOCKING": ("rgba(236, 72, 153, 0.25)", "#f472b6"),
            "SEARCH_MARKER": ("rgba(234, 179, 8, 0.35)", "#facc15"),
            "STAGING_SETTLE": ("rgba(234, 179, 8, 0.35)", "#facc15"),
            "STAGING_AIM": ("rgba(2, 132, 199, 0.35)", "#38bdf8"),
            "CARROT_ALIGN": ("rgba(249, 115, 22, 0.35)", "#fb923c"),
            "STANDOFF_SETTLE": ("rgba(168, 85, 247, 0.35)", "#c084fc"),
            "CRAWL_CONTACT": ("rgba(249, 115, 22, 0.35)", "#fb923c"),
            "BACKUP_STANDOFF": ("rgba(239, 68, 68, 0.35)", "#f87171"),
            "STANDOFF_AIM_CARROT": ("rgba(2, 132, 199, 0.35)", "#38bdf8"),
            "STANDOFF_DRIVE_CARROT": ("rgba(59, 130, 246, 0.35)", "#60a5fa"),
            "STANDOFF_ALIGN_BOX": ("rgba(168, 85, 247, 0.35)", "#c084fc"),
            "STATION_VERIFY": ("rgba(20, 184, 166, 0.35)", "#2dd4bf"),
            "STATION_CRAWL": ("rgba(249, 115, 22, 0.35)", "#fb923c"),
            "STAGE1_APPROACH": ("rgba(2, 132, 199, 0.35)", "#38bdf8"),
            "STAGE1_VISUAL_SERVO": ("rgba(2, 132, 199, 0.35)", "#38bdf8"),
            "STAGE2_STANDOFF_SETTLE": ("rgba(234, 179, 8, 0.35)", "#facc15"),
            "STAGE3_BLIND_CRAWL": ("rgba(249, 115, 22, 0.35)", "#fb923c"),
            "BACKUP_RETRY": ("rgba(168, 85, 247, 0.35)", "#c084fc"),
            "VERIFYING": ("rgba(20, 184, 166, 0.25)", "#2dd4bf"),
            "SUCCESS": ("rgba(34, 197, 94, 0.35)", "#4ade80"),
            "ABORTED": ("rgba(239, 68, 68, 0.35)", "#f87171"),
        }
        bg, fg = color_map.get(effective_state, ("#1e293b", "#94a3b8"))
        self.state_badge.setStyleSheet(f"""
            background-color: {bg};
            color: {fg};
            border: 1px solid {fg};
            border-radius: 12px;
            padding: 3px 12px;
            font-size: 11px;
            font-weight: bold;
        """)

        # Substate confirmation announcement
        if substate_val == "WAIT_USER_CONFIRM" and not getattr(self, '_announced_confirm_wait', False):
            self._announced_confirm_wait = True
            confirm_msg = "🥕 <b>[당근점 생성 완료]</b> RViz에 당근점 좌표가 표시되었습니다. 하단의 <b>'🚀 당근점 이동 승인'</b> 버튼을 누르면 이동합니다."
            self.dialog_box.append(f"<span style='color: #facc15;'>{confirm_msg}</span>")
            sb = self.dialog_box.verticalScrollBar()
            sb.setValue(sb.maximum())
        elif substate_val != "WAIT_USER_CONFIRM":
            self._announced_confirm_wait = False

        # Standoff evaluation announcement & live refresh
        if substate_val == "EVALUATE_STANDOFF":
            if not getattr(self, '_announced_eval_standoff', False):
                self._announced_eval_standoff = True
                eval_start_msg = "📏 <b>[스탠드오프 도착]</b> 로봇 정지 및 카메라 안정화 중... 상자 정면 오차를 실시간 갱신합니다."
                self.dialog_box.append(f"<span style='color: #38bdf8;'>{eval_start_msg}</span>")
                sb = self.dialog_box.verticalScrollBar()
                sb.setValue(sb.maximum())
            now_t = time.time()
            if not hasattr(self, '_last_eval_gui_log_time') or (now_t - self._last_eval_gui_log_time) >= 0.8:
                self._last_eval_gui_log_time = now_t
                pass_icon = "🟢" if (abs(self.target_lateral_err*100) <= 5.0 and abs(self.target_yaw_deg) <= 5.0) else "🟡"
                err_detail = f"{pass_icon} [오차 실시간 갱신] 거리: {self.target_z:.2f}m | 측면(ey): {self.target_lateral_err*100:+.1f}cm | 각도: {self.target_yaw_deg:+.1f}° (기준: 5cm/5°)"
                self.dialog_box.append(f"<span style='color: #94a3b8; font-size: 11px;'>{err_detail}</span>")
                sb = self.dialog_box.verticalScrollBar()
                sb.setValue(sb.maximum())
        else:
            self._announced_eval_standoff = False

        # State transition dialogue guidance (only on state changes)
        if effective_state != prev_state:
            announcements = {
                "SEARCH_MARKER": "🔍 <b>[마커 탐색]</b> 전방 ArUco 마커 검출 여부를 확인합니다...",
                "STAGING_SETTLE": "⏸️ <b>[대기점 안정화]</b> 1.2초 정지 및 마커 기준 당근점 산출 중...",
                "STAGING_AIM": "🎯 <b>[당근점 조준]</b> 가상 당근점을 향해 제자리 회전 조준 중...",
                "CARROT_ALIGN": "🥕 <b>[당근점 추종]</b> S-Curve 당근점 Pure Pursuit 추종 주행 중...",
                "STANDOFF_SETTLE": "📏 <b>[스탠드오프 검증]</b> 마커 정면 정렬 및 정적 오차 검증 중...",
                "CRAWL_CONTACT": "🚀 <b>[크롤링 안착]</b> 최종 3.0cm/s 직진 크롤링 전진 시작 (물리 범퍼 감시 중)!",
                "BACKUP_STANDOFF": "🔁 <b>[안전 후진]</b> 오차 초과로 14cm 안전 후진 후 재정렬합니다.",
                "STANDOFF_AIM_CARROT": "🔄 <b>[대기점 조준]</b> 대기점 방향 제자리 회전 시작",
                "STANDOFF_DRIVE_CARROT": "🚗 <b>[대기점 안착]</b> 오도메트리 기반 대기점 직진 전진 중...",
                "STANDOFF_ALIGN_BOX": "🎯 <b>[상자 정면 조준]</b> 상자 중심을 향해 회전 및 카메라 락온 중...",
                "STATION_VERIFY": "🔍 <b>[형상 검증]</b> 상자 마커 위치 및 자세 정밀 확인 중...",
                "STATION_CRAWL": "🚀 <b>[크롤링 도킹]</b> 최종 3.0cm/s 직진 크롤링 전진 시작 (물리 범퍼 및 안전 뎁스 감시)!",
                "STAGE1_APPROACH": "🎯 <b>[정밀 도킹 개시]</b> 상자 위치 및 Standoff 당근점 산출을 시작합니다.",
                "STAGE1_VISUAL_SERVO": "🎯 <b>[정밀 도킹 개시]</b> 상자 위치 및 Standoff 당근점 산출을 시작합니다.",
                "STAGE2_STANDOFF_SETTLE": "⏸️ <b>[스탠드오프 정지]</b> 정지 및 자세 감쇠 중...",
                "STAGE3_BLIND_CRAWL": "🕶️ <b>[블라인드 직진]</b> 오차 검증 통과! 최종 크롤링을 시작합니다.",
                "BACKUP_RETRY": "🔁 <b>[재시도]</b> 스탠드오프 오차 초과. 후진 후 재진입합니다.",
                "SUCCESS": ("🎉 <b>[도킹 성공]</b> 상자 전면 정밀 도킹이 완료되었습니다!"
                            if is_docking_task
                            else "🎉 <b>[이동 완료]</b> 지정된 위치로 이동을 성공적으로 마쳤습니다.")
            }
            if effective_state in announcements:
                self.dialog_box.append(f"<span style='color: {fg};'>{announcements[effective_state]}</span>")
                sb = self.dialog_box.verticalScrollBar()
                sb.setValue(sb.maximum())

    def _append_agent_response(self, text: str):
        self.dialog_box.append(f"🤖 [Qwen Agent]: {text}")
        sb = self.dialog_box.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _update_target_pose(self, x: float, y: float, z: float, norm_yaw_rad: float = 0.0):
        # In base_footprint: x = forward distance (m), y = lateral offset (m)
        self.target_time = time.time()
        docking_active = (self.docking_fsm_state in (
            "INIT", "CHECK_STAGING", "STAGING_SETTLE", "STAGING_AIM",
            "CARROT_ALIGN", "STANDOFF_SETTLE", "CRAWL_CONTACT",
            "BACKUP_STANDOFF", "BACKUP_RETRY", "SEARCH_MARKER", "DOCKED"
        )) or (self.current_state in (
            "INIT", "CHECK_STAGING", "STAGING_SETTLE", "STAGING_AIM",
            "CARROT_ALIGN", "STANDOFF_SETTLE", "CRAWL_CONTACT",
            "BACKUP_STANDOFF", "BACKUP_RETRY", "SEARCH_MARKER", "DOCKED"
        ))

        if docking_active:
            # Fallback update if status string hasn't updated yet
            if self.target_dist is None:
                self.target_dist = x
                self._set_card_value(self.dist_card, f"{x:.2f} m")
            if self.target_yaw_deg is None:
                norm_yaw_deg = math.degrees(norm_yaw_rad)
                self.target_yaw_deg = norm_yaw_deg
                self._set_card_value(self.yaw_card, f"{norm_yaw_deg:+.1f} °")
            if self.target_lateral_err_cm is None:
                lat_cm = y * 100.0
                self.target_lateral_err_cm = lat_cm
                self._set_card_value(self.axis_card, f"{lat_cm:+.1f} cm")

    def _update_bumper(self, engaged: bool):
        self.bumper_engaged = engaged
        if engaged:
            self._set_card_value(self.bumper_card, "💥 CONTACT!", "#ef4444")
        else:
            self._set_card_value(self.bumper_card, "SAFE", "#10b981")

    def _update_velocity(self, vx: float, wz: float):
        self.current_vx = vx
        self.current_wz = wz
        self._set_card_value(self.vel_card, f"{vx:+.2f} / {wz:+.2f}")

    def _update_battery(self, voltage: float, percentage: float, status_str: str):
        self.bat_voltage_lbl.setText(f"VOLTAGE: {voltage:.2f} V ({percentage:.0f}%)")
        self.bat_progress.setValue(int(percentage))
        self.bat_status_lbl.setText(f"STATUS: {status_str}")

        # Update header badge
        self.hdr_bat_badge.setText(f"🔋 BAT: {voltage:.1f}V ({percentage:.0f}%)")

        if voltage < 11.0 and voltage > 5.0:
            self.bat_warning_lbl.setText("⚠️ LOW VOLTAGE WARNING (< 11.0V)!")
            self.bat_warning_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #ef4444;")
            self.hdr_bat_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #ef4444; background: #2d1515; border: 1px solid #ef4444; border-radius: 4px; padding: 4px 8px;")
        else:
            self.bat_warning_lbl.setText("● VOLTAGE NORMAL (11.0V+)")
            self.bat_warning_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981;")
            self.hdr_bat_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #38bdf8; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")

    def _update_odom_pose(self, x: float, y: float, yaw_deg: float, total_dist: float):
        self.pose_coord_lbl.setText(f"X: {x:+.3f} m  |  Y: {y:+.3f} m")
        self.pose_yaw_lbl.setText(f"HEADING (YAW): {yaw_deg:+.1f} °")
        self.odom_dist_lbl.setText(f"ODOM DISTANCE: {total_dist:.2f} m")

    def _update_topic_health(self, stats: dict):
        ok_count = 0
        total_count = len(stats)

        for name, data in stats.items():
            if name not in self.topic_labels:
                continue
            rate_lbl, status_dot = self.topic_labels[name]
            hz = data["hz"]
            status = data["status"]

            rate_lbl.setText(f"{hz:.1f} Hz")
            if status == "OK":
                ok_count += 1
                status_dot.setText("🟢 ACTIVE")
                status_dot.setStyleSheet("font-size: 10px; font-weight: bold; color: #10b981; background: rgba(16, 185, 129, 0.15); border: 1px solid #10b981; border-radius: 3px; padding: 1px 6px;")
            elif status == "DELAY":
                status_dot.setText("🟡 DELAY")
                status_dot.setStyleSheet("font-size: 10px; font-weight: bold; color: #f59e0b; background: rgba(245, 158, 11, 0.15); border: 1px solid #f59e0b; border-radius: 3px; padding: 1px 6px;")
            elif name == "TargetPose":
                # Event-driven detection topic: Normal to be standby until target appears in camera
                ok_count += 1
                status_dot.setText("⚪ STANDBY")
                status_dot.setStyleSheet("font-size: 10px; font-weight: bold; color: #94a3b8; background: #1e293b; border: 1px solid #334155; border-radius: 3px; padding: 1px 6px;")
            else:
                status_dot.setText("🔴 OFFLINE")
                status_dot.setStyleSheet("font-size: 10px; font-weight: bold; color: #ef4444; background: rgba(239, 68, 68, 0.15); border: 1px solid #ef4444; border-radius: 3px; padding: 1px 6px;")

        # Update header badge
        if ok_count == total_count:
            self.hdr_topic_badge.setText(f"🟢 TOPICS: {ok_count}/{total_count} OK")
            self.hdr_topic_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981; background: #0c121e; border: 1px solid #10b981; border-radius: 4px; padding: 4px 8px;")
        else:
            self.hdr_topic_badge.setText(f"🟡 TOPICS: {ok_count}/{total_count} ACTIVE")
            self.hdr_topic_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #f59e0b; background: #0c121e; border: 1px solid #f59e0b; border-radius: 4px; padding: 4px 8px;")

    def _update_connectivity(self, conn_dict: dict):
        ping_ms = conn_dict.get("pi_ping_ms")
        ollama_ok = conn_dict.get("ollama_ok", False)

        if ping_ms is not None:
            self.net_ping_lbl.setText(f"ROBOT (192.168.0.148): {ping_ms:.1f} ms")
            self.hdr_net_badge.setText(f"📡 ROBOT: {ping_ms:.0f}ms")
            self.hdr_net_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")
        else:
            self.net_ping_lbl.setText("ROBOT (192.168.0.148): NO PING / LOCAL")
            self.hdr_net_badge.setText("📡 ROBOT: OFFLINE")
            self.hdr_net_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8; background: #0c121e; border: 1px solid #1e293b; border-radius: 4px; padding: 4px 8px;")

        if ollama_ok:
            self.vlm_status_lbl.setText("OLLAMA VLM: ONLINE (qwen2.5vl:3b)")
            self.hdr_ollama_badge.setText("🧠 VLM: ONLINE")
            self.hdr_ollama_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #a78bfa; background: #0c121e; border: 1px solid #a78bfa; border-radius: 4px; padding: 4px 8px;")
        else:
            self.vlm_status_lbl.setText("OLLAMA VLM: OFFLINE (PORT 11434)")
            self.hdr_ollama_badge.setText("🧠 VLM: OFFLINE")
            self.hdr_ollama_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #ef4444; background: #0c121e; border: 1px solid #ef4444; border-radius: 4px; padding: 4px 8px;")

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    ros_node = Node('qwen_semantic_gui_node')

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = QwenMissionControlGUI(ros_node)
    gui.show()

    ret = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()

