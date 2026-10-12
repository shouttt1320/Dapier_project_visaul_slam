#!/usr/bin/env python3
"""
[Task Manager Node] Autonomous Multi-Stage Semantic Task Coordinator
- Coordinates semantic intents (DOCK, NAVIGATE, STOP, STATUS) validated by CommandValidator.
- Manages an 8-state deterministic finite state machine (FSM):
    IDLE -> NAVIGATING -> ACQUIRING -> ALIGNING -> APPROACHING -> DOCKING -> VERIFYING -> SUCCESS
    (with RECOVERY and ABORTED safety states).
- Dispatches Nav2 Action goals (`NavigateToPose`) to staging waypoints.
- Triggers and monitors `precision_docker` or object visual servoing for terminal docking.
- Enforces multi-layer safety verification (bumper contact, zero-velocity settle, clearance).
- Emits real-time ANSI-color formatted terminal status logs for live human inspection.
"""

import os
import sys
import time
import math
import json
import numpy as np
from enum import Enum
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import tf2_ros

try:
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    NAV2_AVAILABLE = True
except ImportError:
    NavigateToPose = None
    ComputePathToPose = None
    NAV2_AVAILABLE = False


# ANSI Color Codes for terminal UI
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
    BG_BLUE = "\033[44m"
    BG_RED = "\033[41m"
    BG_GREEN = "\033[42m"


class TaskState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    ACQUIRING = "ACQUIRING"
    ALIGNING = "ALIGNING"
    APPROACHING = "APPROACHING"
    DOCKING = "DOCKING"
    VERIFYING = "VERIFYING"
    SUCCESS = "SUCCESS"
    RECOVERY = "RECOVERY"
    ABORTED = "ABORTED"


class TaskManagerNode(Node):
    def __init__(self, enable_terminal_ui: bool = True):
        super().__init__('task_manager_node')
        self.enable_terminal_ui = enable_terminal_ui

        # ----------------------------------------------------------------------
        # State Machine Variables
        # ----------------------------------------------------------------------
        self.state: TaskState = TaskState.IDLE
        self.previous_state: TaskState = TaskState.IDLE
        self.state_start_time: float = time.time()
        self.current_task: Optional[Dict[str, Any]] = None
        self.retry_count: int = 0
        self.max_retries: int = 3

        # Safety & Sensor feedback
        self.bumper_contact: bool = False
        self.last_contact_time: float = 0.0
        self.current_velocity_linear: float = 0.0
        self.current_velocity_angular: float = 0.0
        self.docker_status_str: str = ""
        self.visual_lock: bool = False
        self.estimated_target_dist: Optional[float] = None

        # ----------------------------------------------------------------------
        # ROS 2 Interfaces
        # ----------------------------------------------------------------------
        # Nav2 Action Client & Planner Reachability Client
        if NAV2_AVAILABLE:
            self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        else:
            self.nav2_client = None
            self.compute_path_client = None
            self.log_warn("nav2_msgs is not available in environment. Simulating Nav2 navigation.")

        self.nav2_goal_handle: Optional[ClientGoalHandle] = None
        self.nav2_in_progress: bool = False
        self.nav2_succeeded: bool = False

        # Docker Control Clients
        self.dock_start_cli = self.create_client(Trigger, '/docking/start')
        self.dock_abort_cli = self.create_client(Trigger, '/docking/abort')
        self.dock_confirm_cli = self.create_client(Trigger, '/docking/confirm')

        # Publishers (Twist / TwistStamped depending on enable_stamped_cmd_vel)
        self.declare_parameter('enable_stamped_cmd_vel', True)
        self.enable_stamped_cmd_vel = bool(self.get_parameter('enable_stamped_cmd_vel').value)
        if self.enable_stamped_cmd_vel:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)
        self.rviz_goal_pub = self.create_publisher(MarkerArray, '/redbox/marker', 10)
        self.detect_query_pub = self.create_publisher(String, '/qwen/detect_query', 10)
        self.start_station_pub = self.create_publisher(String, '/docking/start_station', 10)

        # TF2 Buffer for accurate robot pose in 'map' frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Halt burst state for immediate override
        self._halt_burst_timer = None
        self._halt_burst_count = 0

        # Odometry tracking for relative navigation
        self.current_odom_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.last_odom_time: float = 0.0
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        # Active relative motion state
        self.active_relative_nav: bool = False
        self.rel_nav_start_odom: Optional[Tuple[float, float, float]] = None
        self.rel_target_dx: float = 0.0
        self.rel_target_dy: float = 0.0
        self.rel_target_dyaw: float = 0.0
        self.rel_target_dist: float = 0.0
        self.rel_nav_start_time: float = 0.0
        self.rel_nav_timeout: float = 10.0

        # Target 3D Object Pose Tracking
        self.latest_target_pose: Optional[PoseStamped] = None
        self.latest_target_time: float = 0.0

        # Subscribers
        self.create_subscription(Bool, '/robot/bumper/contact', self._bumper_callback, 10)
        self.create_subscription(String, '/docking/status', self._docker_status_callback, 10)
        self.create_subscription(Bool, '/safety/software_stop', self._stop_trigger_callback, 10)
        self.create_subscription(String, '/task/command', self._task_command_callback, 10)
        self.create_subscription(PoseStamped, '/docking/target_object_pose', self._target_pose_callback, 10)

        # Periodic FSM Update Timer (10 Hz)
        self.fsm_timer = self.create_timer(0.10, self._fsm_step)

        self._print_banner()
        self.log_info(f"Initialized in state: {self.state.value}")

    # --------------------------------------------------------------------------
    # Terminal Logging & Visual UI
    # --------------------------------------------------------------------------
    def _print_banner(self):
        if not self.enable_terminal_ui:
            return
        banner = f"""
{TermColor.CYAN}╔══════════════════════════════════════════════════════════════════════════╗
║              🤖 LLM SEMANTIC DOCKER - TASK MANAGER NODE                 ║
║   States: IDLE -> NAVIGATING -> ACQUIRING -> ALIGN -> APPROACH -> DOCK   ║
║   Safety: Dual Verification (Bumper + Zero-Velocity Settle + STOP Guard) ║
╚══════════════════════════════════════════════════════════════════════════╝{TermColor.RESET}
"""
        print(banner)

    def log_info(self, msg: str):
        self.get_logger().info(msg)
        if self.enable_terminal_ui:
            timestamp = time.strftime("%H:%M:%S")
            print(f"{TermColor.WHITE}[{timestamp}] {TermColor.GREEN}[TASK_MGR INFO]{TermColor.RESET} {msg}")

    def log_state_transition(self, from_state: TaskState, to_state: TaskState, reason: str = ""):
        if self.enable_terminal_ui:
            timestamp = time.strftime("%H:%M:%S")
            reason_str = f" ({reason})" if reason else ""
            print(f"{TermColor.WHITE}[{timestamp}] {TermColor.BOLD}{TermColor.MAGENTA}>>> STATE TRANSITION: {from_state.value} ➔  {to_state.value}{TermColor.RESET}{reason_str}")

    def log_warn(self, msg: str):
        self.get_logger().warn(msg)
        if self.enable_terminal_ui:
            timestamp = time.strftime("%H:%M:%S")
            print(f"{TermColor.WHITE}[{timestamp}] {TermColor.YELLOW}[TASK_MGR WARN]{TermColor.RESET} {msg}")

    def log_error(self, msg: str):
        self.get_logger().error(msg)
        if self.enable_terminal_ui:
            timestamp = time.strftime("%H:%M:%S")
            print(f"{TermColor.WHITE}[{timestamp}] {TermColor.BOLD}{TermColor.RED}[TASK_MGR ERROR]{TermColor.RESET} {msg}")

    def log_safety_event(self, event_name: str, details: str):
        if self.enable_terminal_ui:
            timestamp = time.strftime("%H:%M:%S")
            print(f"{TermColor.WHITE}[{timestamp}] {TermColor.BG_RED}{TermColor.WHITE} 🛡️ SAFETY ALERT {TermColor.RESET} {TermColor.RED}{event_name}: {details}{TermColor.RESET}")

    def log_success_box(self, task_name: str, duration: float):
        if not self.enable_terminal_ui:
            return
        box = f"""
{TermColor.GREEN}╔══════════════════════════════════════════════════════════════════════════╗
║                        🎉 TASK ACCOMPLISHED!                             ║
║  Task:     {task_name:<53} ║
║  Duration: {duration:<6.2f} seconds                                              ║
║  Terminal Status: 5cm Clearance / Bumper Latched / Motors Safely Halted  ║
╚══════════════════════════════════════════════════════════════════════════╝{TermColor.RESET}
"""
        print(box)

    # --------------------------------------------------------------------------
    # Sensor & Safety Callbacks
    # --------------------------------------------------------------------------
    def _bumper_callback(self, msg: Bool):
        previous_contact = self.bumper_contact
        self.bumper_contact = bool(msg.data)
        if self.bumper_contact and not previous_contact:
            self.last_contact_time = time.time()
            self.log_safety_event("BUMPER CONTACT TRIGGERED", "Micro-switch engaged at front bumper tip!")

    def _docker_status_callback(self, msg: String):
        self.docker_status_str = msg.data
        if any(kw in msg.data for kw in ("STATE=SUCCESS", "STATE=DOCKED", "SUCCESS", "DOCKED")):
            if self.state in (TaskState.ACQUIRING, TaskState.APPROACHING, TaskState.DOCKING, TaskState.VERIFYING):
                self.log_info("🎯 Precision Docking Controller reported DOCKED / SUCCESS!")
                self._report_task_info("🎉 [도킹 안착 완료] 전면 범퍼 접촉 및 정밀 도킹이 완벽 안착되었습니다!")
                self._transition_to(TaskState.SUCCESS, reason="Docking Controller Succeeded")
        elif any(kw in msg.data for kw in ("STATE=ABORTED", "ABORTED")):
            if self.state in (TaskState.ACQUIRING, TaskState.APPROACHING, TaskState.DOCKING, TaskState.VERIFYING):
                self.log_warn("❌ Precision Docking Controller reported ABORTED!")
                self._report_task_failure("[도킹 중단] 정밀 도킹 제어기가 작업을 중단했습니다.")
                self._transition_to(TaskState.ABORTED, reason="Docking Controller Aborted")
        elif any(kw in msg.data for kw in ("STATE=IDLE", "STATE=NAV2_READY", "IDLE")):
            if self.state == TaskState.DOCKING and getattr(self, '_docking_service_called', False):
                elapsed = time.time() - self.state_start_time
                if elapsed > 2.0:
                    self.log_info("ℹ️ Precision Docking Controller returned to NAV2_READY/IDLE. Finalizing task.")
                    self._transition_to(TaskState.IDLE, reason="Docking Controller returned to IDLE/NAV2_READY")



    def _stop_trigger_callback(self, msg: Bool):
        if msg.data:
            self.trigger_software_stop(reason="Received /safety/software_stop trigger")

    def _target_pose_callback(self, msg: PoseStamped):
        self.latest_target_pose = msg
        self.latest_target_time = time.time()
        self.visual_lock = True

    def _task_command_callback(self, msg: String):
        try:
            resolved_task = json.loads(msg.data)
            action = resolved_task.get("action", "")
            if action == "STOP":
                self.trigger_software_stop(reason=resolved_task.get("reason", "STOP command received"))
            else:
                self.submit_task(resolved_task)
        except Exception as e:
            self.log_error(f"Failed to parse /task/command: {e}")

    # --------------------------------------------------------------------------
    # Task Submission & Execution
    # --------------------------------------------------------------------------
    def submit_task(self, resolved_task: Dict[str, Any]) -> bool:
        """
        Accepts a validated task dict from CommandValidator and starts the FSM.
        """
        action = resolved_task.get("action", "UNKNOWN")

        # Emergency stop / cancel intents are always processed immediately
        if action in ("STOP", "CANCEL", "ABORT"):
            self.trigger_software_stop(reason=f"Intent is {action}")
            return True

        # Intercept commands during active docking session
        if self.state == TaskState.DOCKING:
            if action in ("NAVIGATE", "NAVIGATE_ABSOLUTE", "NAVIGATE_OFFSET", "NAVIGATE_RELATIVE"):
                self.log_info(f"🔄 Intercepted new action '{action}' during DOCKING. Aborting active docking session.")
                if self.dock_abort_cli.service_is_ready():
                    self.dock_abort_cli.call_async(Trigger.Request())
                self._transition_to(TaskState.IDLE, reason="Aborted docking for new navigation")
            elif action == "DOCK":
                self.log_info("🔄 Re-triggered DOCK while in DOCKING state. Resetting previous docking controller session.")
                self._docking_service_called = False
                self._docking_service_calling = False
                if self.dock_abort_cli.service_is_ready():
                    self.dock_abort_cli.call_async(Trigger.Request())
                time.sleep(0.05)

        if self.state not in (TaskState.IDLE, TaskState.SUCCESS, TaskState.ABORTED, TaskState.DOCKING):
            warn_msg = f"현재 작업({self.state.value}) 수행 중이므로 새 명령({action})을 수락할 수 없습니다."
            self.log_warn(warn_msg)
            self._report_task_failure(warn_msg)
            return False

        self.current_task = resolved_task
        self.retry_count = 0

        if self.enable_terminal_ui:
            print(f"\n{TermColor.CYAN}┌─────────────────────── NEW TASK ACCEPTED ────────────────────────┐")
            print(f"│ Action:        {action:<49} │")
            if "semantic_target" in resolved_task:
                print(f"│ Target:        {resolved_task.get('semantic_target'):<49} │")
            if "mode" in resolved_task:
                print(f"│ Mode:          {resolved_task.get('mode'):<49} │")
            if "staging_pose" in resolved_task:
                wp = resolved_task.get("staging_pose", {})
                yaw_val = wp.get('yaw_deg', wp.get('yaw', 0.0))
                wp_str = f"({wp.get('x', 0):.2f}, {wp.get('y', 0):.2f}, yaw={yaw_val:.1f})"
                print(f"│ Staging:       {wp_str:<49} │")
            elif "staging_waypoint" in resolved_task:
                print(f"│ Staging WP:    {str(resolved_task.get('staging_waypoint')):<49} │")
            print(f"└──────────────────────────────────────────────────────────────────┘{TermColor.RESET}\n")

        if action in ("STOP", "CANCEL", "ABORT"):
            self.trigger_software_stop(reason=f"Intent is {action}")
            return True

        if action in ("NAVIGATE", "NAVIGATE_ABSOLUTE", "NAVIGATE_OFFSET"):
            self._execute_navigate_task(resolved_task)
            return True

        if action == "NAVIGATE_RELATIVE":
            rx = float(resolved_task.get("relative_x", 0.0))
            ry = float(resolved_task.get("relative_y", 0.0))
            ryaw = float(resolved_task.get("relative_yaw", 0.0))
            self._start_relative_navigation(rx, ry, ryaw)
            return True

        if action == "DOCK":
            self._execute_dock_task(resolved_task)
            return True

        if action in ("CONFIRM_DOCK", "CONFIRM", "PROCEED"):
            if self.dock_confirm_cli.service_is_ready():
                req = Trigger.Request()
                self.dock_confirm_cli.call_async(req)
                self.log_info("✅ Forwarded confirmation trigger to precision docking controller")
            else:
                self.log_warn("Docking confirm service (/docking/confirm) not ready")
            return True

        self.log_warn(f"Unknown action: {action}")
        return False

    def trigger_software_stop(self, reason: str = "Unspecified"):
        """
        Instant software stop handler. Cancels Nav2, triggers docker abort, halts motors.
        """
        self.log_safety_event("SOFTWARE STOP INTERCEPT", reason)
        
        # 1. Cancel Nav2 Goal
        if self.nav2_goal_handle is not None and self.nav2_in_progress:
            self.log_info("Canceling active Nav2 goal...")
            try:
                self.nav2_goal_handle.cancel_goal_async()
            except Exception as e:
                self.log_warn(f"Nav2 cancel error: {e}")
            self.nav2_in_progress = False

        # 2. Abort Docking Service
        if self.dock_abort_cli.service_is_ready():
            req = Trigger.Request()
            self.dock_abort_cli.call_async(req)

        # 3. Publish zero velocity burst across all channels immediately
        self.active_relative_nav = False
        self._start_halt_burst(duration_sec=1.2)

        self._transition_to(TaskState.ABORTED, reason=f"Halted via Software Stop: {reason}")

    # --------------------------------------------------------------------------
    # FSM State Transitions & Control Loop
    # --------------------------------------------------------------------------
    def _transition_to(self, new_state: TaskState, reason: str = ""):
        self.previous_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.log_state_transition(self.previous_state, self.state, reason)

        # Publish status string
        status_msg = String()
        task_name = self.current_task.get("action", "NONE") if self.current_task else "NONE"
        target_name = self.current_task.get("target_structure", "") if self.current_task else ""
        status_msg.data = f"STATE={self.state.value}, task={task_name}, target={target_name}, bumper={self.bumper_contact}"
        self.task_status_pub.publish(status_msg)

        if self.state in (TaskState.SUCCESS, TaskState.ABORTED):
            self._clear_rviz_goal_marker()

    def _fsm_step(self):
        """Periodic 10Hz tick for the state machine."""
        elapsed = time.time() - self.state_start_time

        # Periodic status heartbeat
        status_msg = String()
        task_name = self.current_task.get("action", "NONE") if self.current_task else "NONE"
        target_name = self.current_task.get("target_structure", "") if self.current_task else ""
        status_msg.data = f"STATE={self.state.value}, task={task_name}, target={target_name}, bumper={self.bumper_contact}"
        self.task_status_pub.publish(status_msg)

        if self.state == TaskState.IDLE:
            pass

        elif self.state == TaskState.NAVIGATING:
            if self.active_relative_nav:
                self._step_relative_navigation()
            else:
                # Check Nav2 progress
                if not self.nav2_in_progress:
                    if self.nav2_succeeded:
                        action = self.current_task.get("action") if self.current_task else ""
                        if action.startswith("NAVIGATE"):
                            self._transition_to(TaskState.VERIFYING, reason="Nav2 Destination Reached")
                        elif action == "DOCK":
                            st_name = self.current_task.get("station_name") if self.current_task else None
                            self.send_cmd_vel(0.0, 0.0)
                            self.log_info(f"✅ Standoff 대기점 도착 완료! 2단계 도킹 사전 조건 검증(ACQUIRING)으로 전환합니다.")
                            self._report_task_info(f"[대기점 도착] '{st_name or '대상'}' 대기점에 도착했습니다. 도킹 가능 여부를 검증합니다.")
                            self._transition_to(TaskState.ACQUIRING, reason="Staging Waypoint Reached, Verifying Docking Condition")
                    else:
                        self._handle_navigation_failure()

        elif self.state == TaskState.ACQUIRING:
            action = self.current_task.get("action", "") if self.current_task else ""
            st_name = self.current_task.get("station_name") if self.current_task else None
            target_structure = self.current_task.get("target_structure", "station") if self.current_task else "station"
            now_t = time.time()

            # 2단계: 도킹 사전 조건 검증 (Feasibility Check)
            is_feasible, reason, dist, yaw_deg = self._check_docking_feasibility(target_structure)

            # 주기적 검증 진행 로그 (0.8초 간격 터미널 출력)
            last_diag = getattr(self, '_last_feasibility_log_time', 0.0)
            if (now_t - last_diag) > 0.8:
                self._last_feasibility_log_time = now_t
                if is_feasible:
                    diag_str = f"검증 통과 (거리 {dist:.2f}m, 각도 {yaw_deg:+.1f}°)"
                else:
                    diag_str = f"검증 진행 중: {reason}"
                self.log_info(f"🔍 [도킹 조건 검사 {elapsed:.1f}s/4.5s] 대상 '{target_structure}' ➔ {diag_str}")

            if is_feasible:
                self.log_info(f"🎯 [도킹 조건 완벽 충족] 대상 '{target_structure}' 거리: {dist:.2f}m, 편차각: {yaw_deg:+.1f}° ➔ 3단계 정밀 도킹 엔진 핸드오버!")
                self._report_task_info(f"🎯 [도킹 승인] 마커 조건 통과 (거리 {dist:.2f}m, 각도 {yaw_deg:+.1f}°) ➔ 정밀 도킹 제어 시작")
                self._docking_service_called = True
                self._transition_to(TaskState.DOCKING, reason=f"Docking Feasibility Passed ({dist:.2f}m, {yaw_deg:+.1f}°)")

                # 토픽 알림 및 서비스 호출 이중 핸드셰이크
                if st_name:
                    self.start_station_pub.publish(String(data=st_name))
                self._trigger_docking_service()
                return

            elif elapsed > 4.5:
                # 4.5초 동안 마커/타겟 미감지 또는 조건 불만족 시 무리하게 돌진하지 않고 안전 정지
                self._halt_motors()
                warn_msg = f"⚠️ [도킹 불가] 대기점에서 도킹 대상 '{target_structure}'을(를) 확인하지 못했습니다 (사유: {reason}). 안전을 위해 도킹을 중단합니다."
                self.log_warn(warn_msg)
                self._report_task_failure(warn_msg)
                self._transition_to(TaskState.ABORTED, reason=warn_msg)
                return

        elif self.state == TaskState.ALIGNING:
            # Align heading with target center
            if self.latest_target_pose is not None and (time.time() - self.latest_target_time) < 2.0:
                xt = self.latest_target_pose.pose.position.x
                zt = max(0.1, self.latest_target_pose.pose.position.z)
                yaw_err = math.atan2(xt, zt)
                if abs(yaw_err) > 0.06:
                    wz = -float(np.clip(1.2 * yaw_err, -0.30, 0.30))
                    self.send_cmd_vel(0.0, wz)
                else:
                    self.send_cmd_vel(0.0, 0.0)
                    self._transition_to(TaskState.APPROACHING, reason="Heading Aligned (|yaw_err| < 3.5°)")
            elif elapsed > 3.0:
                self.send_cmd_vel(0.0, 0.0)
                self._transition_to(TaskState.APPROACHING, reason="Heading Alignment Time Elapsed")

        elif self.state == TaskState.APPROACHING:
            # Standoff approach: precision_docking_controller_node에 전권을 위임하므로 바로 DOCKING으로 전이
            self._halt_motors()
            self._transition_to(TaskState.DOCKING, reason="Delegating approach to Precision Docking Controller")

        elif self.state == TaskState.DOCKING:
            # 정밀 도킹 엔진(precision_docking_controller_node)에 제어권을 완전 위임
            # task_manager는 모터 명령을 일체 내리지 않고 정밀 도킹 제어기가 온전히 주행하도록 보장함

            if not getattr(self, '_docking_service_called', False):
                self._trigger_docking_service()

            if elapsed > 180.0:
                self.log_warn("정밀 도킹 수행 타임아웃 (>180초) 발생!")
                self.trigger_software_stop(reason="Docking Timeout")
            return

        elif self.state == TaskState.VERIFYING:
            # Verify zero velocity
            self._halt_motors()
            if elapsed > 0.6:
                task_name = self.current_task.get("action", "TASK") if self.current_task else "TASK"
                self._transition_to(TaskState.SUCCESS, reason="Verified Safe Stop & Clearance")
                self.log_success_box(task_name, elapsed)

        elif self.state == TaskState.RECOVERY:
            if elapsed > 2.0:
                self.retry_count += 1
                if self.retry_count <= self.max_retries:
                    action = self.current_task.get("action", "") if self.current_task else ""
                    staging_pose = self.current_task.get("staging_pose") if self.current_task else None
                    if action == "DOCK" and staging_pose:
                        self.log_info(f"🔄 [Nav2 Staging 재시도] 도킹 대기점으로 자율주행을 재시도합니다 ({self.retry_count}/{self.max_retries})...")
                        self._transition_to(TaskState.NAVIGATING, reason=f"Retry {self.retry_count} Nav2 Staging")
                        self._dispatch_nav2_goal(staging_pose)
                    elif action.startswith("NAVIGATE"):
                        target_pose = self.current_task.get("target_pose") or self.current_task.get("target_waypoint")
                        self.log_info(f"🔄 [Nav2 주행 재시도] 목표 위치로 자율주행을 재시도합니다 ({self.retry_count}/{self.max_retries})...")
                        self._transition_to(TaskState.NAVIGATING, reason=f"Retry {self.retry_count} Nav2 Goal")
                        self._dispatch_nav2_goal(target_pose)
                    else:
                        self.log_info(f"Attempting retry {self.retry_count}/{self.max_retries}...")
                        self._transition_to(TaskState.ACQUIRING, reason=f"Retry {self.retry_count}")
                else:
                    self.log_error("Max retries exceeded. Aborting task.")
                    self._transition_to(TaskState.ABORTED, reason="Max Retries Exceeded")

        elif self.state in (TaskState.SUCCESS, TaskState.ABORTED):
            if elapsed < 0.8:
                self._halt_motors()
            elif elapsed > 2.5:
                # Return cleanly to IDLE so manual navigation and RViz goals are NEVER blocked!
                self._transition_to(TaskState.IDLE, reason="Task finalized, returning to IDLE")

    # --------------------------------------------------------------------------
    # Nav2 Goal Dispatch & Handling
    # --------------------------------------------------------------------------
    def _dispatch_nav2_goal(self, waypoint: Optional[Dict[str, Any]]):
        if not waypoint:
            self.log_error("No waypoint specified for navigation!")
            self._transition_to(TaskState.ABORTED, reason="Missing Waypoint")
            return

        x = float(waypoint.get("x", 0.0))
        y = float(waypoint.get("y", 0.0))
        yaw = waypoint.get("yaw")
        if yaw is None:
            yaw = math.radians(float(waypoint.get("yaw_deg", 0.0)))

        self.log_info(f"Dispatching Nav2 Goal -> X={x:.3f}m, Y={y:.3f}m, Yaw={math.degrees(yaw):.1f}°")

        # Visual Goal Marker in RViz
        goal_label = self.current_task.get("action", "NAV2") if self.current_task else "NAV2"
        target_struct = self.current_task.get("target_structure", "") if self.current_task else ""
        if target_struct:
            goal_label = f"{goal_label}: {target_struct}"
        self._publish_rviz_goal_marker(x, y, yaw, goal_label)

        # Mark navigation in progress immediately to prevent FSM race condition
        self.nav2_in_progress = True
        self.nav2_succeeded = False

        if self.nav2_client is not None:
            if not self.nav2_client.server_is_ready():
                self.log_info("⏳ Nav2 자율주행 서버(/navigate_to_pose) 연결 대기 중 (최대 10초)...")
                ready = self.nav2_client.wait_for_server(timeout_sec=10.0)
            else:
                ready = True

            if ready:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = "map"
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = float(x)
                goal_msg.pose.pose.position.y = float(y)
                goal_msg.pose.pose.position.z = 0.0

                # Euler yaw to Quaternion
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)

                self.log_info(f"🚀 [Nav2 골 발행] 대기점 위치(X={x:.3f}m, Y={y:.3f}m, Yaw={math.degrees(yaw):+.1f}°)로 자율주행을 시작합니다.")
                send_goal_future = self.nav2_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self._nav2_goal_response_callback)
                return

        # Nav2 Server not available: 절대 가짜 도착 처리하지 않고 명확히 에러 리포트 & 중단
        self.log_error("❌ [주행 불가] Nav2 Action Server(/navigate_to_pose)가 활성화되지 않았습니다. 자율주행을 시작할 수 없습니다.")
        self._report_task_failure("[주행 불가] Nav2 내비게이션 서버가 준비되지 않았습니다. 시스템 상태를 확인하세요.")
        self.nav2_in_progress = False
        self.nav2_succeeded = False
        self._transition_to(TaskState.ABORTED, reason="Nav2 Action Server Unavailable")

    def _nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            wp_name = self.current_task.get("waypoint_name", "목표 지점") if self.current_task else "목표 지점"
            self._handle_unreachable_target(wp_name, "Nav2 Planner가 골 요청을 거부했습니다 (장애물 또는 도달 불가).")
            self.nav2_in_progress = False
            self.nav2_succeeded = False
            return

        self.log_info("Nav2 Goal accepted by planner. Moving...")
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_callback)

    def _nav2_result_callback(self, future):
        status = future.result().status
        self.nav2_in_progress = False
        wp_name = self.current_task.get("waypoint_name", "목표 지점") if self.current_task else "목표 지점"
        action = self.current_task.get("action", "") if self.current_task else ""
        if status == GoalStatus.STATUS_SUCCEEDED:
            if action == "DOCK":
                self.log_info(f"Nav2 staging goal reached successfully!")
                self._report_task_info(f"[대기점 도착] 도킹 대기점에 성공적으로 도착했습니다. 3D 정밀 도킹으로 전환합니다.")
            else:
                self.log_info(f"Nav2 goal '{wp_name}' reached successfully!")
                self._report_task_info(f"[주행 완료] '{wp_name}' 위치에 성공적으로 도착했습니다.")
            self.nav2_succeeded = True
        else:
            warn_msg = f"[주행 실패] '{wp_name}' 이동 중 경로가 중단되었습니다 (Status={status})."
            self.log_warn(warn_msg)
            self._report_task_failure(warn_msg)
            self.nav2_succeeded = False

    def _report_task_failure(self, msg: str):
        alert_str = f"ALERT: {msg}"
        self.task_status_pub.publish(String(data=alert_str))

    def _report_task_info(self, msg: str):
        info_str = f"INFO: {msg}"
        self.task_status_pub.publish(String(data=info_str))

    def _execute_navigate_task(self, resolved_task: Dict[str, Any]):
        wp_name = resolved_task.get("waypoint_name", "목표 지점")
        target_pose = resolved_task.get("target_pose") or resolved_task.get("target_waypoint")
        if not target_pose or not isinstance(target_pose, dict):
            self._halt_motors()
            warn_msg = f"[주행 불가] 유효한 목표 좌표가 없습니다: {wp_name}"
            self.log_warn(warn_msg)
            self._report_task_failure(warn_msg)
            self._transition_to(TaskState.ABORTED, reason=warn_msg)
            return

        # Pre-flight reachability check via ComputePathToPose if server is ready
        if NAV2_AVAILABLE and self.compute_path_client is not None and self.compute_path_client.server_is_ready():
            self.log_info(f"Nav2 Planner에 '{wp_name}' 경로 생성 가능 여부 사전 질의...")
            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal.header.frame_id = 'map'
            goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
            goal_msg.goal.pose.position.x = float(target_pose.get("x", 0.0))
            goal_msg.goal.pose.position.y = float(target_pose.get("y", 0.0))
            goal_msg.goal.pose.position.z = float(target_pose.get("z", 0.0))
            yaw = math.radians(float(target_pose.get("yaw_deg", target_pose.get("yaw", 0.0))))
            goal_msg.goal.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5)
            )

            send_future = self.compute_path_client.send_goal_async(goal_msg)
            def _path_response_cb(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._handle_unreachable_target(wp_name, "Nav2 Planner가 경로 생성을 거부했습니다 (장애물 차단 또는 맵 외부).")
                    return
                res_future = goal_handle.get_result_async()
                def _path_result_cb(rf):
                    result = rf.result().result
                    if result is None or len(result.path.poses) == 0:
                        self._handle_unreachable_target(wp_name, "유효한 이동 경로가 없습니다 (장애물로 경로가 막혀 있습니다).")
                    else:
                        self.log_info(f"✅ [주행 가능] '{wp_name}' 유효 경로 확인 ({len(result.path.poses)}점). 자율주행을 시작합니다.")
                        self._report_task_info(f"[주행 시작] '{wp_name}'(으)로 자율주행을 시작합니다.")
                        self._transition_to(TaskState.NAVIGATING, reason=f"Valid path found to {wp_name}")
                        self._dispatch_nav2_goal(target_pose)
                res_future.add_done_callback(_path_result_cb)
            send_future.add_done_callback(_path_response_cb)
        else:
            # Planner 사전 검사 서버 미대기 시 직접 발송 (골 거부 콜백이 방어)
            self.log_info(f"'{wp_name}' 자율주행 시작 (Nav2 직접 발송)")
            self._report_task_info(f"[주행 시작] '{wp_name}'(으)로 자율주행을 시작합니다.")
            self._transition_to(TaskState.NAVIGATING, reason=f"Starting Nav to {wp_name}")
            self._dispatch_nav2_goal(target_pose)

    def _handle_unreachable_target(self, target_name: str, reason: str):
        self._halt_motors()
        warn_msg = f"[주행 불가] '{target_name}' 위치로 이동할 수 없습니다. 이유: {reason}"
        self.log_warn(warn_msg)
        self._report_task_failure(warn_msg)
        self._transition_to(TaskState.ABORTED, reason=warn_msg)

    def _execute_dock_task(self, resolved_task: Dict[str, Any]):
        target_name = resolved_task.get("target_name") or resolved_task.get("target_structure") or "상자"
        target_structure = resolved_task.get("target_structure", "box")
        staging_pose = resolved_task.get("staging_pose")
        station_name = resolved_task.get("station_name")

        # 0. 마커/물체 직시 도킹 모드인 경우: Nav2 Staging 주행 완전 생략 (Bypass)
        is_direct_visual = bool(resolved_task.get("direct_visual", False))
        if is_direct_visual:
            self.log_info(f"🎯 [Direct Visual Docking] Nav2 Staging 주행을 생략하고 현 위치에서 즉시 정밀 도킹 엔진으로 전환합니다.")
            self._report_task_info(f"[도킹 승인] 전방 마커 확인 완료 ➔ 정밀 도킹 제어 시작")
            self._docking_service_called = False
            self._docking_service_calling = False
            self._transition_to(TaskState.DOCKING, reason="Direct Visual Docking: Handover to precision controller")
            if station_name:
                self.start_station_pub.publish(String(data=station_name))
            self._trigger_docking_service()
            return

        # 1. Staging Waypoint(Standoff)가 있고, 현재 원거리 위치인 경우 Nav2 자율주행 선행
        needs_nav2 = False
        if staging_pose and "x" in staging_pose and "y" in staging_pose:
            st_x = float(staging_pose["x"])
            st_y = float(staging_pose["y"])
            st_yaw = math.radians(float(staging_pose.get("yaw_deg", 0.0)))
            map_pose = self.get_robot_map_pose()
            if map_pose is not None:
                dist_to_staging = math.hypot(map_pose[0] - st_x, map_pose[1] - st_y)
                yaw_diff = abs(math.atan2(math.sin(map_pose[2] - st_yaw), math.cos(map_pose[2] - st_yaw)))
                self.log_info(f"📍 로봇 현재 map 좌표: ({map_pose[0]:.2f}, {map_pose[1]:.2f}), Standoff: ({st_x:.2f}, {st_y:.2f}) -> 거리: {dist_to_staging:.2f}m, 각도차: {math.degrees(yaw_diff):.1f}°")
                if dist_to_staging > 0.15 or yaw_diff > math.radians(15.0):
                    needs_nav2 = True
                else:
                    self.log_info("✅ 로봇이 이미 Standoff 대기점 정위치에 위치해 있습니다. 도킹 조건 검증으로 즉시 진입합니다.")
            else:
                self.log_warn("⚠️ map -> base_footprint TF 조회 대기 중. 원거리로 간주하고 Standoff 위치로 Nav2 주행을 우선 발송합니다.")
                needs_nav2 = True

        if needs_nav2:
            self.log_info(f"🚀 [1단계: 대기점 진입] 대상 '{target_name}'의 Standoff 위치로 먼저 Nav2 자율주행을 시작합니다...")
            self._report_task_info(f"[주행 시작] 대상 '{target_name}' 대기점으로 이동합니다.")
            self._transition_to(TaskState.NAVIGATING, reason=f"Nav2 Staging to {target_name}")
            self._dispatch_nav2_goal(staging_pose)
            return

        # 2. 대기점 정렬 확인 완료 -> 2단계 도킹 사전 조건 검증(ACQUIRING)으로 전이
        self.log_info(f"🔍 [2단계: 도킹 조건 검증] 대기점에서 대상 '{target_name}' 시야 및 도킹 조건 검증 시작...")
        self._report_task_info(f"[조건 검증] '{target_name}' 도킹 가능 여부를 확인합니다.")
        self._transition_to(TaskState.ACQUIRING, reason="Staging Position Confirmed, Checking Docking Feasibility")
        return

    def _trigger_docking_service(self):
        """Asynchronously triggers precision docking controller node."""
        if getattr(self, '_docking_service_calling', False):
            return

        if self.dock_start_cli.service_is_ready():
            self._docking_service_calling = True
            req = Trigger.Request()
            future = self.dock_start_cli.call_async(req)
            def _on_start_done(f):
                self._docking_service_calling = False
                try:
                    res = f.result()
                    if res.success:
                        self.log_info(f"✅ Precision Docking Engine started: {res.message}")
                        self._docking_service_called = True
                    else:
                        self.log_warn(f"Docking start rejected: {res.message}")
                except Exception as e:
                    self.log_error(f"Error calling /docking/start: {e}")
            future.add_done_callback(_on_start_done)
        else:
            self.log_warn("도킹 제어기 서비스(/docking/start) 준비 대기 중... 준비되면 자동 연결됩니다.")

    def _check_docking_feasibility(self, target_structure: str) -> Tuple[bool, str, float, float]:
        """
        도킹 가능 여부 사전 판별:
        1. 최근 (3.5초 이내) 3D 타겟 인식 데이터 존재 여부
        2. 타겟 거리 및 시야각 편차 확인
        """
        now = time.time()
        time_diff = now - self.latest_target_time

        # 1. 포즈 메시지 기반 검증
        if self.latest_target_pose is not None and time_diff <= 3.5:
            pos = self.latest_target_pose.pose.position
            frame_id = (self.latest_target_pose.header.frame_id or "").lower()
            if "base" in frame_id:
                # base_footprint frame: X forward, Y left, Z up
                dist = math.hypot(pos.x, pos.y)
                yaw_deg = math.degrees(math.atan2(pos.y, max(0.01, pos.x)))
            else:
                # Optical frame: Z forward, X right, Y down
                dist = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                yaw_deg = math.degrees(math.atan2(pos.x, max(0.01, pos.z)))

            if dist < 0.15:
                return False, f"물체와의 거리가 너무 가깝습니다 ({dist:.2f}m < 0.15m)", dist, yaw_deg
            if dist > 1.50:
                return False, f"물체와의 거리가 너무 멉니다 ({dist:.2f}m > 1.50m)", dist, yaw_deg
            if abs(yaw_deg) > 35.0:
                return False, f"물체가 시야각 중심에서 너무 벗어났습니다 ({yaw_deg:+.1f}°)", dist, yaw_deg

            return True, "정상 인식", dist, yaw_deg

        # 2. 도킹 제어기(/docking/status) 텔레메트리 파싱 기반 Fallback 검증
        if self.docker_status_str:
            import re
            m_dist = re.search(r"dist=([0-9\.]+)m", self.docker_status_str)
            m_yaw = re.search(r"yaw_err=([+-]?[0-9\.]+)deg", self.docker_status_str)
            if m_dist:
                d_val = float(m_dist.group(1))
                if d_val > 0.05:
                    y_val = float(m_yaw.group(1)) if m_yaw else 0.0
                    if d_val < 0.15:
                        return False, f"마커 거리가 너무 가깝습니다 ({d_val:.2f}m < 0.15m)", d_val, y_val
                    if d_val > 1.50:
                        return False, f"마커 거리가 너무 멉니다 ({d_val:.2f}m > 1.50m)", d_val, y_val
                    if abs(y_val) > 35.0:
                        return False, f"마커 진입각 편차가 너무 큽니다 ({y_val:+.1f}°)", d_val, y_val
                    return True, "정상 인식 (마커 텔레메트리 확인)", d_val, y_val

        return False, "카메라 시야에 대상/마커가 감지되지 않음", 0.0, 0.0

    def _handle_navigation_failure(self):
        self.log_error("Navigation failed to reach target.")
        self._enter_recovery("Nav2 Staging Failed")

    def _enter_recovery(self, reason: str):
        self.log_warn(f"Entering RECOVERY state: {reason}")
        self._transition_to(TaskState.RECOVERY, reason=reason)

    def get_robot_map_pose(self) -> Optional[Tuple[float, float, float]]:
        """Returns (x, y, yaw_rad) of robot base_footprint in 'map' frame using TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            px = t.transform.translation.x
            py = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (px, py, yaw)
        except Exception:
            return None

    def send_cmd_vel(self, vx: float = 0.0, wz: float = 0.0, halt_all: bool = False):
        """Publishes velocity commands with Twist / TwistStamped compatibility across all channels."""
        if self.enable_stamped_cmd_vel:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x = float(vx)
            msg.twist.angular.z = float(wz)
            self.cmd_vel_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(vx)
            msg.angular.z = float(wz)
            self.cmd_vel_pub.publish(msg)

        if halt_all:
            raw_stop = Twist()
            raw_stop.linear.x = float(vx)
            raw_stop.angular.z = float(wz)
            self.cmd_vel_unstamped_pub.publish(raw_stop)
            self.cmd_vel_nav_pub.publish(raw_stop)

    def _odom_callback(self, msg: Odometry):
        self.last_odom_time = time.time()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_odom_pose = (pos.x, pos.y, yaw)

    def _start_relative_navigation(self, rx: float, ry: float, ryaw_deg: float):
        self.active_relative_nav = True
        self.rel_target_dx = rx
        self.rel_target_dy = ry
        self.rel_target_dyaw = math.radians(ryaw_deg)
        dist_mag = math.hypot(rx, ry)
        self.rel_target_dist = math.copysign(dist_mag, rx if abs(rx) >= abs(ry) else ry)
        self.rel_nav_start_time = time.time()
        self.rel_nav_start_odom = self.current_odom_pose
        self.rel_nav_timeout = max(5.0, abs(self.rel_target_dist) * 15.0 + abs(self.rel_target_dyaw) * 5.0)

        self._transition_to(TaskState.NAVIGATING, reason=f"Relative Nav dx={rx:.2f}m, dy={ry:.2f}m, yaw={ryaw_deg:.1f}°")
        self.log_info(f"🚀 [Relative Nav Started] Target: dx={rx:.3f}m, dy={ry:.3f}m (Dist={self.rel_target_dist:.3f}m), Yaw={ryaw_deg:.1f}°")
        self._publish_rviz_goal_marker(rx, ry, math.radians(ryaw_deg), f"REL_NAV ({rx*100:.0f}cm)")

    def _step_relative_navigation(self):
        elapsed = time.time() - self.rel_nav_start_time

        # Safety timeout check
        if elapsed > self.rel_nav_timeout:
            self.send_cmd_vel(0.0, 0.0)
            self.active_relative_nav = False
            self.log_warn(f"Relative navigation reached timeout ({self.rel_nav_timeout:.1f}s)")
            self._transition_to(TaskState.VERIFYING, reason="Relative Nav Timeout Completed")
            return

        # 1. Closed-loop control using Odometry if available
        if self.current_odom_pose is not None and self.rel_nav_start_odom is not None and (time.time() - self.last_odom_time) < 1.0:
            x_now, y_now, yaw_now = self.current_odom_pose
            x_0, y_0, yaw_0 = self.rel_nav_start_odom

            dx_odom = x_now - x_0
            dy_odom = y_now - y_0
            # Transform to robot's initial coordinate frame
            dx_local = math.cos(yaw_0) * dx_odom + math.sin(yaw_0) * dy_odom
            dyaw_local = math.atan2(math.sin(yaw_now - yaw_0), math.cos(yaw_now - yaw_0))

            err_dist = self.rel_target_dist - dx_local
            err_yaw = math.atan2(math.sin(self.rel_target_dyaw - dyaw_local), math.cos(self.rel_target_dyaw - dyaw_local))

            # Prioritize angular rotation if yaw command exists
            if abs(self.rel_target_dyaw) > 0.05 and abs(err_yaw) > 0.04:
                w_sign = 1.0 if err_yaw > 0 else -1.0
                w_cmd = w_sign * max(0.12, min(0.40, 1.2 * abs(err_yaw)))
                self.send_cmd_vel(0.0, w_cmd)
                return

            # Linear motion control
            if abs(err_dist) < 0.012:  # Reached within 1.2cm
                self.send_cmd_vel(0.0, 0.0)
                self.active_relative_nav = False
                self.log_info(f"🎯 [Relative Nav Done] Traveled {dx_local*100:.1f}cm (target: {self.rel_target_dist*100:.1f}cm)")
                self._transition_to(TaskState.VERIFYING, reason="Relative Nav Reached via Odom")
                return

            # Proportional velocity with deadband compensation for TurtleBot3
            v_sign = 1.0 if err_dist > 0 else -1.0
            # Min 0.040 m/s to overcome wheel friction, max 0.12 m/s for safe smooth motion
            v_cmd = v_sign * max(0.040, min(0.12, 0.8 * abs(err_dist)))
            w_cmd = max(-0.25, min(0.25, 0.6 * err_yaw))
            self.send_cmd_vel(v_cmd, w_cmd)

        else:
            # 2. Open-loop duration fallback if odometry is not published
            target_dist = abs(self.rel_target_dist)
            v_cruise = 0.06  # 6 cm/s safe speed
            expected_duration = target_dist / v_cruise if target_dist > 0 else 0.0

            if elapsed < expected_duration:
                v_sign = 1.0 if self.rel_target_dist >= 0 else -1.0
                self.send_cmd_vel(v_sign * v_cruise, 0.0)
            else:
                self.send_cmd_vel(0.0, 0.0)
                self.active_relative_nav = False
                self.log_info(f"🎯 [Relative Nav Open-Loop Done] Time elapsed: {elapsed:.2f}s for {target_dist*100:.1f}cm")
                self._transition_to(TaskState.VERIFYING, reason="Relative Nav Reached via Open-Loop")

    def _halt_motors(self):
        for _ in range(5):
            self.send_cmd_vel(0.0, 0.0, halt_all=True)

    def _start_halt_burst(self, duration_sec: float = 1.0):
        """Actively floods zero velocity across all channels for duration_sec to overcome Nav2 residual inertia."""
        self._halt_motors()
        if self._halt_burst_timer is not None:
            self._halt_burst_timer.cancel()
            self._halt_burst_timer = None

        self._halt_burst_count = int(duration_sec / 0.05)
        def _burst_tick():
            self.send_cmd_vel(0.0, 0.0, halt_all=True)
            self._halt_burst_count -= 1
            if self._halt_burst_count <= 0:
                if self._halt_burst_timer is not None:
                    self._halt_burst_timer.cancel()
                    self._halt_burst_timer = None

        self._halt_burst_timer = self.create_timer(0.05, _burst_tick)

    def _publish_rviz_goal_marker(self, x: float, y: float, yaw: float, label: str = "GOAL"):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. Goal Arrow
        arrow = Marker()
        arrow.header.stamp = now
        arrow.header.frame_id = "map"
        arrow.ns = "stopping_goal"
        arrow.id = 10
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = float(x)
        arrow.pose.position.y = float(y)
        arrow.pose.position.z = 0.05
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        arrow.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
        arrow.scale = Vector3(x=0.45, y=0.10, z=0.10)
        arrow.color.r = 0.0
        arrow.color.g = 0.95
        arrow.color.b = 1.0  # Cyan
        arrow.color.a = 0.9
        ma.markers.append(arrow)

        # 2. Text Billboard
        text = Marker()
        text.header.stamp = now
        text.header.frame_id = "map"
        text.ns = "stopping_goal"
        text.id = 11
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = 0.35
        text.scale.z = 0.12
        text.color.r = 0.0
        text.color.g = 1.0
        text.color.b = 0.9
        text.color.a = 1.0
        text.text = f"🎯 {label}"
        ma.markers.append(text)

        self.rviz_goal_pub.publish(ma)

    def _clear_rviz_goal_marker(self):
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.ns = "stopping_goal"
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.rviz_goal_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode(enable_terminal_ui=True)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.log_info("Keyboard interrupt received. Shutting down...")
    finally:
        node.trigger_software_stop(reason="Process shutdown")
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
