#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[LLM Agent Node v5.0] Qwen2.5-VL Multimodal Unified ROS 2 Agent
- 자연어 음성/텍스트 명령을 파싱하여 5대 자율주행 모드 및 도킹 Action JSON 발행
- Astra S 실시간 카메라 영상 기반 Qwen2.5-VL Visual Grounding (2D BBox 추출 및 시각 검증)
- 0.1ms 이내 Emergency Software STOP 인터셉트
- CommandValidator를 통한 Pydantic 규격 및 물리적 안전 범위 검증
- Task Manager(/task/command) 및 3D 비전 도킹(/qwen/target_bbox) 연동
"""

import os
import sys
import time
import json
import base64
import urllib.request
import urllib.error
import threading
import re
import yaml
from typing import Optional, Dict, Any, Tuple, List

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw as PILImageDraw, ImageFont as PILImageFont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from llm_semantic_docker.command_validator import CommandValidator, RobotIntent, ActionEnum, TargetSpec
from llm_semantic_docker.software_stop_interceptor import SoftwareStopInterceptor


TARGET_CONFIG_PATH = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/floor_target_objects.yaml"


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
    BG_CYAN = "\033[46m"
    BG_RED = "\033[41m"


class LLMAgentNode(Node):
    def __init__(self, enable_cli_input: bool = True):
        super().__init__('llm_agent_node')
        self.enable_cli_input = enable_cli_input
        self.bridge = CvBridge()

        # ----------------------------------------------------------------------
        # Target Registry & YAML Configuration
        # ----------------------------------------------------------------------
        self.target_profiles: Dict[str, Any] = {}
        self.alias_to_canonical: Dict[str, str] = {}
        self.sorted_aliases: List[str] = []
        self._load_target_profiles()

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434/api/generate')
        self.declare_parameter('model_name', 'qwen2.5:3b')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('keep_alive', -1)

        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.keep_alive = self.get_parameter('keep_alive').get_parameter_value().integer_value

        # Validator 초기화
        self.validator = CommandValidator()
        self.current_anchor_cue = "red_object"

        # Frame state
        self.latest_color_img: Optional[np.ndarray] = None
        self.lock = threading.Lock()

        # ----------------------------------------------------------------------
        # ROS 2 Interfaces
        # ----------------------------------------------------------------------
        # Subscriptions
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(String, '/user_voice_command', self._voice_command_cb, 10)
        self.create_subscription(String, '/qwen/detect_query', self._detect_query_cb, 10)
        self.create_subscription(Image, self.color_topic, self._color_img_cb, qos_best_effort)
        self.create_subscription(CompressedImage, f"{self.color_topic}/compressed", self._color_comp_cb, qos_best_effort)

        # Publishers
        self.task_cmd_pub = self.create_publisher(String, '/task/command', 10)
        self.task_seq_pub = self.create_publisher(String, '/task/command_sequence', 10)
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)
        self.stop_pub = self.create_publisher(Bool, '/safety/software_stop', 10)
        self.target_bbox_pub = self.create_publisher(String, '/qwen/target_bbox', 10)
        self.debug_img_pub = self.create_publisher(Image, '/qwen/debug_image', 10)
        self.agent_resp_pub = self.create_publisher(String, '/llm/agent_response', 10)
        self.dock_station_pub = self.create_publisher(String, '/docking/start_station', 10)
        self.dock_start_cli = self.create_client(Trigger, '/docking/start')

        # OpenCV ArUco detector for direct visual docking pre-flight check
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Services
        self.verify_srv = self.create_service(Trigger, '/qwen/verify_target', self._handle_verify_srv)

        self._print_banner()

        # VLM GPU VRAM 사전 워밍업
        self._warmup_vlm()

        # Interactive Console CLI (백그라운드 스레드)
        if self.enable_cli_input:
            self.cli_thread = threading.Thread(target=self._cli_worker, daemon=True)
            self.cli_thread.start()

    def _load_target_profiles(self):
        """Loads floor_target_objects.yaml registry if present"""
        if os.path.exists(TARGET_CONFIG_PATH):
            try:
                with open(TARGET_CONFIG_PATH, "r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f) or {}
                    self.target_profiles = cfg.get("targets", {})
                    self.alias_to_canonical.clear()
                    for c_id, profile in self.target_profiles.items():
                        self.alias_to_canonical[c_id.lower()] = c_id
                        d_name = profile.get("display_name", "")
                        if d_name:
                            self.alias_to_canonical[d_name.lower()] = c_id
                            self.alias_to_canonical[d_name.replace(" ", "").lower()] = c_id
                        for al in profile.get("aliases", []):
                            self.alias_to_canonical[al.lower()] = c_id
                            self.alias_to_canonical[al.replace(" ", "").lower()] = c_id
                    self.sorted_aliases = sorted(self.alias_to_canonical.keys(), key=lambda s: len(s), reverse=True)
                    self.get_logger().info(f"📄 LLM Agent: Target profiles loaded ({len(self.target_profiles)} targets, {len(self.sorted_aliases)} aliases)")
            except Exception as e:
                self.get_logger().warn(f"Failed to load target config: {e}")

    def _print_banner(self):
        banner = f"""
{TermColor.CYAN}╔══════════════════════════════════════════════════════════════════════════╗
║               🧠 QWEN2.5-VL MULTIMODAL AGENT NODE (v5.0)                ║
║  Model: {self.model_name:<16} | Engine: Local Ollama (GPU 100%)         ║
║  Capabilities: 5-Mode Navigation Planning + 3D Visual Grounding Verification ║
╚══════════════════════════════════════════════════════════════════════════╝{TermColor.RESET}
"""
        print(banner)

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------
    def _color_img_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_color_img = cv_img
        except Exception:
            pass

    def _color_comp_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.latest_color_img = cv_img
        except Exception:
            pass

    def _voice_command_cb(self, msg: String):
        user_text = msg.data.strip()
        if user_text:
            self.process_command(user_text)

    def _detect_query_cb(self, msg: String):
        query = msg.data.strip()
        if query:
            self.ground_and_publish_target(query)

    def _handle_verify_srv(self, request, response):
        """Task Manager가 목적지 도착 후 시각 검증 요청 시 호출"""
        cue = self.current_anchor_cue or "box"
        self.get_logger().info(f"[Verify Service] Verifying target visual anchor: '{cue}'...")
        res = self.ground_and_publish_target(cue)
        if res.get("target_found", False):
            response.success = True
            response.message = f"Target '{cue}' verified with BBox: {res.get('bbox_2d')}"
        else:
            response.success = False
            response.message = f"Target '{cue}' NOT found in camera view."
        return response

    # --------------------------------------------------------------------------
    # 1. 자연어 명령 파싱 (5대 주행 모드 + 정밀 도킹)
    # --------------------------------------------------------------------------
    def check_visual_marker(self) -> Tuple[bool, str, Optional[int]]:
        """실시간 카메라 프레임에서 전방 ArUco 마커 감지 여부 사전 검사"""
        with self.lock:
            if self.latest_color_img is None:
                return False, "카메라 영상이 아직 수신되지 않았습니다.", None
            img = self.latest_color_img.copy()

        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None and len(ids) > 0:
                marker_id = int(ids[0][0])
                return True, f"ArUco 마커(ID:{marker_id}) 감지 성공", marker_id
            return False, "카메라 화면에 ArUco 마커가 보이지 않습니다.", None
        except Exception as e:
            return False, f"마커 검출 중 오류: {e}", None

    def _quick_parse_single_clause(self, clause: str) -> Optional[Dict[str, Any]]:
        """Parses a single atomic clause into an Action dictionary."""
        cl = clause.strip().lower()
        if not cl:
            return None

        # 1. Rotation (angle check)
        m_angle = re.search(r'(?<![a-zA-Z0-9_가-힣])([+-]?\d+(?:\.\d+)?)\s*(도|deg|°)?', cl)
        is_left = any(w in cl for w in ["좌회전", "왼쪽", "좌", "반시계", "ccw"])
        is_right = any(w in cl for w in ["우회전", "오른쪽", "우", "시계", "cw"])
        is_rot = any(w in cl for w in ["회전", "돌아", "틀어", "turn", "rotate"])

        if (is_left or is_right or is_rot) and m_angle:
            try:
                deg = float(m_angle.group(1))
                if is_right:
                    deg = -abs(deg)
                else:
                    deg = abs(deg)
                return {
                    "action": "NAVIGATE",
                    "target": {
                        "mode": "RELATIVE",
                        "relative_x": 0.0,
                        "relative_y": 0.0,
                        "relative_yaw": deg
                    }
                }
            except Exception:
                pass

        # 2. Docking (if docking keyword or station key present)
        if "도킹" in cl or "dock" in cl:
            st_key = self.validator.resolve_station_key(cl)
            target_st = st_key or "marker"
            return {
                "action": "DOCK",
                "target": {
                    "target_structure": target_st,
                    "docking_face": "front",
                    "direct_visual": (target_st == "marker")
                }
            }

        # 3. Waypoint & Station Standoff Navigation (Checked before generic distance numbers)
        wp_key = self.validator.resolve_waypoint_key(cl)
        if wp_key:
            return {
                "action": "NAVIGATE",
                "target": {
                    "mode": "WAYPOINT",
                    "semantic_name": wp_key
                }
            }

        st_key = self.validator.resolve_station_key(cl)
        if st_key:
            # If explicit dock keyword absent, navigate to the station's standoff waypoint
            return {
                "action": "NAVIGATE",
                "target": {
                    "mode": "WAYPOINT",
                    "semantic_name": st_key
                }
            }

        # 4. Linear displacement (Distance check with word boundary protection)
        m_dist = re.search(r'(?<![a-zA-Z0-9_가-힣])([+-]?\d+(?:\.\d+)?)\s*(cm|m|미터|센티|센치)?', cl)
        is_fwd = any(w in cl for w in ["앞으로", "전진", "앞", "직진", "가줘", "가", "이동"])
        is_bwd = any(w in cl for w in ["뒤로", "후진", "뒤"])

        if (is_fwd or is_bwd) and m_dist:
            try:
                val = float(m_dist.group(1))
                unit = m_dist.group(2) or "m"
                if unit in ("cm", "센티", "센치"):
                    dist_m = val / 100.0
                else:
                    dist_m = val
                if is_bwd:
                    dist_m = -abs(dist_m)
                else:
                    dist_m = abs(dist_m)
                return {
                    "action": "NAVIGATE",
                    "target": {
                        "mode": "RELATIVE",
                        "relative_x": dist_m,
                        "relative_y": 0.0,
                        "relative_yaw": 0.0
                    }
                }
            except Exception:
                pass

        return None

    def _quick_parse_sequence(self, user_text: str) -> Optional[Dict[str, Any]]:
        """Splits compound natural language instructions into a sequence of atomic actions (0ms)."""
        delimiters = r'하고나서|하고|해서|가서|한\s*뒤에?|한\s*후에?|이동\s*후에?|(?<=\s)후(?!진|방|면)(?:에)?(?:\s+|$)|한\s*다음에?|그\s*다음에?|갔다가|그리고|,\s*|\.\s*'
        clauses = [c.strip() for c in re.split(delimiters, user_text) if c.strip()]
        if len(clauses) < 2:
            return None

        tasks = []
        for c in clauses:
            sub_task = self._quick_parse_single_clause(c)
            if sub_task is None:
                return None
            tasks.append(sub_task)

        if len(tasks) >= 2:
            seq_id = f"seq_{int(time.time()*1000)%1000000}"
            return {
                "is_sequence": True,
                "sequence_id": seq_id,
                "title": user_text,
                "mode": "PREEMPT_AND_REPLACE",
                "failure_policy": "STOP_ON_FAILURE",
                "tasks": tasks,
                "rationale": f"0ms 고속 정규식 복합 시퀀스 분해 ({len(tasks)}개 작업)"
            }
        return None

    def _quick_parse_command(self, user_text: str) -> Optional[Dict[str, Any]]:
        """정형화된 주요 명령을 LLM 대기 없이 0ms에 즉시 확정 파싱 (오류/부호 역전 완전 방지)"""
        # 0. Check multi-step compound instruction first
        seq_res = self._quick_parse_sequence(user_text)
        if seq_res is not None:
            return seq_res

        cleaned = user_text.strip().lower()

        # 1. 마커 직시 도킹 명령
        if any(kw in cleaned for kw in ["마커 보고 도킹", "마커보고 도킹", "마커보고도킹", "마커 도킹", "마커도킹", "지금 마커"]):
            return {
                "action": "DOCK",
                "target": {
                    "target_structure": "marker",
                    "perception_type": "aruco",
                    "direct_visual": True
                },
                "rationale": "마커 직시 도킹",
                "user_text": user_text
            }

        # 2. 바닥 물체 직시 접근 명령 (Custom YOLO / Open-Vocabulary 타겟 매칭)
        # 2.1 등록된 타겟 별칭(Alias) 우선 매칭
        matched_canonical = None
        matched_alias = None
        cleaned_no_space = cleaned.replace(" ", "")

        for al in self.sorted_aliases:
            if al in cleaned or al.replace(" ", "") in cleaned_no_space:
                matched_canonical = self.alias_to_canonical[al]
                matched_alias = al
                break

        if matched_canonical is not None:
            profile = self.target_profiles.get(matched_canonical, {})
            disp_name = profile.get("display_name", matched_canonical)
            default_standoff_m = float(profile.get("standoff_m", 0.25))

            cm_match = re.search(r'([0-9]+)\s*cm', cleaned)
            clearance = float(cm_match.group(1)) if cm_match else (default_standoff_m * 100.0)

            return {
                "action": "DOCK",
                "target": {
                    "target_structure": "floor_object",
                    "perception_type": "floor_object",
                    "direct_visual": True,
                    "clearance_cm": clearance,
                    "target_name": matched_canonical
                },
                "rationale": f"바닥 타겟 직시 접근 ({disp_name} / {matched_canonical}, {clearance:.0f}cm)",
                "user_text": user_text
            }

        # 2.2 일반 바닥 물체 키워드 폴백 매칭
        if any(kw in cleaned for kw in ["바닥 물체", "바닥에 있는 물체", "물체로 다가가", "물체 다가가", "물체 접근", "상자 접근", "큐브 접근"]):
            cm_match = re.search(r'([0-9]+)\s*cm', cleaned)
            clearance = float(cm_match.group(1)) if cm_match else 25.0

            target_query = "small object on the floor"
            if any(k in cleaned for k in ["상자", "박스", "box"]):
                target_query = "box on the floor"
            elif any(k in cleaned for k in ["큐브", "cube"]):
                target_query = "cube on the floor"
            elif any(k in cleaned for k in ["공", "볼", "ball"]):
                target_query = "ball on the floor"

            return {
                "action": "DOCK",
                "target": {
                    "target_structure": "floor_object",
                    "perception_type": "floor_object",
                    "direct_visual": True,
                    "clearance_cm": clearance,
                    "target_name": target_query
                },
                "rationale": f"일반 바닥 물체 직시 접근 ({clearance:.0f}cm)",
                "user_text": user_text
            }

        # 3. 회전 명령 (왼쪽: 양수(+) / 오른쪽: 음수(-))
        is_turn = any(kw in cleaned for kw in ["회전", "돌아", "돌아줘", "좌회전", "우회전", "turn", "rotate"])
        if is_turn:
            angle_match = re.search(r'([0-9]+)\s*(?:도|deg|°)', cleaned)
            angle = float(angle_match.group(1)) if angle_match else 15.0

            is_left = any(kw in cleaned for kw in ["왼쪽", "왼", "좌회전", "좌", "left", "ccw", "반시계"])
            is_right = any(kw in cleaned for kw in ["오른쪽", "오른", "우회전", "우", "right", "cw", "시계"])

            if is_left and not is_right:
                return {
                    "action": "NAVIGATE",
                    "target": {
                        "mode": "RELATIVE",
                        "relative_x": 0.0,
                        "relative_y": 0.0,
                        "relative_yaw": angle
                    },
                    "rationale": f"왼쪽 {angle}° 회전",
                    "user_text": user_text
                }
            elif is_right and not is_left:
                return {
                    "action": "NAVIGATE",
                    "target": {
                        "mode": "RELATIVE",
                        "relative_x": 0.0,
                        "relative_y": 0.0,
                        "relative_yaw": -angle
                    },
                    "rationale": f"오른쪽 {angle}° 회전",
                    "user_text": user_text
                }

        # 4. 상대 직선 이동 명령
        cm_match = re.search(r'([0-9]+)\s*cm', cleaned)
        m_match = re.search(r'([0-9]+(?:\.[0-9]+)?)\s*(?:m|미터)', cleaned)
        dist_m = None
        if cm_match:
            dist_m = float(cm_match.group(1)) / 100.0
        elif m_match:
            dist_m = float(m_match.group(1))

        if dist_m is not None:
            if any(kw in cleaned for kw in ["앞", "전진", "fwd", "forward"]):
                return {
                    "action": "NAVIGATE",
                    "target": {
                        "mode": "RELATIVE",
                        "relative_x": dist_m,
                        "relative_y": 0.0,
                        "relative_yaw": 0.0
                    },
                    "rationale": f"전진 {dist_m*100:.0f}cm",
                    "user_text": user_text
                }
            elif any(kw in cleaned for kw in ["뒤", "후진", "back", "backward"]):
                return {
                    "action": "NAVIGATE",
                    "target": {
                        "mode": "RELATIVE",
                        "relative_x": -dist_m,
                        "relative_y": 0.0,
                        "relative_yaw": 0.0
                    },
                    "rationale": f"후진 {dist_m*100:.0f}cm",
                    "user_text": user_text
                }

        return None

    def process_command(self, user_text: str):
        t0 = time.perf_counter()
        print(f"\n{TermColor.YELLOW}💬 [User Command Received]{TermColor.RESET} \"{user_text}\"")

        # 1. 초고속 Emergency STOP 인터셉트 (<0.1ms)
        is_stop, check_time = SoftwareStopInterceptor.inspect(user_text)
        if is_stop:
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            print(f"{TermColor.BG_RED}{TermColor.WHITE} 🛡️ EMERGENCY STOP INTERCEPTED ({check_time*1000:.2f}ms) {TermColor.RESET}")
            resp_str = "긴급 정지 신호가 감지되어 즉시 모든 모터를 정지했습니다."
            self.agent_resp_pub.publish(String(data=resp_str))
            stop_task = {"action": "STOP", "reason": f"Voice stop keyword '{user_text}'"}
            self.task_cmd_pub.publish(String(data=json.dumps(stop_task)))
            return

        # 1.5 초고속 결정론적 정규식 파서 매칭 (<0.1ms)
        quick_parsed = self._quick_parse_command(user_text)
        if quick_parsed is not None:
            parsed_dict = quick_parsed
            print(f">> 고속 확정 파서 매칭 ({time.perf_counter() - t0:.2f}ms): {parsed_dict.get('rationale')}")
        else:
            # 규칙 기반 파싱 실패 시 명시적 로그 출력
            miss_msg = f"[LLM Agent] 규칙 기반 빠른 파싱 매칭 실패 -> LLM(Qwen) 기반 추론 진행: '{user_text}'"
            self.get_logger().info(miss_msg)
            print(f"{TermColor.YELLOW}>> [Fast Parser Miss] {miss_msg}{TermColor.RESET}")

            # 2. Qwen2.5-VL 프롬프트 작성 (등록된 웨이포인트 포함)
            registered_places = list(self.validator.waypoints.keys())
            system_prompt = f"""You are an autonomous robot command planner.
Analyze the user command and output ONLY valid JSON matching one of these two schemas:

Case 1: Single Action:
{{
  "action": "NAVIGATE" or "DOCK" or "STATUS",
  "target": {{
    "semantic_name": "place name from registered list if navigating to a fixed waypoint",
    "target_structure": "structure to dock with (e.g. desk, box, charger)",
    "anchor_cue": "visual object on/near target to look for (e.g. red_object, box, pen, tumbler, or null)",
    "docking_face": "front",
    "mode": "WAYPOINT" or "RELATIVE" or "ABSOLUTE" or "LANDMARK_OFFSET",
    "coordinates": [x, y, yaw_deg] (for ABSOLUTE),
    "relative_x": float in meters (for RELATIVE, forward+),
    "relative_y": float in meters (for RELATIVE, left+),
    "relative_yaw": float in degrees (for RELATIVE),
    "offset_from": "base place name" (for LANDMARK_OFFSET),
    "offset_x": float in meters (for LANDMARK_OFFSET),
    "offset_y": float in meters (for LANDMARK_OFFSET)
  }},
  "rationale": "short explanation"
}}

Case 2: Multiple / Sequential Actions (e.g. "~갔다가 ~해", "~이동 후 ~도킹해"):
{{
  "is_sequence": true,
  "title": "short title of sequence",
  "tasks": [
    {{"action": "NAVIGATE", "target": {{"semantic_name": "place1", "mode": "WAYPOINT"}}}},
    {{"action": "NAVIGATE", "target": {{"semantic_name": "place2", "mode": "WAYPOINT"}}}}
  ],
  "rationale": "short explanation"
}}

Registered places: {registered_places}

Examples:
- "앞으로 1미터 이동해": {{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": 1.0, "relative_y": 0.0, "relative_yaw": 0.0}}}}
- "move toward 20cm": {{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": 0.20, "relative_y": 0.0, "relative_yaw": 0.0}}}}
- "뒤로 10cm 후진": {{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": -0.10, "relative_y": 0.0, "relative_yaw": 0.0}}}}
- "좌회전 15도": {{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": 0.0, "relative_y": 0.0, "relative_yaw": 15.0}}}}
- "오른쪽으로 90도 회전해": {{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": 0.0, "relative_y": 0.0, "relative_yaw": -90.0}}}}
- "마커 보고 도킹해": {{"action": "DOCK", "target": {{"target_structure": "marker", "perception_type": "aruco", "direct_visual": true}}}}
- "workplace1로 도킹해": {{"action": "DOCK", "target": {{"target_structure": "workplace1", "docking_face": "front"}}}}
- "책상으로 가줘": {{"action": "NAVIGATE", "target": {{"semantic_name": "desk", "mode": "WAYPOINT"}}}}
- "waypoint1로 갔다가 workplace1로 가": {{"is_sequence": true, "title": "웨이포인트1 경유 워크플레이스1 이동", "tasks": [{{"action": "NAVIGATE", "target": {{"semantic_name": "wp1", "mode": "WAYPOINT"}}}}, {{"action": "NAVIGATE", "target": {{"semantic_name": "workplace1", "mode": "WAYPOINT"}}}}]}}
- "workplace1로 가서 마커 보고 도킹해": {{"is_sequence": true, "title": "워크플레이스1 이동 후 마커 도킹", "tasks": [{{"action": "NAVIGATE", "target": {{"semantic_name": "workplace1", "mode": "WAYPOINT"}}}}, {{"action": "DOCK", "target": {{"target_structure": "marker", "perception_type": "aruco", "direct_visual": true}}}}]}}
- "앞으로 50cm 갔다가 우회전 90도 해줘": {{"is_sequence": true, "title": "직진 50cm 후 우회전 90도", "tasks": [{{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": 0.5, "relative_y": 0.0, "relative_yaw": 0.0}}}}, {{"action": "NAVIGATE", "target": {{"mode": "RELATIVE", "relative_x": 0.0, "relative_y": 0.0, "relative_yaw": -90.0}}}}]}}

Important Rules:
1. When parsing place or station names with numbers (e.g. workplace1), ALWAYS keep the number with the name. Note that 'waypoint1' or '웨이포인트1' corresponds to 'wp1'.
2. Note for rotation: relative_yaw is in degrees (Left/CCW/좌회전/왼쪽: positive +, Right/CW/우회전/오른쪽: negative -).
3. If the user command specifies multiple steps connected by sequential phrases (e.g. '~갔다가', '~하고 나서', '~한 뒤', 'first ... then ...'), ALWAYS output the Case 2 sequential schema with "is_sequence": true.

User Command: "{user_text}"
JSON:"""

            payload = {
                "model": self.model_name,
                "prompt": system_prompt,
                "stream": False,
                "format": "json",
                "options": {"temperature": 0.1, "num_predict": 256},
                "keep_alive": self.keep_alive
            }

            try:
                req = urllib.request.Request(
                    self.ollama_url,
                    data=json.dumps(payload).encode('utf-8'),
                    headers={"Content-Type": "application/json"}
                )
                with urllib.request.urlopen(req, timeout=10) as resp:
                    body = json.loads(resp.read().decode('utf-8'))
                    raw_json = body.get("response", "{}")
                    parsed_dict = json.loads(raw_json)

                elapsed = time.perf_counter() - t0
                summary_action = "SEQUENCE" if parsed_dict.get("is_sequence") else parsed_dict.get('action')
                print(f">> Qwen2.5-VL 추론 완료 ({elapsed*1000:.1f}ms): {summary_action}")
            except Exception as e:
                self.get_logger().error(f"Ollama inference error: {e}")
                resp_str = f"명령 분석 중 오류가 발생했습니다: {e}"
                self.agent_resp_pub.publish(String(data=resp_str))
                return

        parsed_dict["user_text"] = user_text

        try:
            # 3. CommandValidator를 통한 Pydantic 검증
            val_result = self.validator.validate(parsed_dict)

            if val_result.is_valid:
                resolved_task = val_result.resolved_task
                print(f"{TermColor.GREEN}✅ 검증 성공 (VALID): {resolved_task.get('action')}{TermColor.RESET}")

                # 도킹 타겟 앵커 기억
                target_dict = parsed_dict.get("target", {})
                target_cue = target_dict.get("anchor_cue") or target_dict.get("target_structure") or "상자"
                if target_dict.get("anchor_cue"):
                    self.current_anchor_cue = target_dict["anchor_cue"]

                # DOCK 액션 처리
                if resolved_task.get("action") == "DOCK":
                    is_direct = resolved_task.get("direct_visual", False)
                    p_type = resolved_task.get("perception_type", "aruco")

                    # 직시 마커 도킹 시: 사전 카메라 프레임 마커 검사 필수 수행
                    if is_direct and p_type == "aruco":
                        has_marker, marker_info, marker_id = self.check_visual_marker()
                        if not has_marker:
                            warn_msg = "⚠️ [도킹 불가] 현재 카메라 화면에 마커가 보이지 않습니다. 로봇 전방에 마커가 보이도록 위치를 조정해주세요."
                            print(f"{TermColor.YELLOW}{warn_msg}{TermColor.RESET}")
                            self.agent_resp_pub.publish(String(data=warn_msg))
                            self.task_status_pub.publish(String(data=f"ALERT: {warn_msg}"))
                            return
                        else:
                            print(f"{TermColor.GREEN}🎯 [Direct Visual Docking] 전방 마커(ID:{marker_id}) 감지 완료! 정밀 도킹 엔진으로 직접 인계합니다.{TermColor.RESET}")
                            resp_str = f"🎯 [마커 확인] 전방 마커(ID:{marker_id})를 확인했습니다. 정밀 도킹 작업을 시작합니다."
                            self.agent_resp_pub.publish(String(data=resp_str))

                            # 정밀 도킹 제어기 직접 가동 트리거 (중간 FSM/YAML 지연 없이 즉시 진입)
                            if self.dock_start_cli.service_is_ready():
                                self.dock_start_cli.call_async(Trigger.Request())
                            self.dock_station_pub.publish(String(data="marker"))
                    elif is_direct and p_type == "floor_object":
                        # 바닥 물체 직시 접근 알림
                        t_name = resolved_task.get("target_name", "물체")
                        profile = self.target_profiles.get(t_name, {})
                        disp_name = profile.get("display_name", t_name)
                        cl_cm = float(resolved_task.get("clearance_cm", 25.0))
                        resp_str = f"🎯 [바닥 물체 확인] 대상 '{disp_name}' 시각 검출 및 {cl_cm:.0f}cm 정밀 접근 모드를 시작합니다."
                        self.agent_resp_pub.publish(String(data=resp_str))
                    elif resolved_task.get("station_name"):
                        st_name = resolved_task.get("station_name")
                        print(f"\n{TermColor.GREEN}🎯 [Direct Station Docking] 등록된 스테이션 '{st_name}'으로 직접 좌표 도킹을 수행합니다. (VLM 시각 탐색 스킵){TermColor.RESET}")
                    else:
                        if "상자" in user_text:
                            target_cue = "상자"
                        elif "책상" in user_text:
                            target_cue = "책상"
                        elif "충전" in user_text:
                            target_cue = "충전기"

                        print(f"\n{TermColor.CYAN}🎯 [Docking Pre-Grounding] 도킹 대상 '{target_cue}' 실시간 VLM 시각 탐색 수행...{TermColor.RESET}")
                        ground_res = self.ground_and_publish_target(target_cue)
                        if ground_res.get("target_found", False):
                            print(f"{TermColor.GREEN}✅ 도킹 대상 '{target_cue}' VLM 락온 완료 (BBox: {ground_res.get('pixel_box')}){TermColor.RESET}")
                        else:
                            print(f"{TermColor.YELLOW}⚠️ 도킹 대상 '{target_cue}' VLM 미감지 (3D Depth Fallback 연계 대기){TermColor.RESET}")
                        time.sleep(0.15)

                # Multi-Action Sequence인 경우 /task/command_sequence로 우선 발행
                if resolved_task.get("is_sequence"):
                    task_str = json.dumps(resolved_task, ensure_ascii=False)
                    self.task_seq_pub.publish(String(data=task_str))
                    task_cnt = len(resolved_task.get("tasks", []))
                    resp_str = f"연속 복합 명령({task_cnt}개 작업)을 접수했습니다: [{resolved_task.get('title')}] 순차 실행을 시작합니다."
                    print(f"\n{TermColor.CYAN}🚀 [Multi-Task Sequence Dispatched] {resp_str}{TermColor.RESET}")
                    self.agent_resp_pub.publish(String(data=resp_str))
                    return

                # Task Manager로 단일 태스크 발행
                task_str = json.dumps(resolved_task, ensure_ascii=False)
                self.task_cmd_pub.publish(String(data=task_str))

                if not (resolved_task.get("action") == "DOCK" and resolved_task.get("direct_visual", False)):
                    resp_str = f"명령을 확인했습니다: [{resolved_task.get('action')}] 작업을 시작합니다."
                    self.agent_resp_pub.publish(String(data=resp_str))
            else:
                print(f"{TermColor.RED}❌ 유효성 검증 실패: {val_result.error_msg}{TermColor.RESET}")
                resp_str = f"명령을 수행할 수 없습니다: {val_result.error_msg}"
                self.agent_resp_pub.publish(String(data=resp_str))

        except Exception as e:
            self.get_logger().error(f"Command processing error: {e}")
            resp_str = f"명령 분석 중 오류가 발생했습니다: {e}"
            self.agent_resp_pub.publish(String(data=resp_str))

    # --------------------------------------------------------------------------
    # 2. 실시간 시각 검증 및 Visual Grounding (2D BBox 추출)
    # --------------------------------------------------------------------------
    def ground_and_publish_target(self, target_query: str) -> Dict[str, Any]:
        with self.lock:
            if self.latest_color_img is None:
                print(f"{TermColor.YELLOW}⚠️ 카메라 영상이 아직 수신되지 않았습니다. (/camera/color/image_raw 확인 필요){TermColor.RESET}", flush=True)
                self.get_logger().warn("No camera frame available for visual grounding!")
                return {"target_found": False, "error": "No camera frame"}
            img_bgr = self.latest_color_img.copy()

        h, w = img_bgr.shape[:2]
        success, buffer = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            return {"target_found": False, "error": "Image encoding failed"}
        b64_img = base64.b64encode(buffer).decode('utf-8')

        prompt = f"""Locate the foreground target object '{target_query}' resting on the floor in the lower half of the image.
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
            "model": self.model_name,
            "prompt": prompt,
            "images": [b64_img],
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1, "num_predict": 64},
            "keep_alive": self.keep_alive
        }

        t0 = time.perf_counter()
        try:
            req = urllib.request.Request(
                self.ollama_url,
                data=json.dumps(payload).encode('utf-8'),
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                body = json.loads(resp.read().decode('utf-8'))
                raw_json = body.get("response", "{}")
                result = json.loads(raw_json)

            elapsed = time.perf_counter() - t0
            result["latency_sec"] = elapsed

            # 2D BBox 좌표를 max(w, h) 비율 보정하여 실제 픽셀 좌표로 환산
            if result.get("target_found", False) and result.get("bbox_2d"):
                xmin_n, ymin_n, xmax_n, ymax_n = result["bbox_2d"]
                scale = max(w, h) / 1000.0
                xmin = max(0, min(w - 1, int(xmin_n * scale)))
                ymin = max(0, min(h - 1, int(ymin_n * scale)))
                xmax = max(0, min(w, int(xmax_n * scale)))
                ymax = max(0, min(h, int(ymax_n * scale)))
                if xmin > xmax: xmin, xmax = xmax, xmin
                if ymin > ymax: ymin, ymax = ymax, ymin

                result["pixel_box"] = [xmin, ymin, xmax, ymax]
                print(f">> 👁️ Target Found: '{target_query}' at [{xmin}, {ymin}, {xmax}, {ymax}] ({elapsed*1000:.1f}ms)")

                # 디버그 시각화 영상 생성 및 발행
                vis_img = img_bgr.copy()
                cv2.rectangle(vis_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.circle(vis_img, (int((xmin+xmax)/2), int((ymin+ymax)/2)), 4, (0, 0, 255), -1)
                vis_img = self._draw_korean_text(vis_img, f"{result.get('label', target_query)}", (xmin, max(5, ymin - 15)))

                try:
                    img_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
                    self.debug_img_pub.publish(img_msg)
                except Exception:
                    pass
            else:
                print(f">> 👁️ Target NOT Found: '{target_query}' ({elapsed*1000:.1f}ms)")

            # /qwen/target_bbox 토픽으로 전송
            self.target_bbox_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
            return result

        except Exception as e:
            self.get_logger().error(f"Visual grounding error: {e}")
            err_dict = {"target_found": False, "error": str(e)}
            self.target_bbox_pub.publish(String(data=json.dumps(err_dict)))
            return err_dict

    def _draw_korean_text(self, cv_img, text, position, text_color=(0, 255, 0), font_size=16):
        font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
        if os.path.exists(font_path):
            try:
                img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                pil_img = PILImage.fromarray(img_rgb)
                draw = PILImageDraw.Draw(pil_img)
                font = PILImageFont.truetype(font_path, font_size)
                x, y = position
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    draw.text((x + dx, y + dy), text, font=font, fill=(0, 0, 0))
                draw.text((x, y), text, font=font, fill=(text_color[2], text_color[1], text_color[0]))
                return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
            except Exception:
                pass
        cv2.putText(cv_img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
        return cv_img

    def _warmup_vlm(self):
        """시작 시 모델을 GPU VRAM에 상주시킴"""
        print(">> Qwen2.5-VL GPU VRAM 사전 워밍업 중...")
        try:
            payload = {
                "model": self.model_name,
                "prompt": "warmup",
                "stream": False,
                "keep_alive": self.keep_alive
            }
            req = urllib.request.Request(
                self.ollama_url,
                data=json.dumps(payload).encode('utf-8'),
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                pass
            print(f"   {TermColor.GREEN}✅ VLM GPU VRAM 적재 완료 (즉시 추론 준비 완료){TermColor.RESET}")
        except Exception as e:
            print(f"   ⚠️ 워밍업 경고: {e}")

    # --------------------------------------------------------------------------
    # Interactive CLI Input Worker
    # --------------------------------------------------------------------------
    def _cli_worker(self):
        time.sleep(1.0)
        print(f"\n{TermColor.WHITE}────────────────────────────────────────────────────────────────────────{TermColor.RESET}", flush=True)
        print(" ⌨️  [인터랙티브 콘솔 활성화] 터미널에 명령어를 입력할 수 있습니다.", flush=True)
        print("  - 주행 예시: '책상으로 가줘', '앞으로 1m 이동해', '좌표 X 1.5, Y 0.5로 가'", flush=True)
        print("  - 도킹 예시: '빨간 상자가 있는 책상에 도킹해', '상자에 도킹해줘'", flush=True)
        print("  - 시각 질의: '? 텀블러', '? 펜', '? 상자'", flush=True)
        print("  - 비상 정지: '정지', '멈춰', 'stop'", flush=True)
        print(f"{TermColor.WHITE}────────────────────────────────────────────────────────────────────────{TermColor.RESET}\n", flush=True)

        tty_fp = None
        try:
            # Under ros2 launch, sys.stdin is a pipe. Connect directly to /dev/tty if available
            if not sys.stdin.isatty():
                tty_fp = open('/dev/tty', 'r', encoding='utf-8', errors='replace')
                in_stream = tty_fp
            else:
                in_stream = sys.stdin
        except Exception:
            in_stream = sys.stdin

        while rclpy.ok():
            try:
                print(f"{TermColor.CYAN}🤖 명령 입력 >> {TermColor.RESET}", end="", flush=True)
                line = in_stream.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                line = line.strip()
                if not line:
                    continue
                if line.startswith("?"):
                    # 시각 검색 질의
                    query = line[1:].strip()
                    if query:
                        self.ground_and_publish_target(query)
                else:
                    self.process_command(line)
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"CLI error: {e}", flush=True)

        if tty_fp:
            try:
                tty_fp.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode(enable_cli_input=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

