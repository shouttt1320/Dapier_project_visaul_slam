#!/usr/bin/env python3
"""
[Command Validator v3.3] Pydantic-based Schema, Dynamic Waypoints & Docking Profile Validator
- LLM이 출력한 JSON 규격(Pydantic Schema)을 엄격하게 검증
- 역할 분리 반영:
  * target_structure (도킹 대상 구조체: desk, box, charger)
  * anchor_cue (위치 기준 앵커: red_object, blue_cup, None)
  * docking_face (도킹 기준면: front, side, auto)
- 동적 웨이포인트 생성 지원:
  * 1) 절대 좌표: coordinates = [x, y, yaw]
  * 2) 로봇 기준 상대 이동: relative_x, relative_y
  * 3) 랜드마크 기준 상대 오프셋: offset_from, offset_x, offset_y
- 미등록 장소, 불가능한 명령("화성으로 가")은 100% 사전 차단
"""

import os
import math
import yaml
from enum import Enum
from typing import Optional, Dict, Any, List, Union
from pydantic import BaseModel, Field, ValidationError, model_validator


class ActionEnum(str, Enum):
    DOCK = "DOCK"
    NAVIGATE = "NAVIGATE"
    STOP = "STOP"
    STATUS = "STATUS"


class TargetSpec(BaseModel):
    # 1. 시맨틱 이름 및 구조물 (하위 호환 및 통합)
    semantic_name: Optional[str] = Field(default=None, description="Target location or structure name (e.g., desk, box)")
    target_structure: Optional[str] = Field(default=None, description="Physical docking structure (e.g., desk, box, shelf)")
    
    # 2. 시각적 앵커 및 도킹 기준면
    feature_cue: Optional[str] = Field(default=None, description="Legacy visual cue name")
    anchor_cue: Optional[str] = Field(default=None, description="Visual grounding anchor cue (e.g., red_object, blue_cup, None)")
    docking_face: Optional[str] = Field(default="front", description="Docking reference face (front, side, top, auto)")
    clearance_cm: Optional[float] = Field(default=5.0, description="Final clearance distance in cm")

    # 3. 주행 모드 및 동적 좌표 생성 필드
    mode: Optional[str] = Field(default="WAYPOINT", description="WAYPOINT, RELATIVE, ABSOLUTE, or LANDMARK_OFFSET")
    coordinates: Optional[List[float]] = Field(default=None, description="Direct absolute coordinates [x, y] or [x, y, yaw_deg]")
    relative_x: Optional[float] = Field(default=None, description="Relative X offset in meters from robot")
    relative_y: Optional[float] = Field(default=None, description="Relative Y offset in meters from robot")
    relative_yaw: Optional[float] = Field(default=None, description="Relative Yaw rotation in degrees")
    offset_from: Optional[str] = Field(default=None, description="Base landmark name for offset calculation")
    offset_x: Optional[float] = Field(default=0.0, description="Offset X in meters from base landmark")
    offset_y: Optional[float] = Field(default=0.0, description="Offset Y in meters from base landmark")

    # 4. 직시 도킹 / 바닥 물체 접근 지원 필드
    direct_visual: Optional[bool] = Field(default=False, description="Direct visual approach/docking bypassing Nav2 staging")
    perception_type: Optional[str] = Field(default=None, description="Perception backend (aruco, floor_object, etc.)")
    target_name: Optional[str] = Field(default=None, description="Specific target name or canonical ID (e.g., red_cube, blue_ball)")

    @model_validator(mode='after')
    def sync_legacy_fields(self):
        # target_structure <-> semantic_name 동기화
        if not self.target_structure and self.semantic_name:
            self.target_structure = self.semantic_name
        elif not self.semantic_name and self.target_structure:
            self.semantic_name = self.target_structure

        # anchor_cue <-> feature_cue 동기화
        if not self.anchor_cue and self.feature_cue:
            self.anchor_cue = self.feature_cue
        elif not self.feature_cue and self.anchor_cue:
            self.feature_cue = self.anchor_cue

        # 'none' 문자열 처리
        if isinstance(self.anchor_cue, str) and self.anchor_cue.strip().lower() in ["none", "null", "false", ""]:
            self.anchor_cue = None

        return self


class RobotIntent(BaseModel):
    action: ActionEnum
    target: Optional[TargetSpec] = None
    navigation: Optional[Dict[str, Any]] = None
    docking: Optional[Dict[str, Any]] = None
    rationale: Optional[str] = None


class ValidationResult:
    def __init__(self, is_valid: bool, action: str = "UNKNOWN", error_code: str = "",
                 error_msg: str = "", resolved_task: Optional[Dict[str, Any]] = None):
        self.is_valid = is_valid
        self.action = action
        self.error_code = error_code
        self.error_msg = error_msg
        self.resolved_task = resolved_task or {}

    def to_dict(self) -> Dict[str, Any]:
        return {
            "is_valid": self.is_valid,
            "action": self.action,
            "error_code": self.error_code,
            "error_msg": self.error_msg,
            "resolved_task": self.resolved_task
        }

    def __repr__(self):
        return f"<ValidationResult valid={self.is_valid} action={self.action} err={self.error_code}>"


def _locate_config_file(filename: str) -> str:
    # 1순위: 항상 개발 소스 디렉토리의 마스터 파일 (Single Source of Truth)
    src_path = f"/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/{filename}"
    if os.path.exists(src_path):
        return os.path.abspath(src_path)

    # 2순위: ROS 2 ament share 디렉토리
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('llm_semantic_docker')
        share_path = os.path.join(share_dir, "config", filename)
        if os.path.exists(share_path):
            return os.path.abspath(share_path)
    except Exception:
        pass

    this_dir = os.path.dirname(os.path.abspath(__file__))
    for c in [
        os.path.join(this_dir, "..", "config", filename),
        os.path.join(this_dir, "..", "..", "config", filename),
        os.path.join(this_dir, "config", filename),
    ]:
        if os.path.exists(c):
            return os.path.abspath(c)
    return src_path


class CommandValidator:
    def __init__(self, waypoints_file: Optional[str] = None, profiles_file: Optional[str] = None,
                 colors_file: Optional[str] = None):
        self.waypoints_path = waypoints_file or _locate_config_file("semantic_waypoints.yaml")
        self.profiles_path = profiles_file or _locate_config_file("docking_profiles.yaml")
        self.colors_path = colors_file or _locate_config_file("color_profiles.yaml")

        self.waypoints: Dict[str, Any] = {}
        self.profiles: Dict[str, Any] = {}
        self.colors: Dict[str, Any] = {}
        self.alias_map: Dict[str, str] = {}

        self.reload_configs()

    def reload_configs(self):
        # 1. Semantic Waypoints 로드
        if os.path.exists(self.waypoints_path):
            with open(self.waypoints_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                self.waypoints = data.get("waypoints", {})

        # Fallback waypoints
        if not self.waypoints:
            self.waypoints = {
                "box_staging": {
                    "description": "1.0m Staging approach pose in front of ArUco marker box",
                    "x": 1.25, "y": 0.45, "z": 0.0, "yaw_deg": 0.0,
                    "docking_profile": "marker_box",
                    "aliases": ["box", "상자", "택배상자", "박스", "마커상자"]
                },
                "desk_staging": {
                    "description": "1.0m Staging approach pose in front of office desk",
                    "x": 2.80, "y": -0.90, "z": 0.0, "yaw_deg": 90.0,
                    "docking_profile": "desk_red_object",
                    "aliases": ["desk", "책상", "테이블", "업무책상", "빨간물체책상", "연구실책상"]
                },
                "charger_staging": {
                    "description": "1.0m Staging approach pose in front of charging dock",
                    "x": 0.50, "y": 0.20, "z": 0.0, "yaw_deg": 180.0,
                    "docking_profile": "charger_station",
                    "aliases": ["charger", "충전기", "충전소", "충전스테이션"]
                }
            }

        # 2. Docking Profiles 로드
        if os.path.exists(self.profiles_path):
            with open(self.profiles_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                self.profiles = data.get("docking_profiles", {})

        # 3. Docking Stations 로드 (interactive_docking_teacher로 티칭된 3D 점군 ICP 스테이션)
        self.stations_path = _locate_config_file("docking_stations.yaml")
        self.docking_stations: Dict[str, Any] = {}
        if os.path.exists(self.stations_path):
            try:
                with open(self.stations_path, 'r', encoding='utf-8') as f:
                    st_data = yaml.safe_load(f) or {}
                    self.docking_stations = st_data.get("stations", {})
                    for st_name, st_info in self.docking_stations.items():
                        standoff = st_info.get("standoff", {})
                        # Standoff 좌표를 고정 웨이포인트로 등록
                        base_stem = st_name.rstrip("0123456789_ ")
                        st_aliases = [
                            st_name,
                            st_name.replace("_", " "),
                            st_name.replace("_", ""),
                            base_stem,
                            base_stem.replace("_", " "),
                            "작업대",
                            "작업대1",
                            "워크플레이스",
                            "워크플레이스1"
                        ]
                        self.waypoints[st_name] = {
                            "description": st_info.get("description", f"Taught 3D Docking Station: {st_name}"),
                            "x": float(standoff.get("x", 0.0)),
                            "y": float(standoff.get("y", 0.0)),
                            "z": 0.0,
                            "yaw_deg": float(standoff.get("yaw_deg", 0.0)),
                            "docking_profile": "station_icp",
                            "station_name": st_name,
                            "aliases": list(set([a.lower() for a in st_aliases if a]))
                        }
            except Exception as e:
                print(f"[WARN] Failed to load docking_stations.yaml: {e}")

        # station_icp 프로파일 기본 등록
        if "station_icp" not in self.profiles:
            self.profiles["station_icp"] = {
                "description": "3D Point Cloud Template ICP Matching Precision Docking",
                "perception_type": "pointcloud_icp",
                "target_structure": "station",
                "default_face": "front",
                "clearance_reference": "front_bumper",
                "final_clearance": 0.05,
                "coarse_speed": 0.035,
                "fine_speed": 0.02,
                "final_crawl_speed": 0.01,
                "timeout_sec": 90.0
            }

        if not self.profiles:
            self.profiles = {
                "marker_box": {
                    "description": "ArUco Marker-based Box Docking (Front Astra S solvePnP + 4-Corner Depth)",
                    "perception_type": "aruco", "marker_id": 2, "support_surface": "box",
                    "target_structure": "box", "anchor_cue": "marker", "default_face": "front",
                    "staging_waypoint": "box_staging", "clearance_reference": "front_bumper",
                    "final_clearance": 0.05, "coarse_speed": 0.04, "fine_speed": 0.02,
                    "final_crawl_speed": 0.01, "visual_lock_distance": 0.38, "contact_mode": "bumper",
                    "timeout_sec": 75.0
                },
                "desk_red_object": {
                    "description": "2-Stage Anchor-Guided Surface Docking (HSV Red Anchor X + Desk Plane Normal)",
                    "perception_type": "anchor_guided_plane", "primary_detector": "yolo_world",
                    "fallback_detector": "hsv_red", "target_structure": "desk",
                    "anchor_cue": "red_object", "default_face": "front",
                    "staging_waypoint": "desk_staging", "clearance_reference": "front_bumper",
                    "final_clearance": 0.05, "coarse_speed": 0.035, "fine_speed": 0.02,
                    "final_crawl_speed": 0.01, "visual_lock_distance": 0.35, "contact_mode": "bumper",
                    "timeout_sec": 75.0
                },
                "desk_direct": {
                    "description": "Direct Dominant Desk Plane Docking (No Anchor: RANSAC Vertical/Horizontal Plane)",
                    "perception_type": "dominant_plane", "anchor_detector": "none",
                    "target_structure": "desk", "anchor_cue": None, "default_face": "front",
                    "staging_waypoint": "desk_staging", "clearance_reference": "front_bumper",
                    "final_clearance": 0.05, "coarse_speed": 0.035, "fine_speed": 0.02,
                    "final_crawl_speed": 0.01, "visual_lock_distance": 0.35, "contact_mode": "bumper",
                    "timeout_sec": 75.0
                },
                "charger_station": {
                    "description": "Charging Station Docking with Rear Backing",
                    "perception_type": "rear_ir_charger", "target_structure": "charger",
                    "default_face": "rear", "staging_waypoint": "charger_staging",
                    "clearance_reference": "rear_dock_contact", "final_clearance": 0.01,
                    "coarse_speed": 0.03, "fine_speed": 0.015, "final_crawl_speed": 0.008,
                    "contact_mode": "charging_pins", "timeout_sec": 60.0
                }
            }

        # 3. Color Profiles 로드
        if os.path.exists(self.colors_path):
            with open(self.colors_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                self.colors = data.get("colors", {})
        else:
            self.colors = {}

        # 4. Alias 매핑 구축 (한글 및 동의어 색인)
        self.alias_map.clear()
        for wp_key, wp_val in self.waypoints.items():
            self.alias_map[wp_key.lower()] = wp_key
            for alias in wp_val.get("aliases", []):
                self.alias_map[alias.lower()] = wp_key

    def resolve_waypoint_key(self, raw_name: Optional[str]) -> Optional[str]:
        if not raw_name:
            return None
        cleaned = raw_name.strip().lower()
        cleaned_under = cleaned.replace(" ", "_")

        # 1. Direct exact match in alias_map
        if cleaned in self.alias_map:
            return self.alias_map[cleaned]
        if cleaned_under in self.alias_map:
            return self.alias_map[cleaned_under]

        # 2. Check whitespace/underscore-stripped exact match (e.g., "workplace 1" -> "workplace1")
        clean_stripped = cleaned.replace(" ", "").replace("_", "")
        for alias, key in self.alias_map.items():
            if clean_stripped == alias.replace(" ", "").replace("_", ""):
                return key

        # 3. Exact full-word boundary match only (never allow "box" to match "box_01" or "box_02")
        import re
        for alias, key in self.alias_map.items():
            pattern = r'(?<![a-zA-Z0-9_])' + re.escape(alias) + r'(?![a-zA-Z0-9_])'
            if re.search(pattern, cleaned, re.IGNORECASE):
                return key

        return None

    def resolve_station_key(self, raw_target: Optional[str]) -> Optional[str]:
        """Resolves target string to a taught docking station key with robust Korean particle stripping & stem matching."""
        if not raw_target:
            return None
        cleaned = raw_target.strip().lower()

        # 1. Direct match in docking_stations
        for st_name in self.docking_stations.keys():
            if cleaned == st_name.lower() or cleaned.replace(" ", "_") == st_name.lower():
                return st_name

        # 2. Check via resolve_waypoint_key
        wp_key = self.resolve_waypoint_key(cleaned)
        if wp_key and (wp_key in self.docking_stations or (wp_key in self.waypoints and self.waypoints[wp_key].get("docking_profile") == "station_icp")):
            return self.waypoints.get(wp_key, {}).get("station_name", wp_key)

        # 3. Strip Korean particles ('로', '으로', '에', '에서', '쪽으로')
        korean_particles = ["쪽으로", "으로", "에서", "에게", "로", "에"]
        for p in korean_particles:
            if cleaned.endswith(p) and len(cleaned) > len(p):
                stripped = cleaned[:-len(p)].strip()
                res = self.resolve_station_key(stripped)
                if res:
                    return res

        # 4. If single taught station exists, match generic terms or prefix/stem
        if len(self.docking_stations) == 1:
            only_st = list(self.docking_stations.keys())[0]
            generic_keywords = [
                "상자", "박스", "스테이션", "station", "dock", "도킹", "작업대", "workplace", "워크플레이스"
            ]
            if any(k in cleaned for k in generic_keywords):
                return only_st
            # Prefix check (e.g., 'workplace' matches 'workplace1')
            st_stem = only_st.rstrip("0123456789_ ").lower()
            if st_stem and (st_stem in cleaned or cleaned in st_stem):
                return only_st

        # 5. Fuzzy / prefix match among multiple stations
        for st_name in self.docking_stations.keys():
            st_clean = st_name.lower()
            st_stem = st_name.rstrip("0123456789_ ").lower()
            if cleaned == st_stem or (len(cleaned) >= 4 and cleaned in st_clean):
                return st_name

        return None

    def resolve_color_preset(self, cue_text: Optional[str]) -> Optional[str]:
        """anchor_cue 문자열에서 색상 프로파일 키워드(red, blue, yellow 등) 검출"""
        if not cue_text:
            return None
        cleaned = cue_text.strip().lower()
        for color_key, color_info in self.colors.items():
            if color_key in cleaned:
                return color_key
            for alias in color_info.get("aliases", []):
                if alias.lower() in cleaned:
                    return color_key
        return None

    def validate(self, raw_data: Any) -> ValidationResult:
        """Pydantic 스키마 검증 및 Semantic DB 대조, 동적 좌표 생성"""
        # 1. Pydantic 스키마 유효성 검사
        try:
            if isinstance(raw_data, str):
                import json
                parsed_dict = json.loads(raw_data)
                intent = RobotIntent.model_validate(parsed_dict)
            elif isinstance(raw_data, dict):
                parsed_dict = raw_data
                intent = RobotIntent.model_validate(raw_data)
            elif isinstance(raw_data, RobotIntent):
                intent = raw_data
                parsed_dict = intent.model_dump()
            else:
                return ValidationResult(False, error_code="INVALID_FORMAT", error_msg="데이터 형식이 dict 또는 JSON 문자열이 아닙니다.")
        except ValidationError as e:
            return ValidationResult(False, error_code="SCHEMA_VIOLATION", error_msg=f"Pydantic 스키마 검증 실패: {e.errors()}")
        except Exception as e:
            return ValidationResult(False, error_code="PARSE_ERROR", error_msg=f"JSON 파싱 실패: {str(e)}")

        action = intent.action.value

        # 2. Action: STOP
        if action == ActionEnum.STOP.value:
            return ValidationResult(
                is_valid=True,
                action=action,
                resolved_task={"action": "STOP", "reason": "user_command"}
            )

        target = intent.target
        if not target:
            return ValidationResult(
                is_valid=False,
                action=action,
                error_code="MISSING_TARGET",
                error_msg=f"Action '{action}' 수행을 위한 target 정보가 누락되었습니다."
            )

        # 3. Action: NAVIGATE (동적 좌표 생성 포함)
        if action == ActionEnum.NAVIGATE.value:
            # 3.1 로봇 기준 상대 좌표 이동 (mode == 'RELATIVE' 또는 relative_x/y/yaw 지정 시 최우선)
            is_relative_mode = (target.mode and str(target.mode).upper() == "RELATIVE")
            has_relative_fields = (
                target.relative_x is not None or 
                target.relative_y is not None or 
                target.relative_yaw is not None
            )

            # 세맨틱 이름 및 이유(rationale), 원문 텍스트에 회전 관련 키워드가 들어간 경우의 fallback 처리
            sem_name = (target.semantic_name or "").strip().lower()
            rationale_text = str(getattr(intent, 'rationale', '') or '').lower()
            user_text_hint = str(parsed_dict.get('user_text', '') if isinstance(parsed_dict, dict) else '').lower()
            text_context = f"{sem_name} {rationale_text} {user_text_hint}"

            if not is_relative_mode and not has_relative_fields:
                if any(kw in text_context for kw in ("좌회전", "우회전", "회전", "turn", "rotate", "돌아", "돌아줘")):
                    is_relative_mode = True
                    if any(kw in text_context for kw in ("좌", "왼", "left", "ccw", "반시계")):
                        target.relative_yaw = 15.0 if target.relative_yaw is None else target.relative_yaw
                    elif any(kw in text_context for kw in ("우", "오른", "right", "cw", "시계")):
                        target.relative_yaw = -15.0 if target.relative_yaw is None else target.relative_yaw

            if is_relative_mode or has_relative_fields:
                rel_x = float(target.relative_x or 0.0) if target.relative_x is not None else 0.0
                rel_y = float(target.relative_y or 0.0) if target.relative_y is not None else 0.0
                rel_yaw = float(target.relative_yaw or 0.0) if target.relative_yaw is not None else 0.0

                # 만약 Qwen이 coordinates에 상대거리를 넣은 경우 ([0.2, 0] 등)
                if rel_x == 0.0 and rel_y == 0.0 and rel_yaw == 0.0 and target.coordinates and len(target.coordinates) >= 1:
                    rel_x = float(target.coordinates[0])
                    if len(target.coordinates) >= 2:
                        rel_y = float(target.coordinates[1])
                    if len(target.coordinates) >= 3:
                        rel_yaw = float(target.coordinates[2])

                # 회전 방향 부호 안전 가드레일 (Left/좌/왼 -> positive +, Right/우/오른 -> negative -)
                # LLM이 방위각(compass) 또는 시계방향 착오로 부호를 반대로 출력한 경우 자동 교정
                is_left_intent = any(kw in text_context for kw in ("좌", "왼", "left", "ccw", "반시계"))
                is_right_intent = any(kw in text_context for kw in ("우", "오른", "right", "cw", "시계"))
                if is_left_intent and not is_right_intent and rel_yaw < 0:
                    rel_yaw = abs(rel_yaw)
                elif is_right_intent and not is_left_intent and rel_yaw > 0:
                    rel_yaw = -abs(rel_yaw)

                # cm 단위 오류 자동 변환 가드레일 (예: 20 -> 0.20m, 30 -> 0.30m)
                if abs(rel_x) >= 5.0:  # 5미터 이상의 단일 전진은 보통 cm 오입력(예: 20cm)
                    rel_x = rel_x / 100.0
                if abs(rel_y) >= 5.0:
                    rel_y = rel_y / 100.0

                return ValidationResult(
                    is_valid=True,
                    action=action,
                    resolved_task={
                        "action": "NAVIGATE_RELATIVE",
                        "relative_x": rel_x,
                        "relative_y": rel_y,
                        "relative_yaw": rel_yaw
                    }
                )

            # 3.2 직접 절대 좌표 입력 [x, y] or [x, y, yaw] (global map frame)
            if target.coordinates and len(target.coordinates) >= 2:
                x = float(target.coordinates[0])
                y = float(target.coordinates[1])
                yaw = float(target.coordinates[2]) if len(target.coordinates) >= 3 else 0.0
                return ValidationResult(
                    is_valid=True,
                    action=action,
                    resolved_task={
                        "action": "NAVIGATE_ABSOLUTE",
                        "target_pose": {"x": x, "y": y, "z": 0.0, "yaw_deg": yaw},
                        "frame_id": "map"
                    }
                )

            # 3.3 랜드마크 기준 상대 오프셋 이동
            if target.offset_from:
                base_key = self.resolve_waypoint_key(target.offset_from)
                if not base_key:
                    return ValidationResult(
                        is_valid=False,
                        action=action,
                        error_code="UNKNOWN_BASE_LANDMARK",
                        error_msg=f"기준 랜드마크 '{target.offset_from}'를 찾을 수 없습니다."
                    )
                base_info = self.waypoints[base_key]
                # 오프셋 합성
                calc_x = float(base_info["x"]) + float(target.offset_x or 0.0)
                calc_y = float(base_info["y"]) + float(target.offset_y or 0.0)
                calc_yaw = float(base_info.get("yaw_deg", 0.0))
                return ValidationResult(
                    is_valid=True,
                    action=action,
                    resolved_task={
                        "action": "NAVIGATE_OFFSET",
                        "base_landmark": base_key,
                        "target_pose": {"x": calc_x, "y": calc_y, "z": 0.0, "yaw_deg": calc_yaw},
                        "offset_applied": {"dx": target.offset_x, "dy": target.offset_y}
                    }
                )

            # 3.4 사전 정의된 고정 웨이포인트 이동
            wp_key = self.resolve_waypoint_key(target.semantic_name)
            if not wp_key:
                return ValidationResult(
                    is_valid=False,
                    action=action,
                    error_code="UNKNOWN_WAYPOINT",
                    error_msg=f"지도상에 '{target.semantic_name}' 장소를 찾을 수 없습니다. (등록된 장소: {list(self.waypoints.keys())})"
                )

            wp_info = self.waypoints[wp_key]
            return ValidationResult(
                is_valid=True,
                action=action,
                resolved_task={
                    "action": "NAVIGATE",
                    "waypoint_name": wp_key,
                    "target_pose": {
                        "x": float(wp_info["x"]),
                        "y": float(wp_info["y"]),
                        "z": float(wp_info.get("z", 0.0)),
                        "yaw_deg": float(wp_info.get("yaw_deg", 0.0))
                    }
                }
            )

        # 4. Action: DOCK (Visual Anchor 분리 및 직접 시각 도킹 우선 지원)
        if action == ActionEnum.DOCK.value:
            raw_target = (target.target_structure or target.semantic_name or "station").strip()

            profile_name = None
            station_name = None
            wp_key = None
            is_direct_visual = bool(getattr(target, 'direct_visual', False)) or any(
                k in raw_target.lower() for k in ["marker", "마커", "aruco", "아루코", "직접", "직시", "보고도킹"]
            )
            is_floor_object = (getattr(target, 'perception_type', None) == "floor_object") or any(
                k in raw_target.lower() for k in ["floor_object", "바닥물체", "물체", "바닥", "cube", "큐브", "ball", "공", "box", "상자"]
            )

            # 0순위: 마커 직시 도킹 또는 바닥 물체 직시 접근 (Direct Visual Mode: Nav2 Staging Bypass)
            if is_direct_visual or is_floor_object:
                is_direct_visual = True
                primary_station = list(self.docking_stations.keys())[0] if self.docking_stations else "workplace1"
                station_name = primary_station
                profile_name = "station_icp" if "station_icp" in self.profiles else list(self.profiles.keys())[0]
                structure_type = "floor_object" if is_floor_object else "station"
                wp_key = None  # Staging bypass
            # 1순위: 티칭된 정밀 도킹 스테이션 (docking_stations.yaml) 정밀/퍼지/조사제거 매칭
            elif self.resolve_station_key(raw_target) and self.resolve_station_key(raw_target) in self.docking_stations:
                resolved_station = self.resolve_station_key(raw_target)
                station_name = resolved_station
                wp_key = resolved_station
                profile_name = "station_icp"
                structure_type = station_name
            elif any(k in raw_target.lower() for k in ["desk", "책상", "테이블"]):
                structure_type = "desk"
                profile_name = "desk_direct" if target.anchor_cue is None else "desk_red_object"
            elif any(k in raw_target.lower() for k in ["charger", "충전"]):
                structure_type = "charger"
                profile_name = "charger_station"
            else:
                registered = list(self.docking_stations.keys())
                return ValidationResult(
                    is_valid=False,
                    action=action,
                    error_code="STATION_NOT_FOUND",
                    error_msg=f"도킹 스테이션 '{raw_target}'을 찾을 수 없습니다. (등록된 스테이션: {registered})"
                )

            detected_color = self.resolve_color_preset(target.anchor_cue)
            profile = self.profiles[profile_name]
            docking_face = target.docking_face or profile.get("default_face", "front")
            clearance_cm = float(target.clearance_cm or 5.0)

            wp_info = self.waypoints.get(wp_key, {}) if wp_key else {}
            staging_pose = None
            if not is_direct_visual and wp_info and "x" in wp_info:
                staging_pose = {
                    "x": float(wp_info["x"]),
                    "y": float(wp_info["y"]),
                    "z": float(wp_info.get("z", 0.0)),
                    "yaw_deg": float(wp_info.get("yaw_deg", 0.0))
                }

            final_target_name = getattr(target, 'target_name', None) or raw_target

            return ValidationResult(
                is_valid=True,
                action=action,
                resolved_task={
                    "action": "DOCK",
                    "target_structure": structure_type,
                    "target_name": final_target_name,
                    "anchor_cue": target.anchor_cue,
                    "anchor_color_preset": detected_color,
                    "docking_face": docking_face,
                    "clearance_cm": clearance_cm,
                    "profile_name": profile_name,
                    "docking_profile": profile,
                    "staging_waypoint": wp_key,
                    "staging_pose": staging_pose,
                    "station_name": station_name,
                    "direct_visual": is_direct_visual,
                    "perception_type": target.perception_type or ("floor_object" if is_floor_object else "aruco" if is_direct_visual else "icp")
                }
            )

        return ValidationResult(False, error_code="UNSUPPORTED_ACTION", error_msg=f"지원하지 않는 Action: {action}")
