#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Docking Teacher Node for ROS 2 / RViz.
Publishes a 3D Interactive Bounding Box Gizmo with Yaw rotation in RViz.
Allows operators to visually place the gizmo on target objects (boxes, desks),
crops the 3D Point Cloud within the gizmo, transforms it to Station Local frame,
and saves the normalized template PCD along with Standoff pose in docking_stations.yaml.
"""

import os
import sys
import time
import threading
import math
from typing import Tuple, Dict, Any, List, Optional
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, MenuEntry
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from llm_semantic_docker.station_manager import StationManager


WAYPOINTS_CONFIG_PATH = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker/config/semantic_waypoints.yaml"


class InteractiveDockingTeacherNode(Node):
    """RViz Interactive Marker Gizmo Server for Docking Station & Waypoint Teaching."""

    def __init__(self):
        super().__init__('interactive_docking_teacher')

        # Declare parameters
        self.declare_parameter('station_name', 'box_01')
        self.declare_parameter('waypoint_name', 'waypoint_01')
        self.declare_parameter('standoff_distance', 0.60)
        self.declare_parameter('box_size_x', 0.35)
        self.declare_parameter('box_size_y', 0.40)
        self.declare_parameter('box_size_z', 0.25)
        self.declare_parameter('map_frame', 'map')

        self.station_name = self.get_parameter('station_name').value
        self.waypoint_name = str(self.get_parameter('waypoint_name').value)
        self.standoff_distance = float(self.get_parameter('standoff_distance').value)
        self.box_size = np.array([
            float(self.get_parameter('box_size_x').value),
            float(self.get_parameter('box_size_y').value),
            float(self.get_parameter('box_size_z').value)
        ], dtype=np.float64)
        self.declare_parameter('floor_clearance', 0.05)       # 5cm clearance from floor surface
        self.declare_parameter('floor_z_level', -0.15)        # Physical ground level in visual SLAM frame (-0.15m)
        self.map_frame = self.get_parameter('map_frame').value
        self.floor_clearance = float(self.get_parameter('floor_clearance').value)
        self.floor_z_level = float(self.get_parameter('floor_z_level').value)

        self.default_box_size = self.box_size.copy()

        # Managers
        self.station_manager = StationManager()

        # Interactive Marker Server
        self.server = InteractiveMarkerServer(self, "docking_teacher_gizmo")

        # Check if station_name already exists in station_manager
        existing_info = self.station_manager.get_station(self.station_name)
        if existing_info:
            target_face = existing_info.get('target_face', {})
            center = target_face.get('center', [1.50, 1.40, 0.0])
            size = target_face.get('size', [0.35, 0.40, 0.25])
            normal = target_face.get('normal', [-1.0, 0.0, 0.0])
            self.gizmo_pos = np.array([float(center[0]), float(center[1]), float(center[2])], dtype=np.float64)
            self.box_size = np.array([float(size[0]), float(size[1]), float(size[2])], dtype=np.float64)
            self.default_box_size = self.box_size.copy()
            self.gizmo_yaw = math.atan2(float(normal[1]), float(normal[0]))
            self.get_logger().info(
                f"📍 Loaded previously saved pose for '{self.station_name}': "
                f"pos=[{self.gizmo_pos[0]:.3f}, {self.gizmo_pos[1]:.3f}, {self.gizmo_pos[2]:.3f}], "
                f"yaw={math.degrees(self.gizmo_yaw):.1f}°"
            )
        else:
            # Gizmo state: Bottom face sits exactly 5cm (0.05m) above the true physical floor (-0.15m)
            init_center_z = self.floor_z_level + self.floor_clearance + (self.box_size[2] / 2.0)
            self.gizmo_pos = np.array([1.50, 1.40, init_center_z], dtype=np.float64)
            self.gizmo_yaw = 0.0

        # Cached map point cloud
        self.cached_points = np.empty((0, 3), dtype=np.float64)

        # Standalone MarkerArray Publisher for guaranteed visibility in any RViz display mode
        self.marker_pub = self.create_publisher(MarkerArray, '/docking_teacher_gizmo/markers', 10)
        self.marker_timer = self.create_timer(0.2, self._publish_preview_markers)

        # Map Cloud Subscriber (RTAB-Map 3D cloud or live pointcloud)
        map_cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self._cloud_callback,
            map_cloud_qos
        )
        self.cloud_map_sub2 = self.create_subscription(
            PointCloud2,
            '/cloud_map',
            self._cloud_callback,
            map_cloud_qos
        )

        # Fallback Live Depth Cloud Subscriber
        self.live_cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self._live_cloud_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Feedback state and publisher
        self.feedback_status_text = ""
        self.feedback_time = 0.0
        self.feedback_pub = self.create_publisher(String, '/docking/teach_station_feedback', 10)

        # Command Subscriber & Trigger Service
        self.cmd_sub = self.create_subscription(
            String,
            '/docking/teach_station_command',
            self._command_callback,
            10
        )
        self.save_srv = self.create_service(
            Trigger,
            '/docking/save_current_station',
            self._save_service_callback
        )
        self.save_wp_srv = self.create_service(
            Trigger,
            '/docking/save_current_waypoint',
            self._save_waypoint_service_callback
        )

        # Dynamic Box Size Subscription
        self.box_size_sub = self.create_subscription(
            Vector3,
            '/docking/set_box_size',
            self._on_set_box_size_topic,
            10
        )

        # Allow 2D Goal Pose tool from RViz toolbar to instantly relocate gizmo
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._on_goal_pose,
            10
        )

        # Menu Handler
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("💾 Save Docking Station (docking_stations.yaml)", callback=self._menu_save_callback)
        self.menu_handler.insert("📍 Save Waypoint (semantic_waypoints.yaml)", callback=self._menu_save_waypoint_callback)

        # Submenu for Dynamic Box Size Adjustment
        sub_size = self.menu_handler.insert("📐 Adjust Box Dimensions")
        self.menu_handler.insert("➕ Scale All (+10%)", parent=sub_size, callback=self._menu_scale_up)
        self.menu_handler.insert("➖ Scale All (-10%)", parent=sub_size, callback=self._menu_scale_down)
        self.menu_handler.insert("↔️ Width X +5cm (0.05m)", parent=sub_size, callback=self._menu_width_inc)
        self.menu_handler.insert("↔️ Width X -5cm (0.05m)", parent=sub_size, callback=self._menu_width_dec)
        self.menu_handler.insert("↕️ Depth Y +5cm (0.05m)", parent=sub_size, callback=self._menu_depth_inc)
        self.menu_handler.insert("↕️ Depth Y -5cm (0.05m)", parent=sub_size, callback=self._menu_depth_dec)
        self.menu_handler.insert("⏫ Height Z +5cm (0.05m)", parent=sub_size, callback=self._menu_height_inc)
        self.menu_handler.insert("⏬ Height Z -5cm (0.05m)", parent=sub_size, callback=self._menu_height_dec)
        self.menu_handler.insert("🔄 Reset to Default Size", parent=sub_size, callback=self._menu_reset_size)

        # Initialize Marker
        self._init_interactive_marker()
        self.get_logger().info(
            f"InteractiveDockingTeacher ready for station '{self.station_name}' & waypoint '{self.waypoint_name}'."
        )

        # Interactive Console Thread (실행 중 실시간 이름 변경 및 저장 지원)
        self.cli_thread = threading.Thread(target=self._cli_worker, daemon=True)
        self.cli_thread.start()

    def _cli_worker(self):
        """Interactive console reader allowing dynamic station and waypoint name changing during runtime."""
        time.sleep(1.2)
        banner = f"""
\033[1;36m────────────────────────────────────────────────────────────────────────
 ⌨️  [기즈모 티칭 콘솔] 실행 중 실시간 이름 변경 및 즉시 저장 지원:
  - 스테이션 이름 변경:   station <이름>    (예: station workplace2)
  - 웨이포인트 이름 변경: wp <이름>         (예: wp my_desk)
  - 현재 위치 즉시 저장:  save (스테이션)  / save_wp (웨이포인트)
  - 현재 상태 확인:       status
────────────────────────────────────────────────────────────────────────\033[0m
"""
        print(banner, flush=True)

        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                line_str = line.strip()
                if not line_str:
                    continue
                if line_str.lower() in ("help", "?"):
                    print(banner, flush=True)
                else:
                    self._process_command_string(line_str)
            except Exception:
                pass

    def _cloud_callback(self, msg: PointCloud2):
        """Caches full RTAB-Map 3D point cloud."""
        try:
            pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts_list = [[p[0], p[1], p[2]] for p in pts_gen]
            if len(pts_list) > 0:
                was_empty = (self.cached_points.shape[0] == 0)
                self.cached_points = np.array(pts_list, dtype=np.float64)
                if was_empty:
                    self.get_logger().info(
                        f"✅ Successfully cached 3D Map Cloud: {self.cached_points.shape[0]} points."
                    )
        except Exception as e:
            self.get_logger().warn(f"Failed to parse cloud_map: {e}")

    def _live_cloud_callback(self, msg: PointCloud2):
        """Fallback live depth cloud if map cloud is not yet published."""
        if self.cached_points.shape[0] == 0:
            try:
                pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                pts_list = [[p[0], p[1], p[2]] for p in pts_gen]
                if len(pts_list) > 100:
                    self.cached_points = np.array(pts_list, dtype=np.float64)
            except Exception:
                pass

    def _init_interactive_marker(self):
        """Constructs 6-DoF Interactive Bounding Box Gizmo with Yaw handle."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.map_frame
        int_marker.name = "station_gizmo"
        int_marker.description = f"Docking Target: {self.station_name}"
        # Make gizmo control handles large and easily clickable outside the bounding box
        int_marker.scale = max(float(self.box_size[0]) * 1.5, 0.85)

        int_marker.pose.position.x = float(self.gizmo_pos[0])
        int_marker.pose.position.y = float(self.gizmo_pos[1])
        int_marker.pose.position.z = float(self.gizmo_pos[2])
        int_marker.pose.orientation = self._yaw_to_quaternion(self.gizmo_yaw)

        # 1. Visual 3D Bounding Box Marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = float(self.box_size[0])
        box_marker.scale.y = float(self.box_size[1])
        box_marker.scale.z = float(self.box_size[2])
        box_marker.color.r = 0.1
        box_marker.color.g = 0.85
        box_marker.color.b = 0.2
        box_marker.color.a = 0.50

        # 2. Front Approach Arrow Marker (Points along +X)
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = float(self.box_size[0] / 2.0 + 0.35)
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.15
        arrow_marker.color.b = 0.1
        arrow_marker.color.a = 0.95

        # 3. Standoff Ghost Marker (Yellow cylinder showing arrival point)
        standoff_marker = Marker()
        standoff_marker.type = Marker.CYLINDER
        standoff_marker.scale.x = 0.30
        standoff_marker.scale.y = 0.30
        standoff_marker.scale.z = 0.05
        standoff_marker.color.r = 1.0
        standoff_marker.color.g = 0.85
        standoff_marker.color.b = 0.0
        standoff_marker.color.a = 0.70
        standoff_marker.pose.position.x = float(self.box_size[0] / 2.0 + self.standoff_distance)
        standoff_marker.pose.position.z = -float(self.box_size[2] / 2.0)

        # 4. Box Body Control: Set to MOVE_PLANE so clicking & dragging the box directly moves it across XY plane!
        # Right-clicking anywhere on the box pops up the MenuHandler (Save Station).
        box_plane_ctrl = InteractiveMarkerControl()
        box_plane_ctrl.name = "box_plane_move"
        box_plane_ctrl.always_visible = True
        box_plane_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # XY-plane movement: Normal vector along Z axis (pitch 90 deg rotation)
        box_plane_ctrl.orientation.w = 0.7071068
        box_plane_ctrl.orientation.x = 0.0
        box_plane_ctrl.orientation.y = 0.7071068
        box_plane_ctrl.orientation.z = 0.0
        box_plane_ctrl.markers.extend([box_marker, arrow_marker, standoff_marker])
        int_marker.controls.append(box_plane_ctrl)

        # 5. Yaw Rotation Ring Control (around Z-axis)
        ctrl_yaw = InteractiveMarkerControl()
        ctrl_yaw.name = "rotate_z"
        ctrl_yaw.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ctrl_yaw.orientation.w = 0.7071068
        ctrl_yaw.orientation.x = 0.0
        ctrl_yaw.orientation.y = 0.7071068
        ctrl_yaw.orientation.z = 0.0
        ctrl_yaw.always_visible = True
        int_marker.controls.append(ctrl_yaw)

        # 6. Vertical Height Control (Z-axis Move Handle for fine vertical adjustment)
        ctrl_z = InteractiveMarkerControl()
        ctrl_z.name = "move_z"
        ctrl_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl_z.orientation.w = 0.7071068
        ctrl_z.orientation.x = 0.0
        ctrl_z.orientation.y = 0.7071068
        ctrl_z.orientation.z = 0.0
        ctrl_z.always_visible = True
        int_marker.controls.append(ctrl_z)

        # 7. Fine X-axis translation handle
        ctrl_x = InteractiveMarkerControl()
        ctrl_x.name = "move_x"
        ctrl_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl_x.orientation.w = 0.7071068
        ctrl_x.orientation.x = 0.7071068
        ctrl_x.orientation.y = 0.0
        ctrl_x.orientation.z = 0.0
        ctrl_x.always_visible = True
        int_marker.controls.append(ctrl_x)

        # 8. Fine Y-axis translation handle
        ctrl_y = InteractiveMarkerControl()
        ctrl_y.name = "move_y"
        ctrl_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        ctrl_y.orientation.w = 0.7071068
        ctrl_y.orientation.x = 0.0
        ctrl_y.orientation.y = 0.0
        ctrl_y.orientation.z = 0.7071068
        ctrl_y.always_visible = True
        int_marker.controls.append(ctrl_y)

        # 9. Insert and apply MenuHandler
        self.server.insert(int_marker, feedback_callback=self._process_feedback)
        self.menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()

    def _menu_save_callback(self, feedback):
        """Right-click menu save trigger for docking station."""
        self.get_logger().info(f"🖱️ [Menu] Save Docking Station triggered for '{self.station_name}'...")
        success, msg = self.save_station_template()
        self.get_logger().info(f"💾 Result: {msg}")

    def _menu_save_waypoint_callback(self, feedback):
        """Right-click menu save trigger for waypoint."""
        self.get_logger().info(f"🖱️ [Menu] Save Waypoint triggered for '{self.waypoint_name}'...")
        success, msg = self.save_waypoint()
        self.get_logger().info(f"📍 Result: {msg}")

    def _menu_scale_up(self, feedback):
        self.set_box_size(self.box_size[0] * 1.10, self.box_size[1] * 1.10, self.box_size[2] * 1.10)

    def _menu_scale_down(self, feedback):
        self.set_box_size(self.box_size[0] * 0.90, self.box_size[1] * 0.90, self.box_size[2] * 0.90)

    def _menu_width_inc(self, feedback):
        self.set_box_size(self.box_size[0] + 0.05, self.box_size[1], self.box_size[2])

    def _menu_width_dec(self, feedback):
        self.set_box_size(max(0.10, self.box_size[0] - 0.05), self.box_size[1], self.box_size[2])

    def _menu_depth_inc(self, feedback):
        self.set_box_size(self.box_size[0], self.box_size[1] + 0.05, self.box_size[2])

    def _menu_depth_dec(self, feedback):
        self.set_box_size(self.box_size[0], max(0.10, self.box_size[1] - 0.05), self.box_size[2])

    def _menu_height_inc(self, feedback):
        self.set_box_size(self.box_size[0], self.box_size[1], self.box_size[2] + 0.05)

    def _menu_height_dec(self, feedback):
        self.set_box_size(self.box_size[0], self.box_size[1], max(0.05, self.box_size[2] - 0.05))

    def _menu_reset_size(self, feedback):
        self.set_box_size(self.default_box_size[0], self.default_box_size[1], self.default_box_size[2])

    def _on_set_box_size_topic(self, msg: Vector3):
        """Allows setting box dimensions via /docking/set_box_size topic."""
        self.set_box_size(msg.x, msg.y, msg.z)

    def set_box_size(self, sx: float, sy: float, sz: float):
        """Dynamically updates the box size, adjusts center Z to keep bottom at ground+5cm, and refreshes gizmo."""
        self.box_size[0] = round(max(0.05, float(sx)), 3)
        self.box_size[1] = round(max(0.05, float(sy)), 3)
        self.box_size[2] = round(max(0.05, float(sz)), 3)

        # Keep box bottom face at ground clearance level (-0.15m + 0.05m = -0.10m)
        self.gizmo_pos[2] = self.floor_z_level + self.floor_clearance + (self.box_size[2] / 2.0)

        # Refresh interactive marker with new dimensions
        self._init_interactive_marker()
        self.get_logger().info(
            f"📏 Resized Box: Width(X)={self.box_size[0]:.2f}m, Depth(Y)={self.box_size[1]:.2f}m, Height(Z)={self.box_size[2]:.2f}m "
            f"(Bottom sits 5cm above floor at Z={self.floor_z_level + self.floor_clearance:.2f}m)"
        )

    def _on_goal_pose(self, msg: PoseStamped):
        """Allows clicking '2D Goal Pose' or 'Publish Point' in RViz toolbar to instantly teleport gizmo."""
        self.gizmo_pos[0] = msg.pose.position.x
        self.gizmo_pos[1] = msg.pose.position.y
        self.gizmo_pos[2] = self.floor_z_level + self.floor_clearance + (self.box_size[2] / 2.0)
        self.gizmo_yaw = self._quaternion_to_yaw(msg.pose.orientation)

        new_pose = Pose()
        new_pose.position.x = float(self.gizmo_pos[0])
        new_pose.position.y = float(self.gizmo_pos[1])
        new_pose.position.z = float(self.gizmo_pos[2])
        new_pose.orientation = self._yaw_to_quaternion(self.gizmo_yaw)

        self.server.setPose("station_gizmo", new_pose)
        self.server.applyChanges()
        self.get_logger().info(
            f"📍 Teleported Gizmo to 2D Click Position: X={self.gizmo_pos[0]:.2f}, Y={self.gizmo_pos[1]:.2f}, Yaw={math.degrees(self.gizmo_yaw):.1f}°"
        )

    def _publish_preview_markers(self):
        """Continuously publishes standard MarkerArray so gizmo is guaranteed visible in all RViz views."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        ori_q = self._yaw_to_quaternion(self.gizmo_yaw)

        # 1. Target Bounding Box (Translucent Green)
        m_box = Marker()
        m_box.header.frame_id = self.map_frame
        m_box.header.stamp = now
        m_box.ns = "gizmo_preview"
        m_box.id = 100
        m_box.type = Marker.CUBE
        m_box.action = Marker.ADD
        m_box.pose.position.x = float(self.gizmo_pos[0])
        m_box.pose.position.y = float(self.gizmo_pos[1])
        m_box.pose.position.z = float(self.gizmo_pos[2])
        m_box.pose.orientation = ori_q
        m_box.scale.x = float(self.box_size[0])
        m_box.scale.y = float(self.box_size[1])
        m_box.scale.z = float(self.box_size[2])
        m_box.color.r = 0.1
        m_box.color.g = 0.9
        m_box.color.b = 0.2
        m_box.color.a = 0.55
        ma.markers.append(m_box)

        # 2. Front Approach Arrow (Red)
        m_arrow = Marker()
        m_arrow.header.frame_id = self.map_frame
        m_arrow.header.stamp = now
        m_arrow.ns = "gizmo_preview"
        m_arrow.id = 101
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.pose.position.x = float(self.gizmo_pos[0])
        m_arrow.pose.position.y = float(self.gizmo_pos[1])
        m_arrow.pose.position.z = float(self.gizmo_pos[2])
        m_arrow.pose.orientation = ori_q
        m_arrow.scale.x = float(self.box_size[0] / 2.0 + 0.35) # length
        m_arrow.scale.y = 0.05                                 # width
        m_arrow.scale.z = 0.05                                 # height
        m_arrow.color.r = 1.0
        m_arrow.color.g = 0.15
        m_arrow.color.b = 0.1
        m_arrow.color.a = 0.95
        ma.markers.append(m_arrow)

        # 3. Standoff Arrival Goal (Yellow Cylinder)
        nx = math.cos(self.gizmo_yaw)
        ny = math.sin(self.gizmo_yaw)
        so_x = self.gizmo_pos[0] + (self.box_size[0] / 2.0 + self.standoff_distance) * nx
        so_y = self.gizmo_pos[1] + (self.box_size[0] / 2.0 + self.standoff_distance) * ny

        m_so = Marker()
        m_so.header.frame_id = self.map_frame
        m_so.header.stamp = now
        m_so.ns = "gizmo_preview"
        m_so.id = 102
        m_so.type = Marker.CYLINDER
        m_so.action = Marker.ADD
        m_so.pose.position.x = float(so_x)
        m_so.pose.position.y = float(so_y)
        m_so.pose.position.z = float(self.gizmo_pos[2] - self.box_size[2] / 2.0)
        m_so.pose.orientation = ori_q
        m_so.scale.x = 0.30
        m_so.scale.y = 0.30
        m_so.scale.z = 0.05
        m_so.color.r = 1.0
        m_so.color.g = 0.85
        m_so.color.b = 0.0
        m_so.color.a = 0.75
        ma.markers.append(m_so)

        # 4. Text Label
        m_text = Marker()
        m_text.header.frame_id = self.map_frame
        m_text.header.stamp = now
        m_text.ns = "gizmo_preview"
        m_text.id = 103
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = float(self.gizmo_pos[0])
        m_text.pose.position.y = float(self.gizmo_pos[1])
        m_text.pose.position.z = float(self.gizmo_pos[2] + self.box_size[2] / 2.0 + 0.18)
        m_text.scale.z = 0.13 # text size
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.color.a = 0.95
        label = f"🎯 Station: [{self.station_name}] | 📍 WP: [{self.waypoint_name}]"
        if (time.time() - self.feedback_time < 6.0) and self.feedback_status_text:
            label += f"\n✨ {self.feedback_status_text}"
        m_text.text = label
        ma.markers.append(m_text)

        # 5. Display All Other Previously Saved Stations from YAML
        st_idx = 0
        for st_name, st_info in self.station_manager.stations.items():
            if st_name == self.station_name:
                continue
            tf_data = st_info.get('target_face', {})
            st_center = tf_data.get('center', [0, 0, 0])
            st_size = tf_data.get('size', [0.35, 0.40, 0.25])
            st_normal = tf_data.get('normal', [-1.0, 0.0, 0.0])
            st_yaw = math.atan2(float(st_normal[1]), float(st_normal[0]))
            st_ori_q = self._yaw_to_quaternion(st_yaw)

            # Saved Box (Cyan / Semi-transparent)
            s_box = Marker()
            s_box.header.frame_id = self.map_frame
            s_box.header.stamp = now
            s_box.ns = "gizmo_preview"
            s_box.id = 200 + st_idx * 10
            s_box.type = Marker.CUBE
            s_box.action = Marker.ADD
            s_box.pose.position.x = float(st_center[0])
            s_box.pose.position.y = float(st_center[1])
            s_box.pose.position.z = float(st_center[2])
            s_box.pose.orientation = st_ori_q
            s_box.scale.x = float(st_size[0])
            s_box.scale.y = float(st_size[1])
            s_box.scale.z = float(st_size[2])
            s_box.color.r = 0.0
            s_box.color.g = 0.7
            s_box.color.b = 0.9
            s_box.color.a = 0.45
            ma.markers.append(s_box)

            # Saved Station Arrow (Cyan)
            s_arrow = Marker()
            s_arrow.header.frame_id = self.map_frame
            s_arrow.header.stamp = now
            s_arrow.ns = "gizmo_preview"
            s_arrow.id = 200 + st_idx * 10 + 1
            s_arrow.type = Marker.ARROW
            s_arrow.action = Marker.ADD
            s_arrow.pose.position.x = float(st_center[0])
            s_arrow.pose.position.y = float(st_center[1])
            s_arrow.pose.position.z = float(st_center[2])
            s_arrow.pose.orientation = st_ori_q
            s_arrow.scale.x = float(st_size[0] / 2.0 + 0.30)
            s_arrow.scale.y = 0.04
            s_arrow.scale.z = 0.04
            s_arrow.color.r = 0.0
            s_arrow.color.g = 0.8
            s_arrow.color.b = 1.0
            s_arrow.color.a = 0.85
            ma.markers.append(s_arrow)

            # Saved Station Standoff (Teal Cylinder)
            standoff = st_info.get('standoff', {})
            so_x = float(standoff.get('x', st_center[0]))
            so_y = float(standoff.get('y', st_center[1]))
            s_so = Marker()
            s_so.header.frame_id = self.map_frame
            s_so.header.stamp = now
            s_so.ns = "gizmo_preview"
            s_so.id = 200 + st_idx * 10 + 2
            s_so.type = Marker.CYLINDER
            s_so.action = Marker.ADD
            s_so.pose.position.x = so_x
            s_so.pose.position.y = so_y
            s_so.pose.position.z = float(st_center[2] - st_size[2] / 2.0)
            s_so.pose.orientation = st_ori_q
            s_so.scale.x = 0.25
            s_so.scale.y = 0.25
            s_so.scale.z = 0.04
            s_so.color.r = 0.1
            s_so.color.g = 0.9
            s_so.color.b = 0.7
            s_so.color.a = 0.65
            ma.markers.append(s_so)

            # Saved Text Label
            s_text = Marker()
            s_text.header.frame_id = self.map_frame
            s_text.header.stamp = now
            s_text.ns = "gizmo_preview"
            s_text.id = 200 + st_idx * 10 + 3
            s_text.type = Marker.TEXT_VIEW_FACING
            s_text.action = Marker.ADD
            s_text.pose.position.x = float(st_center[0])
            s_text.pose.position.y = float(st_center[1])
            s_text.pose.position.z = float(st_center[2] + st_size[2] / 2.0 + 0.15)
            s_text.scale.z = 0.10
            s_text.color.r = 0.3
            s_text.color.g = 0.9
            s_text.color.b = 1.0
            s_text.color.a = 0.95
            s_text.text = f"✔ [Saved] {st_name}"
            ma.markers.append(s_text)

            st_idx += 1

        # 3. Saved Waypoints from semantic_waypoints.yaml (Golden Pin & Text)
        if os.path.exists(WAYPOINTS_CONFIG_PATH):
            try:
                with open(WAYPOINTS_CONFIG_PATH, 'r', encoding='utf-8') as f:
                    wp_data = yaml.safe_load(f) or {}
                    wps = wp_data.get("waypoints", {})
                    wp_idx = 0
                    for wp_name, wp_info in wps.items():
                        wx = float(wp_info.get("x", 0.0))
                        wy = float(wp_info.get("y", 0.0))
                        wz = float(wp_info.get("z", 0.0))
                        wyaw = math.radians(float(wp_info.get("yaw_deg", 0.0)))
                        w_q = self._yaw_to_quaternion(wyaw)

                        # Cylinder Pin
                        wp_cyl = Marker()
                        wp_cyl.header.frame_id = self.map_frame
                        wp_cyl.header.stamp = now
                        wp_cyl.ns = "gizmo_preview"
                        wp_cyl.id = 500 + wp_idx * 10
                        wp_cyl.type = Marker.CYLINDER
                        wp_cyl.action = Marker.ADD
                        wp_cyl.pose.position.x = wx
                        wp_cyl.pose.position.y = wy
                        wp_cyl.pose.position.z = wz + 0.10
                        wp_cyl.scale.x = 0.20
                        wp_cyl.scale.y = 0.20
                        wp_cyl.scale.z = 0.20
                        wp_cyl.color.r = 1.0
                        wp_cyl.color.g = 0.75
                        wp_cyl.color.b = 0.0
                        wp_cyl.color.a = 0.75
                        ma.markers.append(wp_cyl)

                        # Direction Arrow
                        wp_arrow = Marker()
                        wp_arrow.header.frame_id = self.map_frame
                        wp_arrow.header.stamp = now
                        wp_arrow.ns = "gizmo_preview"
                        wp_arrow.id = 500 + wp_idx * 10 + 1
                        wp_arrow.type = Marker.ARROW
                        wp_arrow.action = Marker.ADD
                        wp_arrow.pose.position.x = wx
                        wp_arrow.pose.position.y = wy
                        wp_arrow.pose.position.z = wz + 0.20
                        wp_arrow.pose.orientation = w_q
                        wp_arrow.scale.x = 0.35
                        wp_arrow.scale.y = 0.04
                        wp_arrow.scale.z = 0.04
                        wp_arrow.color.r = 1.0
                        wp_arrow.color.g = 0.85
                        wp_arrow.color.b = 0.2
                        wp_arrow.color.a = 0.90
                        ma.markers.append(wp_arrow)

                        # Text Label
                        wp_txt = Marker()
                        wp_txt.header.frame_id = self.map_frame
                        wp_txt.header.stamp = now
                        wp_txt.ns = "gizmo_preview"
                        wp_txt.id = 500 + wp_idx * 10 + 2
                        wp_txt.type = Marker.TEXT_VIEW_FACING
                        wp_txt.action = Marker.ADD
                        wp_txt.pose.position.x = wx
                        wp_txt.pose.position.y = wy
                        wp_txt.pose.position.z = wz + 0.35
                        wp_txt.scale.z = 0.10
                        wp_txt.color.r = 1.0
                        wp_txt.color.g = 0.85
                        wp_txt.color.b = 0.2
                        wp_txt.color.a = 0.95
                        wp_txt.text = f"📍 [WP] {wp_name}"
                        ma.markers.append(wp_txt)

                        wp_idx += 1
            except Exception:
                pass

        self.marker_pub.publish(ma)

    def _process_feedback(self, feedback: InteractiveMarkerFeedback):
        """Handles user dragging, rotating, and menu selections in RViz."""
        if feedback.event_type in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            self.gizmo_pos[0] = feedback.pose.position.x
            self.gizmo_pos[1] = feedback.pose.position.y
            self.gizmo_pos[2] = feedback.pose.position.z
            self.gizmo_yaw = self._quaternion_to_yaw(feedback.pose.orientation)

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self.get_logger().info(f"RViz Menu: Saving station '{self.station_name}'...")
                success, msg = self.save_station_template()
                self.get_logger().info(f"Save Station Result: {msg}")

    def save_station_template(self) -> Tuple[bool, str]:
        """
        Saves station pose, bounding box size, and approach heading to docking_stations.yaml.
        """
        # Approach direction: Facing into the box front face from robot perspective
        # If gizmo arrow points towards robot (+X in local), robot approach yaw is opposite (+pi)
        approach_yaw = (self.gizmo_yaw + math.pi) % (2.0 * math.pi)

        success = self.station_manager.register_station(
            station_name=self.station_name,
            center=list(self.gizmo_pos),
            size=list(self.box_size),
            approach_yaw=approach_yaw,
            points_local=None,
            description=f"Taught station {self.station_name} via RViz Gizmo",
            standoff_distance=self.standoff_distance
        )

        if success:
            # Auto-sync station standoff position as a waypoint in semantic_waypoints.yaml
            station_info = self.station_manager.get_station(self.station_name)
            if station_info and "standoff" in station_info:
                so = station_info["standoff"]
                self._save_waypoint_entry(
                    wp_name=self.station_name,
                    x=so["x"],
                    y=so["y"],
                    z=self.gizmo_pos[2],
                    yaw_deg=so["yaw_deg"],
                    desc=f"Taught station {self.station_name} standoff waypoint via RViz Gizmo"
                )

            msg = (f"Successfully saved station '{self.station_name}' "
                   f"(Pos: [{self.gizmo_pos[0]:.3f}, {self.gizmo_pos[1]:.3f}, {self.gizmo_pos[2]:.3f}], "
                   f"Yaw: {math.degrees(approach_yaw):.1f}°) to docking_stations.yaml "
                   f"& auto-synced to semantic_waypoints.yaml.")
            self.get_logger().info(f"💾 {msg}")
            return True, msg
        else:
            return False, "Failed to write station configuration to disk."

    def _save_waypoint_entry(self, wp_name: str, x: float, y: float, z: float, yaw_deg: float, desc: str = "") -> bool:
        """Helper to write or update a waypoint entry in semantic_waypoints.yaml."""
        data = {}
        if os.path.exists(WAYPOINTS_CONFIG_PATH):
            try:
                with open(WAYPOINTS_CONFIG_PATH, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().warn(f"Failed to load existing semantic_waypoints.yaml: {e}")

        if "waypoints" not in data or not isinstance(data["waypoints"], dict):
            data["waypoints"] = {}

        data["waypoints"][wp_name] = {
            "description": desc or f"Taught waypoint {wp_name} via RViz Gizmo",
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "z": round(float(z), 4),
            "yaw_deg": round(float(yaw_deg), 2),
            "aliases": [wp_name]
        }

        try:
            install_wp_path = "/home/jjj/Documents/Dapier/Project/robot_ws/install/llm_semantic_docker/share/llm_semantic_docker/config/semantic_waypoints.yaml"
            for p in [WAYPOINTS_CONFIG_PATH, install_wp_path]:
                if os.path.exists(os.path.dirname(p)):
                    with open(p, 'w', encoding='utf-8') as f:
                        yaml.dump(data, f, allow_unicode=True, sort_keys=False)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoint '{wp_name}' to disk: {e}")
            return False

    def save_waypoint(self, wp_name: Optional[str] = None) -> Tuple[bool, str]:
        """
        Saves waypoint position and yaw orientation directly to semantic_waypoints.yaml.
        """
        target_name = (wp_name or self.waypoint_name).strip()
        if not target_name:
            target_name = "waypoint_01"

        yaw_deg = round(float(math.degrees(self.gizmo_yaw)), 2)
        ok = self._save_waypoint_entry(
            wp_name=target_name,
            x=self.gizmo_pos[0],
            y=self.gizmo_pos[1],
            z=self.gizmo_pos[2],
            yaw_deg=yaw_deg,
            desc=f"Taught waypoint {target_name} via RViz Gizmo"
        )
        if ok:
            msg = (f"Successfully saved waypoint '{target_name}' "
                   f"(Pos: [{self.gizmo_pos[0]:.3f}, {self.gizmo_pos[1]:.3f}, {self.gizmo_pos[2]:.3f}], "
                   f"Yaw: {yaw_deg:.1f}°) to semantic_waypoints.yaml.")
            self.get_logger().info(f"📍 {msg}")
            return True, msg
        else:
            return False, f"Failed to save waypoint '{target_name}' to disk."

    def _send_feedback(self, text: str):
        """Sends user-facing feedback to ROS topic, logger, and updates RViz 3D text badge."""
        self.feedback_status_text = text
        self.feedback_time = time.time()
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"📢 [Gizmo Feedback] {text}")

    def _process_command_string(self, raw_cmd: str):
        """Parses and executes commands from CLI or ROS topic."""
        cmd = raw_cmd.strip().split()
        if not cmd:
            return
        action = cmd[0].lower()

        # Helper to strip '<>', quotes, whitespace
        clean = lambda s: s.strip("<> \"'")

        # 1. Station Name Change: 'station [name]', 'st [name]'
        if action in ("station", "set_station", "st", "set_name", "set_station_name"):
            if len(cmd) > 1:
                self.station_name = clean(cmd[1])
                self._init_interactive_marker()
                self._send_feedback(f"✔ 도킹 스테이션 대상 이름 변경: '{self.station_name}'")
            else:
                self._send_feedback(f"ℹ 현재 스테이션 이름: '{self.station_name}' (변경: station <이름>)")

        # 2. Waypoint Name Change: 'wp [name]', 'set_wp [name]'
        elif action in ("wp", "set_wp", "set_waypoint", "set_waypoint_name", "set_wp_name"):
            if len(cmd) > 1:
                self.waypoint_name = clean(cmd[1])
                self._init_interactive_marker()
                self._send_feedback(f"✔ 웨이포인트 대상 이름 변경: '{self.waypoint_name}'")
            else:
                self._send_feedback(f"ℹ 현재 웨이포인트 이름: '{self.waypoint_name}' (변경: wp <이름>)")

        # 3. Save Waypoint: 'save_wp', 'save wp', 'save waypoint'
        elif action in ("save_wp", "save_waypoint") or (action == "save" and len(cmd) > 1 and cmd[1].lower() in ("wp", "waypoint")):
            target_wp = self.waypoint_name
            if action in ("save_wp", "save_waypoint") and len(cmd) > 1:
                target_wp = clean(cmd[1])
            elif action == "save" and len(cmd) > 2:
                target_wp = clean(cmd[2])

            self.waypoint_name = target_wp
            success, res = self.save_waypoint(target_wp)
            self._init_interactive_marker()
            self._send_feedback(f"📍 웨이포인트 '{self.waypoint_name}': {res}")

        # 4. Save Station: 'save', 'save_station', 'save st', 'save station'
        elif action in ("save", "save_station"):
            target_st = self.station_name
            if len(cmd) > 1:
                if cmd[1].lower() in ("st", "station"):
                    if len(cmd) > 2:
                        target_st = clean(cmd[2])
                else:
                    target_st = clean(cmd[1])

            self.station_name = target_st
            success, res = self.save_station_template()
            self._init_interactive_marker()
            self._send_feedback(f"💾 도킹 스테이션 '{self.station_name}': {res}")

        # 5. Status / Info
        elif action in ("status", "info"):
            status_str = f"Station='{self.station_name}', WP='{self.waypoint_name}', Pose=[X={self.gizmo_pos[0]:.2f}, Y={self.gizmo_pos[1]:.2f}, Yaw={math.degrees(self.gizmo_yaw):.1f}°]"
            self._send_feedback(f"📍 상태: {status_str}")

        # 6. Box dimensions
        elif action in ("box", "size", "set_box_size") and len(cmd) >= 4:
            try:
                sx, sy, sz = float(cmd[1]), float(cmd[2]), float(cmd[3])
                self.set_box_size(sx, sy, sz)
                self._send_feedback(f"📐 박스 크기 변경: {sx:.2f}x{sy:.2f}x{sz:.2f}m")
            except Exception as e:
                self._send_feedback(f"⚠️ 크기 파싱 실패: {e}")

        else:
            self._send_feedback(f"⚠️ 알 수 없는 명령어: '{raw_cmd}' (사용 가능: station <이름>, wp <이름>, save, save_wp, status)")

    def _command_callback(self, msg: String):
        """Processes string commands sent via /docking/teach_station_command topic."""
        self._process_command_string(msg.data)

    def _save_service_callback(self, request, response):
        """Trigger service callback for station."""
        success, message = self.save_station_template()
        response.success = success
        response.message = message
        return response

    def _save_waypoint_service_callback(self, request, response):
        """Trigger service callback for waypoint."""
        success, message = self.save_waypoint()
        response.success = success
        response.message = message
        return response

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        return q

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveDockingTeacherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
