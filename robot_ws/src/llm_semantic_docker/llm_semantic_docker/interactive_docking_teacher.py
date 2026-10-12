#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Docking Teacher Node for ROS 2 / RViz.
Publishes a 3D Interactive Bounding Box Gizmo with Yaw rotation in RViz.
Allows operators to visually place the gizmo on target objects (boxes, desks),
crops the 3D Point Cloud within the gizmo, transforms it to Station Local frame,
and saves the normalized template PCD along with Standoff pose in docking_stations.yaml.
"""

import math
from typing import Tuple, Dict, Any, List, Optional
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
from llm_semantic_docker.pointcloud_icp_matcher import PointCloudICPMatcher


class InteractiveDockingTeacherNode(Node):
    """RViz Interactive Marker Gizmo Server for Docking Station Teaching."""

    def __init__(self):
        super().__init__('interactive_docking_teacher')

        # Declare parameters
        self.declare_parameter('station_name', 'box_01')
        self.declare_parameter('standoff_distance', 0.60)
        self.declare_parameter('box_size_x', 0.35)
        self.declare_parameter('box_size_y', 0.40)
        self.declare_parameter('box_size_z', 0.25)
        self.declare_parameter('map_frame', 'map')

        self.station_name = self.get_parameter('station_name').value
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
        self.matcher = PointCloudICPMatcher(voxel_size=0.01)

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
        self.menu_handler.insert("💾 Save Station (PCD + YAML)", callback=self._menu_save_callback)

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
            f"InteractiveDockingTeacher ready for station '{self.station_name}'. "
            "Use RViz Interactive Markers to align the gizmo."
        )

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
        """Right-click menu save trigger."""
        self.get_logger().info(f"🖱️ [Menu] Save Station triggered for '{self.station_name}'...")
        success, msg = self.save_station_template()
        self.get_logger().info(f"💾 Result: {msg}")

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
        m_text.pose.position.z = float(self.gizmo_pos[2] + self.box_size[2] / 2.0 + 0.15)
        m_text.scale.z = 0.12 # text size
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.color.a = 0.95
        m_text.text = f"{self.station_name} [{self.box_size[0]:.2f} x {self.box_size[1]:.2f} x {self.box_size[2]:.2f}m]"
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
        Crops point cloud inside gizmo box, transforms to Station Local frame,
        and saves template PCD + YAML config.
        """
        if self.cached_points.shape[0] == 0:
            return False, "No point cloud available to crop. Ensure /rtabmap/cloud_map is published."

        R_yaw = StationManager.rotation_matrix_from_yaw(self.gizmo_yaw)

        # 1. Transform entire cached points to Station Local Frame: p_local = R^T * (p_map - center)
        pts_local = StationManager.transform_to_station_local(
            self.cached_points, self.gizmo_pos, R_yaw
        )

        # 2. Crop by Oriented Bounding Box (OBB) in Local Frame
        # Gizmo box centered at local origin (0, 0, 0)
        half_x = self.box_size[0] / 2.0
        half_y = self.box_size[1] / 2.0
        half_z = self.box_size[2] / 2.0

        mask = (
            (np.abs(pts_local[:, 0]) <= half_x) &
            (np.abs(pts_local[:, 1]) <= half_y) &
            (np.abs(pts_local[:, 2]) <= half_z)
        )
        cropped_local = pts_local[mask]

        if cropped_local.shape[0] < 20:
            return False, f"Cropped points too sparse ({cropped_local.shape[0]} points). Expand box or verify cloud."

        # 3. Preprocess / Voxel Downsample
        filtered_local, _ = self.matcher.preprocess_cloud(cropped_local)

        # 4. Save via StationManager
        # Approach direction: Facing into the box front face from robot perspective
        # If gizmo arrow points towards robot (+X in local), robot approach yaw is opposite (+pi)
        approach_yaw = (self.gizmo_yaw + math.pi) % (2.0 * math.pi)

        success = self.station_manager.register_station(
            station_name=self.station_name,
            center=list(self.gizmo_pos),
            size=list(self.box_size),
            approach_yaw=approach_yaw,
            points_local=filtered_local,
            description=f"Taught station {self.station_name} via RViz Gizmo",
            standoff_distance=self.standoff_distance
        )

        if success:
            msg = (f"Successfully saved station '{self.station_name}' with "
                   f"{filtered_local.shape[0]} template points to docking_stations.yaml.")
            return True, msg
        else:
            return False, "Failed to write station configuration to disk."

    def _command_callback(self, msg: String):
        """Processes string commands (e.g. 'save <name>' or 'set_size <x> <y> <z>')."""
        cmd = msg.data.strip().split()
        if not cmd:
            return
        action = cmd[0].lower()
        if action == "save":
            if len(cmd) > 1:
                self.station_name = cmd[1]
            success, res = self.save_station_template()
            self.get_logger().info(f"[Command] Save: {res}")
        elif action == "set_name" and len(cmd) > 1:
            self.station_name = cmd[1]
            self.get_logger().info(f"Target station name updated to '{self.station_name}'")

    def _save_service_callback(self, request, response):
        """Trigger service callback."""
        success, message = self.save_station_template()
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
