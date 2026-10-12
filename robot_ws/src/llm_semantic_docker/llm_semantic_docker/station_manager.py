#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Station Manager for Precision Autonomous Docking System.
Manages docking stations configuration (docking_stations.yaml),
Station Local coordinate transformations, and Point Cloud (PCD) templates.
"""

import os
import yaml
import numpy as np
from typing import Dict, Any, Optional, Tuple, List


class StationManager:
    """Manages taught docking stations, templates, and geometric frames."""

    def __init__(self, config_path: Optional[str] = None, templates_dir: Optional[str] = None):
        src_pkg_root = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker"
        if os.path.exists(src_pkg_root):
            package_root = src_pkg_root
        else:
            try:
                from ament_index_python.packages import get_package_share_directory
                package_root = get_package_share_directory('llm_semantic_docker')
            except Exception:
                package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        if config_path is None:
            self.config_path = os.path.join(package_root, "config", "docking_stations.yaml")
        else:
            self.config_path = config_path

        if templates_dir is None:
            self.templates_dir = os.path.join(package_root, "templates")
        else:
            self.templates_dir = templates_dir

        os.makedirs(self.templates_dir, exist_ok=True)
        self.stations: Dict[str, Any] = {}
        self.load_stations()

    def load_stations(self) -> Dict[str, Any]:
        """Loads stations configuration from YAML."""
        if not os.path.exists(self.config_path):
            self.stations = {}
            return self.stations

        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                self.stations = data.get('stations', {})
        except Exception as e:
            print(f"[StationManager] Error loading {self.config_path}: {e}")
            self.stations = {}
        return self.stations

    def save_stations(self) -> bool:
        """Saves current stations dictionary to YAML."""
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        try:
            data = {'stations': self.stations}
            with open(self.config_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

            # Sync to install share directory if present
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('llm_semantic_docker')
                share_cfg = os.path.join(share_dir, "config", "docking_stations.yaml")
                if os.path.abspath(share_cfg) != os.path.abspath(self.config_path):
                    os.makedirs(os.path.dirname(share_cfg), exist_ok=True)
                    with open(share_cfg, 'w', encoding='utf-8') as sf:
                        yaml.dump(data, sf, default_flow_style=False, sort_keys=False, allow_unicode=True)
            except Exception:
                pass

            return True
        except Exception as e:
            print(f"[StationManager] Error saving {self.config_path}: {e}")
            return False

    def get_station(self, station_name: str) -> Optional[Dict[str, Any]]:
        """Retrieves a station configuration by name."""
        return self.stations.get(station_name)

    def get_all_stations(self) -> Dict[str, Any]:
        """Returns all registered stations."""
        return self.stations

    def list_stations(self) -> List[str]:
        """Lists all registered station names."""
        return list(self.stations.keys())

    @staticmethod
    def rotation_matrix_from_yaw(yaw_rad: float) -> np.ndarray:
        """Constructs a 3x3 rotation matrix for yaw (Z-axis rotation)."""
        c = np.cos(yaw_rad)
        s = np.sin(yaw_rad)
        return np.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    @staticmethod
    def transform_to_station_local(points_map: np.ndarray,
                                   center: np.ndarray,
                                   rotation_matrix: np.ndarray) -> np.ndarray:
        """
        Transforms points from Map coordinate frame to Station Local coordinate frame.
        p_local = R^T * (p_map - center)
        """
        if points_map.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float64)
        translated = points_map[:, :3] - center.reshape((1, 3))
        # (N, 3) @ R = (R^T @ translated.T).T
        points_local = translated @ rotation_matrix
        return points_local

    @staticmethod
    def transform_to_map(points_local: np.ndarray,
                         center: np.ndarray,
                         rotation_matrix: np.ndarray) -> np.ndarray:
        """
        Transforms points from Station Local coordinate frame back to Map coordinate frame.
        p_map = R * p_local + center
        """
        if points_local.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float64)
        rotated = points_local[:, :3] @ rotation_matrix.T
        points_map = rotated + center.reshape((1, 3))
        return points_map

    @staticmethod
    def calculate_standoff_pose(center: np.ndarray,
                                size: np.ndarray,
                                approach_yaw: float,
                                standoff_distance: float = 0.60) -> Tuple[float, float, float]:
        """
        Calculates the Standoff Map Pose (x, y, yaw) from the target front face.
        approach_yaw: Direction robot faces when looking at target (pointing from robot to target).
        Normal vector pointing outward from target toward robot: n_robot = [-cos(yaw), -sin(yaw), 0]
        Front Face = center + (depth/2) * n_robot
        Standoff Pose = Front Face + standoff_distance * n_robot
        Robot yaw at Standoff = approach_yaw
        """
        # Direction from robot to target
        nx = np.cos(approach_yaw)
        ny = np.sin(approach_yaw)
        
        # Depth along approach axis (assumed size[1] or size[0])
        half_depth = size[1] / 2.0 if len(size) > 1 else 0.15
        
        # Target front face position (closest to robot)
        front_x = center[0] - half_depth * nx
        front_y = center[1] - half_depth * ny
        
        # Standoff position (backed off by standoff_distance)
        standoff_x = front_x - standoff_distance * nx
        standoff_y = front_y - standoff_distance * ny
        
        return float(standoff_x), float(standoff_y), float(approach_yaw)

    def register_station(self,
                         station_name: str,
                         center: List[float],
                         size: List[float],
                         approach_yaw: float,
                         points_local: np.ndarray,
                         description: str = "",
                         standoff_distance: float = 0.60,
                         verification_thresholds: Optional[Dict[str, Any]] = None) -> bool:
        """
        Registers a new docking station, exports its local PCD template,
        and saves updated YAML config.
        """
        center_np = np.array(center, dtype=np.float64)
        size_np = np.array(size, dtype=np.float64)
        
        # 1. Calculate Standoff Pose
        sx, sy, syaw = self.calculate_standoff_pose(center_np, size_np, approach_yaw, standoff_distance)
        
        # 2. Save Local PCD Template
        pcd_filename = f"{station_name}_local.pcd"
        pcd_path = os.path.join(self.templates_dir, pcd_filename)
        self.save_pcd(pcd_path, points_local)
        
        rel_pcd_path = os.path.join("templates", pcd_filename)
        
        # 3. Default Verification Thresholds
        if verification_thresholds is None:
            verification_thresholds = {
                'min_fitness': 0.65,
                'max_rmse': 0.025,
                'max_allowable_dx': 0.15,
                'max_allowable_dy': 0.10,
                'max_allowable_dyaw_deg': 15.0,
                'required_stable_frames': 3
            }

        # 4. Update station record
        approach_normal = [-float(np.cos(approach_yaw)), -float(np.sin(approach_yaw)), 0.0]
        self.stations[station_name] = {
            'description': description or f"Taught station {station_name}",
            'frame': "map",
            'standoff': {
                'x': float(round(float(sx), 4)),
                'y': float(round(float(sy), 4)),
                'yaw_deg': float(round(float(np.degrees(syaw)), 2))
            },
            'target_face': {
                'center': [float(round(float(c), 4)) for c in center],
                'size': [float(round(float(s), 4)) for s in size],
                'normal': [float(round(float(n), 4)) for n in approach_normal]
            },
            'template': {
                'pcd_file': rel_pcd_path,
                'resolution_m': 0.01
            },
            'verification_thresholds': verification_thresholds
        }

        return self.save_stations()

    @staticmethod
    def save_pcd(filepath: str, points: np.ndarray) -> bool:
        """
        Saves Nx3 points array to standard PCD (ASCII) format.
        Compatible with Open3D and PCL without external dependencies.
        """
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            n_points = points.shape[0]
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                f"WIDTH {n_points}\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {n_points}\n"
                "DATA ascii\n"
            )
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(header)
                for pt in points[:, :3]:
                    f.write(f"{pt[0]:.5f} {pt[1]:.5f} {pt[2]:.5f}\n")
            return True
        except Exception as e:
            print(f"[StationManager] Failed to write PCD {filepath}: {e}")
            return False

    @staticmethod
    def load_pcd(filepath: str) -> np.ndarray:
        """
        Loads standard ASCII PCD file into Nx3 float64 numpy array.
        """
        if not os.path.exists(filepath):
            return np.empty((0, 3), dtype=np.float64)

        try:
            points = []
            is_data = False
            with open(filepath, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith("DATA ascii"):
                        is_data = True
                        continue
                    if is_data:
                        parts = line.split()
                        if len(parts) >= 3:
                            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            return np.array(points, dtype=np.float64)
        except Exception as e:
            print(f"[StationManager] Failed to read PCD {filepath}: {e}")
            return np.empty((0, 3), dtype=np.float64)
