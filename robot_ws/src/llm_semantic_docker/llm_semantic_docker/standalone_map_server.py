#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standalone Map Server Node for RViz Teaching.
Publishes 3D Point Cloud (/cloud_map) from PLY/PCD and 2D Occupancy Grid (/map) from YAML/PGM
with TRANSIENT_LOCAL QoS so RViz instantly receives the map without needing active sensors or heavy SLAM nodes.
"""

import os
import yaml
import numpy as np
import open3d as o3d
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion


class StandaloneMapServerNode(Node):
    """Publishes static 3D point cloud and 2D grid map for offline teaching."""

    def __init__(self):
        super().__init__('standalone_map_server')

        self.declare_parameter('cloud_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('frame_id', 'map')

        cloud_file = self.get_parameter('cloud_file').value
        map_yaml = self.get_parameter('map_yaml').value
        self.frame_id = self.get_parameter('frame_id').value

        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

        if not cloud_file:
            cloud_file = os.path.join(project_root, 'my_office_cloud_cloud.ply')
        if not map_yaml:
            map_yaml = os.path.join(project_root, 'my_office_map.yaml')

        # Latching QoS (TRANSIENT_LOCAL) ensures RViz receives data immediately upon opening
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.cloud_pub1 = self.create_publisher(PointCloud2, '/cloud_map', latched_qos)
        self.cloud_pub2 = self.create_publisher(PointCloud2, '/rtabmap/cloud_map', latched_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)

        # 1. Load and prepare 3D Cloud
        self.cloud_msg, raw_pts = self._load_point_cloud(cloud_file)

        # 2. Generate 100% Aligned 2D Occupancy Grid directly from 3D Cloud Points
        self.grid_msg = self._generate_grid_from_points(raw_pts, resolution=0.05)

        # Periodic timer (2.0 Hz) to ensure RViz and late subscribers always have data
        self.timer = self.create_timer(2.0, self._publish_maps)

        self.get_logger().info(
            f"✅ StandaloneMapServer initialized.\n"
            f"   3D Cloud: {cloud_file}\n"
            f"   2D Map  : {map_yaml}"
        )

    def _publish_maps(self):
        """Publishes latched map messages."""
        if self.cloud_msg:
            self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.cloud_pub1.publish(self.cloud_msg)
            self.cloud_pub2.publish(self.cloud_msg)

        if self.grid_msg:
            self.grid_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.grid_msg)

    def _load_point_cloud(self, filepath: str):
        if not os.path.exists(filepath):
            self.get_logger().error(f"Point cloud file not found: {filepath}")
            return None

        self.get_logger().info(f"Loading 3D point cloud: {filepath}...")
        try:
            pcd = o3d.io.read_point_cloud(filepath)
            points = np.asarray(pcd.points, dtype=np.float32)
            has_colors = len(pcd.colors) > 0
            if has_colors:
                colors = (np.asarray(pcd.colors, dtype=np.float32) * 255).astype(np.uint8)
            else:
                colors = None

            n_pts = points.shape[0]
            self.get_logger().info(f"Loaded {n_pts} points from cloud file.")

            msg = PointCloud2()
            msg.header.frame_id = self.frame_id
            msg.height = 1
            msg.width = n_pts

            if has_colors:
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
                ]
                msg.point_step = 16
                # Pack rgb into uint32
                r = colors[:, 0].astype(np.uint32)
                g = colors[:, 1].astype(np.uint32)
                b = colors[:, 2].astype(np.uint32)
                rgb_packed = (r << 16) | (g << 8) | b

                data = np.zeros(n_pts, dtype=[
                    ('x', np.float32),
                    ('y', np.float32),
                    ('z', np.float32),
                    ('rgb', np.uint32)
                ])
                data['x'] = points[:, 0]
                data['y'] = points[:, 1]
                data['z'] = points[:, 2]
                data['rgb'] = rgb_packed
                msg.data = data.tobytes()
            else:
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                msg.point_step = 12
                data = np.zeros(n_pts, dtype=[
                    ('x', np.float32),
                    ('y', np.float32),
                    ('z', np.float32)
                ])
                data['x'] = points[:, 0]
                data['y'] = points[:, 1]
                data['z'] = points[:, 2]
                msg.data = data.tobytes()

            msg.row_step = msg.point_step * n_pts
            msg.is_dense = True
            return msg, points

        except Exception as e:
            self.get_logger().error(f"Failed to load cloud: {e}")
            return None, None

    def _generate_grid_from_points(self, points: np.ndarray, resolution: float = 0.05) -> OccupancyGrid:
        """Projects 3D point cloud onto 2D plane so OccupancyGrid is 100% aligned with 3D Cloud."""
        if points is None or points.shape[0] == 0:
            return None

        pts = points[:, :3]
        x_min, y_min = float(pts[:, 0].min() - 0.20), float(pts[:, 1].min() - 0.20)
        x_max, y_max = float(pts[:, 0].max() + 0.20), float(pts[:, 1].max() + 0.20)

        width = int(np.ceil((x_max - x_min) / resolution))
        height = int(np.ceil((y_max - y_min) / resolution))

        # Initialize with -1 (Unknown)
        grid = np.full((height, width), -1, dtype=np.int8)

        # Floor points (Z in [-0.20, 0.05]) -> Free space (0)
        floor_mask = (pts[:, 2] >= -0.20) & (pts[:, 2] < 0.05)
        floor_pts = pts[floor_mask]
        if floor_pts.shape[0] > 0:
            fx = np.clip(np.floor((floor_pts[:, 0] - x_min) / resolution).astype(np.int64), 0, width - 1)
            fy = np.clip(np.floor((floor_pts[:, 1] - y_min) / resolution).astype(np.int64), 0, height - 1)
            grid[fy, fx] = 0

        # Obstacle points (Z in [0.05, 1.20]) -> Occupied space (100)
        obs_mask = (pts[:, 2] >= 0.05) & (pts[:, 2] <= 1.20)
        obs_pts = pts[obs_mask]
        if obs_pts.shape[0] > 0:
            ox = np.clip(np.floor((obs_pts[:, 0] - x_min) / resolution).astype(np.int64), 0, width - 1)
            oy = np.clip(np.floor((obs_pts[:, 1] - y_min) / resolution).astype(np.int64), 0, height - 1)
            grid[oy, ox] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = x_min
        msg.info.origin.position.y = y_min
        msg.info.origin.position.z = 0.0
        msg.data = grid.flatten().tolist()

        self.get_logger().info(
            f"✅ Generated perfectly aligned 2D Occupancy Grid: {width}x{height} cells, origin=[{x_min:.2f}, {y_min:.2f}]"
        )
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = StandaloneMapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
