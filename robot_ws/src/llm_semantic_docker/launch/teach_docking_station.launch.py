#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file for RViz 3D Interactive Docking Station Teaching.
Loads the stored 3D visual SLAM map DB, launches RViz with Interactive Markers,
and starts the interactive_docking_teacher_node for one-click station registration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_docker = get_package_share_directory('llm_semantic_docker')
    pkg_vslam = get_package_share_directory('turtlebot3_visual_slam')

    station_name = LaunchConfiguration('station_name').perform(context)
    waypoint_name = LaunchConfiguration('waypoint_name').perform(context)
    box_x = float(LaunchConfiguration('box_size_x').perform(context))
    box_y = float(LaunchConfiguration('box_size_y').perform(context))
    box_z = float(LaunchConfiguration('box_size_z').perform(context))
    db_path = LaunchConfiguration('database_path').perform(context)
    start_map_server = LaunchConfiguration('start_map_server').perform(context).lower() in ['true', '1']

    rviz_config = os.path.join(pkg_vslam, 'config', 'vslam_rviz.rviz')

    nodes = []

    # 1. Map Publisher Node (If standalone teaching mode requested)
    if start_map_server:
        # Static TF: map -> odom -> base_footprint (for standalone visualization)
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']
        ))

        # Standalone Map Server: Instantly loads and publishes 3D point cloud & 2D occupancy grid
        db_dir = os.path.dirname(db_path)
        db_stem = os.path.splitext(os.path.basename(db_path))[0]
        cloud_ply = os.path.join(db_dir, f'{db_stem}_cloud.ply')
        if not os.path.exists(cloud_ply):
            legacy_ply = os.path.join(db_dir, 'my_office_cloud_cloud.ply')
            if os.path.exists(legacy_ply):
                cloud_ply = legacy_ply
        map_yaml = os.path.join(db_dir, 'my_office_map.yaml')

        nodes.append(Node(
            package='llm_semantic_docker',
            executable='standalone_map_server',
            name='standalone_map_server',
            output='screen',
            parameters=[{
                'cloud_file': cloud_ply,
                'map_yaml': map_yaml,
                'frame_id': 'map'
            }]
        ))

    # 2. Interactive Docking Teacher Node
    nodes.append(Node(
        package='llm_semantic_docker',
        executable='interactive_docking_teacher_node',
        name='interactive_docking_teacher',
        output='screen',
        parameters=[{
            'station_name': station_name,
            'waypoint_name': waypoint_name,
            'standoff_distance': 0.60,
            'box_size_x': box_x,
            'box_size_y': box_y,
            'box_size_z': box_z,
            'map_frame': 'map'
        }]
    ))

    # 3. RViz2 with Interactive Markers & 3D Cloud display
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_teaching',
        arguments=['-d', rviz_config],
        output='screen'
    ))

    return nodes


def generate_launch_description():
    project_root = os.path.expanduser('~/Documents/Dapier/Project')
    default_db = os.path.join(project_root, 'my_office_3d.db')

    return LaunchDescription([
        DeclareLaunchArgument(
            'station_name',
            default_value='box_01',
            description='Name of the station to teach (e.g. box_01, desk_01)'
        ),
        DeclareLaunchArgument(
            'waypoint_name',
            default_value='waypoint_01',
            description='Name of the waypoint to teach (e.g. my_desk, entrance)'
        ),
        DeclareLaunchArgument(
            'box_size_x',
            default_value='0.35',
            description='Target Bounding Box width X (meters)'
        ),
        DeclareLaunchArgument(
            'box_size_y',
            default_value='0.40',
            description='Target Bounding Box depth Y (meters)'
        ),
        DeclareLaunchArgument(
            'box_size_z',
            default_value='0.25',
            description='Target Bounding Box height Z (meters)'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=default_db,
            description='Path to rtabmap 3D database file'
        ),
        DeclareLaunchArgument(
            'start_map_server',
            default_value='true',
            description='Whether to start rtabmap DB loader if not already running'
        ),
        OpaqueFunction(function=launch_setup)
    ])
