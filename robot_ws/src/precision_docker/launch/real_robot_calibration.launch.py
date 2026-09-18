#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Arguments
    show_window = LaunchConfiguration('show_window', default='false')
    launch_os30a = LaunchConfiguration('launch_os30a', default='true')
    cam_pitch = LaunchConfiguration('cam_pitch', default='1.1344')  # 65 degrees
    table_z_min = LaunchConfiguration('table_z_min', default='0.090')
    table_z_max = LaunchConfiguration('table_z_max', default='0.125')

    declare_show_window = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='Display local OpenCV GUI window if true (run on PC)'
    )
    declare_launch_os30a = DeclareLaunchArgument(
        'launch_os30a',
        default_value='true',
        description='Automatically launch OS30A depth camera driver if true'
    )
    declare_cam_pitch = DeclareLaunchArgument(
        'cam_pitch',
        default_value='1.1344',
        description='OS30A camera pitch angle in radians (default 65 deg = 1.1344 rad)'
    )
    declare_table_z_min = DeclareLaunchArgument(
        'table_z_min',
        default_value='0.090',
        description='Minimum desk surface height in base_footprint frame (m)'
    )
    declare_table_z_max = DeclareLaunchArgument(
        'table_z_max',
        default_value='0.125',
        description='Maximum desk surface height in base_footprint frame (m)'
    )

    # 1. Include OS30A Camera Driver
    try:
        os30a_launch_dir = os.path.join(get_package_share_directory('depth_ydlidar_os30a'), 'launch', 'os30a_docking.launch.py')
        os30a_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os30a_launch_dir),
            condition=IfCondition(launch_os30a)
        )
    except Exception:
        os30a_launch = None

    # 2. Micro-switch Contact Bumper Node
    bumper_node = Node(
        package='precision_docker',
        executable='bumper_sensor',
        name='bumper_sensor_node',
        output='screen',
        parameters=[{
            'pin': 23,
            'poll_rate_hz': 50.0,
            'debounce_ms': 30.0,
            'active_low': True,
        }]
    )

    # 3. Precision Approacher Node in Static Calibration Mode (Passive, zero motor cmd_vel)
    approacher_node = Node(
        package='precision_docker',
        executable='precision_approacher',
        name='precision_approacher_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'show_window': show_window,
            'autostart': False,
            'calibration_mode': True,  # Completely halts motors while streaming HUD & markers
            'depth_topic': '/apc/depth/image_raw',
            'color_topic': '/apc/left/image_color',
            'cam_info_topic': '/apc/depth/camera_info',
            'bumper_topic': '/robot/bumper/contact',
            'cam_pitch': cam_pitch,
            'cam_x_base': 0.050,
            'cam_z_base': 0.400,
            'table_z_min': table_z_min,
            'table_z_max': table_z_max,
            'min_cliff_drop': 0.045,
            'floor_z_max': 0.055,
            'publish_debug_img': True,
            'debug_img_stride': 3,
            'ransac_iterations': 20,
            'min_crawl_speed': 0.025,
            'reference_x': 0.100,
            'target_clearance': 0.05,
        }]
    )

    # 4. Static TF: base_footprint -> base_link (if not running full robot bringup)
    tf_base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_link_calib',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    )

    actions = [
        declare_show_window,
        declare_launch_os30a,
        declare_cam_pitch,
        declare_table_z_min,
        declare_table_z_max,
        bumper_node,
        approacher_node,
        tf_base_to_link,
    ]
    if os30a_launch is not None:
        actions.insert(5, os30a_launch)

    return LaunchDescription(actions)
