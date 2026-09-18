#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch Configurations
    color_topic = LaunchConfiguration('color_topic', default='/camera/color/image_raw')
    cam_info_topic = LaunchConfiguration('cam_info_topic', default='/camera/color/camera_info')
    marker_id = LaunchConfiguration('marker_id', default='1')
    marker_size = LaunchConfiguration('marker_size', default='0.060')
    calibration_mode = LaunchConfiguration('calibration_mode', default='false')
    show_window = LaunchConfiguration('show_window', default='false')
    autostart = LaunchConfiguration('autostart', default='false')

    # Arguments
    declare_color_topic = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/color/image_raw',
        description='Astra S RGB color topic (/camera/color/image_raw on Pi or /camera/color/image_decompressed on PC)'
    )
    declare_cam_info_topic = DeclareLaunchArgument(
        'cam_info_topic',
        default_value='/camera/color/camera_info',
        description='Astra S RGB CameraInfo topic (/camera/color/camera_info on Pi)'
    )
    declare_marker_id = DeclareLaunchArgument(
        'marker_id',
        default_value='1',
        description='Target ArUco marker ID (DICT_4X4_50)'
    )
    declare_marker_size = DeclareLaunchArgument(
        'marker_size',
        default_value='0.060',
        description='Marker side length in meters (default 0.06m = 60mm)'
    )
    declare_calibration_mode = DeclareLaunchArgument(
        'calibration_mode',
        default_value='false',
        description='If true, runs in passive calibration mode (motors halted, active HUD & detection)'
    )
    declare_show_window = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='Display local OpenCV visualizer window if true (run on PC)'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='If true, starts docking immediately on launch instead of waiting in NAV2_READY'
    )

    # 1. Micro-switch Contact Bumper Node (Runs on Raspberry Pi 4 GPIO Pin 16 / BCM 23)
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

    # 2. Precision Marker Approacher Node
    approacher_node = Node(
        package='precision_docker',
        executable='precision_approacher',
        name='precision_approacher_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'docking_mode': 'marker',
            'marker_id': marker_id,
            'marker_size': marker_size,
            'color_topic': color_topic,
            'cam_info_topic': cam_info_topic,
            'bumper_topic': '/robot/bumper/contact',
            'calibration_mode': calibration_mode,
            'show_window': show_window,
            'autostart': autostart,
            'target_clearance': 0.05,
            'reference_x': 0.100,
            'carrot_lookahead': 0.22,
            'max_linear_speed': 0.06,
            'crawl_linear_speed': 0.025,
            'max_angular_speed': 0.22,
            'publish_debug_img': True,
            'debug_img_stride': 3,
            'max_retries': 3,
            'backup_retry_distance': 0.25,
            'astra_x_base': 0.080,
            'astra_y_base': 0.015,
            'astra_z_base': 0.134,
            'astra_pitch': 0.0,
        }]
    )

    return LaunchDescription([
        declare_color_topic,
        declare_cam_info_topic,
        declare_marker_id,
        declare_marker_size,
        declare_calibration_mode,
        declare_show_window,
        declare_autostart,
        bumper_node,
        approacher_node,
    ])
