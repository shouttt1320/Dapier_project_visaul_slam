#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[LLM Navigation & Docking Integrated Master Launch]
==================================================
Unified Master Launch File for:
1. TurtleBot3 State Publisher (URDF + TF Tree)
2. PC-Side Compressed Image Decompression Bridge (Wi-Fi Optimized)
3. Static Camera-to-Optical Frame TF Publisher
4. RTAB-Map 3D Visual SLAM (Pure Localization with my_office_3d.db)
5. Nav2 Full Navigation Stack (Path Planning & Obstacle Avoidance)
6. Unified 3D RViz2 (Map, Costmaps, Paths, Robot Model, and 3D BBox Markers)
7. Qwen2.5-VL Multimodal Vision-Language Agent Node
8. 3D Object Detector & Plane Fitting Node
9. 8-State Task Manager FSM (Nav2 Global + Visual Servoing Docking)
10. Qwen Semantic Cockpit GUI Node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    vslam_pkg_dir = get_package_share_directory('turtlebot3_visual_slam')
    llm_pkg_dir = get_package_share_directory('llm_semantic_docker')
    tb3_bringup_pkg = get_package_share_directory('turtlebot3_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    rtabmap_params_file = os.path.join(vslam_pkg_dir, 'config', 'rtabmap_params.yaml')
    nav2_params_file = os.path.join(vslam_pkg_dir, 'config', 'nav2_vslam_params.yaml')

    database_path = LaunchConfiguration('database_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz = LaunchConfiguration('open_rviz')
    enable_gui = LaunchConfiguration('enable_gui')
    odom_topic = LaunchConfiguration('odom_topic')
    model_name = LaunchConfiguration('model_name')

    is_sim = context.perform_substitution(use_sim_time).lower() in ['true', '1']

    # Database resolution
    db_raw = context.perform_substitution(database_path)
    if not db_raw or db_raw in ['auto', '']:
        if is_sim:
            sim_db = os.path.join(os.path.expanduser('~'), 'Documents', 'Dapier', 'Project', 'simulation', 'simulation_3d.db')
            if not os.path.exists(sim_db):
                sim_db = os.path.join(os.path.expanduser('~'), 'Documents', 'Dapier', 'Project', 'simulation_3d.db')
            resolved_db_path = sim_db
        else:
            resolved_db_path = os.path.join(os.path.expanduser('~'), 'Documents', 'Dapier', 'Project', 'my_office_3d.db')
    else:
        resolved_db_path = db_raw

    # Camera topic selection (Real Robot uses PC-side decompressed stream)
    if is_sim:
        rgb_topic = '/camera/color/image_raw'
        depth_topic = '/camera/depth/image_raw'
        camera_info_topic = '/camera/color/camera_info'
    else:
        rgb_topic = '/camera/color/image_decompressed'
        depth_topic = '/camera/depth/image_decompressed'
        camera_info_topic = '/camera/color/decompressed/camera_info'

    nodes = []

    # 1. Local Robot State Publisher (Full URDF & TF tree for RViz & Costmaps)
    state_pub_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_pkg, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': ''}.items()
    )
    nodes.append(state_pub_cmd)

    # 2. PC-Side Zero-Lag Decompression & Sync Bridge (Real robot only)
    if not is_sim:
        bridge_node = Node(
            package='turtlebot3_visual_slam',
            executable='compressed_bridge_node',
            name='compressed_bridge_node',
            output='screen'
        )
        nodes.append(bridge_node)

    # 3. 3D Point Cloud Generator (Live cloud for obstacle avoidance and 3D visual SLAM)
    live_point_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='live_point_cloud_node',
        output='screen',
        parameters=[{
            'decimation': 1,
            'voxel_size': 0.0,
            'approx_sync': True,
            'queue_size': 15,
            'qos': 1,
            'qos_camera_info': 1,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('cloud', '/camera/depth/points')
        ]
    )
    nodes.append(live_point_cloud_node)

    # 4. Static TF (Real robot only)
    if not is_sim:
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            arguments=['--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                       '--frame-id', 'camera_link',
                       '--child-frame-id', 'camera_color_optical_frame']
        )
        nodes.append(static_tf_node)

    # 5. RTAB-Map in Pure 3D Visual Localization Mode
    rtabmap_localization_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'use_sim_time': use_sim_time,
                'database_path': resolved_db_path,
                'Mem/IncrementalMemory': 'false',      # Pure localization mode
                'Mem/InitWMWithAllNodes': 'true',       # Pre-load all nodes for global relocalization
                'RGBD/NeighborLinkRefining': 'true',
            }
        ],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('imu', '/imu'),
            ('odom', odom_topic),
            ('grid_map', '/map')
        ]
    )
    nodes.append(rtabmap_localization_node)

    # 6. Nav2 Core Navigation Stack
    nav2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )
    nodes.append(nav2_navigation_cmd)

    # 7. Unified RViz2 for 3D Visual Navigation
    rviz_config = os.path.join(vslam_pkg_dir, 'config', 'vslam_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(open_rviz)
    )
    nodes.append(rviz_node)

    # 8. Qwen2.5-VL Multimodal Vision-Language Agent Node
    llm_agent_node = Node(
        package='llm_semantic_docker',
        executable='llm_agent_node',
        name='llm_agent_node',
        output='screen',
        parameters=[{
            'ollama_url': 'http://127.0.0.1:11434/api/generate',
            'model_name': model_name,
            'color_topic': rgb_topic,
            'keep_alive': -1
        }]
    )
    nodes.append(llm_agent_node)

    # 9. 3D Object Detector & Plane Fitting Node (Legacy node - disabled in favor of taught gizmo & ICP docking)
    # object_detector_3d_node = Node(
    #     package='llm_semantic_docker',
    #     executable='object_detector_3d_node',
    #     name='object_detector_3d_node',
    #     output='screen',
    #     parameters=[{
    #         'color_topic': rgb_topic,
    #         'depth_topic': depth_topic,
    #         'cam_info_topic': camera_info_topic,
    #         'detector_type': 'qwen_vlm',
    #         'publish_rate_hz': 10.0
    #     }]
    # )
    # nodes.append(object_detector_3d_node)

    # 10. 8-State Task Manager FSM (Nav2 Global Navigation + 3D Visual Docking)
    task_manager_node = Node(
        package='llm_semantic_docker',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    nodes.append(task_manager_node)

    # 11. Dual-Mode Precision Docking Controller Engine
    docking_controller_node = Node(
        package='llm_semantic_docker',
        executable='precision_docking_controller_node',
        name='precision_docking_controller_node',
        output='screen',
        parameters=[{
            'enable_stamped_cmd_vel': True,
            'color_topic': rgb_topic,
            'depth_topic': depth_topic,
            'cam_info_topic': camera_info_topic,
            'marker_id': 1,
            'marker_size': 0.060,
            'crawl_speed': 0.030,
            'max_yaw_rate': 0.22,
            'final_clearance': 0.04,
            'docking_timeout_sec': 180.0,
            'crawl_timeout_sec': 35.0,
            'stall_timeout_sec': 20.0,
            'robot_front_radius': 0.145,
            'enable_bumper_docking': True,
            'marker_check_timeout_sec': 3.0,
            'carrot_lookahead': 0.22
        }]
    )
    nodes.append(docking_controller_node)

    # 11.1 Floor Object Approaching Controller Engine
    object_approach_node = Node(
        package='llm_semantic_docker',
        executable='object_approaching_controller_node',
        name='object_approaching_controller_node',
        output='screen',
        parameters=[{
            'color_topic': rgb_topic,
            'depth_topic': depth_topic,
            'cam_info_topic': camera_info_topic,
        }]
    )
    nodes.append(object_approach_node)

    # 12. Qwen Semantic Cockpit Mission Control GUI
    gui_node = Node(
        package='llm_semantic_docker',
        executable='qwen_semantic_gui_node',
        name='qwen_semantic_gui_node',
        output='screen',
        condition=IfCondition(enable_gui)
    )
    nodes.append(gui_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='auto',
            description='Path to RTAB-Map .db database file'
        ),
        DeclareLaunchArgument(
            'open_rviz',
            default_value='true',
            description='Launch unified RViz2 visualization'
        ),
        DeclareLaunchArgument(
            'enable_gui',
            default_value='true',
            description='Launch Qwen Semantic Mission Control GUI'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic name'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='qwen2.5vl:3b',
            description='Ollama VLM model name'
        ),
        OpaqueFunction(function=launch_setup)
    ])
