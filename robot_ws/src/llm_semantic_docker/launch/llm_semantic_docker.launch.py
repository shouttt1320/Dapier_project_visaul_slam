#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[LLM Semantic Docker Integrated Launch]
Launches the full unified autonomous system:
1. LLM Agent Node (Qwen2.5-VL Multimodal Brain)
2. Task Manager Node (Autonomous Navigation & FSM State Machine)
3. Object Detector 3D Node (3D Plane Fitting & Visual Servoing)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('llm_semantic_docker')

    # Arguments
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/color/image_raw',
        description='Color image topic name'
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Depth image topic name'
    )
    cam_info_topic_arg = DeclareLaunchArgument(
        'cam_info_topic',
        default_value='/camera/color/camera_info',
        description='Camera info topic name'
    )
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='qwen2.5vl:3b',
        description='Ollama VLM model name'
    )
    gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='Whether to launch the Qwen Mission Control GUI'
    )

    # 1. LLM Agent Node (Qwen2.5-VL)
    llm_agent_node = Node(
        package='llm_semantic_docker',
        executable='llm_agent_node',
        name='llm_agent_node',
        output='screen',
        parameters=[{
            'ollama_url': 'http://127.0.0.1:11434/api/generate',
            'model_name': LaunchConfiguration('model_name'),
            'color_topic': LaunchConfiguration('color_topic'),
            'keep_alive': -1
        }]
    )

    # 2. Task Manager Node (FSM)
    task_manager_node = Node(
        package='llm_semantic_docker',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        parameters=[{}]
    )

    # 3. Object Detector 3D Node
    object_detector_3d_node = Node(
        package='llm_semantic_docker',
        executable='object_detector_3d_node',
        name='object_detector_3d_node',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'cam_info_topic': LaunchConfiguration('cam_info_topic'),
            'detector_type': 'qwen_vlm',
            'publish_rate_hz': 10.0
        }]
    )

    # 4. Qwen Mission Control GUI Node
    gui_node = Node(
        package='llm_semantic_docker',
        executable='qwen_semantic_gui_node',
        name='qwen_semantic_gui_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gui'))
    )

    return LaunchDescription([
        color_topic_arg,
        depth_topic_arg,
        cam_info_topic_arg,
        model_name_arg,
        gui_arg,
        llm_agent_node,
        task_manager_node,
        object_detector_3d_node,
        gui_node
    ])
