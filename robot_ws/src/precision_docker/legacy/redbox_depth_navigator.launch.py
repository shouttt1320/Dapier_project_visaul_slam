import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('redbox_navigator')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    param_file = os.path.join(pkg_dir, 'config', 'redbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_nav2 = LaunchConfiguration('launch_nav2', default='true')

    nav2_param_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # 1. Nav2 Autonomous Navigation Stack (Planner, Controller, BT Navigator)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_param_file
        }.items(),
        condition=IfCondition(launch_nav2)
    )

    # 2. Red Box Vision Detector Node
    detector_node = Node(
        package='redbox_navigator',
        executable='redbox_detector',
        name='redbox_detector_node',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    # 3. Pure Depth-based 3D Coordinate Node
    coordinate_node = Node(
        package='redbox_navigator',
        executable='target_depth_coordinate',
        name='target_depth_coordinate_node',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    # 4. Autonomous Mission Controller FSM
    mission_node = Node(
        package='redbox_navigator',
        executable='mission_controller',
        name='redbox_mission_controller',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument('launch_nav2', default_value='true', description='Automatically launch Nav2 stack if true'),
        nav2_cmd,
        detector_node,
        coordinate_node,
        mission_node
    ])
