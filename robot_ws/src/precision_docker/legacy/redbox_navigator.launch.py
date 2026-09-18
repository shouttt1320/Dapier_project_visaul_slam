import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('redbox_navigator')
    param_file = os.path.join(pkg_dir, 'config', 'redbox_params.yaml')

    detector_node = Node(
        package='redbox_navigator',
        executable='redbox_detector',
        name='redbox_detector_node',
        output='screen',
        parameters=[param_file]
    )

    coordinate_node = Node(
        package='redbox_navigator',
        executable='target_coordinate',
        name='target_coordinate_node',
        output='screen',
        parameters=[param_file]
    )

    mission_node = Node(
        package='redbox_navigator',
        executable='mission_controller',
        name='redbox_mission_controller',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        detector_node,
        coordinate_node,
        mission_node
    ])
