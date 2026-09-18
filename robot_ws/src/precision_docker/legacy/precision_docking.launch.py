import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    show_window = LaunchConfiguration('show_window', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_show_window = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Open real-time OpenCV GUI display window if true'
    )

    docking_timeout_sec = LaunchConfiguration('docking_timeout_sec', default='35.0')
    declare_docking_timeout = DeclareLaunchArgument(
        'docking_timeout_sec',
        default_value='35.0',
        description='Timeout in seconds for docking routine before retreating and aborting to Nav2 mode'
    )

    autostart = LaunchConfiguration('autostart', default='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='If true, begin docking routine immediately on startup'
    )

    docking_node = Node(
        package='redbox_navigator',
        executable='precision_approacher',
        name='precision_approacher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'staging_x': 1.10,
            'staging_y': 0.40,
            'staging_yaw': 0.0,
            'target_clearance': 0.05,
            'reference_x': 0.100,
            'skip_nav2_if_close': True,
            'show_window': show_window,
            'docking_timeout_sec': docking_timeout_sec,
            'autostart': autostart,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_show_window,
        declare_docking_timeout,
        declare_autostart,
        docking_node
    ])
