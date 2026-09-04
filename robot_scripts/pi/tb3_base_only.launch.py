import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_bringup')
    model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    param_file = os.path.join(pkg_dir, 'param', f'{model}.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        # 1. State Publisher (URDF TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items()
        ),
        # 2. OpenCR Driver (Odometry, IMU, Motor Cmd) - NO LIDAR
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[param_file, {'namespace': namespace}],
            arguments=['-i', '/dev/ttyACM0'],
            output='screen'
        )
    ])
