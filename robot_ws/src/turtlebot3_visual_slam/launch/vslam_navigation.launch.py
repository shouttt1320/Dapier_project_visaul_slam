import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_visual_slam')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(os.path.expanduser('~'), 'Documents', 'Dapier', 'Project', 'my_office_map.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_vslam_params.yaml')

    map_yaml_file = LaunchConfiguration('map', default=default_map)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    # 1. PC-Side Zero-Lag Decompression & Sync Bridge
    bridge_node = Node(
        package='turtlebot3_visual_slam',
        executable='compressed_bridge_node',
        name='compressed_bridge_node',
        output='screen'
    )

    # 2. PC-Side 3D Point Cloud Generator (Obstacle avoidance for Nav2 Costmap)
    live_point_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='live_point_cloud_node',
        output='screen',
        parameters=[{
            'decimation': 1,
            'voxel_size': 0.0,
            'approx_sync': False,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_decompressed'),
            ('depth/image', '/camera/depth/image_decompressed'),
            ('rgb/camera_info', '/camera/color/decompressed/camera_info'),
            ('cloud', '/camera/depth/points')
        ]
    )

    # 3. Static TF (camera_link -> camera_color_optical_frame)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=['--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                   '--frame-id', 'camera_link',
                   '--child-frame-id', 'camera_color_optical_frame']
    )

    # 4. Depth to 2D LaserScan for AMCL Localization (Astra RGB-D depth slice)
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.45,
            'range_max': 5.0,
            'scan_height': 5,
            'output_frame': 'camera_link'
        }],
        remappings=[
            ('depth', '/camera/depth/image_decompressed'),
            ('depth_camera_info', '/camera/color/decompressed/camera_info'),
            ('scan', '/scan')
        ]
    )

    # 5. Nav2 Standard Bringup (Map Server + AMCL + Controller + Planner + BT Navigator)
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )

    # 6. Unified RViz2 for Navigation
    rviz_config = os.path.join(pkg_dir, 'config', 'vslam_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='vslam_rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to map yaml file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('open_rviz', default_value='true', description='Open Unified RViz'),
        bridge_node,
        live_point_cloud_node,
        static_tf_node,
        depth_to_laserscan_node,
        nav2_bringup_cmd,
        rviz_node
    ])
