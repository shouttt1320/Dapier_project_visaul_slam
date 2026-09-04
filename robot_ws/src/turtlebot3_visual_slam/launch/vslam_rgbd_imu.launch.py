import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_visual_slam')
    params_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_viz = LaunchConfiguration('use_viz', default='false')
    open_rviz = LaunchConfiguration('open_rviz', default='true')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf', default='true')

    rviz_config = os.path.join(pkg_dir, 'config', 'vslam_rviz.rviz')

    # 1. PC-Side Zero-Lag Decompression & Exact-Sync Bridge
    # Subscribes to light compressed Wi-Fi stream and republishes matched uncompressed frames
    bridge_node = Node(
        package='turtlebot3_visual_slam',
        executable='compressed_bridge_node',
        name='compressed_bridge_node',
        output='screen'
    )

    # 2. PC-Side 3D Point Cloud Generator (Offloads heavy computation from Raspberry Pi to PC)
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

    # Optional Static Transform if not provided by robot URDF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['--x', '0.08', '--y', '0.0', '--z', '0.15',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'camera_link'],
        condition=IfCondition(publish_camera_tf)
    )

    # 3. RTAB-Map SLAM Core Node (RGB-D + IMU + Wheel Odom)
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/camera/color/image_decompressed'),
            ('depth/image', '/camera/depth/image_decompressed'),
            ('rgb/camera_info', '/camera/color/decompressed/camera_info'),
            ('imu', '/imu'),
            ('odom', odom_topic),
            ('grid_map', '/map')
        ],
        arguments=['-d']  # Delete previous temporary database on startup
    )

    # Optional RTAB-Map 3D Visualizer GUI
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/camera/color/image_decompressed'),
            ('depth/image', '/camera/depth/image_decompressed'),
            ('rgb/camera_info', '/camera/color/decompressed/camera_info'),
            ('imu', '/imu'),
            ('odom', odom_topic)
        ],
        condition=IfCondition(use_viz)
    )

    # 4. RViz2 for Visualizing Map, Camera, Markers, and 3D Cloud
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
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument('use_viz', default_value='false', description='Launch RTAB-Map GUI visualizer if true'),
        DeclareLaunchArgument('open_rviz', default_value='true', description='Launch RViz2 for visualization if true'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic (/odom or /odometry/filtered)'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true', description='Broadcast static TF if not in URDF'),
        bridge_node,
        live_point_cloud_node,
        static_tf_node,
        rtabmap_slam_node,
        rtabmap_viz_node,
        rviz_node
    ])
