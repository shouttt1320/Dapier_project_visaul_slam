import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('turtlebot3_visual_slam')
    params_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_viz = LaunchConfiguration('use_viz')
    open_rviz = LaunchConfiguration('open_rviz')
    odom_topic = LaunchConfiguration('odom_topic')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    is_sim = context.perform_substitution(use_sim_time).lower() in ['true', '1']

    db_raw = context.perform_substitution(database_path)
    if not db_raw or db_raw in ['auto', '']:
        if is_sim:
            resolved_db_path = os.path.join(os.path.expanduser('~'), 'Documents', 'Dapier', 'Project', 'simulation_3d.db')
        else:
            resolved_db_path = os.path.join(os.path.expanduser('~'), 'Documents', 'Dapier', 'Project', 'my_office_3d.db')
    else:
        resolved_db_path = db_raw

    if is_sim:
        rgb_topic = '/camera/color/image_raw'
        depth_topic = '/camera/depth/image_raw'
        camera_info_topic = '/camera/color/camera_info'
    else:
        rgb_topic = '/camera/color/image_decompressed'
        depth_topic = '/camera/depth/image_decompressed'
        camera_info_topic = '/camera/color/decompressed/camera_info'

    nodes = []

    # 1. PC-Side Zero-Lag Decompression Bridge (Real robot only)
    if not is_sim:
        bridge_node = Node(
            package='turtlebot3_visual_slam',
            executable='compressed_bridge_node',
            name='compressed_bridge_node',
            output='screen'
        )
        nodes.append(bridge_node)

    # 2. 3D Point Cloud Generator (Runs on both Real Robot & Simulation for true optical projection)
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
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('cloud', '/camera/depth/points')
        ]
    )
    nodes.append(live_point_cloud_node)

    # 3. Static Transforms (Real robot only; Simulation launch already defines these)
    if not is_sim:
        base_to_camera_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['--x', '0.08', '--y', '0.0', '--z', '0.134',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'camera_link'],
            condition=IfCondition(publish_camera_tf)
        )
        nodes.append(base_to_camera_tf)

        camera_to_optical_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            arguments=['--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                       '--frame-id', 'camera_link',
                       '--child-frame-id', 'camera_color_optical_frame']
        )
        nodes.append(camera_to_optical_tf)

    # 4. RTAB-Map SLAM Core Node (RGB-D + IMU + Wheel Odom)
    delete_db_val = context.perform_substitution(delete_db_on_start).lower() in ['true', '1']
    rtabmap_args = ['--delete_db_on_start'] if delete_db_val else []

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=rtabmap_args,
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'database_path': resolved_db_path
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
    nodes.append(rtabmap_slam_node)

    # Optional RTAB-Map 3D Visualizer GUI
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'database_path': resolved_db_path
            }
        ],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('imu', '/imu'),
            ('odom', odom_topic)
        ],
        condition=IfCondition(use_viz)
    )
    nodes.append(rtabmap_viz_node)

    # 5. RViz2 for Visualizing Map, Camera, Markers, and 3D Cloud
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
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_visual_slam')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument('use_viz', default_value='false', description='Launch RTAB-Map GUI visualizer if true'),
        DeclareLaunchArgument('open_rviz', default_value='true', description='Launch RViz2 for visualization if true'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic (/odom or /odometry/filtered)'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true', description='Broadcast static TF if not in URDF'),
        DeclareLaunchArgument('database_path', default_value='auto', description='Path to rtabmap database file (auto -> simulation_3d.db if use_sim_time else my_office_3d.db)'),
        DeclareLaunchArgument('delete_db_on_start', default_value='false', description='Delete existing DB and start fresh if true'),
        OpaqueFunction(function=launch_setup)
    ])
