import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('turtlebot3_visual_slam')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    rtabmap_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_vslam_params.yaml')

    database_path = LaunchConfiguration('database_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz = LaunchConfiguration('open_rviz')
    odom_topic = LaunchConfiguration('odom_topic')
    launch_gui = LaunchConfiguration('launch_gui')

    is_sim = context.perform_substitution(use_sim_time).lower() in ['true', '1']

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

    if is_sim:
        rgb_topic = '/camera/color/image_raw'
        depth_topic = '/camera/depth/image_raw'
        camera_info_topic = '/camera/color/camera_info'
    else:
        rgb_topic = '/camera/color/image_decompressed'
        depth_topic = '/camera/depth/image_decompressed'
        camera_info_topic = '/camera/color/decompressed/camera_info'

    nodes = []

    # 0. Local Robot State Publisher (Full URDF & TF tree for RViz & Costmaps)
    tb3_bringup_pkg = get_package_share_directory('turtlebot3_bringup')
    state_pub_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_pkg, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': ''}.items()
    )
    nodes.append(state_pub_cmd)

    # 1. PC-Side Zero-Lag Decompression & Sync Bridge (Real robot only)
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

    # 3. Static TF (Real robot only; Simulation launch already defines these)
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

    # 4. RTAB-Map in Pure 3D Visual Localization Mode
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

    # 5. Nav2 Core Navigation Stack
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

    # 6. Unified RViz2 for 3D Visual Navigation
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
    # 7. RedBox Mission Control GUI (Mode Switcher + Real-Time Video Viewport)
    gui_node = Node(
        package='precision_docker',
        executable='docking_gui',
        name='docking_gui_node',
        condition=IfCondition(launch_gui),
        output='screen'
    )
    nodes.append(gui_node)

    # 8. Precision Close-Docking Node (High-Performance ArUco Visual Servoing on PC)
    precision_node = Node(
        package='precision_docker',
        executable='precision_approacher',
        name='precision_approacher_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'color_topic': rgb_topic,
            'depth_topic': depth_topic,
            'cam_info_topic': camera_info_topic,
            'docking_mode': 'marker',
            'enable_stamped_cmd_vel': True,
            'docking_timeout_sec': 75.0,
            'min_crawl_speed': 0.060
        }],
        output='screen'
    )
    nodes.append(precision_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value='auto', description='Path to RTAB-Map 3D database file (auto -> simulation_3d.db if use_sim_time else my_office_3d.db)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('open_rviz', default_value='true', description='Open Unified RViz'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic (/odom or /odometry/filtered)'),
        DeclareLaunchArgument('launch_gui', default_value='true', description='Open Mission Control GUI Dashboard'),
        OpaqueFunction(function=launch_setup)
    ])
