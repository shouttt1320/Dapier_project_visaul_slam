import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vslam_pkg = get_package_share_directory('turtlebot3_visual_slam')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(vslam_pkg, 'worlds', 'redbox_sim_world.world')
    model_dir = os.path.join(vslam_pkg, 'models')
    model_sdf = os.path.join(model_dir, 'turtlebot3_waffle_pi_os30a', 'model.sdf')

    # Environment variables for Gazebo model resources
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    tb3_model_path = os.path.join(tb3_gazebo_pkg, 'models')
    new_resource_path = f"{model_dir}:{tb3_model_path}:{existing_resource_path}"

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )

    # 1. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r {world_file}"}.items(),
    )

    # 2. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_sdf,
            '-name', 'turtlebot3_waffle_pi_os30a',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # 3. ROS-Gazebo Bridge (Clock, Odom, IMU, CmdVel, JointStates, Astra S, OS30A, Bumper)
    core_bridge_params = os.path.join(vslam_pkg, 'config', 'gazebo_bridge.yaml')
    core_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='core_parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={core_bridge_params}'
        ],
        output='screen'
    )

    # 3.1 Camera Mode Manager (Single-Camera Time-Division: Astra S for Nav2, OS30A for Docking)
    astra_bridge_params = os.path.join(vslam_pkg, 'config', 'gazebo_astra_bridge.yaml')
    os30a_bridge_params = os.path.join(vslam_pkg, 'config', 'gazebo_os30a_bridge.yaml')
    camera_mode_manager = Node(
        package='redbox_navigator',
        executable='camera_mode_manager',
        name='camera_mode_manager_node',
        output='screen',
        parameters=[{
            'initial_mode': 'NAV2',
            'astra_config': astra_bridge_params,
            'os30a_config': os30a_bridge_params,
        }]
    )

    # 4. Static TFs for Base, IMU, and Camera
    tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.01',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'base_link']
    )

    tf_imu_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_tf_pub',
        arguments=['--x', '-0.032', '--y', '0.0', '--z', '0.078',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'imu_link']
    )

    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf_pub',
        arguments=['--x', '0.08', '--y', '0.0', '--z', '0.134',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'camera_link']
    )

    tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                   '--frame-id', 'camera_link',
                   '--child-frame-id', 'camera_color_optical_frame']
    )

    # 5. Static TFs for OS30A Camera (Mounted on top of camera mount at z = 0.52m, pitched 60 deg down = 1.047 rad)
    tf_os30a_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os30a_camera_link_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.52',
                   '--roll', '0.0', '--pitch', '1.047', '--yaw', '0.0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'os30a_camera_link']
    )

    tf_os30a_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os30a_camera_optical_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                   '--frame-id', 'os30a_camera_link',
                   '--child-frame-id', 'os30a_camera_color_optical_frame']
    )

    # 6. EKF Sensor Fusion Node (robot_localization: Wheel Odom + 100Hz IMU Gyro)
    ekf_config = os.path.join(vslam_pkg, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
    )

    # 7. Robot State Publisher (Publishes /robot_description for RViz 3D Robot Model)
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle_pi.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        spawn_robot,
        core_bridge,
        camera_mode_manager,
        tf_base_link,
        tf_imu_link,
        tf_camera_link,
        tf_camera_optical,
        tf_os30a_camera_link,
        tf_os30a_camera_optical,
        ekf_node,
        robot_state_pub
    ])
