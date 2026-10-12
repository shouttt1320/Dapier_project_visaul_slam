"""
[TurtleBot3 Base-Only Launch File for Pure Visual SLAM]
- 목적: 라이다(LiDAR)를 제외하고 터틀봇3의 필수 구동계(OpenCR)와 좌표계(URDF TF)만 실행
- 실행 위치: 라즈베리 파이 4 (TurtleBot3 Waffle Pi)
- 주요 기능:
  1. robot_state_publisher: 로봇 외형 3D 모델(URDF) 파싱 및 링크 간 TF(/tf, /tf_static) 브로드캐스팅
  2. turtlebot3_ros: OpenCR 보드(/dev/ttyACM0)와 시리얼 통신 -> /odom, /imu 발행 및 /cmd_vel 모터 구동
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 터틀봇3 기본 패키지(turtlebot3_bringup)의 공유 디렉터리 경로 획득
    pkg_dir = get_package_share_directory('turtlebot3_bringup')
    
    # 2. 환경 변수에서 로봇 모델명(기본값: waffle_pi)을 가져와 하드웨어 파라미터 YAML 파일 경로 완성
    model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    param_file = os.path.join(pkg_dir, 'param', f'{model}.yaml')

    # 3. 런치 실행 시 외부에서 주입 가능한 인자(Launch Configuration) 정의
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        # 런치 아규먼트 선언 (실제 로봇이므로 기본값: use_sim_time=false)
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),

        # ---------------------------------------------------------------------------------
        # [역할 1] State Publisher 실행 (IncludeLaunchDescription 사용)
        # - 왜 Node 대신 Include인가?
        #   URDF XML 파일을 찾아 열고 'robot_description' 파라미터로 주입하는 전처리 과정이
        #   공식 'turtlebot3_state_publisher.launch.py'에 이미 구현되어 있어 통째로 재사용함.
        # - 역할: base_footprint -> base_link -> 바퀴 링크 간의 기구학적 TF 좌표 변환 브로드캐스팅
        # ---------------------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items()
        ),

        # ---------------------------------------------------------------------------------
        # [역할 2] OpenCR 제어 드라이버 노드 실행 (Node 직접 실행)
        # - 왜 공식 bringup 런치를 안 쓰고 Node로 직접 띄우는가?
        #   공식 런치에는 라이다(LDS-01/02) 실행 노드가 포함되어 있음.
        #   우리 시스템은 순수 비전 SLAM이므로, 라이다를 빼고 OpenCR 통신 노드만 단독 실행함.
        # - 역할:
        #   1) /odom (바퀴 엔코더 기반 오도메트리) 발행
        #   2) /imu (내장 자이로/가속도 센서) 발행
        #   3) /battery_state (배터리 전압) 모니터링
        #   4) /cmd_vel (속도 명령) 수신 -> 다이나믹셀 모터 물리 구동
        # ---------------------------------------------------------------------------------
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[param_file, {'namespace': namespace}],
            arguments=['-i', '/dev/ttyACM0'],  # OpenCR 보드가 연결된 USB 시리얼 포트
            output='screen'
        )
    ])
