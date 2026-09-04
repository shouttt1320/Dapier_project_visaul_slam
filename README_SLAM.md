# 🐢 TurtleBot3 (Waffle Pi) SLAM 환경 가이드

현재 시스템 환경: **Ubuntu 24.04 LTS (Noble)** / **ROS 2 Jazzy Jalisco** / **TurtleBot3 Waffle Pi**

---

## 1. 패키지 자동 설치

프로젝트 폴더 내 준비된 스크립트를 실행하여 모든 필수 ROS 2 및 TurtleBot3 SLAM 패키지를 한 번에 설치합니다.

```bash
cd /home/jjj/Documents/Dapier/Project
./install_turtlebot3_slam.sh
```

설치 완료 후 환경 변수 적용:
```bash
source ~/.bashrc
```

---

## 2. 주요 설치 패키지 목록

| 분류 | 패키지명 | 설명 |
| :--- | :--- | :--- |
| **ROS 2 핵심** | `ros-jazzy-desktop`, `ros-dev-tools` | ROS 2 Jazzy 기본 및 빌드 도구 |
| **터틀봇3 패키지** | `ros-jazzy-turtlebot3*` | TB3 드라이버, 원격제어, 시뮬레이션, 메시지 패키지 |
| **시뮬레이션** | `ros-jazzy-ros-gz` | Gazebo Sim(Harmonic) 연동 브릿지 |
| **SLAM 알고리즘** | `ros-jazzy-slam-toolbox`<br>`ros-jazzy-cartographer-ros` | 2D LiDAR 기반 SLAM (Slam Toolbox / Cartographer) |
| **내비게이션** | `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` | Nav2 자율주행 스택 |

---

## 3. [시나리오 A] 가상 시뮬레이션(Gazebo) 환경에서 SLAM

가상 환경에서 맵을 생성하고 테스트할 때 사용합니다. 터미널 3개를 열어 순서대로 실행합니다.

### 1단계: Gazebo 시뮬레이션 월드 실행 (터미널 1)
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2단계: SLAM 노드 실행 (터미널 2)
```bash
# 기본 SLAM (Cartographer)
ros2 launch turtlebot3_cartographer cartographer.launch.py

# 또는 SLAM Toolbox 사용 시
# ros2 launch slam_toolbox online_async_launch.py
```

### 3단계: 키보드 원격 조종으로 맵 매핑 (터미널 3)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
> 키보드 `w`/`a`/`s`/`d`/`x` 키를 사용하여 로봇을 천천히 이동시키며 RViz에서 맵이 완성되는 것을 확인합니다.

### 4단계: 생성된 맵(Map) 저장 (새 터미널)
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
*(홈 디렉터리에 `map.yaml`, `map.pgm` 파일이 저장됩니다)*

---

## 4. [시나리오 B] 실제 터틀봇3 로봇과 연결하여 SLAM

실제 로봇과 원격 PC(현재 컴퓨터)를 같은 Wi-Fi 네트워크에 연결하고 진행합니다.

### 1단계: 통신 설정 확인 (PC & 로봇 공통)
PC와 로봇의 `~/.bashrc`에 설정된 `ROS_DOMAIN_ID`가 동일해야 합니다. (기존 로봇 설정: **101**)
```bash
echo $ROS_DOMAIN_ID   # 101로 설정됨
```

### 2단계: 터틀봇3 SBC(라즈베리파이)에서 Bringup (터틀봇 SSH 접속 후)
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### 3단계: PC에서 SLAM 노드 실행 (PC 터미널 1)
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

### 4단계: PC에서 키보드 원격 조종 (PC 터미널 2)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 5단계: 완성된 맵 저장
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map
```

---

## 5. 유용한 점검 명령어
- 활성화된 토픽 목록 확인: `ros2 topic list`
- 라이다 스캔 데이터 확인: `ros2 topic echo /scan`
- TF 변환 트리 확인: `ros2 run tf2_tools view_frames`
