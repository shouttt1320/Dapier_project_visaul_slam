# 🤖 TurtleBot3 Visual SLAM & Precision Dual-ArUco Docking System

본 저장소는 **TurtleBot3 Waffle Pi** 로봇을 기반으로 한 **Pure 3D Visual SLAM 자율주행** 및 **Dual ArUco 마커 기반 cm 단위 정밀 도킹(Precision Docking)** 통합 솔루션입니다.

---

## 📌 목차
1. [시스템 및 장비 사양 (Hardware Specs)](#1-시스템-및-장비-사양-hardware-specs)
2. [필요 라이브러리 및 소프트웨어 환경 (Prerequisites)](#2-필요-라이브러리-및-소프트웨어-환경-prerequisites)
3. [전체 프로젝트 디렉토리 및 파일 구성 (Project Structure)](#3-전체-프로젝트-디렉토리-및-파일-구성-project-structure)
4. [자율주행 및 정밀 도킹 실행 순서 (Step-by-Step Guide)](#4-자율주행-및-정밀-도킹-실행-순서-step-by-step-guide)
5. [주요 노드 및 기술 사양 (Core Architecture)](#5-주요-노드-및-기술-사양-core-architecture)
6. [트러블슈팅 및 필수 하드웨어 주의사항 (Troubleshooting)](#6-트러블슈팅-및-필수-하드웨어-주의사항-troubleshooting)

---

## 1. 시스템 및 장비 사양 (Hardware Specs)

| 구분 | 장비 / 사양 | 주요 역할 |
| :--- | :--- | :--- |
| **로봇 플랫폼** | **TurtleBot3 Waffle Pi** | 차동 구동(Differential Drive) 모바일 베이스 |
| **SBC (로봇 탑재)** | **Raspberry Pi 4 Model B** (Ubuntu 24.04 Server) | OpenCR 통신, 카메라 드라이버 실행 |
| **모터 제어기** | **ROBOTIS OpenCR 1.0** + DYNAMIXEL XM430 | 모터 구동 및 엔코더 오도메트리(`/odom`), IMU 수집 |
| **비전 센서** | **Orbbec Astra S** (RGB-D Depth Camera) | 3D 포인트클라우드 Visual SLAM 및 Dual ArUco 마커 인식 |
| **접촉 센서** | **전면 마이크로스위치 범퍼 센서** (GPIO 연동) | 최종 도킹 완료 물리 감지 및 긴급 정지 안전장치 |
| **도킹 마커** | **Dual ArUco Markers** (`DICT_4X4_50`) | ID: Left #7, Right #19 (마커 변 50mm, 간격 150mm) |
| **제어 워크스테이션** | **Ubuntu 24.04 LTS (x86_64)** PC | RTAB-Map Visual SLAM 연산, Nav2 경로계획, 도킹 GUI |
| **통신 환경** | **Wi-Fi 무선 네트워크 (`ROS_DOMAIN_ID=101`)** | ROS 2 DDS 분산 통신 |

---

## 2. 필요 라이브러리 및 소프트웨어 환경 (Prerequisites)

### 2.1 OS 및 미들웨어
* **OS**: Ubuntu 24.04 LTS (Noble Numbat)
* **ROS**: ROS 2 Jazzy Jalisco
* **RMW 구현체**: FastDDS 또는 CycloneDDS (`export ROS_DOMAIN_ID=101`)

### 2.2 ROS 2 패키지 의존성
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-slam \
  ros-jazzy-robot-localization \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-cv-bridge \
  ros-jazzy-turtlebot3-msgs
```

### 2.3 Python 필수 라이브러리
```bash
pip install opencv-python opencv-contrib-python numpy PyQt5 pynput
```
> **주의**: ArUco 마커 인식을 위해 반드시 `opencv-contrib-python`이 설치되어 있어야 `cv2.aruco` 모듈이 활성화됩니다.

---

## 3. 전체 프로젝트 디렉토리 및 파일 구성 (Project Structure)

```text
/home/jjj/Documents/Dapier/Project/
├── 1_build_3d_map.sh               # [실행] RTAB-Map 기반 3D Visual SLAM 지도 작성 스크립트
├── 2_start_3d_nav.sh                # [실행] 작성된 3D 지도 기반 Nav2 자율주행 실행 스크립트
├── run_real_robot_pc.sh             # [실행] PC 측 Astra 카메라 압축 브리지 및 상태 모니터링
├── remote_start_robot.sh           # [유틸] 라즈베리파이 로봇 노드 SSH 원격 실행
├── remote_stop_robot.sh            # [유틸] 라즈베리파이 구동 프로세스 원격 긴급 정지
├── run_teleop.py / run_teleop.sh   # [제어] TwistStamped 호환 키보드 원격 주행 툴
├── astra_compressor.py              # [비전] 실시간 대역폭 최적화를 위한 RGB-D 압축 브리지 노드
├── docking_algorithm_notion_guide.md# [문서] 정밀 도킹 알고리즘 및 PND 제어 수학적 기술 문서
├── REAL_ROBOT_DOCKING_IMPLEMENTATION_PLAN.md # [문서] 실로봇 도킹 아키텍처 및 구현 계획서
├── my_office_3d.db                  # [데이터] RTAB-Map으로 생성된 3D 공간 데이터베이스
├── my_office_map.pgm / .yaml       # [데이터] 2D 격자 지도 (Occupancy Grid Map)
│
├── robot_ws/                        # 메인 ROS 2 워크스페이스
│   └── src/
│       ├── precision_docker/       # [핵심] 정밀 도킹 및 안전 패키지 (구 redbox_navigator)
│       │   ├── launch/
│       │   │   ├── real_robot_marker_docking.launch.py # 도킹 컨트롤러 + PyQt5 GUI 동시 실행
│       │   │   └── real_robot_calibration.launch.py   # 카메라 왜곡 및 마커 인식 캘리브레이션
│       │   ├── precision_docker/
│       │   │   ├── precision_approacher_node.py       # 듀얼 마커 검출, 각도/거리 추정, PND 제어
│       │   │   ├── bumper_sensor_node.py              # 하드웨어 범퍼 접촉 감지 및 정지 신호 발행
│       │   │   └── docking_gui_node.py                # PyQt5 기반 상태 대시보드 및 수동/자동 제어
│       │   └── config/
│       │       └── redbox_params.yaml                 # 도킹 제어 게인, 마커 크기 및 안전 임계값
│       │
│       ├── turtlebot3_visual_slam/ # [항법] 3D Visual SLAM 및 Nav2 설정 패키지
│       │   ├── launch/
│       │   │   ├── vslam_rgbd_imu.launch.py          # RTAB-Map 3D Visual SLAM 실행 런치
│       │   │   └── vslam_navigation.launch.py        # 3D 맵 기반 Nav2 통합 내비게이션
│       │   ├── config/
│       │   │   ├── rtabmap_params.yaml                # RTAB-Map 루프 클로저 및 특징점 파라미터
│       │   │   ├── nav2_vslam_params.yaml             # Nav2 Costmap, Controller, Planner 설정
│       │   │   └── ekf.yaml                           # Robot Localization (Odom + IMU) 설정
│       │   └── scripts/
│       │       ├── setup_astra_udev.sh                # Astra S 카메라 USB 권한 부여 룰 등록
│       │       └── setup_os30a_udev.sh                # OS30A 센서 udev 룰 등록
│       │
│       └── turtlebot3/              # [기본] 터틀봇3 하드웨어 드라이버 및 모델 (Source)
│           ├── turtlebot3_bringup/  # 로봇 부팅 런치
│           ├── turtlebot3_description/ # 3D 로봇 모델(URDF) 및 메쉬
│           ├── turtlebot3_node/     # OpenCR 시리얼 통신, TwistStamped 모터 제어
│           └── turtlebot3_teleop/   # 터틀봇 텔레옵
│
└── robot_scripts/                   # 라즈베리파이 설정 스크립트 및 OpenCR 펌웨어
```

---

## 4. 자율주행 및 정밀 도킹 실행 순서 (Step-by-Step Guide)

### [준비 단계] 네트워크 및 환경 변수 일치
모든 터미널과 라즈베리파이에서 반드시 동일한 `ROS_DOMAIN_ID`를 사용해야 토픽이 공유됩니다:
```bash
export ROS_DOMAIN_ID=101
source /opt/ros/jazzy/setup.bash
source ~/Documents/Dapier/Project/robot_ws/install/setup.bash
```

---

### [1단계] 터틀봇3 하드웨어 구동 (라즈베리파이)

#### ⚠️ 다른 컴퓨터에서 처음 실행 시 주의사항 (SSH 비밀번호 / 인증키)
[`remote_start_robot.sh`](file:///home/jjj/Documents/Dapier/Project/remote_start_robot.sh) 스크립트는 내부적으로 SSH(`ssh user@192.168.0.148`)를 통해 라즈베리파이에 명령을 내립니다.
* **현재 개발 PC**: SSH 공개키가 기등록되어 있어 비밀번호 입력 없이 즉시 실행됩니다.
* **다른 새 PC에서 실행할 경우**: 파이 비밀번호 인증이 필요하므로 스크립트가 멈추거나 실패할 수 있습니다. 아래 명령어로 먼저 SSH 키를 등록하거나 터미널에서 비밀번호를 입력해 주어야 합니다:
  ```bash
  # SSH 공개키를 라즈베리파이에 복사 (최초 1회, 비밀번호: user)
  ssh-copy-id user@192.168.0.148
  ```

#### 🔌 필수 하드웨어 조작: 카메라 USB 재연결 (Plug & Re-plug)
> [!IMPORTANT]
> **라즈베리파이 부팅 및 드라이버 실행 후, Astra 카메라의 USB 케이블을 물리적으로 한 번 뽑았다가 다시 꽂아야 카메라 영상 스트림이 정상 인식됩니다.** (USB 버스 초기화 특성으로 인해 재연결 전에는 영상 토픽이 발행되지 않을 수 있습니다.)

```bash
# 옵션 A: PC에서 원격 스크립트 실행
./remote_start_robot.sh

# 옵션 B: 라즈베리파이 터미널에서 직접 실행
ros2 launch turtlebot3_bringup robot.launch.py
```
* **확인 사항**: OpenCR 연결 (`/odom`), Astra 카메라 가동 (`/camera/color/image_raw`).

---

### [2단계] PC 측 센서 수신 및 대역폭 최적화 브리지 구동 (PC 터미널)
Wi-Fi 통신 환경에서 고화질 영상을 원활히 처리하기 위해 영상 압축 브리지를 실행합니다:
```bash
./run_real_robot_pc.sh
```
* **동작**: `/camera/color/image_raw/compressed` 및 깊이 영상 토픽 최적화 스트리밍 시작.

---

### [3단계] 3D Visual SLAM 지도 작성 (초기 1회 수행 시)
새로운 환경에서 3D 맵을 구축할 때 실행합니다:
```bash
./1_build_3d_map.sh
```
* RViz가 열리며 카메라 포인트클라우드가 표시됩니다.
* 다른 터미널에서 `./run_teleop.sh`를 켜고 로봇을 천천히 수동 주행시켜 지도를 완성합니다.
* 완료 시 맵 데이터가 `my_office_3d.db`로 자동 저장됩니다.

---

### [4단계] 3D 맵 기반 Nav2 자율주행 실행
작성된 맵을 바탕으로 목적지(도킹 스테이션 근방 1m 이내)까지 자율주행합니다:
```bash
./2_start_3d_nav.sh
```
* RViz 상단 도구 모음에서 **[Nav2 Goal]**을 선택하여 마커가 위치한 스테이션 전방 0.8m ~ 1.0m 지점을 클릭합니다.
* 로봇이 깊이 센서 기반 장애물 회피를 수행하며 목적지까지 주행 후 정지합니다.

---

### [5단계] 정밀 도킹(Precision Docking) 실행 (목표 영역 도착 후)
로봇이 도킹 스테이션 시야 내에 들어오면 정밀 비전 도킹을 시작합니다:
```bash
ros2 launch precision_docker real_robot_marker_docking.launch.py
```
* **PyQt5 도킹 GUI**가 화면에 표시됩니다:
  1. 실시간 카메라 화면에 좌/우 ArUco 마커 바운딩 박스와 중심선이 오버레이됩니다.
  2. 현재 마커와의 상대 거리($d$), 상대 편각($\theta$), 횡방향 오차($e_y$)가 표시됩니다.
  3. GUI의 **[Start Auto Docking]** 버튼을 클릭하면 도킹 시퀀스가 자동 시작됩니다:
     - **Phase 1 (Approach)**: 마커 중심선을 정면으로 바라보며 고속 접근.
     - **Phase 2 (Fine Alignment)**: 30cm 이내에서 PND(Proportional Navigation)를 통해 회전각 오차를 $\pm 1^\circ$ 이내로 정렬.
     - **Phase 3 (Final Contact)**: 10cm 이내 극저속(2cm/s) 직진 $\to$ 범퍼 센서가 접촉을 감지하면 즉시 모터를 차단하고 도킹 완료 상태를 잠급니다.

---

### [비상 정지 및 종료]
* GUI 창의 **[EMERGENCY STOP]** 버튼 클릭
* 또는 PC 터미널에서 즉시 원격 정지 스크립트 실행:
  ```bash
  ./remote_stop_robot.sh
  ```

---

## 5. 주요 노드 및 기술 사양 (Core Architecture)

### 5.1 `precision_approacher_node.py` (도킹 제어기)
* **비전 감지**: OpenCV ArUco (`cv2.aruco.detectMarkers`)를 사용하여 2개의 마커 좌표를 획득하고, 두 마커의 중점을 가상 도킹 타겟 원점으로 지정.
* **좌표 변환**: 카메라 내부 파라미터($K$)를 역투영하여 로봇 기준 3차원 상대 위치 $(X, Y, Z)$ 계산.
* **제어 알고리즘**:
  $$v_x = \text{clamp}(K_p \cdot d, v_{\min}, v_{\max})$$
  $$\omega_z = K_{\theta} \cdot \theta_{\text{target}} + K_{\text{lat}} \cdot e_y$$
* **속도 명령 발행**: ROS 2 Jazzy 사양에 맞춘 `geometry_msgs/msg/TwistStamped` 사용 (`frame_id='base_footprint'`).

### 5.2 `bumper_sensor_node.py` (접촉 안전 노드)
* 하드웨어 마이크로 스위치를 감시하여 로봇이 충전 단자/거치대에 닿는 순간 `/robot/bumper/contact` 토픽(`std_msgs/Bool`) 발행.
* 도킹 제어기는 접촉 신호 수신 즉시 속도를 0으로 고정하고 모터 락을 작동.

### 5.3 `docking_gui_node.py` (PyQt5 제어기)
* 실시간 마커 검출 영상 스트리밍.
* 배터리 잔량, 범퍼 상태, 도킹 진행률(0~100%) 시각화.
* 비상 정지(E-STOP) 및 수동 텔레옵 모드 전환 지원.

---

## 6. 트러블슈팅 및 필수 하드웨어 주의사항 (Troubleshooting)

### Q1. 파이 구동 후 카메라 영상 토픽이 들어오지 않습니다.
* **필수 조치**: Astra S 카메라의 USB 버스 초기화 문제로 인해, **파이 부팅 및 드라이버 실행 후 카메라 USB 케이블을 물리적으로 한 번 뽑았다가 다시 꽂아주어야** 정상적으로 영상 장치가 잡히고 스트림이 시작됩니다.
* 또한 압축 토픽(`/camera/color/image_raw/compressed`)은 QoS가 **`BEST_EFFORT`**입니다. 일반 `RELIABLE` 구독자로는 수신되지 않으므로 주의가 필요합니다.

### Q2. 다른 PC에서 `./remote_start_robot.sh` 실행 시 반응이 없거나 실패합니다.
* **원인**: 새 PC에서는 라즈베리파이(`user@192.168.0.148`) SSH 인증키가 등록되어 있지 않아 암호 입력 대기 상태에 걸릴 수 있습니다.
* **해결**: `ssh-copy-id user@192.168.0.148` (비밀번호: `user`) 명령으로 PC의 공개키를 파이에 등록해 주시면 이후 비밀번호 없이 원클릭 실행이 가능합니다.

### Q3. 로봇이 `/cmd_vel` 명령을 받고도 움직이지 않습니다.
* **원인**: ROS 2 Jazzy 버전의 `turtlebot3_node`는 레거시 `geometry_msgs/msg/Twist`를 무시하고 **`geometry_msgs/msg/TwistStamped`**만 수신하도록 업데이트되었습니다.
* **해결**: 모든 제어 노드 및 스크립트에서 `TwistStamped` 형식과 `header.stamp`, `header.frame_id='base_footprint'`를 지정해 발행해야 합니다.

### Q4. 마커 인식이 불안정하거나 튕깁니다.
* 실내 형광등의 반사광(글레어)으로 인해 흰색 테두리가 날아가는 경우가 있습니다.
* `markers/` 디렉토리에 있는 고대비 매트 재질 마커를 인쇄하여 사용하고, 카메라 조리개/노출을 일정하게 고정하는 것을 권장합니다.
