# 🤖 TurtleBot3 Visual SLAM & Precision Dual-ArUco Docking System

본 저장소는 **TurtleBot3 Waffle Pi** 로봇을 기반으로 한 **Pure 3D Visual SLAM 자율주행** 및 **Dual ArUco 마커 기반 cm 단위 정밀 도킹(Precision Docking)** 통합 솔루션입니다.

---

## 📌 목차
1. [시스템 및 장비 사양 (Hardware Specs)](#1-시스템-및-장비-사양-hardware-specs)
2. [필요 라이브러리 및 소프트웨어 환경 (Prerequisites)](#2-필요-라이브러리-및-소프트웨어-환경-prerequisites)
3. [전체 프로젝트 디렉토리 및 파일 구성 (Project Structure)](#3-전체-프로젝트-디렉토리-및-파일-구성-project-structure)
4. [운용 워크플로우: 지도 작성 및 자율주행·도킹 (Step-by-Step Guide)](#4-운용-워크플로우-지도-작성-및-자율주행도킹-step-by-step-guide)
   - 4.1 [최초 1회] 3D 지도 작성 (Mapping Mode)
   - 4.2 [일상 운용] 3D 맵 기반 자율주행 및 정밀 도킹 (Navigation & Docking Mode)
5. [주요 노드 및 기술 사양 (Core Architecture)](#5-주요-노드-및-기술-사양-core-architecture)
6. [트러블슈팅 및 필수 하드웨어 주의사항 (Troubleshooting)](#6-트러블슈팅-및-필수-하드웨어-주의사항-troubleshooting)

---

## 1. 시스템 및 장비 사양 (Hardware Specs)

| 구분 | 장비 / 사양 | 주요 역할 |
| :--- | :--- | :--- |
| **로봇 플랫폼** | **TurtleBot3 Waffle Pi** | 차동 구동(Differential Drive) 모바일 베이스 |
| **SBC (로봇 탑재)** | **Raspberry Pi 4 Model B** (Ubuntu 24.04 Server) | OpenCR 통신, 카메라 드라이버 및 실시간 압축 실행 |
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
├── 1_build_3d_map.sh               # [실행] 3D 지도 작성: RTAB-Map Mapping 모드 실행
├── 2_start_3d_nav.sh                # [실행] 3D 자율주행: 파이 구동 + Nav2 내비게이션 통합 원클릭 실행
├── run_real_robot_pc.sh             # [실행] 3D 자율주행: PC 측 Nav2 + RTAB-Map Localization + GUI 실행 (지도 필수)
├── remote_start_robot.sh           # [유틸] 라즈베리파이 로봇 노드 SSH 원격 실행 (모터, 카메라, 압축)
├── remote_stop_robot.sh            # [유틸] 라즈베리파이 구동 프로세스 원격 긴급 정지
├── run_teleop.py / run_teleop.sh   # [제어] TwistStamped 호환 키보드 원격 수동 주행 툴
├── visual_slam_guide.md            # [문서] RGB-D 기반 3D Visual SLAM 파이프라인 및 기술 명세서
├── docking_algorithm_guide.md      # [문서] 정밀 도킹 알고리즘 및 PND 제어 수학적 기술 문서
├── REAL_ROBOT_DOCKING_IMPLEMENTATION_PLAN.md # [문서] 실로봇 도킹 아키텍처 및 구현 계획서
├── my_office_3d.db                  # [데이터] RTAB-Map으로 생성된 3D 공간 데이터베이스 (지도 파일)
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
│       │   │   ├── vslam_rgbd_imu.launch.py          # RTAB-Map 3D Visual SLAM 매핑 실행 런치
│       │   │   └── vslam_navigation.launch.py        # 3D 맵 기반 Nav2 자율주행 및 위치추정 런치
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
└── robot_scripts/                   # 라즈베리파이 설정 스크립트 및 보조 유틸리티
    └── pi/
        ├── astra_compressor.py      # [파이 탑재] Wi-Fi 대역폭 절감을 위한 RGB-D 실시간 영상 압축기
        ├── start_robot_all.sh       # [파이 탑재] 모터 + 카메라 + 압축기 + 범퍼 일괄 시작 스크립트
        └── run_astra.sh             # [파이 탑재] Astra 카메라 및 astra_compressor 실행 스크립트
```

---

## 4. 운용 워크플로우: 지도 작성 및 자율주행·도킹 (Step-by-Step Guide)

### [사전 준비] 네트워크 및 환경 변수 일치
모든 PC 터미널과 라즈베리파이에서 반드시 동일한 `ROS_DOMAIN_ID`를 사용해야 토픽이 연결됩니다:
```bash
export ROS_DOMAIN_ID=101
source /opt/ros/jazzy/setup.bash
source ~/Documents/Dapier/Project/robot_ws/install/setup.bash
```

---

### 4.1 [최초 1회] 3D 지도 작성 (Mapping Mode)
새로운 장소에서 주행 지도가 없거나 맵을 다시 생성해야 할 때 수행합니다.

#### 1) 라즈베리파이 로봇 하드웨어 구동
```bash
./remote_start_robot.sh
```
> [!IMPORTANT]
> **필수 하드웨어 조작**: 파이 구동 후 **Astra 카메라 USB 케이블을 물리적으로 한 번 뺐다가 다시 꽂아주어야** 카메라 영상 토픽(`/camera/color/image_raw`)이 정상 발행됩니다.

#### 2) PC에서 3D Visual Mapping 실행
```bash
./1_build_3d_map.sh
```
* RTAB-Map이 새 매핑 모드로 시작되며 RViz2에 실시간 3D 포인트클라우드가 그려집니다.

#### 3) 키보드 텔레옵으로 공간 스캔
별도의 터미널을 열고 수동 조종 툴을 실행합니다:
```bash
./run_teleop.sh
```
* 로봇을 천천히 전진 및 회전시키며 사무실 구석구석을 스캔합니다.
* 매핑을 마치고 `./1_build_3d_map.sh` 터미널을 `Ctrl+C`로 종료하면 맵 데이터가 **`my_office_3d.db`** 파일로 자동 저장됩니다.

---

### 4.2 [일상 운용] 3D 맵 기반 자율주행 및 정밀 도킹 (Navigation & Docking Mode)
이미 완성된 3D 지도(`my_office_3d.db`)가 준비되어 있는 일상 운용 단계입니다.

#### 1) 라즈베리파이 로봇 하드웨어 구동
```bash
./remote_start_robot.sh
```
*(카메라 USB 재연결 확인)*

#### 2) PC 자율주행 및 미션 컨트롤 실행
```bash
./run_real_robot_pc.sh
# 또는 원클릭 스크립트: ./2_start_3d_nav.sh
```
* **동작 원리**:
  - 기존에 저장된 `my_office_3d.db` 맵을 불러옵니다.
  - RTAB-Map이 **위치 추정(Localization) 모드**로 작동하여 카메라 영상으로 현재 로봇 위치를 실시간 추정합니다.
  - Nav2 자율주행 스택 및 RViz2 화면이 열립니다.
* **주행 명령**:
  - RViz 상단 도구 모음에서 **[Nav2 Goal]**을 선택하고, 도킹 스테이션(파란 박스) 전방 약 0.8m ~ 1.0m 지점을 클릭합니다.
  - 로봇이 3D 장애물을 회피하며 목적지까지 스스로 이동하여 정지합니다.

#### 3) 목표 영역 도착 후 정밀 비전 도킹 (Precision Docking)
로봇이 도킹 스테이션 전방에 도착하면 비전 도킹 런치를 실행합니다:
```bash
ros2 launch precision_docker real_robot_marker_docking.launch.py
```
* **PyQt5 도킹 GUI 화면 조작**:
  1. 실시간 카메라 화면에 좌/우 Dual ArUco 마커(#7, #19) 바운딩 박스가 잡히는지 확인합니다.
  2. GUI의 **[Start Auto Docking]** 버튼을 클릭합니다:
     - **Phase 1 (접근)**: 마커 중앙 축선을 따라 안전 속도로 전진.
     - **Phase 2 (정렬)**: 30cm 이내에서 PND 각도 제어로 회전각 편차를 $\pm 1^\circ$ 이내로 정밀 보정.
     - **Phase 3 (최종 접촉)**: 10cm 이내 극저속(2cm/s) 직진 $\to$ 전면 범퍼 마이크로스위치가 접촉을 감지하면 즉시 모터를 차단하고 도킹 완료 잠금.

#### [비상 정지]
* GUI 창의 **[EMERGENCY STOP]** 버튼 클릭
* 또는 터미널에서 즉시 원격 정지 스크립트 실행:
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
