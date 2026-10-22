# 🤖 TurtleBot3 Visual SLAM, Precision Docking & LLM/YOLO Floor Object Servoing System

본 저장소는 **TurtleBot3 Waffle Pi** 로봇을 기반으로 한 **Pure 3D Visual SLAM 자율주행**, **3D 기즈모 기반 도킹 스테이션 티칭**, **Dual ArUco 마커 정밀 도킹**, 그리고 **Qwen2.5-VL / YOLOv8 기반 바닥 물체 발견 & 정밀 접근(Floor Object Servoing)** 통합 자율로봇 솔루션입니다.

---

## 📌 목차
1. [시스템 및 장비 사양 (Hardware Specs)](#1-시스템-및-장비-사양-hardware-specs)
2. [필요 라이브러리 및 소프트웨어 환경 (Prerequisites)](#2-필요-라이브러리-및-소프트웨어-환경-prerequisites)
3. [전체 프로젝트 디렉토리 및 파일 구성 (Project Structure)](#3-전체-프로젝트-디렉토리-및-파일-구성-project-structure)
4. [통합 운용 워크플로우 (Step-by-Step Execution Guide)](#4-통합-운용-워크플로우-step-by-step-execution-guide)
   - 4.1 [최초 1회] 3D 지도 작성 (`1_build_3d_map.sh`)
   - 4.2 [티칭] 3D 기즈모 기반 도킹 위치 등록 (`teach_docking_station.sh`)
   - 4.3 [일상 운용] 통합 내비게이션 & 미션 컨트롤 GUI (`3_start_nav_docker.sh`)
   - 4.4 [보조] 순수 3D 내비게이션 단독 실행 (`2_start_3d_nav.sh`)
5. [바닥 타겟 물체 접근 시스템 (Floor Object Approach Architecture)](#5-바닥-타겟-물체-접근-시스템-floor-object-approach-architecture)
6. [YOLO 등록 및 학습 툴체인 (YOLO Toolchain)](#6-yolo-등록-및-학습-툴체인-yolo-toolchain)
7. [주요 노드 및 기술 사양 (Core Architecture)](#7-주요-노드-및-기술-사양-core-architecture)
8. [트러블슈팅 및 필수 하드웨어 주의사항 (Troubleshooting)](#8-트러블슈팅-및-필수-하드웨어-주의사항-troubleshooting)
9. [관련 상세 기술 문서 (Documentation Links)](#9-관련-상세-기술-문서-documentation-links)

---

## 1. 시스템 및 장비 사양 (Hardware Specs)

| 구분 | 장비 / 사양 | 주요 역할 |
| :--- | :--- | :--- |
| **로봇 플랫폼** | **TurtleBot3 Waffle Pi** | 차동 구동(Differential Drive) 모바일 베이스 |
| **SBC (로봇 탑재)** | **Raspberry Pi 4 Model B** (Ubuntu 24.04 Server) | OpenCR 통신, 카메라 드라이버 및 실시간 압축 실행 |
| **모터 제어기** | **ROBOTIS OpenCR 1.0** + DYNAMIXEL XM430 | 모터 구동 및 엔코더 오도메트리(`/odom`), IMU 수집 |
| **비전 센서** | **Orbbec Astra S** (RGB-D Depth Camera) | 3D 포인트클라우드 Visual SLAM, ArUco 도킹, 바닥 물체 실시간 검출 |
| **카메라 기하학** | 높이: **14.7cm** (`0.147m`), 피치각: **+4.2°** | 캘리브레이션된 단안 핀홀 기하학 및 지면 투영 거리 추정 |
| **접촉 센서** | **전면 마이크로스위치 범퍼 센서** (GPIO 연동) | 최종 도킹 접촉 감지 및 전진 긴급 차단 안전장치 |
| **도킹 마커** | **Dual ArUco Markers** (`DICT_4X4_50`) | ID: Left #7, Right #19 (마커 변 50mm, 간격 150mm) |
| **제어 워크스테이션** | **Ubuntu 24.04 LTS (x86_64)** PC (RTX 5050 GPU) | RTAB-Map SLAM, Nav2, Qwen2.5-VL VLM, YOLOv8 20Hz 추론, 미션 GUI |
| **통신 환경** | **Wi-Fi 무선 네트워크 (`ROS_DOMAIN_ID=101`)** | ROS 2 DDS 분산 통신 (시간 동기화 오차 5ms 이내 관리) |

---

## 2. 필요 라이브러리 및 소프트웨어 환경 (Prerequisites)

### 2.1 OS 및 미들웨어
* **OS**: Ubuntu 24.04 LTS (Noble Numbat)
* **ROS**: ROS 2 Jazzy Jalisco
* **RMW 구현체**: FastDDS (`export ROS_DOMAIN_ID=101`)

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
pip install opencv-python opencv-contrib-python numpy PyQt5 pynput ultralytics torch torchvision
```

---

## 3. 전체 프로젝트 디렉토리 및 파일 구성 (Project Structure)

```text
Project/
├── 1_build_3d_map.sh               # [실행] 3D 지도 작성: RTAB-Map Mapping 모드 원클릭 실행
├── teach_docking_station.sh        # [실행] 3D 기즈모 도킹 스테이션 & 웨이포인트 인터랙티브 티칭
├── 2_start_3d_nav.sh               # [실행] 3D 순수 내비게이션: Nav2 + RTAB-Map Localization 단독 실행
├── 3_start_nav_docker.sh           # [실행/권장] 통합 내비게이션 + 미션 컨트롤 GUI + 도킹 & 바닥 접근 마스터 실행
├── remote_start_robot.sh           # [유틸] 라즈베리파이 로봇 노드 SSH 원격 기동 (모터/카메라/압축)
├── remote_stop_robot.sh            # [유틸] 라즈베리파이 구동 프로세스 원격 긴급 정지
├── run_teleop.py / run_teleop.sh   # [제어] TwistStamped 호환 키보드 수동 주행 툴
├── custom_floor_detector_best.pt   # [AI 모델] 바닥 물체 검출용 fine-tuned YOLOv8 가중치 (4개 클래스)
├── my_office_3d.db                 # [데이터] RTAB-Map 3D 공간 데이터베이스 지도
├── my_office_map.pgm / .yaml       # [데이터] 2D Occupancy Grid 격자 지도
│
├── docs/                           # 📚 기술 사양서 및 실전 운용 가이드
│   ├── yolo_training_pipeline_guide.md  # YOLO 데이터셋 수집, 라벨링, 학습, 3D 검증 툴체인 가이드
│   ├── floor_object_approach_system.md  # 바닥 타겟 물체 접근(Floor Object Approach) 시스템 구조서
│   ├── docking_algorithm_guide.md       # ArUco 정밀 도킹 PND 제어 알고리즘 수학적 명세서
│   └── visual_slam_guide.md             # 3D Visual SLAM 파이프라인 및 RTAB-Map 파라미터 기술 문서
│
├── tools/                          # 🛠️ 타겟 물체 학습 & 진단 도구
│   ├── yolo_training/              # YOLO 타겟 물체 등록 & 학습 도구
│   │   ├── collect_yolo_dataset.py       # 실시간 스트림 데이터셋 수집기 (수동/인터벌/비디오 슬라이서)
│   │   ├── auto_label_dataset.py         # 자동 바운딩 박스 라벨러 (YOLO-World + 3X 인터랙티브 GUI)
│   │   ├── train_custom_yolo.py          # YOLOv8 파인튜닝 파이프라인 (바닥 소형 물체 최적화)
│   │   ├── test_floor_object_detector.py # 3D 검출 & 거리 측정 검증 GUI (Zero-Motion 안전 모드)
│   │   └── capture_target_image.py       # 기준 이미지(Exemplar) 캡처기
│   │
│   └── camera_calibration/         # 카메라 캘리브레이션 & 스트림 진단
│       ├── capture_calib_snapshot.py     # 캘리브레이션 스냅샷 캡처
│       ├── test_live_astra_feed.py       # Astra S Wi-Fi 스트림 및 3D 비전 노드 연결 진단
│       └── view_camera.py                # 카메라 피드 실시간 뷰어 (압축/비압축 자동 폴백)
│
├── robot_ws/                       # 🤖 메인 ROS 2 워크스페이스
│   └── src/
│       ├── llm_semantic_docker/    # [지능] 자연어 시맨틱 도킹 & 바닥 물체 접근 & 통합 GUI
│       │   ├── launch/
│       │   │   ├── llm_nav_docker_integrated.launch.py # 통합 마스터 런치 (SLAM + Nav2 + GUI + 접근기)
│       │   │   └── teach_docking_station.launch.py     # 3D 기즈모 티칭 런치
│       │   ├── llm_semantic_docker/
│       │   │   ├── llm_agent_node.py                   # Qwen LLM 기반 자연어 명령 해석 및 의도 분류
│       │   │   ├── task_manager.py                     # 8-상태 미션 스케줄러 & 자율주행 인터럽트 총괄
│       │   │   ├── qwen_semantic_gui_node.py           # PyQt5 기반 미션 컨트롤 다크테마 콕핏 GUI
│       │   │   ├── object_approaching_controller_node.py # 20Hz YOLOv8 지면 하이브리드 3D 정밀 접근기
│       │   │   ├── precision_docking_controller_node.py # Dual ArUco 마커 기반 cm 단위 정밀 도킹기
│       │   │   ├── interactive_docking_teacher.py      # RViz 3D 기즈모 인터랙티브 티칭 노드
│       │   │   └── command_validator.py                # Pydantic v2 기반 자연어 명령 안전 검증기
│       │   └── config/
│       │       ├── floor_target_objects.yaml           # 바닥 타겟 물체 선언적 레지스트리 (ID/별칭/스탠드오프)
│       │       ├── docking_stations.yaml               # 티칭된 도킹 스테이션 좌표 및 허용 오차
│       │       ├── semantic_waypoints.yaml             # 티칭된 웨이포인트 목록
│       │       └── camera_pitch_calib.yaml             # 카메라 높이(14.7cm) 및 피치각(+4.2°) 보정값
│       │
│       ├── turtlebot3_visual_slam/ # [항법] RTAB-Map 3D Visual SLAM & Nav2 파라미터 패키지
│       │   ├── launch/
│       │   │   ├── vslam_rgbd_imu.launch.py            # 3D SLAM 매핑 런치
│       │   │   └── vslam_navigation.launch.py          # 3D 맵 기반 자율주행 런치
│       │   └── config/
│       │       ├── nav2_vslam_params.yaml              # Nav2 Controller/Planner 설정 (속도 조절 반영)
│       │       └── rtabmap_params.yaml                 # RTAB-Map 루프 클로저 설정
│       │
│       └── precision_docker/       # [안전] 하드웨어 범퍼 센서 패키지
│           └── precision_docker/
│               └── bumper_sensor_node.py               # 마이크로스위치 GPIO 접촉 감지
│
└── robot_scripts/                  # 라즈베리파이 탑재 스크립트
    └── pi/
        ├── astra_compressor.py      # [파이 탑재] Wi-Fi 대역폭 절감을 위한 RGB-D 10FPS 실시간 압축기
        └── start_robot_all.sh       # [파이 탑재] 모터 + 카메라 + 압축기 + 범퍼 일괄 구동기
```

---

## 4. 통합 운용 워크플로우 (Step-by-Step Execution Guide)

### [사전 준비] 네트워크 환경 변수
모든 PC 터미널에서 동일한 도메인 ID를 설정합니다:
```bash
export ROS_DOMAIN_ID=101
source /opt/ros/jazzy/setup.bash
source ~/Documents/Dapier/Project/robot_ws/install/setup.bash
```

---

### 4.1 [최초 1회] 3D 지도 작성 (`1_build_3d_map.sh`)
새로운 환경에서 3D 맵을 구축할 때 수행합니다.
```bash
./1_build_3d_map.sh
```
* PC가 자동으로 라즈베리파이 시간을 나노초 단위로 동기화하고 로봇 노드를 원격 기동합니다.
* 별도 터미널에서 `./run_teleop.sh`를 실행하여 로봇을 수동 주행하며 공간을 스캔합니다.
* 종료(`Ctrl+C`) 시 맵이 **`my_office_3d.db`**에 자동 저장됩니다.

---

### 4.2 [티칭] 3D 기즈모 기반 도킹 위치 등록 (`teach_docking_station.sh`)
RViz 상에서 대화형 3D 기즈모(큐브)를 마우스로 조작하여 도킹 스테이션과 웨이포인트를 원클릭으로 등록합니다.
```bash
./teach_docking_station.sh workplace1
```
* **마우스 조작**: 상단 `Interact` 툴(단축키 `i`)을 선택하고 초록색 상자를 원하는 위치로 드래그합니다.
* **터미널 콘솔 명령**:
  - `save`: 현재 기즈모 위치를 도킹 스테이션으로 저장 (`docking_stations.yaml`)
  - `save_wp`: 현재 기즈모 위치를 웨이포인트로 저장 (`semantic_waypoints.yaml`)
  - `wp <이름>` / `station <이름>`: 이름 실시간 변경

---

### 4.3 [일상 운용/추천] 통합 내비게이션 & 미션 컨트롤 GUI (`3_start_nav_docker.sh`)
자율주행, 도킹 스테이션 정밀 도킹, 그리고 바닥 물체 정밀 접근을 한 번에 통합 구동하는 마스터 스크립트입니다.
```bash
./3_start_nav_docker.sh
```
* **실행 시 자동 구동되는 컴포넌트**:
  1. 라즈베리파이 로봇 원격 기동 및 시간 자동 동기화
  2. RTAB-Map 3D 위치 추정(Localization) 및 3D 맵 로드
  3. Nav2 자율주행 스택 & 비용지도(Costmap)
  4. Qwen2.5-VL Multimodal Agent (`llm_agent_node`)
  5. 8-상태 임무 총괄 관리자 (`task_manager_node`)
  6. 20Hz 지면 하이브리드 바닥 물체 접근기 (`object_approaching_controller_node`)
  7. Dual ArUco 마커 정밀 도킹기 (`precision_docking_controller_node`)
  8. 미션 컨트롤 다크테마 콕핏 GUI (`qwen_semantic_gui_node`)
* **GUI 주요 기능**:
  - **카메라 뷰포트**: 20Hz 실시간 스트림, 타겟 Bounding Box, 거리(Z), 좌우 편차(Ey), FPS 표시.
  - **자연어 명령 입력창**: `"workplace1으로 가줘"`, `"빨간 큐브 앞으로 가"`, `"검은 막대 접근"`, `"우회전 30도"` 등 입력 시 LLM 자동 해석 및 실행.
  - **퀵 액션 프리셋**: `[🔴 빨간 큐브]`, `[🔵 파란 공]`, `[⬛ 검은 큐브]`, `[🥢 검은 막대]`, `[🎯 마커 도킹]` 원클릭 버튼.
  - **기즈모 도킹 선택기**: 티칭된 스테이션 목록 드롭다운 선택 후 즉시 자율주행 $\to$ 정밀 도킹 연계.

---

### 4.4 [보조] 순수 3D 내비게이션 단독 실행 (`2_start_3d_nav.sh`)
GUI나 지능 도킹 없이 순수 Nav2 3D 주행 기능만 단독으로 테스트할 때 사용합니다:
```bash
./2_start_3d_nav.sh
```

---

## 5. 바닥 타겟 물체 접근 시스템 (Floor Object Approach Architecture)

바닥에 놓인 작은 물체(큐브, 공, 막대 등)를 발견하고 25cm 앞까지 cm 단위로 정밀하게 감속 접근하는 시스템입니다.

* **자율주행 순찰 중 자동 발견 인터럽트 (Patrol Intercept)**:
  - 로봇이 웨이포인트 주행 중 전방 $0.45\text{m} \sim 2.20\text{m}$에서 신뢰도 $0.65$ 이상으로 물체를 3프레임 연속 감지하면 Nav2 주행을 즉시 멈추고 팝업 확인창(`PickConfirmationDialog`)을 띄웁니다.
  - 사용자 승인 시 정밀 접근 모드로 전환, 건너뛰기 시 원래 주행 목적지로 복귀합니다.
* **하이브리드 3D 거리 측정 (Hybrid Depth Perception)**:
  - **> 1.2m (원거리)**: 단안 지면 핀홀 기하학(Pinhole Ground-Contact Geometry) 및 카메라 물리 피치각(+4.2°) 역투영 계산.
  - **0.4m ~ 1.2m (중근거리)**: Astra S 적외선 Structured Light 3D 뎁스 센서 융합.
  - **< 0.4m (블라인드존)**: 카메라 후드 아래로 물체가 사라져도 오도메트리 공간 기억(Dead-Reckoning)을 활용해 25cm 스탠드오프 지점까지 오차 없이 부드럽게 감속 접근.

> 📖 상세 수학 모델과 FSM 구조는 [docs/floor_object_approach_system.md](file:///home/jjj/Documents/Dapier/Project/docs/floor_object_approach_system.md)를 참고하세요.

---

## 6. YOLO 등록 및 학습 툴체인 (YOLO Toolchain)

`tools/yolo_training/` 도구를 통해 새로운 물체를 4단계 원클릭으로 등록하고 학습시킬 수 있습니다:

```bash
# 1. 실시간 데이터 수집 (스페이스바 캡처 or 인터벌 자동 수집)
python3 tools/yolo_training/collect_yolo_dataset.py --label my_new_target

# 2. 3X 확대 인터랙티브 GUI에서 자동 바운딩 박스 검토 & 라벨링
python3 tools/yolo_training/auto_label_dataset.py

# 3. GPU 가속 파인튜닝 (소형 바닥 물체 증강 기법 적용)
python3 tools/yolo_training/train_custom_yolo.py --epochs 60

# 4. 안전한 무동작 3D 검출 & 거리 측정 검증
python3 tools/yolo_training/test_floor_object_detector.py
```

* **선언적 시스템 등록**:
  `robot_ws/src/llm_semantic_docker/config/floor_target_objects.yaml`에 새 타겟 ID와 별칭을 추가하면 파이썬 코드 수정 없이 즉시 자연어 명령, 자율주행 발견, GUI 버튼에 반영됩니다.

> 📖 상세 학습 파라미터와 조작법은 [docs/yolo_training_pipeline_guide.md](file:///home/jjj/Documents/Dapier/Project/docs/yolo_training_pipeline_guide.md)를 참고하세요.

---

## 7. 주요 노드 및 기술 사양 (Core Architecture)

* **`object_approaching_controller_node.py`**:
  - 20Hz 닫힌 루프(Closed-loop) 제어로 차체 중심과 타겟 간의 횡방향 편차($E_y$)를 보정하고 P-감속 곡선을 통해 25cm 목표 거리에 정확히 안착.
  - ROS 2 Jazzy 사양 `geometry_msgs/msg/TwistStamped` 발행 (`frame_id='base_footprint'`).
* **`task_manager.py`**:
  - `NAVIGATING`, `AWAITING_PICK_CONFIRMATION`, `OBJECT_APPROACHING`, `DOCKING`, `PICK_COMPLETED` 등 8단계 FSM 총괄.
* **`llm_agent_node.py`**:
  - 로컬 Ollama (`qwen2.5:3b`)와 연동하여 사용자 자연어 명령을 Pydantic v2 안전 스키마로 파싱.
* **`precision_docking_controller_node.py`**:
  - Dual ArUco 마커(#7, #19)를 동시에 추적하여 $\pm 1^\circ$, $\pm 1\text{cm}$ 오차로 초정밀 도킹 및 하드웨어 마이크로스위치 범퍼 물리 접촉 감지.

---

## 8. 트러블슈팅 및 필수 하드웨어 주의사항 (Troubleshooting)

### Q1. 파이 구동 후 카메라 영상 토픽이 들어오지 않습니다.
* **필수 조치**: Astra S 카메라의 USB 버스 초기화 문제로 인해, **파이 부팅 및 드라이버 실행 후 카메라 USB 케이블을 물리적으로 한 번 뽑았다가 다시 꽂아주어야** 정상적으로 영상 장치가 잡히고 스트림이 시작됩니다.
* 또한 압축 토픽(`/camera/color/image_raw/compressed`)은 QoS가 **`BEST_EFFORT`**입니다.

### Q2. 로봇 시계 오차로 RTAB-Map 지도가 멈춥니다.
* RTAB-Map은 시계 오차가 5.0초를 초과하면 매핑을 중단합니다. 모든 실행 스크립트(`1_build_3d_map.sh`, `3_start_nav_docker.sh` 등) 서두에서 PC 시간을 파이에 나노초 단위로 자동 동기화(`sudo date -s @$CURRENT_TIME`)하여 시계 오차를 5ms 이내로 해결했습니다.

### Q3. 새로운 물체를 학습시켰는데 주행 중에 인식을 못합니다.
* 최신 가중치 파일(`runs/detect/.../best.pt`)이 프로젝트 루트의 `custom_floor_detector_best.pt`로 복사되었는지 확인하세요.
* `floor_target_objects.yaml`에 타겟 canonical_id 및 aliases가 등록되어 있는지 점검하세요.

---

## 9. 관련 상세 기술 문서 (Documentation Links)

* 📚 [YOLO 데이터셋 수집, 라벨링, 파인튜닝 가이드](file:///home/jjj/Documents/Dapier/Project/docs/yolo_training_pipeline_guide.md)
* 📚 [바닥 타겟 물체 정밀 접근 시스템 기술 사양서](file:///home/jjj/Documents/Dapier/Project/docs/floor_object_approach_system.md)
* 📚 [정밀 도킹 알고리즘 및 PND 제어 수학적 명세서](file:///home/jjj/Documents/Dapier/Project/docking_algorithm_guide.md)
* 📚 [RGB-D 3D Visual SLAM 파이프라인 가이드](file:///home/jjj/Documents/Dapier/Project/visual_slam_guide.md)
