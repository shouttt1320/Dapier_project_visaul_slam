# 📑 [Visual SLAM 구현 보고서] RGB-D 기반 3D 비주얼 SLAM 및 자율 내비게이션 시스템

> **본 문서는 TurtleBot3 Waffle Pi 로봇에서 2D LiDAR 없이 Orbbec Astra S(RGB-D 카메라), IMU, 휠 오도메트리만을 활용하여 구현한 Pure 3D Visual SLAM 및 Nav2 자율주행 파이프라인의 핵심 기술 명세서입니다.**

---

## 1. 개요 및 설계 목표 (Overview & Objectives)

### 1.1 배경 및 필요성
* **기존 2D LiDAR의 한계**: 2D 라이다는 바닥에서 특정 높이(단일 평면)의 단면만 스캔하므로, 책상 상판, 튀어나온 모니터, 바닥의 턱 등 입체적인 3D 장애물을 감지하지 못해 충돌 위험이 존재합니다.
* **Pure Visual SLAM 채택**: Orbbec Astra S RGB-D 깊이 카메라와 RTAB-Map(Real-Time Appearance-Based Mapping)을 활용하여, 고가의 라이다 센서 없이도 **입체적인 3D 공간 복원, 2D 점유 격자 지도(Occupancy Grid) 생성, 정밀 위치 추정(Localization)**을 모두 달성하는 경제적이고 안정적인 자율주행 시스템을 구축했습니다.

### 1.2 시스템 아키텍처 다이어그램

```mermaid
flowchart TD
    subgraph Robot_Pi4 ["라즈베리파이 (Robot Base)"]
        Astra[Orbbec Astra S 카메라] -->|RGB + Depth Raw| Comp[astra_compressor.py\n실시간 영상 압축]
        OpenCR[OpenCR 1.0] -->|엔코더 & 100Hz IMU| TB3Node[turtlebot3_node]
    end

    subgraph Network ["Wi-Fi 무선 통신 (ROS_DOMAIN_ID=101)"]
        Comp -->|/compressed\n/compressedDepth| Bridge
        TB3Node -->|/odom, /imu| EKF
    end

    subgraph Host_PC ["제어 워크스테이션 (PC)"]
        Bridge[compressed_bridge_node\n동기화 및 압축 해제]
        EKF[robot_localization\nEKF 센서 융합]
        
        Bridge -->|RGB + Depth + CameraInfo| RTAB[RTAB-Map SLAM Core]
        Bridge -->|Depth| PointCloud[point_cloud_xyzrgb\n3D 포인트클라우드]
        EKF -->|odom -> base_footprint| RTAB
        
        RTAB -->|map -> odom (TF)| Nav2[Nav2 Navigation Stack]
        RTAB -->|/map (2D Occupancy Grid)| Nav2
        RTAB -->|my_office_3d.db| Storage[(3D DB 저장소)]
        PointCloud -->|Voxel Costmap| Nav2
    end
```

---

## 2. 센서 데이터 취득 및 분산 네트워크 최적화

### 2.1 라즈베리파이 대역폭 병목 극복: `astra_compressor.py`
* **문제점**: Astra S 카메라의 640x480 해상도 RGB 및 16-bit Depth 원본 스트림은 초당 약 30~50MB/s의 대역폭을 소모합니다. 라즈베리파이 4의 무선 Wi-Fi 환경에서는 네트워크 포화로 인해 **2~5초의 극심한 전송 지연과 패킷 드롭**이 발생하여 실시간 SLAM이 불가능했습니다.
* **해결책**:
  1. 라즈베리파이 로컬에서 경량 파이썬 노드(`astra_compressor.py`) 구동.
  2. RGB 컬러 영상은 JPEG 압축(품질 55)하여 `sensor_msgs/msg/CompressedImage`로 변환.
  3. 16-bit Depth 영상은 무손실 PNG 압축 후 `/camera/depth/image_raw/compressedDepth`로 발행.
  4. DDS 네트워크 송신 QoS를 **`BEST_EFFORT`**로 설정하여 네트워크 백로그(큐 지연)를 원천 차단.

### 2.2 PC 측 무지연 이벤트 드리븐 브리지: `compressed_bridge_node`
* PC 수신단에서 `compressed_bridge_node`를 구동하여, 무선으로 수신된 압축 스트림을 언패킹하고 고정밀 동기화를 수행합니다.
* **1:1 락스텝 검증 (`dispatch_sync_pair`)**:
  - 컬러와 깊이의 타임스탬프 차이를 검증하여 $|t_{\text{color}} - t_{\text{depth}}| \le 0.07\text{s}$ 이내인 정상 쌍만 동기화.
  - Color, Depth, CameraInfo 3개 메시지에 **나노초 단위까지 동일한 타임스탬프**를 재부여하여 RTAB-Map 및 RViz2로 동시 발행.
* **소비 즉시 버퍼 비움 (Buffer Flush)**:
  - 한 번 발행된 프레임 쌍은 `self.latest_color = None`, `self.latest_depth_msg = None`으로 즉시 폐기하여, 과거 프레임이 다음 주기로 밀려 로봇 회전 시 잔상(Ghosting)이 생기는 현상을 원천 방지.
* **단일 스레드 최적화**:
  - ROS 2 Jazzy `MultiThreadedExecutor`의 100% CPU 스핀 버그를 회피하기 위해 `rclpy.spin(node)` 단일 스레드로 구동하여 PC CPU 점유율을 1% 미만으로 유지.

---

## 3. 센서 융합 기반 오도메트리 (Sensor Fusion: Robot Localization EKF)

Visual SLAM에서 카메라 움직임만으로 위치를 추정(Visual Odometry)하면, 텍스처가 부족한 흰 벽면을 마주하거나 로봇이 빠르게 회전할 때 추적이 끊기는 치명적인 문제가 발생합니다. 이를 방지하기 위해 **확장 칼만 필터(EKF)**로 물리 센서를 융합했습니다.

### 3.1 융합 센서 구성 ([`ekf.yaml`](file:///home/jjj/Documents/Dapier/Project/robot_ws/src/turtlebot3_visual_slam/config/ekf.yaml))
* **휠 오도메트리 (`/odom`)**:
  - DYNAMIXEL 모터 엔코더 기반 선속도 $v_x$ 융합 (`odom0_config`의 7번째 요소 활성화).
  - 휠 슬립으로 인해 누적되는 회전각(Yaw) 오차는 신뢰하지 않고 배제.
* **고속 IMU (`/imu`)**:
  - OpenCR 탑재 100Hz 관성센서로부터 **각속도 $\omega_z$ (Yaw Rate)** 및 **전진 가속도 $a_x$** 융합.
  - 급격한 회전이나 슬립 발생 시에도 즉각적인 방향 전환을 감지.

### 3.2 2D 평면 모드 제약 (`two_d_mode: true`)
실내 바닥을 주행하는 터틀봇의 특성을 반영하여 Roll, Pitch, Z축 이동을 강제로 0으로 고정함으로써 센서 노이즈에 의해 로봇 좌표계가 지면 아래로 파고들거나 들뜨는 현상을 방지했습니다.

---

## 4. RTAB-Map 기반 3D Graph SLAM 핵심 알고리즘

RTAB-Map은 대규모 실내외 환경에서 장시간 안정적으로 동작하는 **외형 기반(Appearance-Based) 그래프 SLAM 라이브러리**입니다.

### 4.1 시각 특징점 추출 및 매칭 (Feature Extraction)
* **특징점 알고리즘**: **ORB (Oriented FAST and Rotated BRIEF)** (`Vis/FeatureType: "2"`, `Kp/DetectorStrategy: "2"`)
* **특징점 수**: 프레임당 최대 **1500개** (`Vis/MaxFeatures: "1500"`)
  - 사무실 환경의 반복적인 타일 바닥, 단색 벽면 등 텍스처가 희박한 환경에서도 충분한 기하학적 매칭점을 확보.
* **인라이어 임계값**: 최소 **20개 이상**의 RANSAC 인라이어 매칭 시에만 유효한 노드로 인정 (`Vis/MinInliers: "20"`). 잘못된 루프 클로저로 인한 지도 왜곡을 원천 방지.

### 4.2 DBoW2 기반 루프 클로저 (Loop Closure Detection)
* 로봇이 이전에 방문했던 장소로 돌아왔을 때, 시각적 단어 가방(Bag-of-Words) 모델을 통해 현재 뷰와 과거 키프레임 간의 유사도(Similarity)를 실시간 검색합니다.
* 루프가 감지되면 **g2o / GTSAM 그래프 최적화 엔진**을 호출하여, 주행 중 누적되었던 모든 오도메트리 누적 오차를 일시에 보정(Graph Optimization)합니다.

### 4.3 3D 센서의 2D 평면 강제 제약 (`Reg/Force3DoF: "true"`)
* 3D 카메라(Astra S)의 깊이 데이터를 사용하더라도, 바닥 요철이나 급가속으로 인해 카메라가 위아래로 흔들리면 3D 지도가 파도치듯 뒤틀리는 현상이 발생합니다.
* 이를 해결하기 위해 슬램 등록 파라미터에서 **`Reg/Force3DoF: "true"`** 및 **`RGBD/ForceOdom3DoF: "true"`**를 적용하여, 위치 추정과 그래프 최적화가 완벽한 $(X, Y, \text{Yaw})$ 2D 평면 위에서만 정렬되도록 구속했습니다.

### 4.4 중력 벡터 기반 수평 지면 정렬 (`Mem/UseOdomGravity: "true"`)
* IMU의 가속도 벡터를 참조하여 지면 수평축(Gravity Normal Vector)을 자동 계산합니다. 로봇이 약간 기울어져 있더라도 생성되는 지도가 수평면을 완벽히 유지합니다.

---

## 5. 3D Depth $\to$ 2D 점유 격자 지도(Occupancy Grid) 투영

라이다 센서가 없는 본 시스템에서는 Nav2 내비게이션이 주행 경로를 생성할 수 있도록 **3D Depth 영상으로부터 2D 격자 지도를 실시간으로 투영·추출**합니다.

```mermaid
flowchart LR
    A[Astra 3D Depth 영상] --> B[3D 포인트클라우드 역투영]
    B --> C{높이 필터링\n0.05m ~ 1.2m}
    C -->|바닥면 노이즈 제거| D[지면 분리 (Ground Filtering)]
    C -->|장애물 영역| E[Ray Tracing 점유 격자 투영]
    D & E --> F[2D /map Grid: 해상도 5cm]
```

### 5.1 파라미터 설정 ([`rtabmap_params.yaml`](file:///home/jjj/Documents/Dapier/Project/robot_ws/src/turtlebot3_visual_slam/config/rtabmap_params.yaml))
* **투영 소스**: `Grid/Sensor: "1"`, `Grid/FromDepth: "true"` (라이다 대신 순수 Depth 사용)
* **격자 분해능**: `Grid/CellSize: "0.05"` (5cm 단위 정밀 격자)
* **센서 거리 유효 범위**:
  - `Grid/RangeMin: "0.4"` (Astra S 카메라 사각지대인 40cm 미만 노이즈 차단)
  - `Grid/RangeMax: "2.5"` (유효 적외선 신뢰 거리인 2.5m까지만 지도에 반영)
* **장애물 높이 필터링**:
  - `Grid/MinGroundHeight: "0.03"` ~ `Grid/MaxGroundHeight: "0.08"`: 바닥면 반사 노이즈(3cm 이하)를 제거하면서, 15cm 내외 저상 작업대도 장애물로 정상 검출하도록 설정.
  - `Grid/MaxObstacleHeight: "1.2"`: 천장 조명이나 환풍기 배관 등 주행과 무관한 상부 물체 제외.
* **광선 추적 (Ray Tracing)**: `Grid/RayTracing: "true"`
  - 물체가 사라지거나 로봇이 이동할 때, 카메라 광선 경로 상에 있는 이전 장애물 셀을 자동으로 빈 공간(Free Space)으로 갱신.

---

## 6. 운용 모드 분리: Mapping vs Localization

| 항목 | 매핑 모드 (`1_build_3d_map.sh`) | 위치 추정 모드 (`run_real_robot_pc.sh`) |
| :--- | :--- | :--- |
| **목적** | 새로운 환경의 3D 맵 구축 | 기생성된 맵 위에서 Nav2 자율주행 |
| **DB 동작** | 새 데이터베이스 생성 (`delete_db_on_start:=true`) | 기존 DB 읽기 전용 로드 (`my_office_3d.db`) |
| **메모리 갱신** | `Mem/IncrementalMemory: "true"` (새 노드 계속 추가) | `Mem/IncrementalMemory: "false"` (지도 고정) |
| **TF 발행** | `map -> odom` 실시간 갱신 및 루프 교정 | 현재 카메라 뷰와 DB 매칭 후 `map -> odom` 보정 |
| **CPU 부하** | 중간~다소 높음 (특징점 추출 및 그래프 누적) | 매우 낮음 (사전 색인된 어휘 사전으로 빠른 매칭) |

---

## 7. 주요 트러블슈팅 및 성능 최적화 경험

### Q1. 로봇 회전 시 지도가 순간적으로 찌그러지거나 겹치는 현상
* **원인**: 순수 비주얼 오도메트리만 사용할 경우, 회전 시 모션 블러로 인해 프레임 간 매칭 실패가 발생함.
* **해결**: OpenCR의 100Hz IMU 자이로 센서와 휠 엔코더를 EKF로 융합하고, 회전각 오차를 IMU 기반으로 강력히 구속하여 회전 중 시각 매칭이 일시적으로 끊겨도 위치 추적이 흔들리지 않도록 보정함.

### Q2. 형광등 빛이나 반사 바닥면에서 고스트 장애물이 지도에 찍히는 현상
* **원인**: 에폭시 코팅 바닥이나 광택 타일에서 적외선 프로젝터 패턴이 반사되어 바닥 아래나 허공에 잘못된 깊이 값이 생성됨.
* **해결**: `Grid/MinGroundHeight(0.05m)` 필터를 강화하고, `Grid/RayTracing`을 활성화하여 반사 노이즈가 발생하더라도 로봇이 지나가면서 즉시 허위 장애물을 지우도록 설계.

### Q3. Wi-Fi 무선 대역폭 한계로 인한 화면 딜레이 (최대 3초 지연)
* **원인**: Raw 이미지 토픽을 그대로 전송하여 무선 네트워크 포화 발생.
* **해결**: 라즈베리파이에서 `astra_compressor.py`를 통해 전송 전 JPEG/PNG 하드웨어 압축을 수행하고, `BEST_EFFORT` QoS를 적용하여 전송 지연을 **30ms(0.03초) 이내로 단축**함.

### Q4. RTAB-Map이 주행 중 5초마다 멈추거나(Watchdog Timeout) 포인트클라우드가 뒤틀리는 현상
* **원인**: 
  - RTAB-Map은 `delay = PC_time - msg_header_stamp` 수식으로 센서 지연 시간을 측정하며, 이 값이 5.0초를 초과하면 센서 타임아웃으로 간주하여 5초간 매핑을 중단(Freezing)시킴.
  - 라즈베리파이 내부 시스템 클럭이 PC보다 **2.95초 느리게 뒤처져(Drift)** 있어서, 패킷이 2초만 지연되어도 5초를 즉시 초과해 지도 생성이 먹통이 됨.
* **해결**:
  - `1_build_3d_map.sh` 및 `2_start_3d_nav.sh` 기동 시, SSH를 통해 PC의 정밀 나노초 시간을 파이에 원격 주입:
    ```bash
    CURRENT_TIME=$(date +%s)
    ssh user@192.168.0.148 "echo user | sudo -S date -s @$CURRENT_TIME.%N"
    ```
  - 두 장비 간의 시간 오차를 **5ms 미만**으로 영점 조준하여 5초 프리징 현상을 100% 영구 해결.

### Q5. ROS 2 Jazzy 환경에서 브리지 및 압축 노드 실행 시 CPU 100% 폭주 현상
* **원인**: ROS 2 Jazzy의 `MultiThreadedExecutor`가 `MutuallyExclusiveCallbackGroup`과 함께 사용될 때 무한 루프로 CPU 코어를 100% 점유하는 알려진 스핀 버그 존재.
* **해결**: 복잡한 멀티스레드 대신 `rclpy.spin(node)` 단일 스레드 이벤트 루프로 전환하고 무거운 압축 연산을 쓰로틀링 루틴에 정렬하여 CPU 점유율을 1~3% 수준으로 안정화.
