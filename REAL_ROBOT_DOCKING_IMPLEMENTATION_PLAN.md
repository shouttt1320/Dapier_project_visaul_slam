# 실물 터틀봇3 정밀 도킹 시스템 상세 구현 계획서 (개정판)

본 문서는 Gazebo 가상환경에서 검증된 Visual SLAM 및 목표 책상 정밀 도킹 알고리즘을 **실물 하드웨어(TurtleBot3 + Raspberry Pi 4 + Astra S + OS30A Depth Camera + 접촉 범퍼)**에 이식하기 위한 최신 상세 엔지니어링 구현 계획서입니다.

---

## 0. 현재 진행 현황 (Progress Status)

| 단계 | 항목 | 상태 | 검증 내용 |
| :--- | :--- | :---: | :--- |
| **Step 1** | **`bumper_sensor_node.py` 개발** | **완료 (DONE)** | BCM 23(물리 16번) 핀, 50Hz, 30ms 디바운스, 라즈베리파이 udev 룰 적용 및 토픽 발행 검증 완료 |
| **Step 1** | **`precision_approacher_node.py` 파라미터화** | **완료 (DONE)** | 65° 피치, 40cm 높이, 9~12.5cm 슬라이스, 2.5cm/s 크롤링, RANSAC 20회 축소, PC & Pi 빌드 완료 |
| **Step 2** | **비전 필터링 및 책상 정적 캘리브레이션** | **진행 예정 (NEXT)** | 10~11cm 책상 배치, Static TF(65°/0.40m) 검증, RViz 디버그 뷰 및 에지 마커 측정 |
| **Step 3** | **정밀 도킹 FSM 단독 주행 테스트** | 대기 | 책상 앞 0.40m에서 `/start_docking` 트리거, 헤딩 정렬 ➔ 저속 접근 ➔ 범퍼 접촉 정지 |
| **Step 4** | **Nav2 + 카메라 시분할 엔드투엔드 통합** | 대기 | Nav2 주행(Astra S) ➔ 도착 후 OS30A 전환 ➔ 5cm 정밀 접촉 도킹 완료 |

---

## 1. 하드웨어 스펙 및 시스템 아키텍처

### 1.1 하드웨어 배치 및 기구학 사양

```
                      [OS30A Depth Cam] (Z = 0.40m, Pitch = 65° = 1.1344 rad 하향)
                             \  (책상 상판 에지 검출 및 정밀 도킹용)
                              \
   [Astra S Cam]               \
  (Z=0.10m, 수평 전방) -------- [TurtleBot3 Body]
  (Visual SLAM/Nav2용)         | (Raspberry Pi 4 + OpenCR)
                               |
                        [Bumper Switch] (전방 최하단, Pin 14 GND & Pin 16 BCM 23)
```

| 구성 요소 | 장착 위치 및 스펙 | 역할 | 연결 인터페이스 |
| :--- | :--- | :--- | :--- |
| **Astra S** | 전방 수평 ($Z=0.10\text{m}$, Pitch $0^\circ$) | Visual SLAM (RTAB-Map), 2D/3D 장애물 회피 (Nav2) | USB 2.0 / UVC |
| **OS30A** | 상단 마운트 ($Z=0.40\text{m}$, Pitch $65^\circ$ 하향) | 책상 상판 에지 검출, 정밀 도킹 거리/각도 측정 | USB 3.0 / V4L2 (`HP-ASC-H201`) |
| **접촉 센서** | 로봇 전방 최하단 범퍼 | 최종 도킹 완료 감지 (기계적 접촉 스위치) | 물리 14(GND), 16(GPIO 23) |
| **Raspberry Pi 4** | 4GB RAM, Ubuntu 24.04 (ROS2 Jazzy) | 로봇 드라이버, 범퍼 노드, **도킹 FSM 로컬 제어** | 로봇 내부 탑재 |
| **원격 PC** | 고성능 CPU/GPU, ROS2 Jazzy | RTAB-Map SLAM, Nav2 내비게이션, RViz2 시각화 | Wi-Fi (DDS 토픽 통신) |

---

### 1.2 노드 분배 및 통신 토폴로지

무거운 원본 Depth 이미지(1280x460 / 640x480)는 Wi-Fi로 쏘지 않고 **라즈베리파이 내부 공유 메모리(Localhost)**에서 직접 처리하여 0.05초 이하의 초저지연 폐루프 제어를 달성합니다. PC에는 스로틀링된 디버그 이미지(3~4Hz)만 전송합니다.

```
+========================================================================================+
|                              [ Raspberry Pi 4 (Robot Local) ]                          |
|                                                                                        |
|  +---------------------+                                                               |
|  | Bumper Node         | --- [/robot/bumper/contact] ---> +-------------------------+  |
|  | (GPIO 23 / 50Hz)    |                                  |                         |  |
|  +---------------------+                                  | Precision Approacher    |  |
|                                                           | Node                    |  |
|  +---------------------+                                  | (FSM & Depth Algorithm) |  |
|  | OS30A Driver        | --- [/camera/depth/image_raw] -> |                         |  |
|  | (USB 3.0 / 15fps)   |                                  +-------------------------+  |
|  +---------------------+                                               |               |
|            ^                                                           v               |
|            | (Process Start/Stop)                                 [/cmd_vel]           |
|  +---------------------+                                               |               |
|  | Camera Mode Manager |                                               v               |
|  +---------------------+                                  +-------------------------+  |
|            | (Process Start/Stop)                         | TurtleBot3 Core Node    |  |
|            v                                              | (OpenCR Motor Control)  |  |
|  +---------------------+                                  +-------------------------+  |
|  | Astra S Driver      |                                                               |
|  +---------------------+                                                               |
+======================================= | =============================== ^ ============+
                                         | (Wi-Fi DDS / 3~4Hz Throttled)   |
                                         v                                 |
+========================================================================= | ============+
|                                  [ Remote PC ]                           |
|                                                                          |
|  +-----------------------+                         +---------------------+----------+  |
|  | RViz2 & Dashboard     | <--- [/docking/debug] - | Nav2 Navigation Stack          |  |
|  | (Visualizer)          |                         | (/start_docking Service Call)  |  |
|  +-----------------------+                         +--------------------------------+  |
|  | RTAB-Map Visual SLAM  | <--- (Astra Color/Depth Image via ROS2 Topic)               |  |
|  +-----------------------+                                                             |  |
+========================================================================================+
```

---

## 2. 기하학 캘리브레이션 및 핵심 파라미터 (검증 완료)

### 2.1 OS30A ➔ `base_footprint` 좌표 변환 모델

카메라가 지면 기준 $Z=0.40\text{m}$ 높이에 위치하고, **수평선 기준 Pitch $\theta = 65^\circ = 1.13437\text{ rad}$ 하향**으로 마운트되어 있습니다.

$$
\begin{bmatrix} X_{base} \\ Y_{base} \\ Z_{base} \end{bmatrix} = \begin{bmatrix} \sin\theta & 0 & \cos\theta \\ 0 & -1 & 0 \\ \cos\theta & 0 & -\sin\theta \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} + \begin{bmatrix} 0.05 \\ 0 \\ 0.40 \end{bmatrix}
$$

### 2.2 화각(FOV) 기하학적 유효 감지 영역

* **수직 화각(FOV)**: 약 $50^\circ$ ($\pm 25^\circ$)
* **지면($Z=0$) 중심 투영점**: $X = 0.40 / \tan(65^\circ) \approx 0.186\text{m}$ (로봇 앞 18.6cm)
* **책상 상판($Z=0.10\text{m}$) 중심 투영점**: $X = (0.40 - 0.10) / \tan(65^\circ) \approx 0.140\text{m}$ (로봇 앞 14.0cm)
* **책상 유효 시야(FOV) 한계선**:
  - 상단 경계 ($40^\circ$ 각도): $X_{max} \approx 0.30 / \tan(40^\circ) \approx \mathbf{0.36\text{m} \sim 0.45\text{m}}$
  - 하단 경계 ($90^\circ$ 수직선): $X_{min} \approx \mathbf{0.05\text{m}}$
* **설계 반영**: Nav2 스테이징 도착 지점을 **책상 전방 $0.40\text{m} \sim 0.45\text{m}$**로 설정하여 로봇이 멈추자마자 OS30A에 책상이 즉시 잡히도록 구성.

### 2.3 ROI 높이 슬라이스 및 평면 수평성 검증

* **실물 책상 높이**: $10\text{cm} \sim 11\text{cm}$ ($0.10\text{m} \sim 0.11\text{m}$)
* **유효 Z 슬라이스**: $0.090\text{m} \le Z_{base} \le 0.125\text{m}$ (바닥 $0.0\text{m}$ 완벽 차단)
* **평면 기울기 검증**: $\text{np.linalg.lstsq}$ 평면 피팅 $Z = aX + bY + d$ 수행, $|slope_x| \le 0.25$ ($14^\circ$ 이내) 통과 시에만 책상으로 승인.

---

## 3. 정밀 도킹 FSM 제어 흐름 (실물 하드웨어 반영)

```
[INIT] ──(거리 멀면)──▶ [WAIT_NAV2] ──(책상 포착 시 취소)──┐
  │                                                        ▼
  └───────(이미 책상 근처)──────────────────────────▶ [CHECK_STAGING] (거리/오차 평가)
                                                             │
         ┌───────────────────┬───────────────────────────────┤
         ▼                   ▼                               ▼
     (거리<35cm)         (횡오차 > 4cm)              (횡오차 1.2~4cm)       (횡오차 ≤ 1.2cm)
 [BACKUP_STANDOFF]    [CASE B: 직각 우회]           [CASE A: 당근점 추종]         │
  (40cm로 후진)       TURN_LATERAL (90° 회전)        CARROT_ALIGN (완만 곡선)      │
         │                   │                               │                    │
         └──────────▶  LATERAL_CRUISE (횡이동)               │                    │
                             │                               │                    │
                       TURN_FACE_DESK (-90° 회전)            │                    │
                             │                               ▼                    ▼
                             └───────────────────────▶ [ALIGN_PARALLEL] (평행 정렬)
                                                             │
                                                             ▼
                                                      [FINAL_APPROACH] (2.5cm/s 저속 전진)
                                                             │
                                                   (범퍼 접촉 or 5.0cm 도달)
                                                             ▼
                                                         [DOCKED] (모터 완전 정지)
```

| 상태 (State) | 제어 속도 | 전환 조건 (Next Trigger) | 실물 안전 장치 (Fail-Safe) |
| :--- | :--- | :--- | :--- |
| **`CHECK_STAGING`** | 정지 / 미세 전진 ($0.04\text{m/s}$) | 거리 $0.35\sim 0.45\text{m}$ 확보 시 오차별 분기 | 25초 미검출 시 Abort |
| **`TURN_LATERAL`** | $w_z = \pm 0.20\text{ rad/s}$ | 제자리 $90^\circ$ 회전 완료 ($|\Delta yaw| \le 2.5^\circ$) | 7.85초 타임아웃 |
| **`LATERAL_CRUISE`**| $v_x = 0.04\text{ m/s}$ | 횡변위 이동량 추종 ($d \ge \|e_y\| - 1\text{cm}$) | 오도메트리 거리 연산 |
| **`TURN_FACE_DESK`**| $w_z = \mp 0.20\text{ rad/s}$ | 원점 헤딩으로 반대 $90^\circ$ 회전 완료 | 7.85초 타임아웃 |
| **`CARROT_ALIGN`**  | $v_x = 0.03\text{ m/s}$, Pure Pursuit | 전방 거리 $\le 0.17\text{m}$ 또는 $\|e_y\| \le 1.0\text{cm}$ | 20cm 당근점 추종 |
| **`ALIGN_PARALLEL`**| $w_z = -K_{yaw} \cdot e_\theta$ | 에지 평행 오차 $\|e_\theta\| \le 1.5^\circ$ | 제자리 회전, 8초 제한 |
| **`FINAL_APPROACH`**| **$v_x = 0.025\text{ m/s}$ (2.5cm/s)** | **범퍼 접촉 (`contact == True`)** or 광학 5cm | **스톨(Stall) 감지 및 모터 보호** |
| **`DOCKED`**        | $v_x = 0.0, w_z = 0.0$ | 도킹 완료 상태 유지 | 모터 완전 락 |

---

## 4. 상세 구현 및 검증 계획 (Step 2 ~ Step 4)

### 📌 Step 2: 비전 필터링 및 책상 정적 캘리브레이션 (차기 작업)

1. **실제 환경 배치**:
   - 로봇을 10~11cm 높이의 책상 앞 약 $0.38\text{m} \sim 0.42\text{m}$ 지점에 배치.
2. **Static TF 및 카메라 드라이버 구동**:
   - 라즈베리파이에서 OS30A 드라이버 실행.
   - `base_footprint` $\to$ `os30a_camera_link` TF 발행 ($Z=0.40\text{m}$, Pitch $65^\circ = 1.1344\text{ rad}$).
3. **정적 검증 스크립트 실행 (`precision_approacher`)**:
   - PC RViz2에서 `/docking/debug_image` 및 `/marker/desk_edge` 토픽 구독.
   - 책상 앞 모서리에 초록색 직선(Marker)이 흔들림 없이 일치하는지 확인.
   - 실제 줄자로 잰 거리와 토픽의 `desk_distance` 오차가 $\pm 1\text{cm}$ 이내인지 검증 및 `cam_pitch` 미세 튜닝.

### 📌 Step 3: FSM 정밀 도킹 단독 동적 테스트

1. 로봇을 책상 앞 $0.45\text{m}$ 지점에 살짝 비틀어진 각도($\pm 10^\circ$)와 횡오차($\pm 3\text{cm}$)로 배치.
2. 도킹 트리거 호출:
   ```bash
   ros2 service call /start_docking std_srvs/srv/Trigger
   ```
3. 상태 전환 순서 및 궤적 검증:
   - `CHECK_STAGING` $\to$ `CARROT_ALIGN` (중앙선 진입) $\to$ `ALIGN_PARALLEL` (평행 정렬) $\to$ `FINAL_APPROACH` (2.5cm/s 크롤링) $\to$ **범퍼 딸깍 접촉 (`/robot/bumper/contact == true`)** $\to$ 즉시 정지 (`DOCKED`).
4. 정지 후 로봇과 책상 사이의 최종 클리어런스가 정확히 $5\text{cm} \pm 0.5\text{cm}$인지 측정.

### 📌 Step 4: Visual SLAM + Nav2 + 카메라 시분할 엔드투엔드 통합

1. **`camera_mode_manager_node.py` 프로세스 수명주기 연동**:
   - Nav2 자율주행 모드: Astra S 구동, OS30A 중지.
   - 도킹 스테이징 도착 시: Astra S 종료 $\to$ USB 안정화 $0.5$초 $\to$ OS30A 실행 ➔ 정밀 도킹 FSM 가동.
2. **원클릭 맵 주행 및 도킹 테스트**:
   - PC에서 `2_start_3d_nav.sh` 실행.
   - 책상 앞 0.45m 지점을 2D Nav Goal로 지정.
   - 로봇이 스스로 찾아가 도착 후 자동으로 OS30A로 전환하여 책상 아래로 5cm 정밀 도킹을 완수하는지 전체 사이클 검증.
