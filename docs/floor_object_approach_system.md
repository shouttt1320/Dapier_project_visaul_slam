# 🤖 바닥 타겟 물체 정밀 접근 (Floor Object Approach) 시스템 명세서

본 문서는 TurtleBot3 Waffle Pi 로봇이 바닥에 놓인 소형 물체(큐브, 공, 막대 등)를 원거리에서 발견하고, 자율주행 중단 후 cm 단위로 정밀하게 접근하여 정지하는 **Floor Object Approach & Visual Servoing 시스템**의 구조와 알고리즘을 설명합니다.

---

## 📌 1. 시스템 아키텍처 개요

시스템은 **[지능 인식 계층] ➔ [임무 스케줄링 계층] ➔ [실시간 서보잉 제어 계층] ➔ [모빌리티 하드웨어 계층]**의 4단계 파이프라인으로 구성됩니다.

```mermaid
flowchart TD
    subgraph SENSORS ["센서 입력"]
        RGB["Astra S Color (320x240)"]
        DEPTH["Astra S Depth (320x240)"]
        ODOM["OpenCR Wheel Odometry (/odom)"]
        BUMPER["Microswitch Bumper (/bumper)"]
    end

    subgraph PERCEPTION ["실시간 20Hz 비전 인식 (object_approaching_controller_node)"]
        YOLO["YOLOv8 Fine-Tuned (CUSTOM Engine)<br/>red_cube, blue_ball, white_tissue"]
        YOLO_WORLD["YOLO-World v2 (Open-Vocabulary Fallback)"]
        HYBRID_DEPTH["하이브리드 3D 뎁스 융합<br/>• Pinhole Ground Geometry (>1.2m)<br/>• Astra S IR 3D Depth (0.4m~1.2m)<br/>• Dead-Reckoning & Spike-Guard (<0.35m)"]
    end

    subgraph MISSION ["임무 제어 FSM (task_manager & llm_agent_node)"]
        PATROL["Nav2 웨이포인트 자율 순찰"]
        INTERRUPT{"바닥 물체 감지?<br/>(0.40m~1.20m, 3프레임 연속)"}
        POPUP["GUI 픽업 승인 대화상자<br/>(PickConfirmationDialog)"]
        RESUME["수거 완료 후 Nav2 복귀"]
    end

    subgraph CONTROLLER ["20Hz 닫힌 루프 제어기"]
        FSM["Approach FSM<br/>SEARCH_ALIGN ➔ APPROACH_OBJECT ➔ FINAL_SETTLE ➔ POST_YAW"]
        PID["적응형 감속 제어 (P-Deceleration & Close-Zone Latch)<br/>Vx = clamp(Kp * error, Vmin, Vmax)<br/>Wz = K_heading * angle + K_lat * Ey"]
        CMD_VEL["/cmd_vel (TwistStamped)"]
    end

    RGB --> YOLO
    DEPTH --> HYBRID_DEPTH
    YOLO --> HYBRID_DEPTH
    YOLO_WORLD --> HYBRID_DEPTH

    HYBRID_DEPTH --> INTERRUPT
    PATROL --> INTERRUPT
    INTERRUPT -- "물체 발견 시 Nav2 정지" --> POPUP
    POPUP -- "사용자 승인 (Approve)" --> FSM
    POPUP -- "건너뛰기 (Skip)" --> RESUME

    ODOM --> HYBRID_DEPTH
    BUMPER --> FSM
    FSM --> PID
    PID --> CMD_VEL
    FSM -- "수거 완료 신호" --> RESUME
```

---

## 🔍 2. 하이브리드 3D 거리 측정 및 급발진 방지 알고리즘 (Hybrid Depth & Spike-Guard)

Astra S 카메라의 물리적 특성(IR 뎁스 유효 거리 0.6m~3.0m, 지면 반사 노이즈, 근접 사각지대)을 극복하고 초근접 거리에서 안전하게 정지하기 위해 **3단계 하이브리드 기하학 융합 + 스파이크 가드**를 적용합니다.

```text
 ┌─────────────────┬─────────────────┬──────────────────────┐
 │  ~ 0.35m 이하   │  0.40m ~ 1.20m  │     1.20m 이상       │
 │ 근접/사각지대   │   안정적 영역   │     원거리 영역      │
 ├─────────────────┼─────────────────┼──────────────────────┤
 │ 스파이크 가드 & │ Astra S IR 뎁스 │ 단안 지면 핀홀 투영  │
 │ 공간 기억 래칭  │ + 지면 기하 융합│ (Pinhole Ground Proj)│
 │  [SPIKE-GUARD]  │    [IR 3D]      │       [GEOM]         │
 └─────────────────┴─────────────────┴──────────────────────┘
```

### 1) 원거리: 단안 지면 핀홀 기하학 (`[GEOM]`)
물체가 1.2m 이상 멀리 떨어져 있을 때는 IR 깊이 센서의 바닥 반사 오차가 커지므로, 물체가 바닥면과 접촉하는 최하단 픽셀 좌표($v_{\text{bottom}}$)와 카메라 장착 높이($h = 14.7\text{cm}$), 물리적 피치 기울기 각도($\theta_{\text{pitch}} = +4.2^\circ$)를 바탕으로 삼각 기하학을 적용합니다:

$$\alpha = \arctan\left(\frac{v_{\text{bottom}} - c_y}{f_y}\right)$$
$$d_z = \frac{h_{\text{lens}}}{\tan(\alpha + \theta_{\text{pitch}})}$$

### 2) 중근거리: Astra S Structured Light 3D 뎁스 (`[IR 3D]`)
물체가 0.4m ~ 1.2m 구간에 있을 때는 물체 바운딩 박스 중앙 $7\times 7$ 픽셀의 깊이 유효값(IQR 필터링)을 추출하여 1cm 단위의 정확한 거리를 획득합니다.

### 3) 초근접 사각지대 & 스파이크 가드 (`[SPIKE-GUARD]` & `[DEAD-RECKONING]`)
로봇이 타겟 정지 거리(`white_tissue`: 13cm, `red_cube`/`blue_ball`: 25cm)까지 접근하면, Astra S의 적외선 프로젝터 사각지대(<0.35m)로 인해 바닥이나 뒷배경 벽면의 먼 깊이 값(0.8m~1.2m)이 물체 영역으로 누출되는 현상이 발생할 수 있습니다. 이를 방어하기 위해 2단계 안전 가드를 탑재했습니다:
* **`[SPIKE-GUARD]` (거리 점프 기각 및 휠 오도메트리 전진 차감)**:
  - 근접 구간에서 이전 필터링 거리 대비 거리가 갑자기 증가(양의 점프 > +3.5cm)하는 스파이크 노이즈가 감지되면, 이를 허위 측정으로 즉시 기각합니다.
  - 대신 실제 로봇 차동 바퀴가 전진한 휠 오도메트리 변위($\Delta d_{\text{odom}}$)를 직전 거리에서 차감(`last_dist - odom_step`)하여 물리적으로 일관된 거리를 보장합니다.
* **`is_close_zone_latched` (근접 최고속도 락 & 5cm 정밀 점진 감속)**:
  - 물체와의 거리가 0.35m 이하로 한 번이라도 떨어지면 래치를 걸어, 어떠한 측정값 변동에도 최고 선속도를 크롤 속도($v_{\text{crawl}} = 0.040\,\text{m/s}$) 이하로 강제 제한합니다.
  - 목표 스탠드오프 도달 5cm 전부터는 최저 $0.020\,\text{m/s}$까지 점진 감속하여 충돌 없이 완벽한 미세 정지를 수행합니다.

---

## 🚦 3. 자율주행 순찰 중 자동 발견 인터럽트 (Patrol Intercept)

로봇이 특정 방이나 목적지로 Nav2 자율주행 중일 때, 바닥에 떨어진 물체를 발견하면 자동으로 반응하는 프로세스입니다:

1. **지속적 백그라운드 스캔**:
   - 로봇이 주행 중일 때 `object_approaching_controller_node`가 20Hz로 카메라 영상을 감시합니다.
2. **신뢰도 연속 검증**:
   - 일시적인 노이즈나 잘못된 검출을 방지하기 위해 **거리 $0.40\text{m} \sim 1.20\text{m}$**, **신뢰도 $0.65$ 이상**으로 **최소 3프레임 연속 감지**되었을 때만 이벤트를 발생시킵니다.
3. **Nav2 주행 즉각 취소 & 모터 정지**:
   - 조건 충족 시 `/floor_object/detected_event` 토픽이 발행됩니다.
   - `task_manager`가 현재 Nav2 자율주행 목표(Goal)를 일시정지하고 로봇을 제자리에 멈춥니다.
4. **GUI 사용자 승인 대화상자**:
   - `PickConfirmationDialog` 창이 뜨며 발견된 물체 이름, 거리, 사진과 함께 승인 대기 카운트다운(15초)을 시작합니다.
   - **승인(Approve)**: 물체 정밀 접근 서보잉을 시작하여 타겟별 목표 정지 거리(`white_tissue`: 13cm, `red_cube`/`blue_ball`: 25cm) 앞까지 이동.
   - **건너뛰기(Skip)**: 물체를 무시하고 원래 목적지로 Nav2 주행 복귀.

---

## 🔄 4. 정밀 접근 제어기 FSM (Finite State Machine)

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> SEARCH_ALIGN: 접근 명령 수신
    SEARCH_ALIGN --> APPROACH_OBJECT: 타겟 3프레임 락온 & 조향 정렬
    APPROACH_OBJECT --> FINAL_SETTLE: 목표 거리 (13cm or 25cm) 도달
    FINAL_SETTLE --> POST_YAW: 속도 0 안정 확인 후 파지 각도 편향 회전
    POST_YAW --> SUCCESS: 각도 편향 회전 완료 (+10° / -30°)
    SUCCESS --> [*]: 로봇팔 수거 대기 (/task/ack 발행)

    SEARCH_ALIGN --> ABORTED: 타임아웃 / 미발견
    APPROACH_OBJECT --> ABORTED: E-STOP / 범퍼 충돌
```

* **`SEARCH_ALIGN`**: 제자리 회전하여 타겟이 카메라 정중앙($|E_y| < 2\text{cm}$)에 오도록 각도를 정렬합니다.
* **`APPROACH_OBJECT`**:
  - 적응형 P-감속 및 근접 구역 속도 캡을 적용합니다:
    - 원거리($d_{\text{err}} > 0.55\text{m}$): $v_x = \text{clamp}(0.50 \cdot d_{\text{err}}, 0.040, 0.065)\,\text{m/s}$
    - 근접 래칭 구역($d \le 0.35\text{m}$): $v_{\text{max}} = 0.040\,\text{m/s}$, 종단 5cm에서 $v_x = \text{clamp}(0.40 \cdot d_{\text{err}} + 0.015, 0.020, 0.040)\,\text{m/s}$
    - $\omega_z = K_{\text{heading}} \cdot \theta_{\text{err}} + K_{\text{lat}} \cdot E_y$
* **`FINAL_SETTLE`**: 목표 거리 도달 시 속도를 0으로 유지하고 5프레임 이상 정적 안정을 확인합니다.
* **`POST_YAW` (파지 최적화 편향 회전)**:
  - 안착 완료 후 로봇팔 그리퍼의 기구적 파지 방향 정렬을 위해 타겟 프로파일(`floor_target_objects.yaml`)에 지정된 편향 각도로 자동 제자리 회전합니다:
    - `white_tissue`: **좌회전 +10.0°**
    - `red_cube`: **좌회전 +10.0°**
    - `blue_ball`: **우회전 -30.0°**
* **`SUCCESS`**: 회전 완료 후 로봇팔 제어기로 `/task/ack` `{"status": "APPROACH_DONE"}` 신호를 발행하고 팔의 수거 완료(`PICK_DONE`)를 대기합니다.

---

## 🛡️ 5. 안전 장치 (Safety Features)

1. **하드웨어 마이크로스위치 범퍼 센서**:
   - 전면 범퍼가 물리적으로 물체나 장애물에 닿는 순간 즉시 전원 차단 수준으로 모터를 정지시킵니다.
2. **동적 타임아웃 가드**:
   - 접근 중 6초 이상 목표 거리에 도달하지 못하거나 바닥 물체가 가려지면 작업을 강제 중단하고 안전 정지합니다.
3. **GUI 비상 정지 (Emergency Stop)**:
   - UI 우측 상단의 빨간색 [긴급 정지] 버튼 클릭 시 즉시 0 속도 발행 및 모든 FSM이 ABORT 됩니다.
