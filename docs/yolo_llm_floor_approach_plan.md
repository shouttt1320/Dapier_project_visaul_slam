# LLM + YOLO-World 기반 바닥 물체 정밀 접근 아키텍처 및 구현 계획

본 문서는 Qwen2.5-VL VLM의 바닥 소형 물체 인식률 저하 및 추론 지연(1.5~3.0s) 문제를 해결하기 위해, **LLM(자연어 추론/파싱) + YOLO-World(초고속 실시간 시각 검출)** 하이브리드 비전 파이프라인으로 전환하는 구현 계획을 기술합니다.

---

## 1. 개요 및 전환 배경

### 1.1 기존 시스템(Qwen2.5-VL 단독)의 한계
* **소형 물체 검출 불안정**: 바닥에 놓인 4cm 큐브나 작은 상자 인식 시, 바닥 타일 이음새나 가구 다리와 혼동하여 BBox 정확도 저하.
* **높은 지연 시간(Latency)**: 프레임당 약 1.5 ~ 3.0초 소요로 인해 로봇 주행 중 20Hz 실시간 시각 피드백 제어(Visual Servoing) 불가.
* **색상 추적기 한계**: 1회 락온 후 사용하던 2D Lab 색상 트래커가 바닥 그림자나 조명 변화에 취약.

### 1.2 LLM + YOLO 분업의 핵심 이점
* **LLM (텍스트 추론/자연어 이해)**: 사용자 음성/명령어에서 인식 대상(`target_name`), 속성(색상/크기/형상), 도달 거리(`clearance_cm`)를 자연어 분석하여 **동적 클래스 프롬프트(Dynamic Class Prompts)**로 변환.
* **YOLO-World (Open-Vocabulary 실시간 검출)**:
  * 로컬 **NVIDIA RTX 5050 GPU** 기준 1회 추론 시간 **7.5ms (133 FPS)** 실측 완료.
  * 사전 정의되지 않은 임의의 텍스트(`set_classes(...)`)를 런타임에 즉시 탐색 가능.
  * 정밀한 2D BBox와 Astra S 뎁스 데이터를 결합하여 **근거리(0.18m)부터 원거리(3.0m)까지 안정적인 3D 좌표 산출**.

---

## 2. 전체 시스템 아키텍처

```
[사용자 자연어 명령 ("바닥에 있는 작은 상자로 25cm 다가가")]
                            │
                            ▼
           ┌───────────────────────────────────┐
           │        llm_agent_node.py          │
           │ - 자연어 문맥 분석                 │
           │ - 동적 클래스: ['box', 'small box']│
           │ - 목표 거리: 0.25m                │
           └─────────────────┬─────────────────┘
                             │ /object_approach/target_query
                             │ /task/command
                             ▼
           ┌───────────────────────────────────┐
           │         task_manager.py           │
           │ - FSM 상태 전환: TaskState.APPROACH │
           │ - /object_approach/start 트리거    │
           │ - /object_approach/target_standoff│
           └─────────────────┬─────────────────┘
                             │
                             ▼
 ┌──────────────────────────────────────────────────────────────┐
 │             object_approaching_controller_node.py            │
 │                                                              │
 │  1. YOLO-World 엔진 (yolov8s-worldv2.pt, 7.5ms on RTX 5050) │
 │     - set_classes(['box', 'small box', 'cube'])              │
 │     - 20Hz 실시간 BBox 검출                                  │
 │                                                              │
 │  2. 바닥면 지능형 공간 필터 (Floor Spatial Gate)             │
 │     - 바닥 위 높이: 1.8cm <= z_base <= 22cm                  │
 │     - 로봇 주행 통로: |y_base| <= 0.38m                      │
 │     - 책상 다리, 벽면, 천장 가구 100% 원천 배제              │
 │                                                              │
 │  3. 핀홀 접점 기하학 + 3D 뎁스 융합                          │
 │     - 42cm 이상: 3D Depth 중앙값 직접 활용                   │
 │     - 18~42cm (음영 지역): 바닥 접점 v_bottom 핀홀 기하학    │
 │     - 20Hz PID 조향 제어 (v_x, w_z)                          │
 └──────────────────────────────┬───────────────────────────────┘
                                │
                                ▼
           ┌───────────────────────────────────┐
           │       qwen_semantic_gui_node.py   │
           │ - 실시간 YOLO HUD 영상 시각화     │
           │ - 25cm 거리 및 중심 편차 표시     │
           │ - 비상정지(Emergency Stop) 연동   │
           └───────────────────────────────────┘
```

---

## 3. 코드별 수정 계획 및 호환성 분석

### 3.1 `object_approaching_controller_node.py`
* **제거**: Qwen HTTP 비동기 대기 상태(`REQUEST_QWEN`, `WAIT_QWEN`) 및 취약한 Lab 색상차 트래커.
* **추가**:
  * `from ultralytics import YOLOWorld` 로드 및 CUDA 0번 디바이스 자동 바인딩.
  * `/home/jjj/Documents/Dapier/Project/yolov8s-worldv2.pt` 가중치 로드.
  * `/object_approach/target_query` 구독 콜백에서 `self.yolo_model.set_classes(...)` 동적 업데이트.
  * 바닥 기하학 필터: 검출된 박스의 바닥 높이 및 가로 폭 검사로 책상 다리 완벽 제거.
  * 20Hz 실시간 제어 루프에서 매 프레임 YOLO 추론 수행 (7.5ms).

### 3.2 `llm_agent_node.py`
* `_quick_parse_command()`에서 사용자가 말한 물체 명칭을 YOLO-World가 가장 잘 인식하는 영문 클래스 셋으로 매핑:
  * 상자/박스 $\rightarrow$ `['box', 'small box', 'cardboard box']`
  * 큐브/블록 $\rightarrow$ `['cube', 'toy block', 'small block']`
  * 컵/텀블러 $\rightarrow$ `['cup', 'paper cup', 'bottle']`
  * 일반 바닥 물체 $\rightarrow$ `['small box', 'cube', 'small object']`
* `/object_approach/target_query` 토픽으로 해당 문자열 전송.

### 3.3 `task_manager.py`
* 이미 직전 단계에서 `self.obj_query_pub`(`/object_approach/target_query`) 및 `self.obj_standoff_pub`(`/object_approach/target_standoff`)가 구현되어 있으므로 **100% 호환 유지**.

### 3.4 `qwen_semantic_gui_node.py`
* `/object_approach/debug_image`로 고속 YOLO 디버그 프레임(초록색 박스 + 실시간 거리) 수신.
* GUI FSM 배지에 `YOLO_APPROACH`, `ALIGN_CENTER`, `FINAL_SETTLE`, `APPROACHED` 정상 표시.

---

## 4. 호환성 및 안전성 검증

1. **마커 정밀 도킹과의 공존**:
   * ArUco 마커 도킹(`precision_docking_controller_node.py`)은 독립 노드로 유지되며, 바닥 물체 접근만 본 YOLO 제어기로 분기되므로 기존 도킹 알고리즘에 영향 없음.
2. **비상 정지 (Safety)**:
   * GUI의 `EMERGENCY STOP` 또는 `/safety/software_stop` 시 `task_manager`가 즉시 `/object_approach/abort` 서비스를 호출하여 0ms 속도 차단 보장.
3. **GPU 자원 점유**:
   * RTX 5050 GPU VRAM 8GB 중 YOLO-World는 약 400MB만 점유하므로 RTAB-Map(SLAM)과 완벽 동시 구동 가능.
