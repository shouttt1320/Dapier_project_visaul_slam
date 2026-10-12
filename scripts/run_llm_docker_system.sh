#!/usr/bin/env bash
# ==============================================================================
# Qwen2.5-VL Multimodal Autonomous Navigation & Precision Docking Launcher
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "======================================================================"
echo " 🤖 Qwen2.5-VL Multimodal Autonomous Navigation & Precision Docking"
echo "======================================================================"

# 1. Ollama 데몬 확인
export PATH="/home/jjj/.local/bin:$PATH"
if ! pgrep -f "ollama serve" > /dev/null; then
    echo ">> Ollama 데몬을 백그라운드에서 시작합니다..."
    ollama serve > /tmp/ollama_daemon.log 2>&1 &
    sleep 2
fi

# 2. ROS 2 환경 설정
source /opt/ros/jazzy/setup.bash
if [ -f "$PROJECT_ROOT/robot_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/robot_ws/install/setup.bash"
fi
export LD_LIBRARY_PATH="/home/jjj/.local/lib:$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=101

# 3. 모드 감지 (로컬 직결 vs 실제 로봇 Wi-Fi 압축 모드)
TARGET_MODE="${1:-auto}"

if [ "$TARGET_MODE" == "auto" ]; then
    if lsusb | grep -qi "2bc5:0402"; then
        TARGET_MODE="local"
        echo ">> [자동 감지] 노트북에 Astra S USB 연결 확인 -> '로컬 직결 모드'로 실행합니다."
    else
        TARGET_MODE="robot"
        echo ">> [자동 감지] 노트북에 Astra S 미연결 -> '실제 로봇 Wi-Fi 압축 모드'로 실행합니다."
    fi
fi

CAM_PID=""
cleanup() {
    echo -e "\n>> 시스템을 안전하게 종료합니다..."
    if [ -n "$CAM_PID" ]; then
        kill -9 $CAM_PID 2>/dev/null || true
    fi
    pkill -9 -f astra_camera_node 2>/dev/null || true
    pkill -9 -f agent_cli.py 2>/dev/null || true
    pkill -9 -f qwen_semantic_gui_node 2>/dev/null || true
}
trap cleanup EXIT

# 4. 전용 인터랙티브 CLI 콘솔 창 (GUI에 텍스트 입력창이 완전 통합되어 별도 팝업 비활성화)
# launch_interactive_cli() {
#     sleep 2.5  # ROS 2 노드 초기화 및 VLM 사전 워밍업 대기
#     if which gnome-terminal > /dev/null 2>&1 && [ -n "$DISPLAY" ]; then
#         echo ">> 🎮 전용 대화형 CLI 콘솔 창(gnome-terminal)을 자동으로 엽니다..."
#         gnome-terminal --title="🤖 Qwen2.5-VL Interactive CLI Console" --geometry=95x28 -- bash -c "
#             source /opt/ros/jazzy/setup.bash
#             if [ -f '$PROJECT_ROOT/robot_ws/install/setup.bash' ]; then
#                 source '$PROJECT_ROOT/robot_ws/install/setup.bash'
#             fi
#             export ROS_DOMAIN_ID=101
#             python3 '$PROJECT_ROOT/scripts/agent_cli.py'
#         " 2>/dev/null || true
#     fi
# }
# launch_interactive_cli &

if [ "$TARGET_MODE" == "local" ]; then
    echo ">> [MODE: LOCAL] Astra S 카메라 드라이버를 백그라운드에서 직접 구동합니다..."
    pkill -9 -f astra_camera_node 2>/dev/null || true
    sleep 0.5

    ros2 launch astra_camera astra.launch.xml \
        depth_width:=320 depth_height:=240 \
        color_width:=320 color_height:=240 \
        enable_point_cloud:=false depth_registration:=true > /tmp/astra_live_system.log 2>&1 &
    CAM_PID=$!

    echo "   카메라 초기화 대기 중 (3초)..."
    sleep 3

    # 4. 로컬 직결 토픽으로 통합 런치 실행
    ros2 launch llm_semantic_docker llm_semantic_docker.launch.py \
        color_topic:=/camera/color/image_raw \
        depth_topic:=/camera/depth/image_raw \
        cam_info_topic:=/camera/color/camera_info

else
    echo ">> [MODE: ROBOT] 실제 로봇(라즈베리파이) Wi-Fi 압축 영상 스트림 모드로 실행합니다."
    echo "   (주의: PC에서 run_real_robot_pc.sh 실행 중이면 decompressed 토픽 자동 연동)"

    # compressed_bridge_node 가 실행 중이면 decompressed 토픽 사용, 아니면 raw/compressed 직접 수신
    if pgrep -f "compressed_bridge_node" > /dev/null; then
        echo "   -> compressed_bridge_node 감지됨: Decompressed 고속 동기화 토픽 사용"
        ros2 launch llm_semantic_docker llm_semantic_docker.launch.py \
            color_topic:=/camera/color/image_decompressed \
            depth_topic:=/camera/depth/image_decompressed \
            cam_info_topic:=/camera/color/decompressed/camera_info
    else
        echo "   -> 라즈베리파이 Wi-Fi 압축 토픽 직접 수신 (/compressed, /compressedDepth)"
        ros2 launch llm_semantic_docker llm_semantic_docker.launch.py \
            color_topic:=/camera/color/image_raw \
            depth_topic:=/camera/depth/image_raw \
            cam_info_topic:=/camera/color/camera_info
    fi
fi
