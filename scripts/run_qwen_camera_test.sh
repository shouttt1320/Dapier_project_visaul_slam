#!/usr/bin/env bash
# ==============================================================================
# Qwen2.5-VL 3D Object & Depth Interactive Test Launcher
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "======================================================================"
echo " 👁️ Qwen2.5-VL 3D Object Detection & Depth Interactive Test"
echo "======================================================================"

# 1. Ollama 데몬 실행 확인
if ! pgrep -f "ollama serve" > /dev/null; then
    echo ">> Ollama 데몬을 백그라운드에서 시작합니다..."
    ollama serve > /dev/null 2>&1 &
    sleep 2
fi

# 2. 환경 변수 설정
source /opt/ros/jazzy/setup.bash
if [ -f "$PROJECT_ROOT/robot_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/robot_ws/install/setup.bash"
fi
export LD_LIBRARY_PATH="/home/jjj/.local/lib:$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=101

# 3. Astra S 카메라 드라이버 실행 (기존 좀비 프로세스 정리 후 신선한 연결 보장)
pkill -9 -f astra_camera_node 2>/dev/null || true
sleep 0.5

echo ">> Astra S 카메라 드라이버를 백그라운드에서 실행합니다..."
ros2 launch astra_camera astra.launch.xml \
    depth_width:=320 depth_height:=240 \
    color_width:=320 color_height:=240 \
    enable_point_cloud:=false depth_registration:=true > /tmp/astra_pc_test.log 2>&1 &
CAM_PID=$!

echo "   카메라 초기화 대기 중 (3초)..."
sleep 3

# 종료 시 카메라 완전 종료
cleanup() {
    echo -e "\n>> 테스트용 카메라 노드를 정리합니다..."
    if [ -n "$CAM_PID" ]; then
        kill -9 $CAM_PID 2>/dev/null || true
    fi
    pkill -9 -f astra_camera_node 2>/dev/null || true
}
trap cleanup EXIT


# 4. Qwen 3D 검출 테스트 스크립트 실행
python3 "$PROJECT_ROOT/diagnostics/test_qwen_depth_detector.py"
