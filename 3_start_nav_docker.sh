#!/usr/bin/env bash
# ==============================================================================
# 3. Autonomous 3D Visual Navigation & Precision Docking (Qwen2.5-VL + RTAB-Map + Nav2)
# ==============================================================================
# One-click Master Launcher for:
# - Ollama Local VLM Daemon (qwen2.5vl:3b)
# - Raspberry Pi Robot Time Sync & Remote Node Bringup (or Local Astra S)
# - RTAB-Map Pure 3D Visual Localization (my_office_3d.db)
# - Nav2 Global/Local Path Planning & Dynamic Obstacle Avoidance
# - 3D Object Detection & RANSAC Surface Plane Fitting
# - 8-State Autonomous FSM Task Manager (Docking & Waypoint Navigation)
# - Qwen Semantic Cockpit GUI + Interactive CLI Terminal
# ==============================================================================

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/home/jjj/.local/bin:$PATH"

echo "======================================================================"
echo " 🚀 [Step 0] Cleaning up stale ROS 2 processes..."
echo "======================================================================"
pkill -9 -f "qwen_semantic_gui" 2>/dev/null || true
pkill -9 -f "compressed_bridge_node" 2>/dev/null || true
pkill -9 -f "llm_agent_node" 2>/dev/null || true
pkill -9 -f "task_manager_node" 2>/dev/null || true
pkill -9 -f "precision_docking_controller_node" 2>/dev/null || true
pkill -9 -f "object_detector_3d" 2>/dev/null || true
pkill -9 -f "rtabmap" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "point_cloud_xyzrgb" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "agent_cli.py" 2>/dev/null || true
pkill -9 -f "astra_camera_node" 2>/dev/null || true
sleep 1

# 1. Ollama 데몬 상태 확인 및 백그라운드 자동 기동
if ! pgrep -f "ollama serve" > /dev/null; then
    echo ">> [Ollama] 데몬을 백그라운드에서 기동합니다..."
    ollama serve > /tmp/ollama_nav_docker.log 2>&1 &
    sleep 2
fi

# 2. 파라미터 파싱 및 구동 모드 결정 (기본 auto, --local, --robot, --sim)
MODE="auto"
OPEN_RVIZ="true"
ENABLE_GUI="true"
USE_SIM="false"
DB_PATH="$PROJECT_ROOT/my_office_3d.db"

for arg in "$@"; do
    case $arg in
        --local) MODE="local" ;;
        --robot) MODE="robot" ;;
        --sim)   MODE="sim"; USE_SIM="true" ;;
        --no-rviz) OPEN_RVIZ="false" ;;
        --no-gui)  ENABLE_GUI="false" ;;
        --db=*)  DB_PATH="${arg#*=}" ;;
    esac
done

if [ "$MODE" == "auto" ]; then
    if lsusb | grep -qi "2bc5:0402"; then
        MODE="local"
        echo ">> [모드 자동감지] PC에 Astra S 카메라 직접 연결 확인 -> '로컬 직결 모드'로 실행합니다."
    else
        MODE="robot"
        echo ">> [모드 자동감지] PC에 카메라 직결 없음 -> '실제 로봇(TurtleBot3 Wi-Fi) 모드'로 실행합니다."
    fi
fi

CAM_PID=""
cleanup() {
    echo -e "\n>> 시스템을 안전하게 종료합니다..."
    if [ -n "$CAM_PID" ]; then
        kill -9 "$CAM_PID" 2>/dev/null || true
    fi
    pkill -9 -f "astra_camera_node" 2>/dev/null || true
    pkill -9 -f "precision_docking_controller_node" 2>/dev/null || true
    pkill -9 -f "qwen_semantic_gui" 2>/dev/null || true
    pkill -9 -f "agent_cli.py" 2>/dev/null || true
}
trap cleanup EXIT

# 3. 모드별 하드웨어 초기화
if [ "$MODE" == "robot" ]; then
    echo "=================================================================="
    echo " 📡 [Step 1] 터틀봇(Raspberry Pi: 192.168.0.148) 시간 동기화 및 노드 확인..."
    echo "=================================================================="
    CURRENT_TIME=$(date +%s.%N)
    if ping -c 1 -W 1 192.168.0.148 > /dev/null 2>&1; then
        echo ">> 로봇 시계 정밀 동기화 및 무선랜 절전 해제 (초저지연 스트리밍)..."
        ssh user@192.168.0.148 "echo user | sudo -S timedatectl set-ntp false 2>/dev/null; echo user | sudo -S date -s @$CURRENT_TIME; echo user | sudo -S iwconfig wlan0 power off 2>/dev/null || true" > /dev/null 2>&1 || true
        
        # 로봇 기본 드라이버 구동 여부 확인
        if ! ssh user@192.168.0.148 "pgrep -f 'turtlebot3_ros' > /dev/null 2>&1"; then
            echo ">> 로봇 기본 노드가 꺼져 있어 원격 시작을 요청합니다..."
            ssh user@192.168.0.148 "nohup ~/start_robot_all.sh > /tmp/start_all.log 2>&1 &" || true
            echo ">> 로봇 노드 및 자이로 초기화 대기 (6초)..."
            sleep 6
        else
            echo ">> 로봇 기본 노드가 이미 정상 동작 중입니다."
        fi
    else
        echo "⚠️  [경고] 192.168.0.148(로봇 IP) 응답 없음. Wi-Fi 연결을 확인해주세요."
        echo "   (오프라인 테스트를 진행하거나 PC 직결 모드는 --local 옵션을 사용하세요)"
    fi

elif [ "$MODE" == "local" ]; then
    echo "=================================================================="
    echo " 📷 [Step 1] PC 직결 Astra S 카메라 드라이버 백그라운드 구동..."
    echo "=================================================================="
    source /opt/ros/jazzy/setup.bash
    if [ -f "$PROJECT_ROOT/robot_ws/install/setup.bash" ]; then
        source "$PROJECT_ROOT/robot_ws/install/setup.bash"
    fi

    ros2 launch astra_camera astra.launch.xml \
        depth_width:=320 depth_height:=240 \
        color_width:=320 color_height:=240 \
        enable_point_cloud:=false depth_registration:=true > /tmp/astra_nav_docker.log 2>&1 &
    CAM_PID=$!
    echo "   카메라 초기화 대기 중 (3초)..."
    sleep 3
fi

# 4. 대화형 인터랙티브 터미널 (GUI에 텍스트 입력창이 완전 통합되어 별도 팝업 비활성화)
# launch_interactive_cli() {
#     sleep 4
#     if which gnome-terminal > /dev/null 2>&1 && [ -n "$DISPLAY" ]; then
#         echo ">> 🎮 대화형 음성/텍스트 CLI 콘솔 창을 새 터미널로 엽니다..."
#         gnome-terminal --title="🤖 Qwen2.5-VL Mission Command Console" --geometry=95x28 -- bash -c "
#             source /opt/ros/jazzy/setup.bash
#             source '$PROJECT_ROOT/robot_ws/install/setup.bash'
#             export ROS_DOMAIN_ID=101
#             python3 '$PROJECT_ROOT/scripts/agent_cli.py'
#         " 2>/dev/null || true
#     fi
# }
# launch_interactive_cli &

# 5. ROS 2 환경 설정 및 통합 마스터 런치 구동
echo "=================================================================="
echo " 🌐 [Step 2] 통합 Navigation & LLM Semantic Docker 기동..."
echo "  - DB 파일: $DB_PATH"
echo "  - RViz2: $OPEN_RVIZ, GUI: $ENABLE_GUI, SimTime: $USE_SIM"
echo "=================================================================="

source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=101
export TURTLEBOT3_MODEL=waffle_pi
export LD_LIBRARY_PATH="/home/jjj/.local/lib:$LD_LIBRARY_PATH"

exec ros2 launch llm_semantic_docker llm_nav_docker_integrated.launch.py \
    use_sim_time:="$USE_SIM" \
    database_path:="$DB_PATH" \
    open_rviz:="$OPEN_RVIZ" \
    enable_gui:="$ENABLE_GUI"
