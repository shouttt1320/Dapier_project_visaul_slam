#!/usr/bin/env bash
# ==============================================================================
# [Teach Docking Station via RViz 3D Gizmo]
# Loads the stored 3D Map, opens RViz, and launches the 3D Interactive Gizmo
# to teach and save docking station coordinates & point cloud template (.pcd).
# ==============================================================================
# Remove strict set -e to prevent premature exit on pipe checks
set +e

# Default arguments
STATION_NAME="${1:-workplace1}"
BOX_X="${2:-0.35}"
BOX_Y="${3:-0.40}"
BOX_Z="${4:-0.25}"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="${PROJECT_ROOT}/robot_ws"
MAP_DB="${5:-${PROJECT_ROOT}/my_office_3d.db}"

# ANSI Colors
CYAN='\033[1;36m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
BLUE='\033[1;34m'
BOLD='\033[1m'
NC='\033[0m'

clear
echo -e "${CYAN}==============================================================================${NC}"
echo -e "${BOLD}       🤖 [TurtleBot3] 3D Visual Docking Station Teaching System             ${NC}"
echo -e "${CYAN}==============================================================================${NC}"
echo -e " 🏷️  Target Station Name : ${GREEN}${STATION_NAME}${NC}"
echo -e " 📦 Bounding Box Size   : ${YELLOW}Width: ${BOX_X}m | Depth: ${BOX_Y}m | Height: ${BOX_Z}m${NC}"
echo -e " 🗺️  3D Map DB           : ${BLUE}${MAP_DB}${NC}"
echo -e "${CYAN}------------------------------------------------------------------------------${NC}"

# Check Map DB
if [ ! -f "$MAP_DB" ]; then
    echo -e "${RED}[ERROR] Map database not found at: ${MAP_DB}${NC}"
    echo -e "Please ensure '$MAP_DB' exists."
    exit 1
fi

# Auto-sync/export latest 3D point cloud from DB if PLY is missing or older than DB
DB_DIR="$(dirname "$MAP_DB")"
DB_STEM="$(basename "$MAP_DB" .db)"
CLOUD_PLY="${DB_DIR}/${DB_STEM}_cloud.ply"
if [ ! -f "$CLOUD_PLY" ] || [ "$MAP_DB" -nt "$CLOUD_PLY" ]; then
    echo -e "${YELLOW}⚡ Synchronizing latest 3D point cloud from ${MAP_DB}...${NC}"
    rtabmap-export --cloud "$MAP_DB" > /dev/null 2>&1 || true
    echo -e "${GREEN}✔ 3D Point Cloud synchronized: ${CLOUD_PLY}${NC}"
fi

# Source ROS 2 Environment
source /opt/ros/jazzy/setup.bash
source "${ROBOT_WS}/install/setup.bash"
export ROS_DOMAIN_ID=101
export TURTLEBOT3_MODEL=waffle_pi

# Check if Map is already active or needs standalone map server
ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null || true)
START_MAP_SERVER="true"
if echo "$ACTIVE_TOPICS" | grep -q "/rtabmap/cloud_map"; then
    echo -e "${GREEN}✔ Active /rtabmap/cloud_map detected! Attaching to running SLAM session.${NC}"
    START_MAP_SERVER="false"
else
    echo -e "${YELLOW}ℹ Starting standalone 3D Map Server from DB for offline teaching...${NC}"
    START_MAP_SERVER="true"
fi

echo -e "\n${BOLD}${GREEN}==============================================================================${NC}"
echo -e "${BOLD}${GREEN}                   📖 [도킹 위치 지정 방법 안내 가이드]                       ${NC}"
echo -e "${BOLD}${GREEN}==============================================================================${NC}"
echo -e " ${BOLD}1. RViz 3D 뷰 조작:${NC}"
echo -e "    - ${CYAN}마우스 좌클릭 드래그${NC}: 3D 맵 화면 회전"
echo -e "    - ${CYAN}마우스 휠 굴리기${NC}    : 줌 인 / 줌 아웃"
echo -e "    - ${CYAN}마우스 휠 누르고 드래그${NC}: 화면 평행 이동 (Pan)"
echo ""
echo -e " ${BOLD}2. 초록색 3D 기즈모(큐브) 조작 (Interact 모드):${NC}"
echo -e "    - 상단 툴바에서 ${CYAN}'Interact'${NC} 툴(단축키: ${BOLD}i${NC})을 선택합니다."
echo -e "    - ${YELLOW}[순간이동]${NC} 상단 ${CYAN}'2D Goal Pose'${NC}로 원하는 바닥을 콕 찍으면 기즈모가 그 자리로 즉각 순간이동합니다!"
echo -e "    - ${YELLOW}[상자 직접 드래그]${NC} ${GREEN}초록색 상자 자체${NC}를 마우스로 잡고 끌면 바닥(XY 평면) 위에서 자유롭게 이동합니다."
echo -e "    - ${YELLOW}[미세 이동]${NC} 상자 주변의 빨강/초록/파랑 화살표 핸들을 드래그하여 X/Y/Z 방향으로 정밀 배치합니다."
echo -e "    - ${YELLOW}[방향 회전]${NC} 원형 링 핸들을 돌려 ${RED}빨간색 전면 화살표${NC}가 ${BOLD}로봇이 들어올 정면${NC}을 향하게 맞춥니다."
echo -e "    - ${YELLOW}[노란 원기둥]${NC} 상자 앞 60cm 위치에 로봇이 멈출 ${BOLD}Standoff 목표점${NC}이 자동 연동 표시됩니다."
echo ""
echo -e " ${BOLD}3. 박스 크기 실시간 조절 (마우스 우클릭 메뉴):${NC}"
echo -e "    - 상자 위에서 ${GREEN}마우스 우클릭${NC} $\to$ ${CYAN}[Adjust Box Dimensions]${NC} 서브메뉴 선택"
echo -e "      • ${BOLD}Scale All (+/- 10%)${NC} : 전체 크기 확대/축소"
echo -e "      • ${BOLD}Width X (+/- 5cm)${NC}   : 가로 폭(X) 미세 조절"
echo -e "      • ${BOLD}Depth Y (+/- 5cm)${NC}   : 깊이 폭(Y) 미세 조절"
echo -e "      • ${BOLD}Height Z (+/- 5cm)${NC}  : 높이(Z) 미세 조절"
echo -e "      • ${BOLD}Reset to Default${NC}    : 초기 크기로 리셋"
echo -e "    - 또는 터미널에서 명령: ${YELLOW}ros2 topic pub --once /docking/set_box_size geometry_msgs/msg/Vector3 \"{x: 0.4, y: 0.4, z: 0.3}\"${NC}"
echo ""
echo -e " ${BOLD}4. 도킹 지점 원클릭 저장:${NC}"
echo -e "    - 상자 위에서 ${GREEN}마우스 우클릭${NC} $\to$ ${BOLD}[Save Station (YAML)]${NC} 클릭"
echo -e "    - 터미널에 ${GREEN}'Successfully saved station...'${NC} 로그가 뜨면 저장 완료!"
echo -e "    - 저장 경로:"
echo -e "      • 도킹 좌표 설정값: ${CYAN}robot_ws/src/llm_semantic_docker/config/docking_stations.yaml${NC}"
echo ""
echo -e " ${BOLD}4. 완료 후:${NC}"
echo -e "    - RViz 창을 닫거나 터미널에서 ${RED}Ctrl + C${NC}를 누르면 안전하게 종료됩니다."
echo -e "${BOLD}${GREEN}==============================================================================${NC}\n"

echo -e "${CYAN}Starting RViz and Interactive Docking Teacher...${NC}"
sleep 2

# Launch
exec ros2 launch llm_semantic_docker teach_docking_station.launch.py \
    station_name:="${STATION_NAME}" \
    box_size_x:="${BOX_X}" \
    box_size_y:="${BOX_Y}" \
    box_size_z:="${BOX_Z}" \
    database_path:="${MAP_DB}" \
    start_map_server:="${START_MAP_SERVER}"
