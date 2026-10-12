#!/usr/bin/env bash
# ==============================================================================
# Orbbec Astra S Depth Camera Launcher for Ubuntu 24.04 / ROS 2 Jazzy (PC Local)
# - Astra S (VID 2bc5, PID 0402) 네이티브 astra_camera 드라이버 실행
# - 컬러/뎁스 스트림 (/camera/color/*, /camera/depth/*) 즉시 발행
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "======================================================================"
echo " 📷 Orbbec Astra S Depth Camera Launcher (Native ROS 2 Jazzy)"
echo "======================================================================"

# 1. USB 장치 연결 확인 (VID: 2bc5)
echo -n ">> 1. USB 연결 상태 확인 중... "
if lsusb | grep -iq "2bc5:"; then
    DEVICE_NAME=$(lsusb | grep -i "2bc5:" | head -n 1)
    echo -e "\033[92m[성공]\033[0m $DEVICE_NAME"
else
    echo -e "\033[91m[오류]\033[0m Astra S 카메라가 USB 포트에 감지되지 않습니다."
    echo "   카메라를 파란색 USB 3.0 포트에 다시 꽂아주세요."
    exit 1
fi

# 2. 이전 실행 잔여 프로세스 및 공유 메모리 청소
pkill -9 -f astra_camera_node 2>/dev/null || true
rm -f /dev/shm/orbbec* /dev/shm/fastrtps* /dev/shm/sem.* 2>/dev/null || true

# 3. 환경 변수 설정
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/robot_ws/install/setup.bash"
export LD_LIBRARY_PATH="/home/jjj/.local/lib:$LD_LIBRARY_PATH"

echo "======================================================================"
echo " 🚀 Astra S 카메라를 시작합니다..."
echo " - 발행 토픽:"
echo "   * /camera/color/image_raw (RGB 320x240)"
echo "   * /camera/depth/image_raw (16-bit 3D Depth 320x240)"
echo "   * /camera/color/camera_info"
echo " - 종료 방법: Ctrl + C"
echo "======================================================================"

ros2 launch astra_camera astra.launch.xml \
    depth_width:=320 depth_height:=240 \
    color_width:=320 color_height:=240 \
    enable_point_cloud:=true depth_registration:=true
