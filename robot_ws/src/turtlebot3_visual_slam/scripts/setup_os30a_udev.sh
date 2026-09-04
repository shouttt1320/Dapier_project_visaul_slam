#!/usr/bin/env bash
set -e

echo "=========================================================="
echo " [YDLIDAR OS30A Depth Camera USB Udev Rules Setup Script] "
echo "=========================================================="

UDEV_FILE="/etc/udev/rules.d/56-ydlidar-os30a.rules"

echo ">> /etc/udev/rules.d/ 에 YDLIDAR OS30A 권한 규칙 생성 중..."

sudo bash -c "cat << 'EOF' > $UDEV_FILE
# YDLIDAR / eYs3D OS30A 3D Depth Camera USB Rules
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"1e4e\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2a0b\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", MODE=\"0666\", GROUP=\"plugdev\"
EOF"

echo ">> udev 규칙 다시 로드 중..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ">> YDLIDAR OS30A udev 설정 완료!"
