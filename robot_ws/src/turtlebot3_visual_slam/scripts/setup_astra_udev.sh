#!/usr/bin/env bash
set -e

echo "=========================================================="
echo " [Orbbec Astra S USB Udev Rules Setup Script] "
echo "=========================================================="

UDEV_FILE="/etc/udev/rules.d/56-orbbec-astra.rules"

echo ">> /etc/udev/rules.d/ 에 Orbbec Astra S 권한 규칙 생성 중..."

sudo bash -c "cat << 'EOF' > $UDEV_FILE
# Orbbec Astra S Camera USB Rules
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0401\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0402\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0403\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0404\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0405\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0501\", MODE=\"0666\", GROUP=\"plugdev\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bc5\", ATTR{idProduct}==\"0601\", MODE=\"0666\", GROUP=\"plugdev\"
EOF"

echo ">> udev 규칙 다시 로드 중..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ">> Orbbec Astra S udev 설정 완료!"
echo "   카메라 USB 케이블을 다시 꽂아주세요."
