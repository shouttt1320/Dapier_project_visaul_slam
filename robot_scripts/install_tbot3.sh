#!/bin/bash
# -----------------------------------------------------
# Install Turtlebot3 on pi shell script by liberman
# Last revision: 2025.10.22 ~
# -----------------------------------------------------

if [ $# -ne 2 ]; then
  echo "usage: install_tbot3.sh jazzy LDS-02"
  exit 1
fi

echo ----------------
echo " Ubuntu 설정"
echo ----------------

echo "자동 update 설정 (1->0 변경)"
sudo sed -i -e 's/1/0/' /etc/apt/apt.conf.d/20auto-upgrades
echo "prevent boot-up delay 설정"
sudo systemctl mask systemd-networkd-wait-online.service
echo "Disable Suspend and Hibernation 설정"
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target  
echo

# 터틀봇3 RAM 2GB 사용할 경우 swap memory 생성하기
echo ---------------------------------------
cat /proc/device-tree/model && echo
free -h              # 메모리 확인
echo ---------------------------------------
#read -p "RAM Memory가 2GB 이하 인가요? (Y/n) " answer

#if [ "$answer" = "y" ] || [ "$answer" = "Y" ]  || [ "$answer" = "" ] ; then
  sudo fallocate -l 2G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab    # 부팅시 자동 swap 설정
  free -h
#fi

echo ------------------
echo " Update & upgrade"
echo ------------------

sudo apt update
sudo apt upgrade -y
#upgrade 후 No VM guests are running outdated hypervisor (qemu) binaries on this host. 표시 제거
sudo apt purge needrestart -y
sudo apt autoremove -y

echo ---------------------
echo " 터틀봇3 패키지 설치"
echo ---------------------

sudo apt install -y python3-argcomplete libboost-system-dev build-essential libudev-dev \
ros-$1-hls-lfcd-lds-driver \
ros-$1-turtlebot3-msgs \
ros-$1-dynamixel-sdk \
ros-$1-rosbridge-server \
ros-$1-xacro -y

sudo dpkg --add-architecture armhf     # add armhf
sudo apt update                              # add 후 update 필요
sudo apt install libc6:armhf -y             # opencr용 lib 설치

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b $1 https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b $1 https://github.com/ROBOTIS-GIT/ld08_driver.git
git clone -b $1 https://github.com/ROBOTIS-GIT/coin_d4_driver

cd ~/ros2_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2

echo ---------------------------------
echo "Turtlebot3 Compile: 약 20분 소요"
echo ---------------------------------

# 컴파일시 릴리즈모드로 시간단축
source /opt/ros/$1/setup.bash
cd ~/ros2_ws/
colcon build --parallel-workers 1 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash

echo ---------------------------
echo " OpenCR 설정 및 upload"
echo ---------------------------

# OpenCR을 위한 usb 포트 설정
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# OpenCR upload 실행
cd
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2 
tar -xvf opencr_update.tar.bz2 
rm -rf ./opencr_update.tar.bz2  
cd ./opencr_update  
./update.sh /dev/ttyACM0 burger.opencr 

echo ------------------------
echo " Turtlebot3 환경설정"
echo ------------------------

echo "export LDS_MODEL=$2" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo "source /opt/ros/$1/setup.bash" >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
tail -n 7 ~/.bashrc        # 마지막 줄 표시

echo --------------------------------
echo " Completed!  source ~/.bashrc"
echo --------------------------------
