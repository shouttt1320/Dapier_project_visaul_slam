#!/bin/bash
# ---------------------------------------
# Install ROS2 shell script by liberman
# Last revision: 2025.10.22~
# ---------------------------------------

if [ $# -lt 1 ]; then
  echo "usage: install_ros2.sh jazzy"
  exit 1
fi

echo -------------------
echo "Update & upgrade"
echo -------------------

# Waiting for cache lock: Could not get lock /var/lib/dpkg/lock-frontend 에러 제거
#sudo rm /var/lib/apt/lists/lock
#sudo rm /var/cache/apt/archives/lock
#sudo rm /var/lib/dpkg/*
#sudo dpkg --configure -a

sudo apt update
sudo apt upgrade -y

echo -------------------------------
echo "Enable required repositories"
echo -------------------------------

sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
#locale       # verify settings

# Repository: Main(필수 sw), Universe (커뮤니티 지원 sw), Restricted (저작권 sw), Mumtiverse (모두 포함)
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update                          # library가 추가되어 정보를 갱신해야 한다.

echo ---------------
echo " Install ROS2"
echo ---------------

# ROS2 GPG key
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# noble-updaes를 추가한다.
echo "deb http://ports.ubuntu.com/ubuntu-ports/ $(. /etc/os-release && echo $UBUNTU_CODENAME)-updates main restricted universe multiverse" | sudo tee /etc/apt/sources.list.d/noble-updates.list

sudo apt update                          # library가 추가되어 정보를 갱신해야 한다.
sudo apt upgrade -y

sudo apt install ros-dev-tools -y                # ROS 개발 툴
sudo apt install ros-$1-rosbridge-server -y     # 스마트폰 앱 연동
sudo apt install python3-colcon-common-extensions -y     # colcon-cmake, colcon-ros 등 빌드도구 확장
sudo apt install net-tools wireless-tools -y

# 라즈베리파이에 설치
if grep -qi "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
  sudo apt install ros-$1-ros-base -y

# RemotePC에 설치
else
  sudo apt install -y git wpasupplicant ifupdown gedit  \
  gstreamer1.0-tools libgstreamer1.0-dev \
  x11-xserver-utils x11-apps nmap \
  libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
  openssh-server

  # ROS2 필수항목
  sudo apt install -y ros-$1-desktop \
  ros-$1-cartographer ros-$1-cartographer-ros \
  ros-$1-navigation2 ros-$1-nav2-bringup

  # ROS2 추가항목
  sudo apt install -y ros-$1-camera-calibration \
  ros-$1-joint-state-publisher ros-$1-joint-state-publisher-gui \
  ros-$1-ros-gz ros-$1-ros-gz-bridge \
  ros-$1-rqt-robot-steering ros-$1-rqt-tf-tree \
  ros-$1-rosidl-default-generators ros-$1-ament-lint-auto

  echo -----------------------------------
  echo "Install system dependencies"
  echo -----------------------------------
#  sudo apt install python3-rospkg -y
#  sudo apt install python3-rosdep -y
#  sudo apt install python3-rosinstall-generator -y
#  sudo apt install python3-pip -y
#  sudo apt install python3-opencv -y

  # python 개발시 의존성 체크 툴 설치
  sudo rosdep init
  rosdep update                        # permission error: don't use sudo

#  python3 -m pip config set global.break-system-packages true     # 다음 pip3 명령을 실행할 수 있다.
  #pip3 install numpy==1.26.4                   #  버전 충돌날 수 있음.
#  pip3 install opencv-contrib-python

fi

echo -------------------------------
echo Set environment for ROS
echo -------------------------------
echo "" >> ~/.bashrc
echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc
echo "alias cbc='cd ~/ros2_ws && unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && unset COLCON_PREFIX_PATH && sudo rm -r build/ install/ log/ && source /opt/ros/$1/setup.bash'" >> ~/.bashrc
echo "alias sc='source ~/ros2_ws/install/setup.bash && source /opt/ros/$1/setup.bash'" >> ~/.bashrc
echo "alias ws='cd ~/ros2_ws/src'" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "source /opt/ros/$1/setup.bash" >> ~/.bashrc              # ros 환경 설정
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/ && colcon build --symlink-install && cd

echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc        # 동일 네트워크에서 Domain ID 설정
tail -n 8 ~/.bashrc

echo -------------------------------
echo "Completed!  source ~/.bashrc"
echo -------------------------------
