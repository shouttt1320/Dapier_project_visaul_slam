#!/usr/bin/env bash
set -e

echo "=========================================================="
echo " [TurtleBot3 SLAM & ROS 2 Jazzy Installation Script] "
echo " Target OS: Ubuntu 24.04 LTS (Noble)"
echo " Robot Model: TurtleBot3 Waffle Pi"
echo "=========================================================="

# 1. Update and install prerequisites
echo ">> [1/5] 패키지 목록 업데이트 및 기본 도구 설치 중..."
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release git

# 2. Add ROS 2 Jazzy Apt Repository
echo ">> [2/5] ROS 2 Jazzy 저장소 키 및 소스 목록 추가 중..."
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# 3. Install ROS 2 Jazzy Desktop and Build Tools
echo ">> [3/5] ROS 2 Jazzy Desktop 및 빌드 툴 설치 중..."
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete

# Initialize rosdep if not already initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo ">> rosdep 초기화 중..."
  sudo rosdep init || true
fi
rosdep update || true

# 4. Install TurtleBot3, Gazebo Sim, SLAM and Navigation Packages
echo ">> [4/5] TurtleBot3, SLAM(Slam Toolbox & Cartographer), Nav2, Gazebo 패키지 설치 중..."
sudo apt install -y \
  ros-jazzy-dynamixel-sdk \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-turtlebot3-teleop \
  ros-jazzy-turtlebot3-cartographer \
  ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-cartographer \
  ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz \
  ros-jazzy-teleop-twist-keyboard

# 5. Configure bash environment (.bashrc)
echo ">> [5/5] 터틀봇3 환경 변수 설정 중 (TURTLEBOT3_MODEL=waffle_pi)..."

BASHRC_FILE="$HOME/.bashrc"

# Check and append ROS 2 Jazzy setup
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC_FILE"; then
  echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC_FILE"
fi

# Check and append TURTLEBOT3_MODEL
if ! grep -q "export TURTLEBOT3_MODEL=" "$BASHRC_FILE"; then
  echo "export TURTLEBOT3_MODEL=waffle_pi" >> "$BASHRC_FILE"
else
  sed -i 's/export TURTLEBOT3_MODEL=.*/export TURTLEBOT3_MODEL=waffle_pi/' "$BASHRC_FILE"
fi

# Check and append ROS_DOMAIN_ID (matches robot configuration)
if ! grep -q "export ROS_DOMAIN_ID=" "$BASHRC_FILE"; then
  echo "export ROS_DOMAIN_ID=101" >> "$BASHRC_FILE"
fi

# Check and append LDS_MODEL (matches robot configuration RPLIDAR-C1)
if ! grep -q "export LDS_MODEL=" "$BASHRC_FILE"; then
  echo "export LDS_MODEL=RPLIDAR-C1" >> "$BASHRC_FILE"
fi

if ! grep -q "export RMW_IMPLEMENTATION=" "$BASHRC_FILE"; then
  echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> "$BASHRC_FILE"
fi

echo "=========================================================="
echo " [설치 완료!] "
echo " 터미널에 다음 명령어를 입력하여 환경 설정을 즉시 적용하세요:"
echo " source ~/.bashrc"
echo "=========================================================="
