echo "----------------------"
echo "Install RPiCam v2 on ROS2 and Ubuntu Server 24.04 64bit (2025.11.09)"
echo "----------------------"

# RAM이 2GB 이하인 경우 compile시에 swap 필요
#sudo fallocate -l 2G /swapfile
#sudo chmod 600 /swapfile
#sudo mkswap /swapfile
#sudo swapon /swapfile
#free -h

# camera-ros 설치 (카메라 실행이 안됨)
sudo apt update
sudo apt install ros-jazzy-camera-ros -y                   # 이때 libcamera도 같이 설치 된다.
sudo apt install ros-jazzy-image-transport-plugins -y

# dependency lib 설치
sudo apt install -y python3-pip git python3-jinja2
sudo apt install -y libboost-dev libyuv-dev libevent-dev
sudo apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5t64 libqt5widgets5t64
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

# libcamera 패키지 설치 (소스를 직접 build)
cd
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build -j 2                #  limit the build to a single process
sudo ninja -C build install
sudo ldconfig

# source로 컴파일 한 libcamera를 LD_LIBRARY_PATH를 설정한다. 그렇지 않으면 에러가 발생한다.
# ERROR IPCPipe ipc_pipe_unixsocket.cpp:131 Call timeout!
# ERROR IPAProxy raspberrypi_ipa_proxy.cpp:316 Failed to call start: -110
echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc

# calibration file 생성
mkdir -p ~/.ros/camera_info/
echo "
image_width: 320   # Update to your camera's actual resolution
image_height: 240   # Update to your camera's actual resolution
camera_name: imx219__base_soc_i2c0mux_i2c_1_imx219_10_320x240   
frame_id: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [161.0352, 0, 99.6340, 0, 160.4337, 77.6267, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1639958, -0.2718400, 0.0010558, -0.0016656, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [164.6242, 0, 99.2051, 0, 0, 164.5522, 77.7529, 0, 0, 0, 1, 0]
" > ~/.ros/camera_info/imx219__base_soc_i2c0mux_i2c_1_imx219_10_320x240.yaml

echo "------------------"
echo "Installed successfully"
echo "source ~/.bashrc"
echo "ros2 run camera_ros camera_node --ros-args -p image_transport:=compressed -p format:='RGB888' -p FrameDurationLimits:='[200000,200000]' -p width:=320 -p height:=240"
echo "------------------"
