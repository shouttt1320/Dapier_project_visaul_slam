#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/user/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=101

pkill -9 -f astra_camera_node 2>/dev/null || true
pkill -9 -f astra_compressor 2>/dev/null || true
rm -f /dev/shm/orbbec* /dev/shm/fastrtps* /dev/shm/sem.* 2>/dev/null || true
ros2 run astra_camera clean_up_shm_node 2>/dev/null || true
sleep 1

python3 /home/user/astra_compressor.py &
COMP_PID=$!

ros2 launch astra_camera astra.launch.xml \
  depth_width:=320 depth_height:=240 \
  color_width:=320 color_height:=240 \
  enable_point_cloud:=false depth_registration:=true

kill $COMP_PID 2>/dev/null || true
