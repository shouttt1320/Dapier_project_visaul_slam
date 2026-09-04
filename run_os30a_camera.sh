#!/usr/bin/env bash
set -e

echo "=========================================================="
echo " [YDLIDAR OS30A Real Depth Camera & RViz Viewer Runner]"
echo "=========================================================="

source /opt/ros/jazzy/setup.bash
source /home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/jjj/Documents/Dapier/Project/robot_ws/src/depth_ydlidar_os30a/eYs3D_wrapper/lib:${LD_LIBRARY_PATH}

# Launch camera driver node and RViz2 visualizer
ros2 launch depth_ydlidar_os30a apc_camera_launch.py norviz:=false
