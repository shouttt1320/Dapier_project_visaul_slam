#!/usr/bin/env bash
set -e

# ==============================================================================
# 1. 3D Office Visual Mapping Scan (Run once when battery is charged)
# ==============================================================================
# Clean up any previous stale GUI, bridge, or navigation nodes
pkill -9 -f "docking_gui" 2>/dev/null || true
pkill -9 -f "compressed_bridge_node" 2>/dev/null || true
pkill -9 -f "vslam_navigation" 2>/dev/null || true
pkill -9 -f "vslam_rgbd_imu" 2>/dev/null || true
pkill -9 -f "rtabmap" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "point_cloud_xyzrgb" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

echo "=================================================================="
echo " [Step 1] Synchronizing time & Starting Robot on Raspberry Pi..."
echo "=================================================================="
CURRENT_TIME=$(date +%s)
ssh user@192.168.0.148 "echo turtlebot | sudo -S date -s @$CURRENT_TIME" > /dev/null 2>&1 || true
ssh user@192.168.0.148 "nohup ~/start_robot_all.sh > /tmp/start_all.log 2>&1 &" || true

echo "Waiting 5 seconds for robot topics to initialize..."
sleep 5

echo "=================================================================="
echo " [Step 2] Starting 3D Visual SLAM (RTAB-Map Mapping + RViz)..."
echo "  Target DB: /home/jjj/Documents/Dapier/Project/my_office_3d.db"
echo "  Note: Drive slowly with teleop to build a dense, clean 3D map."
echo "=================================================================="
source /opt/ros/jazzy/setup.bash
source /home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=101
export TURTLEBOT3_MODEL=waffle_pi

# Run mapping with database reset for clean scan
exec ros2 launch turtlebot3_visual_slam vslam_rgbd_imu.launch.py delete_db_on_start:=true "$@"
