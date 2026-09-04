#!/usr/bin/env bash
set -e

# ==============================================================================
# 1. 3D Office Visual Mapping Scan (Run once when battery is charged)
# ==============================================================================
echo "=================================================================="
echo " [Step 1] Starting Robot Base & Astra Camera on Raspberry Pi..."
echo "=================================================================="
ssh user@192.168.0.148 "~/start_robot_all.sh"

echo "Waiting for robot topics to initialize..."
sleep 3

echo "=================================================================="
echo " [Step 2] Starting 3D Visual SLAM (RTAB-Map Mapping + RViz)..."
echo "  Target DB: /home/jjj/Documents/Dapier/Project/my_office_3d.db"
echo "=================================================================="
source /opt/ros/jazzy/setup.bash
source /home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=101

# Run mapping with database reset for clean scan
ros2 launch turtlebot3_visual_slam vslam_rgbd_imu.launch.py delete_db_on_start:=true
