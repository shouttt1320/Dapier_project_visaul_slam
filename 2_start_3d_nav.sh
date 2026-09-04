#!/usr/bin/env bash
set -e

# ==============================================================================
# 2. Autonomous 3D Visual Navigation (Pure RTAB-Map Localization + Nav2)
# ==============================================================================
echo "=================================================================="
echo " [Step 1] Starting Robot Base & Astra Camera on Raspberry Pi..."
echo "=================================================================="
ssh user@192.168.0.148 "~/start_robot_all.sh"

echo "Waiting for robot topics to initialize..."
sleep 3

echo "=================================================================="
echo " [Step 2] Starting 3D Visual Navigation (RTAB-Map Loc + Nav2)..."
echo "  Loading DB: /home/jjj/Documents/Dapier/Project/my_office_3d.db"
echo "=================================================================="
source /opt/ros/jazzy/setup.bash
source /home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=101

ros2 launch turtlebot3_visual_slam vslam_navigation.launch.py
