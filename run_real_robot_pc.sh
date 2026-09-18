#!/usr/bin/env bash
# ==============================================================================
# RedBox Real Robot PC Mission Control Launcher
# Launches:
#   1. Zero-Lag Wi-Fi Compressed Image & Sync Bridge
#   2. RTAB-Map 3D Visual Localization (on my_office_3d.db)
#   3. Nav2 Autonomous Navigation Stack
#   4. RViz2 3D Visual SLAM & Goal Pose Interface
#   5. PyQt5 Mission Control Dashboard (Mode Switcher + Real-Time Video Viewport)
# ==============================================================================

set -e

# ROS 2 Jazzy Environment
source /opt/ros/jazzy/setup.bash

PROJECT_DIR="/home/jjj/Documents/Dapier/Project"
WS_DIR="$PROJECT_DIR/robot_ws"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

export ROS_DOMAIN_ID=101
export TURTLEBOT3_MODEL=waffle_pi
export DISPLAY="${DISPLAY:-:0}"

# Clean up any previous stale GUI, bridge, or navigation nodes
pkill -9 -f "docking_gui" 2>/dev/null || true
pkill -9 -f "precision_approacher" 2>/dev/null || true
pkill -9 -f "compressed_bridge_node" 2>/dev/null || true
pkill -9 -f "vslam_navigation" 2>/dev/null || true
pkill -9 -f "rtabmap" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "behavior_server" 2>/dev/null || true
pkill -9 -f "smoother_server" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "point_cloud_xyzrgb" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

echo "======================================================================"
echo " Starting RedBox PC Mission Control (DOMAIN ID: $ROS_DOMAIN_ID)"
echo " Map Database: $PROJECT_DIR/my_office_3d.db"
echo " Displays: RViz2 3D Navigation + PyQt5 Mission Control Dashboard"
echo "======================================================================"

exec ros2 launch turtlebot3_visual_slam vslam_navigation.launch.py "$@"
