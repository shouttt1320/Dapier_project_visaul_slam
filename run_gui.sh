#!/usr/bin/env bash
# ==============================================================================
# TurtleBot3 RedBox Mode Switcher GUI (Nav2 Mode ⇄ Precision Docking Mode)
# ==============================================================================

TARGET_DOMAIN=42
if [ "$1" == "real" ] || [ "$1" == "101" ]; then
    TARGET_DOMAIN=101
elif [ "$1" == "sim" ] || [ "$1" == "42" ]; then
    TARGET_DOMAIN=42
elif [ -n "$1" ]; then
    TARGET_DOMAIN=$1
fi

export ROS_DOMAIN_ID=$TARGET_DOMAIN

echo "========================================================"
echo "  Launching RedBox TurtleBot3 Mode Controller GUI...   "
if [ "$ROS_DOMAIN_ID" == "42" ]; then
    echo "  - Target:        [SIMULATION] (Gazebo Harmonic)       "
elif [ "$ROS_DOMAIN_ID" == "101" ]; then
    echo "  - Target:        [REAL ROBOT] (Raspberry Pi)          "
fi
echo "  - ROS_DOMAIN_ID: $ROS_DOMAIN_ID                       "
echo "========================================================"

ros2 run redbox_navigator docking_gui
