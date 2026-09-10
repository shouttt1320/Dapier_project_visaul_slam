#!/usr/bin/env bash
# ==============================================================================
# Precision Close-Docking One-Click Execution Script
# Runs the FSM-based precision approacher node with proper ROS 2 Jazzy environment
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
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}

echo "========================================================"
echo "  TurtleBot3 Precision Close-Docking Controller        "
echo "  - Mode: 5cm Desk Approacher with Parallel Alignment  "
echo "  - Sensing: OS30A 3D Depth + Single Contact Bumper    "
if [ "$ROS_DOMAIN_ID" == "42" ]; then
    echo "  - Target:   [SIMULATION] (Gazebo Harmonic)            "
elif [ "$ROS_DOMAIN_ID" == "101" ]; then
    echo "  - Target:   [REAL ROBOT] (Raspberry Pi)               "
fi
echo "  - Domain ID: $ROS_DOMAIN_ID                           "
echo "========================================================"

# Run precision approacher node
ros2 run redbox_navigator precision_approacher
