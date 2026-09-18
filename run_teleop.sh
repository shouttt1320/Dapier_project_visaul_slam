#!/usr/bin/env bash
# ==============================================================================
# TurtleBot3 Teleop Keyboard Controller (Compatible with Gazebo Sim & Real Robot)
# ==============================================================================

# ROS 2 Environment
source /opt/ros/jazzy/setup.bash
if [ -f "/home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash" ]; then
    source "/home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash"
fi

# Target Domain resolution (Default: 101 for Real Robot, 42 for Simulation)
TARGET_DOMAIN=101
if [ "$1" == "sim" ] || [ "$1" == "42" ]; then
    TARGET_DOMAIN=42
elif [ "$1" == "real" ] || [ "$1" == "101" ]; then
    TARGET_DOMAIN=101
elif [ -n "$1" ]; then
    TARGET_DOMAIN=$1
fi

export ROS_DOMAIN_ID=$TARGET_DOMAIN
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}

echo "========================================================"
echo "  TurtleBot3 Manual Teleop Keyboard Controller"
if [ "$ROS_DOMAIN_ID" == "101" ]; then
    echo "  - Target:    [REAL ROBOT] (Raspberry Pi)"
elif [ "$ROS_DOMAIN_ID" == "42" ]; then
    echo "  - Target:    [SIMULATION] (Gazebo Harmonic)"
else
    echo "  - Target:    Custom Domain $ROS_DOMAIN_ID"
fi
echo "  - Domain ID: $ROS_DOMAIN_ID"
echo "  - Model:     $TURTLEBOT3_MODEL"
echo "  - Topic:     /cmd_vel (TwistStamped)"
echo "========================================================"

# Run high-responsiveness continuous teleop controller
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/run_teleop.py"
