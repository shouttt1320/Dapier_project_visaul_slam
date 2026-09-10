#!/usr/bin/env bash
# ==============================================================================
# TurtleBot3 Teleop Keyboard Controller (Compatible with Gazebo Sim & Real Robot)
# ==============================================================================

# Target Domain resolution (Default: 42 for Simulation, 101 for Real Robot)
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
echo "  TurtleBot3 Manual Teleop Keyboard Controller"
if [ "$ROS_DOMAIN_ID" == "42" ]; then
    echo "  - Target:    [SIMULATION] (Gazebo Harmonic)"
elif [ "$ROS_DOMAIN_ID" == "101" ]; then
    echo "  - Target:    [REAL ROBOT] (Raspberry Pi)"
else
    echo "  - Target:    Custom Domain $ROS_DOMAIN_ID"
fi
echo "  - Domain ID: $ROS_DOMAIN_ID"
echo "  - Model:     $TURTLEBOT3_MODEL"
echo "  - Publishing to: /cmd_vel (Twist)"
echo "========================================================"

# In ROS 2 Jazzy, teleop_keyboard publishes TwistStamped unless ROS_DISTRO=humble.
# Using ROS_DISTRO=humble publishes geometry_msgs/msg/Twist to /cmd_vel.
ROS_DISTRO=humble ros2 run turtlebot3_teleop teleop_keyboard
