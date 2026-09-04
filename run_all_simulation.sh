#!/bin/bash
# ==============================================================================
# Autonomous Red Box Navigation with Pure Depth Camera (OS30A) + IMU + Wheel Odom
# Master One-Click Execution Script
# ==============================================================================

# Cleanup function when pressing Ctrl+C
cleanup() {
    echo ""
    echo "========================================================"
    echo " [SHUTDOWN] Terminating all simulation & SLAM nodes...  "
    echo "========================================================"
    if [ -n "$NAV_PID" ]; then kill -TERM "$NAV_PID" 2>/dev/null || true; fi
    if [ -n "$SLAM_PID" ]; then kill -TERM "$SLAM_PID" 2>/dev/null || true; fi
    if [ -n "$SIM_PID" ]; then kill -TERM "$SIM_PID" 2>/dev/null || true; fi
    sleep 2
    killall -9 gz sim parameter_bridge rtabmap rviz2 2>/dev/null || true
    echo ">>> All simulation nodes stopped cleanly."
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Environment Setup
source /opt/ros/jazzy/setup.bash
source /home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

echo "========================================================"
echo "  TurtleBot3 Pure Visual-Inertial SLAM & Navigation    "
echo "  - Sensor: YDLIDAR OS30A (RGB-D) + IMU + Wheel Odom   "
echo "  - Mode: Simulation (Gazebo Harmonic, Domain 42)      "
echo "========================================================"

# 2. Kill any old lingering processes
killall -9 gz sim parameter_bridge rtabmap rviz2 2>/dev/null || true
sleep 1

# 3. Launch Gazebo Simulation
echo ""
echo ">>> [1/3] Launching Gazebo Simulation & Virtual Robot..."
ros2 launch turtlebot3_visual_slam simulation_vslam_pure_depth.launch.py &
SIM_PID=$!
sleep 5

# 4. Launch Visual SLAM & RViz
echo ""
echo ">>> [2/3] Launching RTAB-Map Visual SLAM & RViz2..."
ros2 launch turtlebot3_visual_slam vslam_rgbd_imu.launch.py use_sim_time:=true open_rviz:=true &
SLAM_PID=$!
sleep 4

# 5. Launch Nav2 & Autonomous Mission Controller
echo ""
echo ">>> [3/3] Launching Nav2 Stack & Red Box Mission Controller..."
ros2 launch redbox_navigator redbox_depth_navigator.launch.py use_sim_time:=true &
NAV_PID=$!

echo ""
echo "========================================================"
echo " [RUNNING] All 3 subsystems are active and running!      "
echo " - Gazebo Sim: Rendering textured world + robot         "
echo " - RTAB-Map: Building 2D /map & 3D cloud map            "
echo " - Nav2: Autonomous path planning with smooth turning   "
echo "                                                        "
echo " >>> Press [Ctrl + C] in this terminal to stop all <<<  "
echo "========================================================"

# Wait indefinitely until user terminates
wait
