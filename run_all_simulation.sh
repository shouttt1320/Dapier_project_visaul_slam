#!/bin/bash
# ==============================================================================
# Autonomous Navigation with Orbbec Astra S (Native 3D RGB-D) + IMU + Wheel Odom
# Master One-Click Execution Script
# ==============================================================================

# Cleanup function when pressing Ctrl+C
cleanup() {
    echo ""
    echo "========================================================"
    echo " [SHUTDOWN] Terminating all simulation & control nodes... "
    echo "========================================================"
    if [ -n "$GUI_PID" ]; then kill -TERM "$GUI_PID" 2>/dev/null || true; fi
    if [ -n "$DOCK_PID" ]; then kill -TERM "$DOCK_PID" 2>/dev/null || true; fi
    if [ -n "$NAV_PID" ]; then kill -TERM "$NAV_PID" 2>/dev/null || true; fi
    if [ -n "$SLAM_PID" ]; then kill -TERM "$SLAM_PID" 2>/dev/null || true; fi
    if [ -n "$SIM_PID" ]; then kill -TERM "$SIM_PID" 2>/dev/null || true; fi
    sleep 2
    pkill -9 -f 'docking_gui' 2>/dev/null || true
    pkill -9 -f 'precision_approacher' 2>/dev/null || true
    pkill -9 -f 'camera_mode_manager' 2>/dev/null || true
    pkill -9 -f 'ekf_node' 2>/dev/null || true
    pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
    pkill -9 -f 'point_cloud_xyzrgb' 2>/dev/null || true
    pkill -9 -f 'gz sim' 2>/dev/null || true
    killall -9 gz-sim-server gz-sim-gui gz sim parameter_bridge rtabmap rtabmap_viz rviz2 2>/dev/null || true
    killall -9 controller_server planner_server behavior_server bt_navigator smoother_server opennav_docking collision_monitor route_server lifecycle_manager 2>/dev/null || true
    echo ">>> All simulation and control nodes stopped cleanly."
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Environment Setup (Simulation is strictly isolated to Domain 42)
source /opt/ros/jazzy/setup.bash
source /home/jjj/Documents/Dapier/Project/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

echo "========================================================"
echo "  TurtleBot3 Full Integrated Autonomous Simulation     "
echo "  - Sensor: Orbbec Astra S (3D RGB-D) + IMU + Odom     "
echo "  - Manipulation: YDLIDAR OS30A (Tilted 105° Down)     "
echo "  - Navigation: Nav2 Stack + RTAB-Map SLAM              "
echo "  - Docking: 5cm Precision Desk Block Docking Engine   "
echo "  - Control: PyQt5 Mode Controller GUI Dashboard        "
echo "  - Mode: Simulation (Gazebo Harmonic, Isolated Domain 42)"
echo "========================================================"

# 2. Kill any old lingering processes
pkill -9 -f 'docking_gui' 2>/dev/null || true
pkill -9 -f 'precision_approacher' 2>/dev/null || true
pkill -9 -f 'camera_mode_manager' 2>/dev/null || true
pkill -9 -f 'ekf_node' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
pkill -9 -f 'point_cloud_xyzrgb' 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
killall -9 gz-sim-server gz-sim-gui gz sim parameter_bridge rtabmap rtabmap_viz rviz2 2>/dev/null || true
killall -9 controller_server planner_server behavior_server bt_navigator smoother_server opennav_docking collision_monitor route_server lifecycle_manager 2>/dev/null || true
sleep 1

# 3. Launch Gazebo Simulation
echo ""
echo ">>> [1/5] Launching Gazebo Simulation & Virtual Robot..."
ros2 launch turtlebot3_visual_slam simulation_vslam_pure_depth.launch.py &
SIM_PID=$!
sleep 5

# 4. Launch Visual SLAM & RViz
echo ""
echo ">>> [2/5] Launching RTAB-Map Visual SLAM & RViz2 (Simulation DB: simulation_3d.db)..."
ros2 launch turtlebot3_visual_slam vslam_rgbd_imu.launch.py \
    use_sim_time:=true \
    odom_topic:=/odometry/filtered \
    database_path:=/home/jjj/Documents/Dapier/Project/simulation_3d.db \
    delete_db_on_start:=true \
    open_rviz:=true &
SLAM_PID=$!
sleep 4

# 5. Launch Nav2 Stack for User 2D Goal Pose Navigation
echo ""
echo ">>> [3/5] Launching Nav2 Stack for User Commands (RViz 2D Goal Pose)..."
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true \
    autostart:=true \
    params_file:=/home/jjj/Documents/Dapier/Project/robot_ws/src/redbox_navigator/config/nav2_params.yaml &
NAV_PID=$!
sleep 3

# 6. Launch Precision Approacher Node (Standby in NAV2_READY mode)
echo ""
echo ">>> [4/5] Launching 5cm Precision Desk Docking Engine..."
ros2 launch redbox_navigator precision_docking.launch.py \
    use_sim_time:=true \
    show_window:=true \
    autostart:=false &
DOCK_PID=$!
sleep 1

# 7. Launch PyQt5 Mode Controller GUI Dashboard
echo ""
echo ">>> [5/5] Launching RedBox Mode Controller GUI Dashboard..."
ros2 run redbox_navigator docking_gui &
GUI_PID=$!

echo ""
echo "========================================================"
echo " [RUNNING] All Simulation & Control Modules Active!     "
echo " - Gazebo Sim: Rendering brown desk & 50mm red block    "
echo " - RTAB-Map: Building 2D /map & 3D pointcloud map       "
echo " - Nav2: Ready for User Command (RViz 2D Goal Pose)     "
echo " - Docking Engine: Ready on standby (Astra S Active)    "
echo " - Mode Controller GUI: Open for One-Click Switching!   "
echo "                                                        "
echo " Manual Teleop keyboard command (in separate terminal): "
echo "   ./run_teleop.sh                                      "
echo "                                                        "
echo " >>> Press [Ctrl + C] in this terminal to stop all <<<  "
echo "========================================================"

# Wait indefinitely until user terminates
wait
