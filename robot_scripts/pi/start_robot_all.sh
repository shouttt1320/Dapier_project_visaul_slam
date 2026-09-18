#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/user/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=101
export TURTLEBOT3_MODEL=waffle_pi

# 1. Clean previous processes & locks
pkill -9 -f turtlebot3_ros 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f hlds_laser_publisher 2>/dev/null || true
pkill -9 -f astra_camera_node 2>/dev/null || true
pkill -9 -f astra_compressor 2>/dev/null || true
rm -f /dev/shm/orbbec* /dev/shm/fastrtps* /dev/shm/sem.* 2>/dev/null || true
sleep 1

# 2. Start TurtleBot3 Base (Motor & Odom & IMU only, NO LIDAR) in background
echo '[1/2] Starting TurtleBot3 base (No LiDAR)...'
ros2 launch ~/tb3_base_only.launch.py > /tmp/turtlebot3.log 2>&1 &

sleep 3

# 3. Start Astra Camera Stream
echo '[2/3] Starting Astra S stream...'
~/run_astra.sh > /tmp/astra.log 2>&1 &

sleep 3

# 4. Start Micro-switch Contact Bumper Sensor (GPIO Pin 16 / BCM 23)
echo '[3/3] Starting Contact Bumper Sensor...'
ros2 run precision_docker bumper_sensor --ros-args -p pin:=23 -p poll_rate_hz:=50.0 -p debounce_ms:=15.0 -p active_low:=true > /tmp/bumper.log 2>&1 &

echo 'All robot nodes started! (Logs: /tmp/turtlebot3.log, /tmp/astra.log, /tmp/bumper.log)'
