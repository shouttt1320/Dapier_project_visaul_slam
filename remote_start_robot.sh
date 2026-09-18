#!/usr/bin/env bash
# One-click remote start for TurtleBot3 Base & Astra Camera on Raspberry Pi
echo "==> [1/3] Synchronizing Raspberry Pi system clock to PC..."
CURRENT_TIME=$(date +%s)
ssh user@192.168.0.148 "echo user | sudo -S date -s @$CURRENT_TIME" > /dev/null 2>&1

echo "==> [2/3] Launching TurtleBot3 Base & Astra Camera on Raspberry Pi..."
ssh user@192.168.0.148 "nohup ~/start_robot_all.sh > /tmp/start_all.log 2>&1 &"

echo "==> [3/3] Waiting 8 seconds for nodes to initialize..."
sleep 8

echo "==> Status Check on Raspberry Pi:"
ssh user@192.168.0.148 "ps aux | grep -E 'turtlebot3_ros|astra_camera_node|bumper_sensor|astra_compressor' | grep -v grep"

echo ""
echo "======================================================================"
echo " All Raspberry Pi nodes are running successfully!"
echo " Now you can run ./run_real_robot_pc.sh on this PC."
echo "======================================================================"
