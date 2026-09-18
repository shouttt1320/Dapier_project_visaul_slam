#!/usr/bin/env bash
# ==============================================================================
# Save 2D Occupancy Grid Map to my_office_map (.yaml & .pgm)
# ==============================================================================
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=101

MAP_PATH="/home/jjj/Documents/Dapier/Project/my_office_map"
echo "==> Saving 2D Occupancy Map to: ${MAP_PATH}.yaml & .pgm..."

ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"

echo "==> Map saved successfully!"
