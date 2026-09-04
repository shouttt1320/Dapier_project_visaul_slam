#!/usr/bin/env bash
# One-click remote stop for TurtleBot3 Base & Astra Camera on Raspberry Pi
echo "Stopping all robot processes on Raspberry Pi..."
ssh user@192.168.0.148 "~/stop_robot_all.sh"
echo "Done."
