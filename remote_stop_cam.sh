#!/usr/bin/env bash
# Stop Astra S Camera Stream on Raspberry Pi
echo "Stopping Astra S stream on Raspberry Pi (192.168.0.148)..."
ssh user@192.168.0.148 "~/stop_astra.sh"
