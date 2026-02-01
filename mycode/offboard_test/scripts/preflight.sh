#!/bin/bash
# Preflight check: restart DDS agent and verify topic discovery before flight.
# Usage: ./preflight.sh

set -e

RPI_HOST="ubuntu@192.168.20.123"
SSH_KEY="$HOME/.ssh/rpi_key"
SSH_CMD="ssh -i $SSH_KEY -o StrictHostKeyChecking=no -o ConnectTimeout=5 $RPI_HOST"

# Ensure FastDDS peers XML is loaded
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_peers.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/fastdds_peers.xml"

echo "[1/4] Checking RPi connectivity..."
if ! ping -c 1 -W 2 192.168.20.123 > /dev/null 2>&1; then
    echo "FAIL: Cannot reach RPi at 192.168.20.123. Check WiFi."
    exit 1
fi
echo "  OK"

echo "[2/4] Restarting DDS agent on RPi..."
$SSH_CMD "sudo systemctl restart dds-agent" 2>/dev/null
echo "  OK"

echo "[3/4] Waiting for DDS discovery..."
sleep 5

echo "[4/4] Checking /fmu topics on laptop..."
TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$TOPICS" | grep -q "/fmu/in/vehicle_visual_odometry"; then
    FMU_COUNT=$(echo "$TOPICS" | grep -c "/fmu/")
    echo "  OK - $FMU_COUNT /fmu/ topics visible"
    echo ""
    echo "Preflight PASSED. Ready to launch."
else
    echo "FAIL: /fmu/ topics not visible on laptop."
    echo "  Check: echo \$FASTDDS_DEFAULT_PROFILES_FILE"
    echo "  Should be: $HOME/fastdds_peers.xml"
    exit 1
fi
