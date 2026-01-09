#!/bin/bash

echo "========================================="
echo "   Clean Startup - Drone Control GUI"
echo "========================================="
echo ""

# Kill ALL existing processes thoroughly
echo "Cleaning up existing processes..."
killall -9 px4 2>/dev/null
killall -9 gz 2>/dev/null
killall -9 gzserver 2>/dev/null
killall -9 gzclient 2>/dev/null
killall -9 MicroXRCEAgent 2>/dev/null
killall -9 python3 2>/dev/null
killall -9 ruby 2>/dev/null

sleep 3

echo "Verifying cleanup..."
REMAINING=$(ps aux | grep -E "(px4|gz)" | grep -v grep | wc -l)
if [ $REMAINING -gt 0 ]; then
    echo "WARNING: Some processes still running:"
    ps aux | grep -E "(px4|gz)" | grep -v grep
    echo ""
    echo "Forcing cleanup..."
    killall -9 gz 2>/dev/null
    sleep 2
fi

echo "All clean!"
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/local_setup.bash

echo "Starting components..."
echo ""

# Start MicroXRCE Agent
echo "[1/4] Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/agent_clean.log 2>&1 &
sleep 3

# Start PX4 SITL
echo "[2/4] Starting PX4 SITL (this takes ~10 seconds)..."
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth > /tmp/px4_clean.log 2>&1 &
PX4_PID=$!

# Wait for PX4 to fully initialize
sleep 15

# Check if PX4 is running
if ! ps -p $PX4_PID > /dev/null 2>&1; then
    echo "ERROR: PX4 failed to start!"
    echo "Check /tmp/px4_clean.log for details"
    exit 1
fi

echo "[3/4] Starting Gazebo GUI..."
gz sim -g > /tmp/gazebo_clean.log 2>&1 &
sleep 4

# Wait a bit more for Gazebo to connect to simulation
sleep 3

echo "[4/4] Starting Drone Control GUI..."
echo ""
echo "========================================="
echo "  Checking for data from PX4..."
echo "========================================="
echo ""

# Run GUI in foreground
cd ~/ws_sensor_combined
drone_control_gui

# Cleanup on exit
echo ""
echo "Shutting down..."
killall -9 px4 gz MicroXRCEAgent 2>/dev/null
echo "Done!"
