#!/bin/bash

echo "========================================="
echo "   Quick Test - Drone Control GUI"
echo "========================================="
echo ""

# Kill any existing processes
killall -9 px4 gz MicroXRCEAgent 2>/dev/null
sleep 2

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/local_setup.bash

echo "Starting components in background..."
echo ""

# Start MicroXRCE Agent
echo "[1/4] Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
sleep 2

# Start PX4
echo "[2/4] Starting PX4 SITL..."
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth > /tmp/px4.log 2>&1 &
PX4_PID=$!
sleep 12

# Start Gazebo GUI
echo "[3/4] Starting Gazebo GUI..."
gz sim -g > /tmp/gazebo_gui.log 2>&1 &
sleep 3

# Start Drone Control GUI
echo "[4/4] Starting Drone Control GUI..."
echo ""
echo "========================================="
echo "  GUI should appear in a few seconds!"
echo "========================================="
echo ""
echo "To use:"
echo "  1. Wait for status panel to show data"
echo "  2. Click 'ENABLE OFFBOARD'"
echo "  3. Click 'ARM' (will force arm)"
echo "  4. Click 'TAKEOFF'"
echo "  5. Use joystick to fly!"
echo ""
echo "Press Ctrl+C to stop everything"
echo ""

# Run GUI in foreground
drone_control_gui

# Cleanup on exit
echo ""
echo "Shutting down..."
killall -9 px4 gz MicroXRCEAgent 2>/dev/null
