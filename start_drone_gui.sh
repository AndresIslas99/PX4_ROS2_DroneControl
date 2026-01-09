#!/bin/bash

# Startup script for Drone Control GUI
# This script starts all necessary components in separate terminals

echo "Starting Drone Control System..."
echo "================================="
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/local_setup.bash

# Terminal 1: MicroXRCE-DDS Agent
echo "Terminal 1: Starting MicroXRCE-DDS Agent..."
gnome-terminal -- bash -c "echo 'MicroXRCE-DDS Agent'; MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 2

# Terminal 2: PX4 SITL with x500_depth
echo "Terminal 2: Starting PX4 SITL..."
gnome-terminal -- bash -c "echo 'PX4 SITL'; cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth; exec bash"
sleep 10

# Terminal 3: Gazebo GUI
echo "Terminal 3: Starting Gazebo GUI..."
gnome-terminal -- bash -c "echo 'Gazebo GUI'; gz sim -g; exec bash"
sleep 3

# Terminal 4: Drone Control GUI
echo "Terminal 4: Starting Drone Control GUI..."
gnome-terminal -- bash -c "echo 'Drone Control GUI'; source /opt/ros/humble/setup.bash; source ~/ws_sensor_combined/install/local_setup.bash; drone_control_gui; exec bash"

echo ""
echo "All components started!"
echo "======================="
echo ""
echo "Wait ~15 seconds for everything to initialize, then:"
echo "1. Check that all status indicators show data"
echo "2. Click 'ENABLE OFFBOARD' button"
echo "3. Click 'ARM' button"
echo "4. Click 'TAKEOFF' button"
echo "5. Use the joystick to fly!"
echo ""
echo "To stop: Close all terminal windows or press Ctrl+C in each"
