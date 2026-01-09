#!/bin/bash

###############################################################################
# PX4 ROS 2 Feature Test Script
# This script validates all components of the PX4 ROS 2 workspace
###############################################################################

set -e  # Exit on error

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "========================================="
echo "   PX4 ROS 2 Feature Test Script"
echo "========================================="
echo -e "${NC}"

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

###############################################################################
# Test 1: Check if workspace is built
###############################################################################
echo -e "${YELLOW}[1/8] Checking workspace build...${NC}"
if [ -d "$HOME/ws_sensor_combined/install" ]; then
    echo -e "${GREEN}✓ Workspace is built${NC}"
else
    echo -e "${RED}✗ Workspace not built. Run: colcon build${NC}"
    exit 1
fi

###############################################################################
# Test 2: Check if PX4-Autopilot exists
###############################################################################
echo -e "${YELLOW}[2/8] Checking PX4-Autopilot installation...${NC}"
if [ -d "$HOME/PX4-Autopilot" ]; then
    echo -e "${GREEN}✓ PX4-Autopilot found at ~/PX4-Autopilot${NC}"
else
    echo -e "${RED}✗ PX4-Autopilot not found${NC}"
    exit 1
fi

###############################################################################
# Test 3: Check if MicroXRCEAgent is installed
###############################################################################
echo -e "${YELLOW}[3/8] Checking MicroXRCEAgent installation...${NC}"
if command -v MicroXRCEAgent &> /dev/null; then
    echo -e "${GREEN}✓ MicroXRCEAgent is installed${NC}"
else
    echo -e "${RED}✗ MicroXRCEAgent not found${NC}"
    exit 1
fi

###############################################################################
# Test 4: Check ROS 2 packages
###############################################################################
echo -e "${YELLOW}[4/8] Checking ROS 2 packages...${NC}"

packages=("px4_msgs" "px4_ros_com" "px4_ros2_cpp" "drone_control_gui")
all_found=true

for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo -e "${GREEN}  ✓ ${pkg}${NC}"
    else
        echo -e "${RED}  ✗ ${pkg}${NC}"
        all_found=false
    fi
done

if ! $all_found; then
    echo -e "${RED}Some packages are missing. Rebuild workspace.${NC}"
    exit 1
fi

###############################################################################
# Test 5: Clean up existing processes
###############################################################################
echo -e "${YELLOW}[5/8] Cleaning up existing processes...${NC}"
killall -9 px4 gz gzserver gzclient MicroXRCEAgent 2>/dev/null || true
sleep 2
echo -e "${GREEN}✓ Processes cleaned${NC}"

###############################################################################
# Test 6: Start simulation and test communication
###############################################################################
echo -e "${YELLOW}[6/8] Starting simulation for communication test...${NC}"

# Start MicroXRCEAgent
echo "  Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/agent_test.log 2>&1 &
AGENT_PID=$!
sleep 2

# Start PX4 SITL
echo "  Starting PX4 SITL (this takes ~15 seconds)..."
cd ~/PX4-Autopilot
timeout 30 make px4_sitl gz_x500_depth > /tmp/px4_test.log 2>&1 &
PX4_PID=$!

# Wait for PX4 to initialize
sleep 18

# Check if PX4 is running
if ps -p $PX4_PID > /dev/null 2>&1; then
    echo -e "${GREEN}  ✓ PX4 SITL is running${NC}"
else
    echo -e "${RED}  ✗ PX4 SITL failed to start${NC}"
    echo "Check /tmp/px4_test.log for details"
    kill $AGENT_PID 2>/dev/null || true
    exit 1
fi

###############################################################################
# Test 7: Verify ROS 2 communication
###############################################################################
echo -e "${YELLOW}[7/8] Testing ROS 2 communication...${NC}"

# Test if topics are available
echo "  Checking for ROS 2 topics..."
if timeout 5 ros2 topic list | grep -q "/fmu/out/sensor_combined"; then
    echo -e "${GREEN}  ✓ ROS 2 topics are available${NC}"
else
    echo -e "${RED}  ✗ ROS 2 topics not found${NC}"
    kill $PX4_PID $AGENT_PID 2>/dev/null || true
    exit 1
fi

# Test if sensor data is being published
echo "  Testing sensor data reception..."
if timeout 5 ros2 topic echo /fmu/out/sensor_combined --once > /dev/null 2>&1; then
    echo -e "${GREEN}  ✓ Sensor data is being received${NC}"
else
    echo -e "${RED}  ✗ No sensor data received${NC}"
    kill $PX4_PID $AGENT_PID 2>/dev/null || true
    exit 1
fi

# Count available topics
TOPIC_COUNT=$(timeout 3 ros2 topic list 2>/dev/null | wc -l)
echo -e "${GREEN}  ✓ Found ${TOPIC_COUNT} ROS 2 topics${NC}"

###############################################################################
# Test 8: Test available executables
###############################################################################
echo -e "${YELLOW}[8/8] Testing available executables...${NC}"

executables=(
    "px4_ros_com:sensor_combined_listener"
    "px4_ros_com:vehicle_gps_position_listener"
    "px4_ros_com:offboard_control"
    "px4_ros_com:offboard_control.py"
    "drone_control_gui:drone_control_gui"
)

for exec in "${executables[@]}"; do
    pkg="${exec%%:*}"
    exe="${exec##*:}"
    if ros2 pkg executables "$pkg" 2>/dev/null | grep -q "$exe"; then
        echo -e "${GREEN}  ✓ ${exec}${NC}"
    else
        echo -e "${YELLOW}  ⚠ ${exec} (optional)${NC}"
    fi
done

###############################################################################
# Cleanup
###############################################################################
echo ""
echo -e "${YELLOW}Cleaning up test processes...${NC}"
kill $PX4_PID $AGENT_PID 2>/dev/null || true
killall -9 px4 gz gzserver gzclient MicroXRCEAgent 2>/dev/null || true
sleep 2

###############################################################################
# Summary
###############################################################################
echo ""
echo -e "${GREEN}"
echo "========================================="
echo "   ✓ All Tests Passed!"
echo "========================================="
echo -e "${NC}"
echo ""
echo "Next steps:"
echo "  1. Run ./start_drone_gui.sh to launch with GUI"
echo "  2. Or ./start_clean.sh for single-terminal mode"
echo "  3. Run manual commands (see README.md)"
echo ""
echo "Available commands:"
echo "  ros2 run px4_ros_com sensor_combined_listener"
echo "  ros2 run px4_ros_com offboard_control.py"
echo "  drone_control_gui"
echo ""
