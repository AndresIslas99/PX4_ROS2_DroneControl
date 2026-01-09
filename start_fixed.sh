#!/bin/bash

###############################################################################
# Fixed PX4 SITL Startup Script
# This script properly starts PX4 with Gazebo and ensures data flows correctly
###############################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "==========================================="
echo "  PX4 SITL with Gazebo - Fixed Startup"
echo "==========================================="
echo -e "${NC}"

# Step 1: Clean environment
echo -e "${YELLOW}Step 1: Cleaning environment...${NC}"
killall -9 px4 gz gzserver gzclient MicroXRCEAgent ruby 2>/dev/null || true
sleep 3
echo -e "${GREEN}✓ Clean${NC}"
echo ""

# Step 2: Setup environment variables
echo -e "${YELLOW}Step 2: Setting up environment...${NC}"
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastrtps_discovery.yaml
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

cd ~/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}
echo -e "${GREEN}✓ Environment configured${NC}"
echo ""

# Step 3: Start MicroXRCE-DDS Agent
echo -e "${YELLOW}Step 3: Starting MicroXRCE-DDS Agent...${NC}"
MicroXRCEAgent udp4 -p 8888 > /tmp/microxrce_agent.log 2>&1 &
AGENT_PID=$!
sleep 2

if ps -p $AGENT_PID > /dev/null; then
    echo -e "${GREEN}✓ MicroXRCE Agent started (PID: $AGENT_PID)${NC}"
else
    echo -e "${RED}✗ Failed to start MicroXRCE Agent${NC}"
    exit 1
fi
echo ""

# Step 4: Start PX4 SITL with Gazebo
echo -e "${YELLOW}Step 4: Starting PX4 SITL with Gazebo...${NC}"
echo "  This will take about 15-20 seconds..."
echo "  Watch for 'INFO  [uxrce_dds_client] synchronized' message"
echo ""

cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth 2>&1 | tee /tmp/px4_startup.log &
PX4_PID=$!

# Wait for PX4 to initialize
echo "  Waiting for PX4 initialization..."
sleep 15

# Check if PX4 is still running
if ! ps -p $PX4_PID > /dev/null 2>&1; then
    echo -e "${RED}✗ PX4 crashed during startup!${NC}"
    echo "Check /tmp/px4_startup.log for errors"
    kill $AGENT_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✓ PX4 SITL is running${NC}"
echo ""

# Step 5: Wait for DDS synchronization
echo -e "${YELLOW}Step 5: Waiting for DDS synchronization...${NC}"
sleep 5

if grep -q "synchronized with time offset" /tmp/px4_startup.log 2>/dev/null; then
    echo -e "${GREEN}✓ DDS synchronized${NC}"
else
    echo -e "${YELLOW}⚠ DDS synchronization not confirmed, but continuing...${NC}"
fi
echo ""

# Step 6: Verify data flow
echo -e "${YELLOW}Step 6: Verifying data flow...${NC}"
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
source install/setup.bash

# Count topics
sleep 2
TOPIC_COUNT=$(timeout 5 ros2 topic list 2>/dev/null | grep -c "/fmu" || echo "0")
echo "  Found ${TOPIC_COUNT} PX4 topics"

if [ "$TOPIC_COUNT" -lt 10 ]; then
    echo -e "${YELLOW}⚠ Low topic count, waiting more...${NC}"
    sleep 5
    TOPIC_COUNT=$(timeout 5 ros2 topic list 2>/dev/null | grep -c "/fmu" || echo "0")
    echo "  Now found ${TOPIC_COUNT} PX4 topics"
fi

# Test sensor data
echo "  Testing sensor data stream..."
if timeout 8 ros2 topic echo /fmu/out/sensor_combined --once > /tmp/sensor_check.txt 2>&1; then
    if [ -s /tmp/sensor_check.txt ] && grep -q "timestamp:" /tmp/sensor_check.txt; then
        echo -e "${GREEN}✓ Sensor data is streaming!${NC}"
        WORKING=true
    else
        echo -e "${RED}✗ Sensor topic exists but no data${NC}"
        WORKING=false
    fi
else
    echo -e "${RED}✗ No sensor data received${NC}"
    WORKING=false
fi
echo ""

# Step 7: Final status and instructions
echo -e "${BLUE}"
echo "==========================================="
if [ "$WORKING" = true ]; then
    echo "  ✓ System is Ready!"
else
    echo "  ⚠ System Started (with warnings)"
fi
echo "==========================================="
echo -e "${NC}"
echo ""

if [ "$WORKING" = true ]; then
    echo -e "${GREEN}Everything is working correctly!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Start Gazebo GUI (optional):"
    echo "     gz sim -g"
    echo ""
    echo "  2. Run examples:"
    echo "     ros2 run px4_ros_com sensor_combined_listener"
    echo "     ros2 run px4_ros_com offboard_control.py"
    echo "     drone_control_gui"
    echo ""
else
    echo -e "${YELLOW}System started but data flow issues detected.${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check PX4 log:"
    echo "     tail -50 /tmp/px4_startup.log | grep -E '(ERROR|WARN|ekf2)'"
    echo ""
    echo "  2. Check if Gazebo is running:"
    echo "     ps aux | grep gz"
    echo ""
    echo "  3. Check Gazebo topics:"
    echo "     gz topic --list"
    echo ""
    echo "  4. Look for EKF2 errors in PX4 console"
    echo ""
    echo "  5. Try restarting:"
    echo "     killall -9 px4 gz MicroXRCEAgent"
    echo "     ./start_fixed.sh"
    echo ""
fi

echo "Running processes:"
echo "  - MicroXRCE Agent: PID $AGENT_PID"
echo "  - PX4 SITL: PID $PX4_PID"
echo ""
echo "Logs:"
echo "  - PX4: /tmp/px4_startup.log"
echo "  - Agent: /tmp/microxrce_agent.log"
echo "  - Sensor test: /tmp/sensor_check.txt"
echo ""
echo "To stop all: killall -9 px4 gz MicroXRCEAgent"
echo ""

# Keep terminal open
if [ "$WORKING" = true ]; then
    echo "Press Ctrl+C to stop..."
    wait
else
    echo -e "${YELLOW}Exiting due to errors. Processes left running for debugging.${NC}"
    echo "Use 'killall -9 px4 gz MicroXRCEAgent' to stop them."
fi
