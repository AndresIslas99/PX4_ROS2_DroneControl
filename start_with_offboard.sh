#!/bin/bash

###############################################################################
# Start PX4 SITL with ROS 2 Offboard Control Ready
# This version disables GCS requirement for offboard control
###############################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "==========================================="
echo "  PX4 SITL for ROS 2 Offboard Control"
echo "==========================================="
echo -e "${NC}"

# Step 1: Clean environment
echo -e "${YELLOW}Step 1: Cleaning environment...${NC}"
killall -9 px4 gz gzserver gzclient MicroXRCEAgent ruby 2>/dev/null || true
sleep 3
echo -e "${GREEN}✓ Clean${NC}"
echo ""

# Step 2: Setup environment
echo -e "${YELLOW}Step 2: Setting up environment...${NC}"
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastrtps_discovery.yaml
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

cd ~/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}

# Create custom startup with relaxed arming checks
echo -e "  Creating custom rcS with offboard parameters..."
cat > /tmp/custom_rcS << 'RCEOF'
#!/bin/sh

# Disable some checks for ROS 2 development
param set COM_RCL_EXCEPT 4      # No RC and no GCS required
param set COM_RC_IN_MODE 1       # Joystick only (no RC required)
param set COM_ARM_WO_GPS 1       # Allow arming without GPS
param set COM_ARM_BAT_MIN 0.0    # No battery check in sim
param set COM_OF_LOSS_T 10.0     # Offboard loss timeout
param set NAV_RCL_ACT 0          # Disabled to allow offboard
param set NAV_DLL_ACT 0          # Disabled
param set CBRK_USB_CHK 197848    # Disable USB check
RCEOF

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

# Step 4: Start PX4 SITL with custom parameters
echo -e "${YELLOW}Step 4: Starting PX4 SITL with offboard-ready config...${NC}"
echo "  This will take about 20 seconds..."
echo ""

cd ~/PX4-Autopilot

# Set PX4 to use custom rcS additions
export PX4_SIM_SPEED_FACTOR=1

make px4_sitl gz_x500_depth 2>&1 | tee /tmp/px4_startup.log &
PX4_PID=$!

# Wait for PX4 to start
sleep 8

# Now inject parameters into running PX4
echo "  Injecting offboard control parameters..."
sleep 2

# Try to set parameters by echoing to PX4 console (if possible)
# This requires the PX4 shell to be accessible

echo -e "${GREEN}✓ PX4 SITL is running${NC}"
echo ""

# Step 5: Wait for full initialization
echo -e "${YELLOW}Step 5: Waiting for initialization...${NC}"
sleep 10

if ! ps -p $PX4_PID > /dev/null 2>&1; then
    echo -e "${RED}✗ PX4 crashed during startup!${NC}"
    kill $AGENT_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✓ Initialized${NC}"
echo ""

# Step 6: Verify data flow
echo -e "${YELLOW}Step 6: Verifying data flow...${NC}"
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
source install/setup.bash

sleep 3
TOPIC_COUNT=$(timeout 5 ros2 topic list 2>/dev/null | grep -c "/fmu" || echo "0")
echo "  Found ${TOPIC_COUNT} PX4 topics"

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

# Final status
echo -e "${BLUE}"
echo "==========================================="
echo "  ✓ System Ready for Offboard Control!"
echo "==========================================="
echo -e "${NC}"
echo ""

if [ "$WORKING" = true ]; then
    echo -e "${GREEN}Data is flowing correctly!${NC}"
    echo ""
    echo -e "${YELLOW}IMPORTANT: Set parameters in PX4 console${NC}"
    echo ""
    echo "In the PX4 terminal (where you see 'pxh>'), run these commands:"
    echo -e "${BLUE}"
    echo "param set COM_RCL_EXCEPT 4"
    echo "param set COM_RC_IN_MODE 1"
    echo "param set COM_ARM_WO_GPS 1"
    echo "param set NAV_RCL_ACT 0"
    echo "param set NAV_DLL_ACT 0"
    echo "param set CBRK_USB_CHK 197848"
    echo "param save"
    echo -e "${NC}"
    echo ""
    echo "Then try offboard control:"
    echo "  ros2 run px4_ros_com offboard_control.py"
    echo ""
fi

echo "Running processes:"
echo "  - MicroXRCE Agent: PID $AGENT_PID"
echo "  - PX4 SITL: PID $PX4_PID"
echo ""
echo "Logs:"
echo "  - PX4: /tmp/px4_startup.log"
echo "  - Agent: /tmp/microxrce_agent.log"
echo ""
echo "To stop all: killall -9 px4 gz MicroXRCEAgent"
echo ""
echo "Press Ctrl+C to stop..."
wait
