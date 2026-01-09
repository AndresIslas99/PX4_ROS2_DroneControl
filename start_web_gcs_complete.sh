#!/bin/bash

###############################################################################
# Complete Web GCS Startup Script
# Starts PX4, video server, and Web GCS all together
###############################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "============================================="
echo "  Professional Web GCS Complete Startup"
echo "============================================="
echo -e "${NC}"

# Step 1: Clean environment
echo -e "${YELLOW}[1/6] Cleaning environment...${NC}"
killall -9 px4 gz gzserver gzclient MicroXRCEAgent ruby web_video_server 2>/dev/null || true
sleep 3
echo -e "${GREEN}✓ Clean${NC}"
echo ""

# Step 2: Setup environment
echo -e "${YELLOW}[2/6] Setting up environment...${NC}"
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastrtps_discovery.yaml
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
cd ~/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}
echo -e "${GREEN}✓ Environment configured${NC}"
echo ""

# Step 3: Start MicroXRCE-DDS Agent
echo -e "${YELLOW}[3/6] Starting MicroXRCE-DDS Agent...${NC}"
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

# Step 4: Start PX4 SITL
echo -e "${YELLOW}[4/6] Starting PX4 SITL (takes ~20 seconds)...${NC}"
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth > /tmp/px4_startup.log 2>&1 &
PX4_PID=$!

echo "  Waiting for PX4 initialization..."
sleep 18

if ! ps -p $PX4_PID > /dev/null 2>&1; then
    echo -e "${RED}✗ PX4 crashed during startup!${NC}"
    kill $AGENT_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✓ PX4 SITL is running (PID: $PX4_PID)${NC}"
echo ""

# Step 5: Configure PX4 parameters
echo -e "${YELLOW}[5/6] Waiting for PX4 ready status...${NC}"
sleep 5

echo -e "${CYAN}${NC}"
echo -e "${CYAN}  IMPORTANT: Set PX4 parameters in the PX4 console${NC}"
echo -e "${CYAN}  (In the terminal where PX4 is running):${NC}"
echo -e "${MAGENTA}"
echo "    param set COM_RCL_EXCEPT 4"
echo "    param set COM_RC_IN_MODE 1"
echo "    param set COM_ARM_WO_GPS 1"
echo "    param set NAV_RCL_ACT 0"
echo "    param set NAV_DLL_ACT 0"
echo "    param set CBRK_USB_CHK 197848"
echo "    param save"
echo -e "${NC}"
echo ""

# Step 6: Start video server
echo -e "${YELLOW}[6/6] Starting video server...${NC}"
cd ~/ws_sensor_combined
source install/setup.bash

# Check if web_video_server is installed
if ros2 pkg list | grep -q "web_video_server"; then
    ros2 run web_video_server web_video_server --ros-args -p port:=8080 > /tmp/video_server.log 2>&1 &
    VIDEO_PID=$!
    sleep 2

    if ps -p $VIDEO_PID > /dev/null; then
        echo -e "${GREEN}✓ Video server started (PID: $VIDEO_PID)${NC}"
        echo -e "${GREEN}  Video available at: http://localhost:8080${NC}"
    else
        echo -e "${YELLOW}⚠ Video server failed, but continuing...${NC}"
        VIDEO_PID=""
    fi
else
    echo -e "${YELLOW}⚠ web_video_server not installed${NC}"
    echo -e "${CYAN}  Install with: sudo apt install ros-humble-web-video-server${NC}"
    VIDEO_PID=""
fi
echo ""

# Step 7: Verify data flow
echo -e "${YELLOW}Verifying data flow...${NC}"
sleep 3

TOPIC_COUNT=$(timeout 5 ros2 topic list 2>/dev/null | grep -c "/fmu" || echo "0")
echo -e "  Found ${CYAN}${TOPIC_COUNT}${NC} PX4 topics"

if [ "$TOPIC_COUNT" -lt 20 ]; then
    echo -e "${YELLOW}⚠ Low topic count, waiting more...${NC}"
    sleep 5
fi

# Test sensor data
if timeout 5 ros2 topic echo /fmu/out/sensor_combined --once > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Sensor data is flowing!${NC}"
    DATA_FLOWING=true
else
    echo -e "${YELLOW}⚠ Sensor data not detected yet${NC}"
    DATA_FLOWING=false
fi
echo ""

# Final status
echo -e "${CYAN}"
echo "============================================="
echo "  🚀 System Started Successfully!"
echo "============================================="
echo -e "${NC}"
echo ""

echo -e "${GREEN}Running Services:${NC}"
echo "  • MicroXRCE Agent: PID $AGENT_PID (port 8888)"
echo "  • PX4 SITL: PID $PX4_PID"
if [ -n "$VIDEO_PID" ]; then
    echo "  • Video Server: PID $VIDEO_PID (port 8080)"
fi
echo ""

echo -e "${MAGENTA}Now starting Web GCS...${NC}"
echo ""

# Save PIDs for cleanup
echo "$AGENT_PID" > /tmp/web_gcs_pids.txt
echo "$PX4_PID" >> /tmp/web_gcs_pids.txt
if [ -n "$VIDEO_PID" ]; then
    echo "$VIDEO_PID" >> /tmp/web_gcs_pids.txt
fi

echo -e "${CYAN}============================================="
echo "  Starting Web GCS Server"
echo "=============================================${NC}"
echo ""
echo -e "${GREEN}✓ Open your browser to: ${YELLOW}http://localhost:5000${NC}"
echo ""
echo -e "${CYAN}Web GCS Features:${NC}"
echo "  ✓ Real-time telemetry (position, attitude, battery)"
echo "  ✓ Live camera feed"
echo "  ✓ Flight control (arm, takeoff, land)"
echo "  ✓ Status monitoring"
echo "  ✓ WebSocket updates at 10 Hz"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all services${NC}"
echo ""

# Trap Ctrl+C to cleanup
trap 'echo ""; echo "Shutting down..."; killall -9 px4 gz MicroXRCEAgent web_video_server 2>/dev/null; exit 0' INT TERM

# Start Web GCS (this will block)
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run Web GCS directly with Python
python3 install/web_gcs/lib/python3.10/site-packages/web_gcs/gcs_server.py

# Cleanup on exit
killall -9 px4 gz MicroXRCEAgent web_video_server 2>/dev/null || true
echo "All services stopped."
