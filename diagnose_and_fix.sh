#!/bin/bash

echo "==========================================="
echo "  PX4 ROS 2 Diagnostics & Fix Script"
echo "==========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Clean up
echo -e "${YELLOW}[1/5] Cleaning up existing processes...${NC}"
killall -9 px4 gz gzserver gzclient MicroXRCEAgent ruby 2>/dev/null || true
sleep 3
echo -e "${GREEN}✓ Cleaned${NC}"
echo ""

# Configure FastRTPS for better discovery
echo -e "${YELLOW}[2/5] Configuring FastRTPS discovery...${NC}"
sudo mkdir -p /etc/ros
sudo bash -c 'cat > /etc/ros/fastrtps_discovery.yaml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseDuration>
                        <sec>20</sec>
                    </leaseDuration>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF'
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastrtps_discovery.yaml
echo -e "${GREEN}✓ FastRTPS configured${NC}"
echo ""

# Start MicroXRCE Agent
echo -e "${YELLOW}[3/5] Starting MicroXRCE-DDS Agent...${NC}"
MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT_PID=$!
sleep 3
if ps -p $AGENT_PID > /dev/null; then
    echo -e "${GREEN}✓ Agent running (PID: $AGENT_PID)${NC}"
else
    echo -e "${RED}✗ Agent failed to start${NC}"
    exit 1
fi
echo ""

# Start PX4 SITL
echo -e "${YELLOW}[4/5] Starting PX4 SITL (wait ~20 seconds)...${NC}"
cd ~/PX4-Autopilot

# Source Gazebo environment
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GZ_SIM_SYSTEM_PLUGIN_PATH}

# Start PX4
timeout 60 make px4_sitl gz_x500_depth > /tmp/px4.log 2>&1 &
PX4_PID=$!

# Wait for initialization
echo "  Waiting for PX4 to initialize..."
sleep 20

# Check if still running
if ! ps -p $PX4_PID > /dev/null 2>&1; then
    echo -e "${RED}✗ PX4 crashed during startup${NC}"
    echo "Check /tmp/px4.log for details"
    kill $AGENT_PID 2>/dev/null
    exit 1
fi
echo -e "${GREEN}✓ PX4 is running${NC}"
echo ""

# Test ROS 2 communication
echo -e "${YELLOW}[5/5] Testing ROS 2 data flow...${NC}"
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
source install/setup.bash

# Wait a bit more for topics
sleep 5

echo "  Checking topics..."
TOPIC_COUNT=$(timeout 3 ros2 topic list 2>/dev/null | grep -c "/fmu" || echo "0")
echo "  Found ${TOPIC_COUNT} PX4 topics"

if [ "$TOPIC_COUNT" -lt 10 ]; then
    echo -e "${RED}✗ Not enough topics detected${NC}"
    echo "  Check /tmp/agent.log and /tmp/px4.log"
else
    echo -e "${GREEN}✓ Topics detected${NC}"
fi

echo ""
echo "  Testing sensor_combined data..."
if timeout 5 ros2 topic echo /fmu/out/sensor_combined --once > /tmp/sensor_test.txt 2>&1; then
    if [ -s /tmp/sensor_test.txt ]; then
        echo -e "${GREEN}✓ Sensor data is flowing!${NC}"
        echo ""
        echo "Sample data:"
        head -5 /tmp/sensor_test.txt
    else
        echo -e "${RED}✗ Sensor data is empty${NC}"
        echo "  This usually means Gazebo isn't sending data to PX4"
    fi
else
    echo -e "${RED}✗ Timeout waiting for sensor data${NC}"
    echo "  Check if Gazebo simulation is running properly"
fi

echo ""
echo "==========================================="
echo "  Diagnosis Complete"
echo "==========================================="
echo ""
echo "Logs saved to:"
echo "  - /tmp/agent.log (MicroXRCE Agent)"
echo "  - /tmp/px4.log (PX4 SITL)"
echo "  - /tmp/sensor_test.txt (Sensor data test)"
echo ""
echo "To view logs:"
echo "  tail -f /tmp/px4.log"
echo "  tail -f /tmp/agent.log"
echo ""
echo "Processes still running:"
echo "  - MicroXRCE Agent: PID $AGENT_PID"
echo "  - PX4 SITL: PID $PX4_PID"
echo ""
echo "To stop: killall -9 px4 gz MicroXRCEAgent"
echo ""
