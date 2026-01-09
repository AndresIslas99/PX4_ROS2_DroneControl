#!/bin/bash

###############################################################################
# Gazebo Camera Bridge for Web GCS
# Bridges the X500 depth camera from Gazebo to ROS 2
###############################################################################

echo "========================================="
echo "  Starting Gazebo Camera Bridge"
echo "========================================="
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

# Camera topic from Gazebo
CAMERA_TOPIC="/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image"

echo "Bridging camera topic:"
echo "  Gazebo: $CAMERA_TOPIC"
echo "  ROS 2:  $CAMERA_TOPIC"
echo ""

# Start the bridge
ros2 run ros_gz_bridge parameter_bridge \
    ${CAMERA_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image \
    > /tmp/camera_bridge.log 2>&1 &

BRIDGE_PID=$!
sleep 2

# Verify bridge is running
if ps -p $BRIDGE_PID > /dev/null; then
    echo "✓ Camera bridge started (PID: $BRIDGE_PID)"
    echo ""
    echo "Camera stream available at:"
    echo "  http://localhost:8080/stream?topic=${CAMERA_TOPIC}&type=mjpeg"
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""

    # Save PID for cleanup
    echo "$BRIDGE_PID" > /tmp/camera_bridge.pid

    # Wait for bridge to exit
    wait $BRIDGE_PID
else
    echo "✗ Failed to start camera bridge"
    cat /tmp/camera_bridge.log
    exit 1
fi
