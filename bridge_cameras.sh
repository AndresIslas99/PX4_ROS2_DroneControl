#!/bin/bash

###############################################################################
# Gazebo-ROS 2 Camera Bridge
# Bridges Gazebo camera topics to ROS 2 for GUI visualization
###############################################################################

echo "Starting Gazebo-ROS 2 camera bridge..."
echo ""

source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash

# Check if ros-gz-bridge is installed
if ! ros2 pkg list | grep -q "ros_gz_bridge"; then
    echo "Installing ros-gz-bridge for Gazebo camera topics..."
    echo "This bridges Gazebo topics to ROS 2"
    echo ""
    echo "Run: sudo apt install ros-humble-ros-gz-bridge"
    echo ""
fi

# Method 1: Use ros_gz_bridge if available
if ros2 pkg list | grep -q "ros_gz_bridge"; then
    echo "Using ros_gz_bridge for camera bridging..."
    echo ""

    # Bridge Gazebo camera to ROS 2
    ros2 run ros_gz_bridge parameter_bridge \
        /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[ignition.msgs.Image \
        /depth_camera@sensor_msgs/msg/Image[ignition.msgs.Image \
        --ros-args -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera
else
    echo "ros_gz_bridge not found."
    echo ""
    echo "To install:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-image"
    echo ""
    echo "Then restart this script."
fi
