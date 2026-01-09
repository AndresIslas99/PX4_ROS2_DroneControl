#!/bin/bash

###############################################################################
# Install Web GCS Dependencies
###############################################################################

echo "========================================="
echo "  Installing Web GCS Dependencies"
echo "========================================="
echo ""

# Python packages for web server
echo "Installing Python packages..."
pip3 install --user flask flask-socketio flask-cors python-socketio eventlet simple-websocket

# ROS 2 web video server
echo ""
echo "Installing web_video_server..."
sudo apt update
sudo apt install -y ros-humble-web-video-server

# Optional: Install ros-gz-bridge for camera
echo ""
echo "Installing ros-gz-bridge (optional, for camera)..."
sudo apt install -y ros-humble-ros-gz-bridge ros-humble-ros-gz-image

echo ""
echo "========================================="
echo "  ✓ Installation Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Build the workspace:"
echo "     cd ~/ws_sensor_combined"
echo "     colcon build --packages-select web_gcs"
echo ""
echo "  2. Source the workspace:"
echo "     source install/setup.bash"
echo ""
echo "  3. Start the Web GCS:"
echo "     web_gcs"
echo ""
echo "  4. Open browser to: http://localhost:5000"
echo ""
