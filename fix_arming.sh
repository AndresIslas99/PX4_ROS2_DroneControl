#!/bin/bash

###############################################################################
# Fix PX4 Arming for ROS 2 Development
# This disables GCS requirement and enables offboard control
###############################################################################

echo "============================================"
echo "  Fixing PX4 Arming Configuration"
echo "============================================"
echo ""

# Check if PX4 is running
if ! pgrep -x "px4" > /dev/null; then
    echo "ERROR: PX4 is not running!"
    echo "Please start PX4 first with: ./start_fixed.sh"
    exit 1
fi

echo "Setting PX4 parameters for ROS 2 development..."
echo ""

# Send parameters via MAVLink commander
cd ~/PX4-Autopilot

# Method 1: Set parameters via px4 shell
echo "Configuring parameters..."

# Use px4-listener to send commands
# We'll create a parameter file that gets loaded

cat > /tmp/px4_ros2_params.txt << 'EOF'
# Disable GCS connection requirement
param set COM_RCL_EXCEPT 4

# Allow arming without RC
param set COM_RC_IN_MODE 1

# Disable battery failsafe for simulation
param set COM_ARM_BAT_MIN 0.0

# Enable offboard control
param set COM_OF_LOSS_T 10.0

# Disable preflight checks that aren't needed for sim
param set COM_ARM_WO_GPS 1

# Set to allow arming in simulation
param set CBRK_USB_CHK 197848

# Save parameters
param save
EOF

# Try to set parameters via commander
echo "Attempting to set parameters..."

# Check if we can connect to PX4 shell
if [ -e "/tmp/px4.sock" ] || pgrep -f "px4" > /dev/null; then
    echo "PX4 is running. Parameters should be set via PX4 console."
    echo ""
    echo "You need to set these parameters in the PX4 console:"
    echo "---------------------------------------------------"
    cat /tmp/px4_ros2_params.txt | grep "param set"
    echo "---------------------------------------------------"
    echo ""
    echo "To do this:"
    echo "1. In the terminal where PX4 is running, type these commands:"
    echo "   (or press Enter in that terminal to get the 'pxh>' prompt)"
    echo ""
    echo "2. Copy and paste these commands:"
    cat /tmp/px4_ros2_params.txt | grep "param set"
    echo ""
    echo "3. Then type: param save"
    echo ""
else
    echo "Could not detect PX4 running properly."
fi

echo ""
echo "============================================"
echo "  Alternative: Edit Parameters File"
echo "============================================"
echo ""
echo "If the above doesn't work, you can edit the parameters file directly:"
echo ""
echo "1. Stop PX4: killall -9 px4 gz MicroXRCEAgent"
echo "2. Edit: nano ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4002_gz_x500_depth"
echo "3. Add these lines before 'param save':"
cat /tmp/px4_ros2_params.txt | grep "param set"
echo "4. Restart: ./start_fixed.sh"
echo ""
