#!/bin/bash
# Script to fix PX4 parameters for GUI control without GCS

echo "Fixing PX4 parameters..."

# Create a parameters file to disable GCS requirement
cat > ~/PX4-Autopilot/build/px4_sitl_default/etc/extras.txt << 'EOF'
# Disable GCS connection requirement
param set COM_ARM_WO_GPS 1
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0

# Allow arming without GCS
param set COM_RC_IN_MODE 1

# Enable offboard control without GCS
param set COM_RCL_EXCEPT 4

# Save parameters
param save
EOF

echo "Parameters configured!"
echo "Restart PX4 SITL for changes to take effect"
echo ""
echo "After restarting PX4, you should be able to arm without GCS"
