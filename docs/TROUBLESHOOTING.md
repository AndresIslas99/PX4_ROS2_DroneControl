# PX4 ROS 2 Troubleshooting Guide

## Common Issue: Empty Topics / No Data Flow

### Symptoms
- `ros2 topic list` shows topics exist
- `ros2 topic echo /fmu/out/sensor_combined` shows no data or times out
- GUI shows no telemetry
- PX4 console shows: "WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data"

### Root Cause
PX4's EKF2 (Extended Kalman Filter) is not receiving sensor data from Gazebo. This happens when:
1. Gazebo simulation isn't properly connected to PX4
2. gz_bridge isn't working
3. Timing synchronization issues between Gazebo and PX4

### Solution: Use the Fixed Startup Script

```bash
cd ~/ws_sensor_combined
./start_fixed.sh
```

This script:
- Properly configures the environment
- Starts components in the correct order
- Waits for synchronization
- Verifies data flow
- Provides detailed diagnostics

## Diagnostic Steps

### Step 1: Check if Everything is Running

```bash
# Check processes
ps aux | grep -E "(px4|MicroXRCE|gz)"

# Should see:
# - MicroXRCEAgent (1 process)
# - px4 (multiple processes)
# - gz sim (2-3 processes)
```

### Step 2: Check PX4 Logs for EKF2

```bash
# In the PX4 console, look for:
tail -50 /tmp/px4_startup.log | grep -E "(ekf2|ERROR|WARN)"

# Good signs:
# - "INFO  [ekf2] starting GPS fusion"
# - "INFO  [ekf2] starting vision alignment"

# Bad signs:
# - "WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data"
# - "ERROR [ekf2] ..." anything
```

### Step 3: Check Gazebo Topics

```bash
# List Gazebo topics
gz topic --list

# Should see topics like:
# /world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu
# /world/default/model/x500_depth_0/link/base_link/sensor/air_pressure_sensor/...
```

### Step 4: Check ROS 2 Topics

```bash
source ~/ws_sensor_combined/install/setup.bash

# List topics
ros2 topic list | grep fmu

# Test specific topic
timeout 5 ros2 topic echo /fmu/out/sensor_combined --once

# Check topic frequency (should be ~100-250 Hz)
ros2 topic hz /fmu/out/sensor_combined
```

### Step 5: Check DDS Communication

```bash
# Check if PX4 synchronized with DDS agent
grep "synchronized with time offset" /tmp/px4_startup.log

# Check agent log
tail -20 /tmp/microxrce_agent.log
```

## Common Fixes

### Fix 1: Full Restart

```bash
# Kill everything
killall -9 px4 gz gzserver gzclient MicroXRCEAgent ruby

# Wait
sleep 5

# Restart with fixed script
cd ~/ws_sensor_combined
./start_fixed.sh
```

### Fix 2: Check Gazebo Version

```bash
gz --version

# Should show: Gazebo Harmonic (8.x.x) or Garden (7.x.x)
# PX4 works best with Gazebo Harmonic
```

### Fix 3: Rebuild PX4 Clean

```bash
cd ~/PX4-Autopilot

# Clean build
make distclean
make px4_sitl_default

# Then try again
cd ~/ws_sensor_combined
./start_fixed.sh
```

### Fix 4: Check ROS_DOMAIN_ID

```bash
# In all terminals, ensure ROS_DOMAIN_ID is consistent
echo $ROS_DOMAIN_ID

# If empty or different values, set it:
export ROS_DOMAIN_ID=0

# Add to ~/.bashrc to make permanent:
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### Fix 5: FastRTPS Configuration

```bash
# Verify FastRTPS config exists
cat /etc/ros/fastrtps_discovery.yaml

# If missing, create it:
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

# Set environment variable
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastrtps_discovery.yaml
```

## Specific Error Messages

### "ekf2 missing data"

**Cause**: Gazebo sensors aren't publishing to PX4

**Fix**:
1. Check if Gazebo simulation is running: `ps aux | grep "gz sim"`
2. Check Gazebo topics: `gz topic --list`
3. Restart with proper timing: `./start_fixed.sh`

### "No connection to the GCS"

**Cause**: QGroundControl not running (this is OK for ROS 2 development)

**Fix**: Ignore this warning, or start QGroundControl if you need it

### "Preflight Fail: No GPS"

**Cause**: GPS hasn't locked yet (normal in simulation)

**Fix**: Wait 20-30 seconds, or ignore for local position control

### Topics exist but `ros2 topic echo` shows nothing

**Cause**: Publishers exist but aren't sending data

**Fix**:
1. Check topic type: `ros2 topic info /fmu/out/sensor_combined`
2. Check if PX4 is armed: `ros2 topic echo /fmu/out/vehicle_status_v1 --once`
3. Verify EKF2 is running: Check PX4 logs
4. Restart everything: `./start_fixed.sh`

### Gazebo starts but drone doesn't appear

**Cause**: Model spawn failed

**Fix**:
1. Check PX4 log: `grep "Spawning" /tmp/px4_startup.log`
2. Check Gazebo resources: `echo $GZ_SIM_RESOURCE_PATH`
3. Manually spawn: `gz model --spawn-name=x500 --model-file=...`

## Advanced Diagnostics

### Monitor All Logs in Real-Time

```bash
# Terminal 1: PX4 log
tail -f /tmp/px4_startup.log

# Terminal 2: MicroXRCE Agent log
tail -f /tmp/microxrce_agent.log

# Terminal 3: ROS 2 topic monitoring
watch -n 0.5 'ros2 topic list | grep fmu | wc -l'
```

### Check System Resources

```bash
# CPU usage (Gazebo is heavy)
top -n 1 | grep -E "(px4|gz|ruby)"

# If CPU > 90%, simulation may lag
# If memory > 8GB used, you may have issues
```

### Verify Network Settings

```bash
# Check localhost communication
ping -c 2 127.0.0.1

# Check UDP port 8888 (MicroXRCE)
netstat -tuln | grep 8888

# Should show:
# udp        0      0 0.0.0.0:8888            0.0.0.0:*
```

## Still Not Working?

### Last Resort: Complete Reset

```bash
# 1. Kill everything
killall -9 px4 gz gzserver gzclient MicroXRCEAgent ruby

# 2. Clean PX4 build
cd ~/PX4-Autopilot
make distclean
make px4_sitl_default

# 3. Clean ROS 2 workspace
cd ~/ws_sensor_combined
rm -rf build install log
colcon build

# 4. Reboot (clears all network/IPC issues)
sudo reboot

# 5. After reboot, try fixed script
cd ~/ws_sensor_combined
./start_fixed.sh
```

### Get Help

If still having issues, collect this info:

```bash
# System info
echo "=== System ===" > ~/px4_debug.txt
lsb_release -a >> ~/px4_debug.txt
uname -a >> ~/px4_debug.txt

# Gazebo version
echo "=== Gazebo ===" >> ~/px4_debug.txt
gz --version >> ~/px4_debug.txt

# ROS 2 version
echo "=== ROS 2 ===" >> ~/px4_debug.txt
ros2 --version >> ~/px4_debug.txt
env | grep ROS >> ~/px4_debug.txt

# PX4 log
echo "=== PX4 Log ===" >> ~/px4_debug.txt
tail -100 /tmp/px4_startup.log >> ~/px4_debug.txt

# Agent log
echo "=== Agent Log ===" >> ~/px4_debug.txt
tail -100 /tmp/microxrce_agent.log >> ~/px4_debug.txt

# Topic list
echo "=== Topics ===" >> ~/px4_debug.txt
timeout 5 ros2 topic list >> ~/px4_debug.txt 2>&1

# Share ~/px4_debug.txt when asking for help
cat ~/px4_debug.txt
```

## Prevention

To avoid these issues in the future:

1. **Always use the fixed startup script** (`./start_fixed.sh`)
2. **Set ROS_DOMAIN_ID=0** in ~/.bashrc
3. **Wait 20+ seconds** after starting PX4 before running commands
4. **Don't mix manual starts** with scripts (pick one method)
5. **Fully kill processes** between runs: `killall -9 px4 gz MicroXRCEAgent`
6. **Keep Gazebo updated**: `sudo apt update && sudo apt upgrade gazebo`

## Reference Commands

```bash
# Start system (RECOMMENDED)
./start_fixed.sh

# Quick diagnostics
./diagnose_and_fix.sh

# Full feature test
./test_all_features.sh

# Clean shutdown
killall -9 px4 gz MicroXRCEAgent

# Check what's running
ps aux | grep -E "(px4|gz|MicroXRCE)"

# Monitor sensor data
ros2 topic hz /fmu/out/sensor_combined
```
