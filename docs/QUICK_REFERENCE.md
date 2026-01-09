# PX4 ROS 2 Quick Reference

## Fast Start Commands

```bash
# Start everything with GUI
cd ~/ws_sensor_combined && ./start_drone_gui.sh

# Start in single terminal (better for debugging)
cd ~/ws_sensor_combined && ./start_clean.sh

# Test all features
cd ~/ws_sensor_combined && ./test_all_features.sh
```

## Common ROS 2 Commands

### Topic Monitoring
```bash
# List all topics
ros2 topic list

# Echo sensor data (once)
ros2 topic echo /fmu/out/sensor_combined --once

# Monitor vehicle position
ros2 topic echo /fmu/out/vehicle_local_position_v1

# Check topic frequency
ros2 topic hz /fmu/out/sensor_combined
```

### Run Examples
```bash
# Sensor listener
ros2 run px4_ros_com sensor_combined_listener

# Offboard control (automatic takeoff/land)
ros2 run px4_ros_com offboard_control.py

# GPS listener
ros2 run px4_ros_com vehicle_gps_position_listener

# Drone control GUI
drone_control_gui
```

## Key Topics

### Send Commands to PX4

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Set which control inputs are active |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Position/velocity/acceleration setpoints |
| `/fmu/in/vehicle_command` | `VehicleCommand` | Commands (arm, disarm, mode change) |

### Receive Data from PX4

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/fmu/out/sensor_combined` | `SensorCombined` | IMU data (gyro, accel) |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | Attitude (quaternion) |
| `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | Local position (NED) |
| `/fmu/out/vehicle_gps_position` | `VehicleGpsPosition` | GPS data |
| `/fmu/out/vehicle_status_v1` | `VehicleStatus` | Status, mode, arming |

## Offboard Control Pattern

```python
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

# 1. Publish offboard_control_mode at 2+ Hz
mode_msg = OffboardControlMode()
mode_msg.position = True
mode_msg.velocity = False
mode_msg.acceleration = False
mode_msg.attitude = False
mode_msg.body_rate = False
# Publish to /fmu/in/offboard_control_mode

# 2. Send setpoints continuously
setpoint_msg = TrajectorySetpoint()
setpoint_msg.position = [0.0, 0.0, -5.0]  # NED frame (Z is down)
setpoint_msg.yaw = 1.57  # radians
# Publish to /fmu/in/trajectory_setpoint

# 3. Engage offboard mode
cmd = VehicleCommand()
cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
cmd.param1 = 1.0  # Main mode
cmd.param2 = 6.0  # Offboard mode
# Publish to /fmu/in/vehicle_command

# 4. Arm the vehicle
cmd = VehicleCommand()
cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
cmd.param1 = 1.0  # Arm
# Publish to /fmu/in/vehicle_command
```

## Vehicle Commands

```python
from px4_msgs.msg import VehicleCommand

# Arm
VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0

# Disarm
VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0

# Set mode to offboard
VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0

# Takeoff
VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF

# Land
VehicleCommand.VEHICLE_CMD_NAV_LAND
```

## Coordinate Frames

PX4 uses **NED (North-East-Down)** coordinate system:
- X: North
- Y: East
- Z: Down (negative altitude)

Example:
- Move 5m forward (north): `position = [5.0, 0.0, 0.0]`
- Move 5m up: `position = [0.0, 0.0, -5.0]`

## Troubleshooting Commands

```bash
# Kill all processes
killall -9 px4 gz MicroXRCEAgent

# Check if MicroXRCEAgent is running
ps aux | grep MicroXRCEAgent

# Check PX4 process
ps aux | grep px4

# View PX4 console output
# (If running in separate terminal, check that terminal)

# Check ROS 2 daemon
ros2 daemon status

# Restart ROS 2 daemon if needed
ros2 daemon stop
ros2 daemon start

# Check which packages are available
ros2 pkg list | grep px4
```

## Build Commands

```bash
# Build all packages
cd ~/ws_sensor_combined
colcon build

# Build specific package
colcon build --packages-select px4_ros_com

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build
rm -rf build install log
colcon build
```

## Useful Aliases

Add to your `~/.bashrc`:

```bash
# PX4 ROS 2 aliases
alias px4_source='source /opt/ros/humble/setup.bash && source ~/ws_sensor_combined/install/setup.bash'
alias px4_build='cd ~/ws_sensor_combined && colcon build'
alias px4_clean='cd ~/ws_sensor_combined && rm -rf build install log && colcon build'
alias px4_start='cd ~/ws_sensor_combined && ./start_clean.sh'
alias px4_gui='cd ~/ws_sensor_combined && ./start_drone_gui.sh'
alias px4_test='cd ~/ws_sensor_combined && ./test_all_features.sh'
alias px4_topics='ros2 topic list | grep fmu'
```

Then reload: `source ~/.bashrc`

## Gazebo Commands

```bash
# List available models
gz model --list

# Spawn a model
gz model --spawn-name=<name> --model-file=<sdf_file>

# List topics
gz topic --list

# Echo a topic
gz topic --echo --topic <topic_name>

# Start Gazebo GUI only
gz sim -g

# Start Gazebo server only
gz sim -s
```

## PX4 SITL Models

Available models (from ~/PX4-Autopilot):
- `gz_x500` - Basic quadcopter
- `gz_x500_depth` - Quadcopter with depth camera
- `gz_x500_vision` - Quadcopter with vision sensors
- `gz_standard_vtol` - VTOL aircraft

To change model, edit the startup script or run:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_<model_name>
```

## Resources

- **PX4 Docs**: https://docs.px4.io/main/en/ros2/
- **Message Reference**: See px4_msgs package
- **Examples**: `~/ws_sensor_combined/src/px4_ros_com/src/examples/`
- **Interface Library**: `~/ws_sensor_combined/src/px4-ros2-interface-lib/`

## Support

If you encounter issues:
1. Check README.md troubleshooting section
2. Run test_all_features.sh to diagnose
3. Check logs in `/tmp/px4*.log` and `/tmp/agent*.log`
4. Verify all processes are running: `ps aux | grep -E "(px4|MicroXRCE)"`
