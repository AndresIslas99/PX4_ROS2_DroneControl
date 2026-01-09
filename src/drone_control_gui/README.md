# Drone Control GUI

A user-friendly PyQt5 GUI application for controlling PX4 drones with real-time camera feeds, depth visualization, velocity control, and waypoint mission planning.

## Features

- **RGB Camera Feed**: Real-time video from drone's camera
- **Depth Point Cloud Visualization**: 3D visualization of depth camera point cloud
- **Virtual Joystick**: Intuitive velocity control (forward/backward, left/right, up/down, yaw)
- **Status Display**: Real-time position, velocity, attitude, battery, and connection status
- **Quick Actions**: Arm, disarm, takeoff, land, emergency stop
- **Waypoint Mission Planner**: Create and execute multi-waypoint missions

## Installation

The package is already built in your workspace. Make sure you have all dependencies installed:

```bash
sudo apt install python3-pyqt5 python3-pyqtgraph python3-opencv
```

## Usage

### Option 1: Launch Everything Together (Recommended)

Use the launch file to start the complete system:

```bash
cd ~/ws_sensor_combined
source install/local_setup.bash
ros2 launch drone_control_gui gui_control.launch.py
```

This will automatically start:
1. MicroXRCEAgent
2. PX4 SITL with x500_depth model
3. Gazebo GUI
4. Drone Control GUI

### Option 2: Manual Start

**Terminal 1 - MicroXRCE-DDS Agent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 - PX4 SITL:**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```

**Terminal 3 - Gazebo GUI:**
```bash
gz sim -g
```

**Terminal 4 - Drone Control GUI:**
```bash
cd ~/ws_sensor_combined
source install/local_setup.bash
ros2 run drone_control_gui drone_control_gui
```

## Using the GUI

### 1. Connect and Arm

1. Wait for all status indicators to show data (position, attitude, battery)
2. Click **ENABLE OFFBOARD** to enter offboard control mode
3. Click **ARM** to arm the drone
4. Click **TAKEOFF** to takeoff to 5m altitude

### 2. Velocity Control with Joystick

- **Drag the joystick** to control horizontal velocity (forward/backward, left/right)
- **Altitude slider**: Control vertical velocity (up/down)
- **Yaw Rate slider**: Control rotation speed
- **Center / Stop button**: Return all controls to zero and stop the drone

### 3. Waypoint Missions

1. Click **Add Current Position** to add waypoints as you fly
2. Or click **Add Manual** to add a default waypoint
3. Click **Execute Mission** to start the mission
4. The drone will fly to each waypoint in sequence
5. Click **Stop Mission** to abort at any time

### 4. Emergency Procedures

- **EMERGENCY STOP**: Immediately sets all velocities to zero
- **DISARM**: Disarms the drone (only use when landed)
- **LAND**: Initiates automatic landing

## GUI Layout

```
┌─────────────────────────────────────────────────────────┐
│  RGB Camera  │  Depth Cloud  │  Status Panel           │
├─────────────────────────────────────────────────────────┤
│  Joystick    │  Waypoint Mission Planner               │
└─────────────────────────────────────────────────────────┘
```

## Controls

### Status Panel

- **Position**: X, Y, Z in NED frame (meters)
- **Velocity**: VX, VY, VZ (m/s)
- **Attitude**: Roll, Pitch, Yaw (degrees)
- **Battery**: Percentage with color-coded bar
- **Flight Status**: Current mode and armed state

### Quick Actions

- **ARM**: Arm the motors (required before flight)
- **DISARM**: Disarm the motors (only when landed)
- **TAKEOFF**: Automatic takeoff to 5m
- **LAND**: Automatic landing
- **ENABLE/DISABLE OFFBOARD**: Toggle offboard control mode
- **EMERGENCY STOP**: Immediately stop all movement

## Troubleshooting

### Camera feed not showing

- Check that Gazebo is running with x500_depth model
- Verify `/camera` topic is publishing: `ros2 topic list | grep camera`
- Check topic rate: `ros2 topic hz /camera`

### Point cloud not displaying

- Verify `/depth_camera/points` topic: `ros2 topic hz /depth_camera/points`
- Point cloud may take a few seconds to appear
- Try rotating the 3D view by dragging with mouse

### Drone not responding to commands

- Ensure MicroXRCEAgent is running
- Check PX4 connection: `ros2 topic hz /fmu/out/vehicle_status`
- Make sure OFFBOARD mode is enabled
- Verify drone is armed

### GUI crashes or freezes

- Check ROS 2 node is running: `ros2 node list`
- Restart the GUI: `Ctrl+C` and rerun
- Check for error messages in terminal

## Safety Notes

⚠️ **Important Safety Information:**

- Always test in simulation first
- Keep emergency stop readily accessible
- Monitor battery levels
- Understand all controls before flight
- Have a safety pilot ready to take manual control
- Follow local regulations for drone operations

## Technical Details

### ROS 2 Topics

**Subscribed Topics:**
- `/camera` - RGB camera feed
- `/depth_camera/points` - Point cloud data
- `/fmu/out/vehicle_local_position` - Position and velocity
- `/fmu/out/vehicle_status` - Flight mode and arming state
- `/fmu/out/vehicle_attitude` - Attitude (roll, pitch, yaw)
- `/fmu/out/battery_status` - Battery information

**Published Topics:**
- `/fmu/in/offboard_control_mode` - Offboard control mode
- `/fmu/in/trajectory_setpoint` - Velocity setpoints
- `/fmu/in/vehicle_command` - Vehicle commands (arm, disarm, etc.)

### Velocity Limits

- **Maximum horizontal velocity**: 3.0 m/s
- **Maximum vertical velocity**: 2.0 m/s
- **Maximum yaw rate**: 90°/s

### Coordinate Frames

- **PX4 NED Frame**: X-North, Y-East, Z-Down
- **Joystick**: Forward is +X, Left is +Y, Up is -Z

## Development

### Package Structure

```
drone_control_gui/
├── drone_control_gui/
│   ├── __init__.py
│   ├── main_window.py          # Main GUI window
│   ├── drone_controller.py     # ROS 2 node
│   └── widgets/
│       ├── camera_widget.py    # Camera display
│       ├── depth_widget.py     # Point cloud visualization
│       ├── joystick_widget.py  # Virtual joystick
│       ├── status_widget.py    # Status display
│       └── waypoint_widget.py  # Mission planner
├── launch/
│   └── gui_control.launch.py   # Launch file
├── package.xml
├── setup.py
└── README.md
```

## License

Apache 2.0

## Support

For issues or questions, refer to the PX4 documentation: https://docs.px4.io
