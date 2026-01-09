# 🎮 Web GCS - Remote Control & Mission Planner Guide

## ✨ New Features Added

Your Web GCS now includes two powerful new features in the center panel:

### 1. 🎮 Virtual Joystick Remote Control
Control your drone in real-time with intuitive button controls

### 2. 📍 Waypoint Mission Planner
Create and execute autonomous flight missions with multiple waypoints

---

## 🎮 Remote Control Features

### Controls Available

#### Altitude Control
- **UP ▲** - Ascend by 1 meter
- **DOWN ▼** - Descend by 1 meter

#### Horizontal Movement
- **↑ North** - Move 5 meters north
- **↓ South** - Move 5 meters south
- **← West** - Move 5 meters west (negative east)
- **→ East** - Move 5 meters east
- **⬛ STOP** - Hold current position

#### Yaw Rotation
- **↻ CW** - Rotate 30° clockwise
- **↺ CCW** - Rotate 30° counter-clockwise

### How to Use Remote Control

1. **First, TAKEOFF**
   ```
   Click "TAKEOFF" button → Drone arms and ascends to 5m
   ```

2. **Wait for stable hover**
   - Status shows "Offboard"
   - Armed badge is RED
   - Altitude stabilizes at ~5m

3. **Use the controls**
   - Click any direction button to move
   - Drone will fly to the new position
   - Click STOP to hold current position

4. **Monitor telemetry**
   - Position updates in real-time
   - Watch X, Y, Z coordinates change
   - Velocity shows movement direction

### Important Notes

⚠️ **Remote control requires Offboard mode**
- TAKEOFF automatically enables offboard mode
- If you manually ARM, click "OFFBOARD MODE" first

⚠️ **Position is relative to current location**
- Each button press moves from current position
- North = +X, East = +Y, Down = +Z (NED frame)

---

## 📍 Mission Planner Features

### Creating a Mission

1. **Add waypoints**
   - Enter **X (North)** coordinate in meters
   - Enter **Y (East)** coordinate in meters
   - Enter **Z (Altitude)** - use negative values (e.g., -5 for 5m altitude)
   - Click **+ ADD** button

2. **Waypoint appears in list**
   ```
   1. N:10m, E:0m, Alt:5m
   ```

3. **Add multiple waypoints**
   - Repeat the process
   - Each waypoint is numbered
   - Click ✖ to remove individual waypoints

4. **Clear all waypoints**
   - Click **✖ CLEAR** to remove all waypoints
   - Confirmation dialog appears

### Executing a Mission

1. **Ensure drone is in Offboard mode**
   ```
   Click TAKEOFF → Wait for stable hover
   ```

2. **Click ▶ START MISSION**

3. **Watch mission progress**
   - Status updates: "Flying to waypoint 1/3..."
   - Drone flies to each waypoint sequentially
   - Holds position for 2 seconds at each waypoint
   - Status updates when complete: "Mission complete!"

4. **Mission automatically stops when finished**

### Example Mission

**Simple square pattern at 5m altitude:**

```
Waypoint 1: X=10, Y=0,  Z=-5  (10m North)
Waypoint 2: X=10, Y=10, Z=-5  (10m North, 10m East)
Waypoint 3: X=0,  Y=10, Z=-5  (10m East)
Waypoint 4: X=0,  Y=0,  Z=-5  (Return to start)
```

**Vertical maneuver:**

```
Waypoint 1: X=0, Y=0, Z=-10  (Climb to 10m)
Waypoint 2: X=5, Y=5, Z=-10  (Move diagonally at 10m)
Waypoint 3: X=5, Y=5, Z=-3   (Descend to 3m)
```

---

## 🔧 Technical Details

### Coordinate System (NED Frame)

- **X (North)**: Positive is north, negative is south
- **Y (East)**: Positive is east, negative is west
- **Z (Down)**: Negative is up! (e.g., Z=-5 means 5m altitude)

### Position Tracking

The system tracks your current position in real-time:
```javascript
Current Position: { x: 0, y: 0, z: -5.0, yaw: 0 }
```

Each movement command updates the target position:
```javascript
Move North 5m:  target_x = current_x + 5
Move East 5m:   target_y = current_y + 5
Move Up 1m:     target_z = current_z - 1  (NED frame)
```

### Mission Execution

**Backend Logic:**
1. Mission runs in background thread
2. Publishes setpoints for each waypoint
3. Waits until within 1m of target
4. Holds position for 2 seconds
5. Proceeds to next waypoint
6. Emits status updates via WebSocket

**Safety:**
- Mission aborts if offboard mode is disabled
- Each waypoint is logged
- Real-time status updates to UI

---

## 🚀 Quick Start Example

### Test Remote Control

1. Start system:
   ```bash
   cd ~/ws_sensor_combined
   ./start_web_gcs_complete.sh
   ```

2. Open browser: **http://localhost:5000**

3. Set PX4 parameters (in PX4 console):
   ```bash
   param set COM_RCL_EXCEPT 4
   param set COM_RC_IN_MODE 1
   param set COM_ARM_WO_GPS 1
   param set NAV_RCL_ACT 0
   param set NAV_DLL_ACT 0
   param set CBRK_USB_CHK 197848
   param save
   ```

4. Test sequence:
   ```
   Click TAKEOFF → Wait for 5m altitude
   Click ↑ (North) → Drone moves north 5m
   Click → (East) → Drone moves east 5m
   Click ▼ (Down) → Drone descends 1m
   Click ⬛ (STOP) → Drone holds position
   Click LAND → Drone descends and lands
   ```

### Test Mission Planner

1. After takeoff, add waypoints:
   ```
   Waypoint 1: X=5,  Y=0,  Z=-5
   Waypoint 2: X=5,  Y=5,  Z=-5
   Waypoint 3: X=0,  Y=5,  Z=-5
   Waypoint 4: X=0,  Y=0,  Z=-5
   ```

2. Click **▶ START MISSION**

3. Watch status updates:
   ```
   "Flying to waypoint 1/4..."
   "Flying to waypoint 2/4..."
   "Flying to waypoint 3/4..."
   "Flying to waypoint 4/4..."
   "Mission complete! Flew to 4 waypoints."
   ```

4. Click **LAND** when done

---

## 🎯 What Changed

### Frontend (index.html)

**Added:**
- Virtual joystick UI with 9 buttons
- Mission planner form (X, Y, Z inputs)
- Waypoint list with remove buttons
- Mission status display

**JavaScript Functions:**
- `moveAltitude(delta)` - Change altitude
- `moveHorizontal(north, east)` - Move horizontally
- `stopMovement()` - Hold position
- `rotateYaw(degrees)` - Change heading
- `addWaypoint()` - Add to mission
- `removeWaypoint(index)` - Remove from mission
- `clearWaypoints()` - Clear all
- `executeMission()` - Start mission
- `updateWaypointList()` - Refresh UI

### Backend (gcs_server.py)

**Added Methods:**
- `goto_position(x, y, z, yaw)` - Move to coordinates
- `execute_mission(waypoints)` - Run waypoint mission

**Added WebSocket Handlers:**
- `'goto'` command - Remote control movements
- `'mission'` command - Mission execution
- `'mission_status'` event - Progress updates

**Added State Variables:**
- `target_position_x` - North coordinate
- `target_position_y` - East coordinate
- `target_yaw` - Heading in radians

**Updated:**
- `publish_offboard_control()` - Uses target position variables

---

## 📊 Benefits

### Remote Control
✅ Intuitive UI - No need to type commands
✅ Instant response - Real-time position updates
✅ Safe operation - Stop button for emergency hold
✅ Visual feedback - Position and velocity displayed

### Mission Planner
✅ Autonomous flight - Set it and watch it fly
✅ Multi-waypoint support - Complex flight patterns
✅ Progress tracking - Know exactly where drone is going
✅ Reusable missions - Save coordinates for repeated flights

---

## ⚠️ Safety Reminders

1. **Always TAKEOFF before using remote control or missions**
   - Offboard mode must be active
   - Continuous setpoint publishing required

2. **Monitor position carefully**
   - Watch X, Y, Z coordinates
   - Verify movement matches expectations
   - Use STOP button if behavior is unexpected

3. **Test in simulation first**
   - Gazebo provides safe environment
   - Verify mission paths before real flight
   - Check for obstacles in waypoint paths

4. **Emergency procedures**
   - Click LAND for immediate controlled descent
   - Click DISARM only when on ground
   - Close browser to stop commands (but offboard continues!)

---

## 🎓 Next Steps

Now that you have remote control and mission planning, you can:

1. **Create complex missions**
   - Survey patterns (grid search)
   - Perimeter patrols
   - Altitude profiles
   - Orbital patterns

2. **Combine with telemetry**
   - Monitor battery during missions
   - Track velocity during movements
   - Verify altitude during climbs

3. **Extend functionality**
   - Add mission templates
   - Save/load mission files
   - Add mission planning map
   - Real-time path visualization

---

## 📁 Modified Files

1. `/home/andres/ws_sensor_combined/src/web_gcs/web_gcs/templates/index.html`
   - Added remote control UI (lines 374-458)
   - Added JavaScript functions (lines 636-815)

2. `/home/andres/ws_sensor_combined/src/web_gcs/web_gcs/gcs_server.py`
   - Added target position variables (lines 140-142)
   - Updated offboard control publishing (lines 267-270)
   - Added `goto_position()` method (lines 352-365)
   - Added `execute_mission()` method (lines 367-444)
   - Added WebSocket handlers (lines 542-554)

---

## ✨ You're Ready to Fly!

Your Web GCS now has professional-grade remote control and mission planning capabilities!

**Start the system:**
```bash
cd ~/ws_sensor_combined
./start_web_gcs_complete.sh
```

**Open browser:**
```
http://localhost:5000
```

**Happy Flying!** 🚁✨
