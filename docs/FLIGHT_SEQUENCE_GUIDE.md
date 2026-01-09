# 🚁 Web GCS Flight Control Guide

## ✅ What Was Fixed

### Offboard Control Issue
**Problem**: Clicking "TAKEOFF" or "OFFBOARD MODE" caused the drone to disarm immediately.

**Root Cause**: PX4 requires **continuous publishing** of:
1. `OffboardControlMode` messages (at least 2 Hz)
2. `TrajectorySetpoint` messages (position commands)

PX4 will **reject offboard mode** or **emergency disarm** if setpoints stop arriving.

**Solution**: The Web GCS now:
- Continuously publishes offboard control mode at **20 Hz**
- Continuously publishes trajectory setpoints at **20 Hz**
- Automatically enables this when you click OFFBOARD MODE or TAKEOFF

## 🎮 Correct Flight Sequence

### Method 1: Single-Click Takeoff (RECOMMENDED ⭐)

**Just click TAKEOFF!** The button now does everything automatically:

1. **Click "TAKEOFF"** in Web GCS
   - Starts publishing setpoints (20 Hz)
   - Waits 0.5s for PX4 to receive setpoints
   - Switches to OFFBOARD mode
   - Arms the drone
   - Drone ascends to 5 meters

2. **Monitor flight** - Watch telemetry update in real-time

3. **Click "LAND"** when ready
   - Sets target altitude to 0m
   - Drone descends smoothly

4. **Click "DISARM"** after landing
   - Stops motors
   - Disables offboard mode

### Method 2: Manual Step-by-Step

If you want more control:

1. **Click "OFFBOARD MODE"**
   - Starts publishing setpoints at 20 Hz
   - Switches to offboard mode
   - Status shows "Offboard"

2. **Click "ARM"**
   - Enables motors
   - Drone stays on ground (setpoint is at current position)

3. **Click "TAKEOFF"**
   - Updates setpoint to 5m altitude
   - Drone ascends

4. **Click "LAND"**
   - Updates setpoint to ground level
   - Drone descends

5. **Click "DISARM"**
   - Stops motors

## 📊 Understanding the Display

### Position (NED Frame)
- **X**: North (+) / South (-)
- **Y**: East (+) / West (-)
- **Z**: Down (+) / **Up (-)** ← Negative = altitude!

**Example**: `Z = -5.0m` means the drone is 5 meters above ground

### GPS Data
In simulation, GPS may show zeros until:
- Gazebo GPS sensor is properly configured
- Or you're flying in outdoor world with GPS enabled

This is **normal** - position data comes from local position estimator instead.

### Flight Mode Display
- **Manual** - Default mode
- **Altitude** - Altitude hold
- **Position** - Position hold
- **Offboard** - ROS 2 control active ✓
- **Auto Loiter** - Hovering automatically
- **Auto Land** - Landing automatically

## 🔧 Technical Details

### What Happens When You Click TAKEOFF

```python
1. self.offboard_mode_active = True
   → Timer starts publishing at 20 Hz:
     - OffboardControlMode (position=True)
     - TrajectorySetpoint (x=0, y=0, z=-5.0)

2. time.sleep(0.5)  # Wait for PX4 to receive >2 setpoints

3. Send VEHICLE_CMD_DO_SET_MODE (offboard)
   → PX4 switches to offboard mode

4. Send VEHICLE_CMD_COMPONENT_ARM_DISARM (arm)
   → Motors spin up

5. Drone follows setpoint → ascends to 5m
```

### Continuous Setpoint Publishing

The Web GCS runs a 20 Hz timer that publishes:

```python
OffboardControlMode:
  - position: True      # We control position
  - velocity: False
  - acceleration: False
  - attitude: False

TrajectorySetpoint:
  - position[0]: 0.0    # North
  - position[1]: 0.0    # East
  - position[2]: -5.0   # Up (negative in NED)
  - yaw: 0.0           # Heading
```

**Critical**: If publishing stops for >0.5 seconds, PX4 will **emergency disarm**!

## 🐛 Troubleshooting

### Problem: Drone still disarms when clicking TAKEOFF

**Check PX4 parameters** (in PX4 terminal):
```bash
param show COM_RCL_EXCEPT  # Should be 4
param show COM_ARM_WO_GPS  # Should be 1
param show NAV_RCL_ACT     # Should be 0
```

**Set if needed**:
```bash
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 1
param set COM_ARM_WO_GPS 1
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set CBRK_USB_CHK 197848
param save
```

### Problem: Position stays at 0, 0, 0

**Cause**: Drone hasn't moved yet

**Solution**: Click TAKEOFF - once armed and flying, position will update

### Problem: GPS shows 0 satellites

**This is normal in simulation**. GPS data in Gazebo requires:
- Outdoor world with GPS sensor configured
- Or manual GPS plugin in Gazebo model

Position control uses **local position** instead, which works fine.

### Problem: Web GCS shows "Cannot connect"

**Check services**:
```bash
ps aux | grep gcs_server
ps aux | grep px4
ps aux | grep MicroXRCE
```

**Restart if needed**:
```bash
cd ~/ws_sensor_combined
./start_web_gcs_complete.sh
```

## 🎯 Testing the Fix

1. **Open Web GCS**: http://localhost:5000
2. **Wait for green "Connected" status**
3. **Click "TAKEOFF"** button
4. **Observe**:
   - Status changes to "Offboard"
   - Armed badge turns RED
   - Position Z starts decreasing (going up!)
   - Drone visible in Gazebo rising to 5m
5. **Click "LAND"**
   - Position Z increases back to ~0
   - Drone descends
6. **Click "DISARM"**
   - Armed badge turns GRAY
   - Motors stop

## 📈 Expected Behavior

**Successful Takeoff:**
```
[INFO] [web_gcs_node]: Offboard control activated, target altitude: 5.0m
[INFO] [web_gcs_node]: Sent command: 176  (DO_SET_MODE)
[INFO] [web_gcs_node]: Sent command: 400  (ARM_DISARM)
[INFO] [web_gcs_node]: Takeoff sequence initiated
```

**PX4 Terminal:**
```
INFO [commander] Offboard mode activated
INFO [commander] Armed by external command
INFO [navigator] Takeoff to 5.0m
```

**Web GCS Display:**
```
Armed: YES (red)
Flight Mode: Offboard
Position Z: -0.5 → -1.0 → -2.0 → -5.0 (ascending)
```

## 🚀 Advanced Usage

### Custom Takeoff Altitude

Modify in browser console:
```javascript
socket.emit('command', { command: 'takeoff', altitude: 10.0 });
```

Or via API:
```bash
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "takeoff", "altitude": 10.0}'
```

### Custom Position Commands

To add waypoint navigation, you can modify `gcs_server.py`:
```python
def goto_position(self, north, east, altitude):
    """Fly to specific position"""
    self.current_setpoint.position[0] = north
    self.current_setpoint.position[1] = east
    self.current_setpoint.position[2] = -abs(altitude)
    self.offboard_mode_active = True
```

## 📚 References

- **PX4 Offboard Control**: https://docs.px4.io/main/en/ros2/offboard_control
- **PX4 Flight Modes**: https://docs.px4.io/main/en/flight_modes/
- **px4_msgs Documentation**: https://github.com/PX4/px4_msgs

---

**Your Web GCS now has proper offboard control!** 🎉

The drone should takeoff, fly, and land correctly using the TAKEOFF button.
