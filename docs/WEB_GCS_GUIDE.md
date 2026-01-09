# Professional Web-based Ground Control Station

## 🎉 What You Have

A complete, professional web-based GCS with:

- ✅ **Real-time telemetry** via WebSocket (10 Hz updates)
- ✅ **Live video streaming** from drone camera
- ✅ **Flight controls** (ARM, TAKEOFF, LAND, RTL)
- ✅ **Beautiful modern UI** with dark theme
- ✅ **Position tracking** (NED coordinates)
- ✅ **Attitude display** (Roll, Pitch, Yaw)
- ✅ **Battery monitoring** with color-coded warnings
- ✅ **GPS information**
- ✅ **Flight mode display**
- ✅ **Altitude indicator**
- ✅ **Connection status**

## 🚀 Quick Start (All-in-One)

### Option 1: Complete Automatic Startup

```bash
cd ~/ws_sensor_combined
./start_web_gcs_complete.sh
```

This will:
1. Clean up old processes
2. Start MicroXRCE Agent
3. Start PX4 SITL with Gazebo
4. Start video server
5. Launch Web GCS

**Then open your browser to:** http://localhost:5000

### Option 2: Manual Step-by-Step

**Terminal 1: Start PX4 System**
```bash
cd ~/ws_sensor_combined
./start_fixed.sh
```

**In the PX4 console (terminal 1), set parameters:**
```bash
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 1
param set COM_ARM_WO_GPS 1
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set CBRK_USB_CHK 197848
param save
```

**Terminal 2: Start Video Server**
```bash
source ~/ws_sensor_combined/install/setup.bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

**Terminal 3: Start Web GCS**
```bash
source ~/ws_sensor_combined/install/setup.bash
web_gcs
```

**Terminal 4: Open browser**
```
http://localhost:5000
```

## 🎮 Using the Web GCS

### Interface Overview

The interface is divided into three panels:

#### **Left Panel: Telemetry**
- Armed/Disarmed status badge
- Flight mode
- Altitude indicator (visual gauge)
- Position (X, Y, Z in NED frame)
- Velocity (VX, VY, VZ)
- Battery (voltage, percentage with color warnings)
- GPS (satellites, lat/lon)

#### **Center Panel: Video & Attitude**
- Live camera feed from drone
- Video FPS and resolution overlay
- Attitude display (Roll, Pitch, Yaw in degrees)

#### **Right Panel: Controls**
- **ARM** - Enable motors
- **DISARM** - Disable motors
- **OFFBOARD MODE** - Switch to ROS 2 control
- **TAKEOFF** - Ascend to 5m
- **LAND** - Descend and land
- **RETURN TO LAUNCH** - RTL mode
- System info (update rate, last update time)
- Quick reference guide

### Control Workflow

**Standard Flight Sequence:**

1. **Check telemetry** - Ensure data is flowing (green status indicator)
2. **Set OFFBOARD MODE** - Click "OFFBOARD MODE" button
3. **ARM** - Click "ARM" button
4. **TAKEOFF** - Click "TAKEOFF" button (drone goes to 5m)
5. **Monitor** - Watch position, attitude, battery
6. **LAND** - Click "LAND" when ready to land
7. **DISARM** - Click "DISARM" after landed

**Emergency Stop:**
- Click "DISARM" immediately
- Or use "RETURN TO LAUNCH" for automatic return

## 📊 Understanding the Data

### Position (NED Frame)
- **X (North)**: Positive = North, Negative = South
- **Y (East)**: Positive = East, Negative = West
- **Z (Down)**: Positive = Down, Negative = Up (altitude)

**Example:** Z = -5.0m means drone is 5 meters above ground

### Attitude
- **Roll**: Bank angle (left/right tilt)
- **Pitch**: Nose up/down
- **Yaw**: Heading direction

### Battery
- **Green**: > 50% remaining
- **Yellow**: 20-50% remaining
- **Red**: < 20% remaining

### Flight Modes
- **Manual**: Pilot manual control
- **Altitude**: Altitude hold
- **Position**: Position hold
- **Offboard**: ROS 2 control
- **Auto Takeoff/Land**: Autonomous takeoff/landing

## 🎥 Video Streaming

The Web GCS uses `web_video_server` for video streaming.

### If Video Shows "Camera feed unavailable":

**1. Check if video server is running:**
```bash
ps aux | grep web_video_server
```

**2. Start video server if not running:**
```bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

**3. Check if camera topic exists:**
```bash
ros2 topic list | grep camera
```

**4. Test video URL directly:**
Open in browser: `http://localhost:8080/stream?topic=/camera&type=mjpeg`

### Troubleshooting Video

**Problem:** "Unknown message type [9]" when bridging Gazebo

**Solution:** The Web GCS uses standard ROS 2 topics. Make sure you're publishing to `/camera` topic:

```bash
# If you have a camera on /camera topic already, it should work
# If not, you need to bridge Gazebo camera or use a different source
ros2 topic echo /camera --once
```

## 🔧 Technical Details

### Architecture

```
┌─────────────┐
│   Browser   │ ← http://localhost:5000
└──────┬──────┘
       │ WebSocket (telemetry) + HTTP (video)
       │
┌──────▼──────────────┐
│   Web GCS Server    │ ← Flask + SocketIO
│  (port 5000)        │
└──────┬──────────────┘
       │ ROS 2 Topics
       │
┌──────▼──────────────┐
│   PX4 via DDS       │ ← /fmu/out/* and /fmu/in/*
└─────────────────────┘
```

### Communication

**Telemetry (WebSocket):**
- Client connects to `ws://localhost:5000/gcs`
- Server pushes updates at 10 Hz
- Real-time, bidirectional

**Video (MJPEG Stream):**
- HTTP streaming from `web_video_server`
- MJPEG format for browser compatibility
- URL: `http://localhost:8080/stream?topic=/camera&type=mjpeg`

**Commands (WebSocket):**
- Client sends: `{command: 'arm'}`, `{command: 'takeoff'}`, etc.
- Server executes and sends acknowledgment
- Commands translated to PX4 VehicleCommand messages

### Data Flow

```
PX4 Topics → WebGCSNode (ROS 2) → Flask-SocketIO → WebSocket → Browser
                                                ↓
                                           Telemetry JSON
```

**Update Rate:**
- Position: ~10 Hz
- Attitude: ~10 Hz
- Status: ~10 Hz
- Battery: ~1 Hz
- GPS: ~1 Hz

## 📝 API Reference

### REST API

**GET /api/telemetry**
```bash
curl http://localhost:5000/api/telemetry
```

Returns current telemetry as JSON.

**POST /api/command**
```bash
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "arm"}'
```

Commands: `arm`, `disarm`, `takeoff`, `land`, `offboard`, `rtl`

### WebSocket Events

**Client → Server:**
```javascript
socket.emit('command', {command: 'arm'});
socket.emit('command', {command: 'takeoff', altitude: 10.0});
```

**Server → Client:**
```javascript
socket.on('telemetry', (data) => {
    console.log('Position:', data.position);
    console.log('Attitude:', data.attitude);
    console.log('Status:', data.status);
});

socket.on('command_ack', (data) => {
    console.log('Command:', data.command, 'Success:', data.success);
});
```

## 🐛 Troubleshooting

### Web GCS won't start

**Error:** `ModuleNotFoundError: No module named 'flask'`

**Fix:**
```bash
pip3 install --user flask flask-socketio flask-cors python-socketio eventlet
```

### No telemetry data

**Check:**
1. Is PX4 running? `ps aux | grep px4`
2. Is MicroXRCE Agent running? `ps aux | grep MicroXRCE`
3. Are topics available? `ros2 topic list | grep fmu`
4. Is data flowing? `ros2 topic echo /fmu/out/sensor_combined --once`

**Fix:** Restart PX4 with `./start_fixed.sh`

### Position not updating in GUI

**Symptoms:** All other data works, but position stays at 0,0,0

**Cause:** Vehicle hasn't moved or isn't armed

**Fix:**
1. ARM the drone
2. TAKEOFF - position should update
3. Check `/fmu/out/vehicle_local_position_v1` directly:
   ```bash
   ros2 topic echo /fmu/out/vehicle_local_position_v1
   ```

### Camera feed not showing

See "Video Streaming" section above.

**Quick fix:**
```bash
# Install video server
sudo apt install ros-humble-web-video-server

# Start it
ros2 run web_video_server web_video_server

# Refresh browser
```

### Commands not working (can't arm)

**Check PX4 parameters:**

In PX4 console:
```bash
param show COM_RCL_EXCEPT  # Should be 4
param show COM_ARM_WO_GPS  # Should be 1
```

**Set if needed:**
```bash
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 1
param set COM_ARM_WO_GPS 1
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set CBRK_USB_CHK 197848
param save
```

### Browser shows "Cannot connect"

**Check Web GCS is running:**
```bash
ps aux | grep gcs_server
```

**Check port 5000 is listening:**
```bash
netstat -tuln | grep 5000
```

**Start Web GCS:**
```bash
source ~/ws_sensor_combined/install/setup.bash
web_gcs
```

## 🌟 Advanced Usage

### Custom Takeoff Altitude

Edit `gcs_server.py` line with `self.takeoff(altitude=5.0)` to change default, or:

Via API:
```bash
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "takeoff", "altitude": 10.0}'
```

### Multiple Drones

The Web GCS can be adapted for multi-drone:

1. Run multiple PX4 instances on different ports
2. Create multiple WebGCSNode instances
3. Modify UI to show multiple drones
4. Use namespace separation

### Remote Access

**To access from other devices on your network:**

1. Find your IP: `ip addr show | grep "inet "`
2. Edit `gcs_server.py`: Already set to `host='0.0.0.0'`
3. Access from other device: `http://YOUR_IP:5000`

**Security Note:** Only for trusted networks. Add authentication for production.

### Integration with Mission Planner

You can use the Web GCS alongside QGroundControl:

1. Web GCS for monitoring (port 5000)
2. QGroundControl for mission planning (port 14550)
3. Both can connect simultaneously

## 📦 Files Created

- `~/ws_sensor_combined/src/web_gcs/` - ROS 2 package
- `web_gcs/gcs_server.py` - Backend server (Python/Flask)
- `web_gcs/templates/index.html` - Frontend UI
- `start_web_gcs_complete.sh` - All-in-one startup
- `install_web_gcs_deps.sh` - Dependency installation

## 🎓 Next Steps

1. **Customize the UI** - Edit `templates/index.html`
2. **Add mission planning** - Waypoint editor
3. **Add map display** - Integrate Leaflet.js or Cesium
4. **Add data logging** - Record telemetry to files
5. **Add real-time graphs** - Chart.js for data visualization
6. **Multi-drone support** - Fleet management
7. **Mobile app** - React Native or Flutter wrapper

## 📚 Resources

- **Flask-SocketIO**: https://flask-socketio.readthedocs.io/
- **Socket.IO Client**: https://socket.io/docs/v4/client-api/
- **PX4 ROS 2 Guide**: https://docs.px4.io/main/en/ros2/
- **web_video_server**: http://wiki.ros.org/web_video_server

---

**Enjoy your professional Web GCS!** 🚁✨

For issues or questions, check the troubleshooting section or logs:
- PX4: `/tmp/px4_startup.log`
- Agent: `/tmp/microxrce_agent.log`
- Video: `/tmp/video_server.log`
