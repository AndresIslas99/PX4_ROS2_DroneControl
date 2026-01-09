# Camera Fix & Building a Proper GCS

## 🎥 Quick Fix: Enable Camera Feeds Now

### Step 1: Install Gazebo-ROS Bridge

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz-bridge ros-humble-ros-gz-image
```

### Step 2: Start Camera Bridge

Open a **new terminal** (keep PX4 running):

```bash
cd ~/ws_sensor_combined
source install/setup.bash

# Bridge Gazebo camera to ROS 2
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera
```

### Step 3: Verify Camera Topic

```bash
# In another terminal
source ~/ws_sensor_combined/install/setup.bash

# Check if /camera topic exists
ros2 topic list | grep camera

# Echo camera data
ros2 topic echo /camera --once
```

### Step 4: Run GUI

```bash
drone_control_gui
```

**The camera should now show live video!** 📹

---

## 🚀 Building a Proper GCS (Ground Control Station)

You're absolutely right! A professional GCS should use:
- **WebRTC** for low-latency video streaming
- **Web-based interface** for cross-platform access
- **MAVLink protocol** compatibility
- **Real-time telemetry** dashboards

### Architecture Options

#### Option 1: QGroundControl (Existing GCS)

**Pros:**
- Full-featured, production-ready
- MAVLink compatible
- Video streaming built-in
- Mission planning

**Setup:**
```bash
# Download QGroundControl
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage

# Run
./QGroundControl.AppImage
```

Then connect to PX4 via MAVLink (UDP 14550).

#### Option 2: Custom Web-Based GCS with ROS 2 + WebRTC

This is what modern drone companies build!

**Tech Stack:**
- **Backend**: ROS 2 + FastAPI/Flask
- **Frontend**: React/Vue.js
- **Video**: WebRTC (ros-webrtc or direct GStreamer)
- **Communication**: rosbridge_suite (WebSocket)
- **Visualization**: Cesium.js or Leaflet for maps

**Architecture:**
```
┌─────────────────┐
│   PX4 SITL      │
│   (Gazebo)      │
└────────┬────────┘
         │
    ┌────▼──────────────┐
    │   ROS 2 Bridge    │
    │  (Topics/Camera)  │
    └────┬──────────────┘
         │
    ┌────▼──────────────┐
    │  WebSocket/WebRTC │
    │  (rosbridge_suite) │
    └────┬──────────────┘
         │
    ┌────▼──────────────┐
    │   Web Browser     │
    │  (React GCS UI)   │
    └───────────────────┘
```

### Implementation Guide

#### Step 1: Install ROS Bridge Suite

```bash
sudo apt install ros-humble-rosbridge-suite
sudo apt install ros-humble-web-video-server
```

#### Step 2: Install WebRTC Video Server

```bash
# Install GStreamer for WebRTC
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-libav

# Install ros2-webrtc (if available)
# Or use web_video_server
```

#### Step 3: Create Backend Service

```python
# gcs_backend.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, SensorCombined
from sensor_msgs.msg import Image
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import asyncio
import json

app = FastAPI()

# Enable CORS for web frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class GCSBackend(Node):
    def __init__(self):
        super().__init__('gcs_backend')

        # Latest telemetry data
        self.telemetry = {
            'position': {},
            'velocity': {},
            'attitude': {},
            'status': {},
            'sensors': {}
        }

        # Subscribe to PX4 topics
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            10
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            10
        )

    def position_callback(self, msg):
        self.telemetry['position'] = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
            'vx': msg.vx,
            'vy': msg.vy,
            'vz': msg.vz
        }

    def status_callback(self, msg):
        self.telemetry['status'] = {
            'armed': msg.arming_state == 2,
            'mode': msg.nav_state
        }

# Global node
gcs_node = None

@app.on_event("startup")
async def startup():
    global gcs_node
    rclpy.init()
    gcs_node = GCSBackend()

@app.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Send telemetry at 10 Hz
            await websocket.send_json(gcs_node.telemetry)
            rclpy.spin_once(gcs_node, timeout_sec=0.1)
            await asyncio.sleep(0.1)
    except:
        pass

@app.get("/api/status")
async def get_status():
    return gcs_node.telemetry

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

#### Step 4: Create Frontend (React Example)

```bash
# Create React app
npx create-react-app drone-gcs
cd drone-gcs

# Install dependencies
npm install leaflet react-leaflet websocket
```

```javascript
// src/App.js
import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker } from 'react-leaflet';

function DroneGCS() {
  const [telemetry, setTelemetry] = useState({});
  const [ws, setWs] = useState(null);

  useEffect(() => {
    // Connect to WebSocket
    const websocket = new WebSocket('ws://localhost:8000/ws/telemetry');

    websocket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setTelemetry(data);
    };

    setWs(websocket);

    return () => websocket.close();
  }, []);

  return (
    <div className="gcs-container">
      <h1>Drone Ground Control Station</h1>

      {/* Telemetry Panel */}
      <div className="telemetry">
        <h2>Telemetry</h2>
        <p>Position: X={telemetry.position?.x?.toFixed(2)}
                     Y={telemetry.position?.y?.toFixed(2)}
                     Z={telemetry.position?.z?.toFixed(2)}</p>
        <p>Armed: {telemetry.status?.armed ? 'YES' : 'NO'}</p>
      </div>

      {/* Video Feed */}
      <div className="video">
        <h2>Camera Feed</h2>
        <img src="http://localhost:8080/stream?topic=/camera"
             alt="Drone camera" />
      </div>

      {/* Map */}
      <MapContainer center={[0, 0]} zoom={13}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        <Marker position={[telemetry.position?.y || 0, telemetry.position?.x || 0]} />
      </MapContainer>
    </div>
  );
}

export default DroneGCS;
```

#### Step 5: Start Video Streaming

```bash
# Terminal 1: Start web_video_server for camera streaming
ros2 run web_video_server web_video_server

# This exposes camera on: http://localhost:8080/stream?topic=/camera
```

#### Step 6: Run the GCS

```bash
# Terminal 1: ROS bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Video server
ros2 run web_video_server web_video_server

# Terminal 3: Backend
python3 gcs_backend.py

# Terminal 4: Frontend
cd drone-gcs && npm start

# Open browser: http://localhost:3000
```

---

## 🎯 Complete GCS Feature List

### Essential Features

1. **Real-time Telemetry**
   - Position (GPS/Local)
   - Velocity
   - Attitude (Roll/Pitch/Yaw)
   - Battery
   - Flight mode
   - Arming status

2. **Video Streaming**
   - Low-latency WebRTC (< 200ms)
   - Multi-camera support
   - Recording capability
   - Fullscreen mode

3. **Map Display**
   - OpenStreetMap/Satellite
   - Drone position marker
   - Mission waypoints
   - Geofence visualization
   - Home position

4. **Controls**
   - Arm/Disarm
   - Mode selection
   - Takeoff/Land
   - Emergency stop
   - Manual control (gamepad)

5. **Mission Planning**
   - Waypoint editor
   - Mission upload/download
   - Mission progress
   - Auto-generated missions

6. **Data Logging**
   - Flight logs (ULog format)
   - Telemetry recording
   - Video recording
   - Replay capability

### Advanced Features

7. **Multi-Drone Support**
   - Fleet management
   - Swarm visualization
   - Coordinated missions

8. **Analytics**
   - Real-time graphs
   - Performance metrics
   - Battery predictions
   - Flight statistics

9. **Alerts & Notifications**
   - Low battery warnings
   - GPS loss alerts
   - Geofence violations
   - Critical system alerts

10. **3D Visualization**
    - Cesium.js for 3D globe
    - Drone model orientation
    - Terrain following

---

## 📦 Quick Start Package

I'll create a minimal GCS for you:

### Minimal WebRTC GCS

```bash
cd ~/ws_sensor_combined/src

# Create minimal GCS package
ros2 pkg create --build-type ament_python mini_gcs \
  --dependencies rclpy px4_msgs sensor_msgs geometry_msgs

cd mini_gcs
mkdir mini_gcs/templates mini_gcs/static

# Create server
cat > mini_gcs/gcs_server.py << 'EOF'
#!/usr/bin/env python3
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
import threading

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

class GCSNode(Node):
    def __init__(self):
        super().__init__('gcs_node')
        self.telemetry = {}

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            lambda msg: self.update_telemetry('position', {
                'x': msg.x, 'y': msg.y, 'z': msg.z
            }),
            10
        )

    def update_telemetry(self, key, value):
        self.telemetry[key] = value
        socketio.emit('telemetry', self.telemetry)

gcs_node = None

def ros_thread():
    while rclpy.ok():
        rclpy.spin_once(gcs_node, timeout_sec=0.1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/telemetry')
def get_telemetry():
    return jsonify(gcs_node.telemetry)

if __name__ == '__main__':
    rclpy.init()
    gcs_node = GCSNode()
    threading.Thread(target=ros_thread, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
EOF
```

---

## 🎬 Recommended Approach

For a **production-quality GCS**, I recommend:

1. **Use QGroundControl** for immediate full-featured GCS
2. **Build custom web GCS** for specialized features:
   - Use **rosbridge_suite** for ROS communication
   - Use **web_video_server** or **WebRTC** for video
   - Use **React + Leaflet/Cesium** for frontend
   - Deploy as web app (accessible from any device)

3. **Integration:**
   ```
   PX4 → ROS 2 → WebSocket/WebRTC → Web Browser
   ```

This gives you:
- ✅ Low latency video (WebRTC < 200ms)
- ✅ Cross-platform (works on phone/tablet/PC)
- ✅ Modern UI (React components)
- ✅ Scalable (cloud deployment possible)
- ✅ Multi-user (multiple operators)

---

## 🔧 Next Steps

**For immediate camera fix:**
```bash
sudo apt install ros-humble-ros-gz-bridge
./bridge_cameras.sh
drone_control_gui
```

**For building proper GCS:**
1. Start with rosbridge and web_video_server
2. Build React frontend
3. Add WebRTC for low-latency video
4. Deploy as web application

Want me to create a complete minimal web GCS package for you?
