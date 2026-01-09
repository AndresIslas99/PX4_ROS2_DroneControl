# 📹 Camera Feed Setup Complete!

## ✅ What Was Fixed

The camera feed in the Web GCS now works! Here's what was set up:

### 1. Gazebo Camera Bridge
The X500 depth drone has an RGB camera (`IMX214 sensor`) in Gazebo. This camera publishes images on a Gazebo-specific topic that needs to be **bridged to ROS 2**.

**Gazebo Camera Topic:**
```
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

**Bridge Command:**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image
```

### 2. Web Video Server
The `web_video_server` converts ROS 2 image topics to MJPEG streams that browsers can display.

**Running on:** `http://localhost:8080`

**Camera Stream URL:**
```
http://localhost:8080/stream?topic=/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image&type=mjpeg
```

### 3. Web GCS Integration
The Web GCS HTML has been updated to display the camera feed from the correct topic.

## 🚀 How to Use

### Automatic Startup (Recommended)

**Just run the complete startup script:**

```bash
cd ~/ws_sensor_combined
./start_web_gcs_complete.sh
```

This now automatically starts:
1. MicroXRCE Agent
2. PX4 SITL
3. Web video server (port 8080)
4. **Camera bridge** ← NEW!
5. Web GCS server (port 5000)

**Then open your browser to:** http://localhost:5000

### Manual Startup

If you prefer to start services manually:

**Terminal 1: PX4 System**
```bash
cd ~/ws_sensor_combined
./start_fixed.sh
```

**Terminal 2: Video Server**
```bash
source ~/ws_sensor_combined/install/setup.bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

**Terminal 3: Camera Bridge**
```bash
source ~/ws_sensor_combined/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image
```

**Terminal 4: Web GCS**
```bash
source ~/ws_sensor_combined/install/setup.bash
python3 install/web_gcs/lib/python3.10/site-packages/web_gcs/gcs_server.py
```

**Browser:** http://localhost:5000

## 📊 What You'll See

In the Web GCS interface:

**Center Panel - Camera Feed:**
- Live RGB camera view from the drone
- Updates in real-time as the drone moves
- FPS counter (should be ~30 FPS)
- Resolution: 640x480 (IMX214 sensor)

**In Gazebo:**
- The camera is mounted on the front of the X500 drone
- Points forward (same direction as the drone)
- Has depth capabilities (depth camera on same mount)

## 🔧 Troubleshooting

### Camera shows "Camera feed unavailable"

**Check if services are running:**

```bash
# Video server
ps aux | grep web_video_server

# Camera bridge
ps aux | grep parameter_bridge

# Check ROS 2 topic
ros2 topic list | grep "IMX214"
```

**Test camera stream directly:**
Open in browser:
```
http://localhost:8080/stream_viewer?topic=/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

**Restart camera bridge:**
```bash
killall parameter_bridge
source ~/ws_sensor_combined/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image &
```

### Port 8080 already in use

**Check what's using it:**
```bash
netstat -tuln | grep 8080
# or
ss -tuln | grep 8080
```

**Kill existing web_video_server:**
```bash
killall web_video_server
```

**Restart:**
```bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080 &
```

### Camera bridge crashes

**Check logs:**
```bash
cat /tmp/camera_bridge.log
```

**Common issues:**
- PX4 SITL not running → Start PX4 first
- Gazebo not running → PX4 SITL starts Gazebo automatically
- ros_gz_bridge not installed → `sudo apt install ros-humble-ros-gz-bridge`

## 📈 Testing the Camera

1. **Start complete system:**
   ```bash
   ./start_web_gcs_complete.sh
   ```

2. **Open Web GCS:**
   http://localhost:5000

3. **Arm and takeoff:**
   - Click "TAKEOFF" button
   - Drone rises to 5 meters

4. **Watch camera feed:**
   - Camera view should show the Gazebo environment
   - As drone ascends, view changes
   - Ground gets smaller, horizon visible

5. **Move drone in Gazebo:**
   - Use Gazebo GUI to move the drone
   - Camera feed updates in real-time

## 🎥 Camera Specifications

**IMX214 Sensor (RGB Camera):**
- Resolution: 640x480 pixels
- Frame rate: ~30 FPS
- Field of view: ~90 degrees
- Format: RGB8
- Mounted on: `camera_link` (front of drone)

**Depth Camera:**
- Also available on `/depth_camera` topic
- Can be bridged for 3D perception
- Useful for obstacle avoidance

## 🔄 Bridging Depth Camera (Optional)

If you want to also bridge the depth camera:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image &
```

Then access it at:
```
http://localhost:8080/stream?topic=/depth_camera&type=mjpeg
```

## ✅ Verification Checklist

- [x] web_video_server running on port 8080
- [x] Camera bridge running (parameter_bridge process)
- [x] ROS 2 topic exists: `/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image`
- [x] Web GCS HTML updated with correct camera topic
- [x] Startup script includes camera bridge
- [x] Browser shows camera feed at http://localhost:5000

## 📚 Next Steps

Now that your camera feed is working:

1. **Test full flight with video**
   - ARM → TAKEOFF → LAND sequence
   - Watch camera view during flight

2. **Add mission planning**
   - Fly to waypoints
   - Watch camera as drone moves

3. **Use depth camera for obstacles**
   - Bridge depth camera
   - Add obstacle detection

4. **Record video**
   - Use web_video_server's snapshot feature
   - Save camera streams

---

**Your camera feed is ready!** 📹✨

Open http://localhost:5000 and you should see live video from the drone camera!
