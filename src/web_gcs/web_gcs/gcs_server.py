#!/usr/bin/env python3
"""
Professional Web-based Ground Control Station for PX4
Real-time telemetry, video streaming, and drone control via web interface
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import threading
import time
import json
import os

# PX4 Messages
from px4_msgs.msg import (
    VehicleStatus,
    VehicleLocalPosition,
    VehicleAttitude,
    SensorGps,
    BatteryStatus,
    SensorCombined,
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint
)

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np


class WebGCSNode(Node):
    """ROS 2 Node for Web GCS - handles all PX4 communication"""

    def __init__(self, socketio):
        super().__init__('web_gcs_node')
        self.socketio = socketio

        # Telemetry data storage
        self.telemetry = {
            'position': {'x': 0, 'y': 0, 'z': 0, 'vx': 0, 'vy': 0, 'vz': 0},
            'attitude': {'roll': 0, 'pitch': 0, 'yaw': 0},
            'gps': {'lat': 0, 'lon': 0, 'alt': 0, 'satellites': 0},
            'battery': {'voltage': 0, 'percentage': 0, 'current': 0},
            'status': {
                'armed': False,
                'flight_mode': 'Unknown',
                'nav_state': 0,
                'failsafe': False,
                'connected': False
            },
            'sensors': {'gyro': [0, 0, 0], 'accel': [0, 0, 0]},
            'timestamp': 0
        }

        # QoS Profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_sensor
        )

        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_sensor
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_sensor
        )

        self.create_subscription(
            SensorGps,
            '/fmu/out/sensor_gps',
            self.gps_callback,
            qos_sensor
        )

        self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_callback,
            qos_sensor
        )

        self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_callback,
            qos_sensor
        )

        # Publishers for commands
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_reliable
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_reliable
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_reliable
        )

        # Offboard control state
        self.offboard_mode_active = False
        self.target_altitude = -5.0  # NED frame: negative is up
        self.target_position_x = 0.0  # North
        self.target_position_y = 0.0  # East
        self.target_yaw = 0.0  # Heading in radians
        self.current_setpoint = TrajectorySetpoint()

        # Heartbeat timer for WebSocket updates
        self.create_timer(0.1, self.publish_telemetry)  # 10 Hz

        # Offboard control timer - must publish at >2Hz for PX4
        self.create_timer(0.05, self.publish_offboard_control)  # 20 Hz

        self.get_logger().info('Web GCS Node started')
        self.last_update = time.time()

    def position_callback(self, msg):
        """Update position telemetry"""
        self.telemetry['position'] = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'vx': float(msg.vx),
            'vy': float(msg.vy),
            'vz': float(msg.vz)
        }
        self.telemetry['status']['connected'] = True
        self.last_update = time.time()

    def attitude_callback(self, msg):
        """Update attitude telemetry"""
        # Convert quaternion to Euler angles
        q = msg.q
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.telemetry['attitude'] = {
            'roll': float(np.degrees(roll)),
            'pitch': float(np.degrees(pitch)),
            'yaw': float(np.degrees(yaw))
        }

    def status_callback(self, msg):
        """Update vehicle status"""
        # Flight mode mapping
        mode_map = {
            0: 'Manual',
            1: 'Altitude',
            2: 'Position',
            3: 'Auto Mission',
            4: 'Auto Loiter',
            5: 'Auto RTL',
            6: 'Auto RC Recover',
            14: 'Offboard',
            17: 'Auto Takeoff',
            18: 'Auto Land'
        }

        self.telemetry['status'] = {
            'armed': msg.arming_state == 2,
            'flight_mode': mode_map.get(msg.nav_state, f'Unknown ({msg.nav_state})'),
            'nav_state': int(msg.nav_state),
            'failsafe': bool(msg.failsafe),
            'connected': True
        }

    def gps_callback(self, msg):
        """Update GPS telemetry"""
        self.telemetry['gps'] = {
            'lat': float(msg.latitude_deg),
            'lon': float(msg.longitude_deg),
            'alt': float(msg.altitude_msl_m),
            'satellites': int(msg.satellites_used)
        }

    def battery_callback(self, msg):
        """Update battery telemetry"""
        self.telemetry['battery'] = {
            'voltage': float(msg.voltage_v),
            'percentage': float(msg.remaining * 100),
            'current': float(msg.current_a)
        }

    def sensor_callback(self, msg):
        """Update sensor telemetry"""
        self.telemetry['sensors'] = {
            'gyro': [float(msg.gyro_rad[0]), float(msg.gyro_rad[1]), float(msg.gyro_rad[2])],
            'accel': [float(msg.accelerometer_m_s2[0]), float(msg.accelerometer_m_s2[1]), float(msg.accelerometer_m_s2[2])]
        }
        self.telemetry['timestamp'] = int(msg.timestamp)

    def publish_telemetry(self):
        """Publish telemetry to WebSocket clients"""
        # Check connection timeout
        if time.time() - self.last_update > 2.0:
            self.telemetry['status']['connected'] = False

        # Emit to all connected WebSocket clients
        self.socketio.emit('telemetry', self.telemetry, namespace='/gcs')

    def publish_offboard_control(self):
        """Continuously publish offboard control mode and setpoints"""
        if not self.offboard_mode_active:
            return

        # Publish offboard control mode - tells PX4 what to control
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_control_mode_pub.publish(offboard_msg)

        # Publish trajectory setpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position[0] = self.target_position_x  # North
        setpoint.position[1] = self.target_position_y  # East
        setpoint.position[2] = self.target_altitude  # Down (negative = up)
        setpoint.yaw = self.target_yaw  # Heading in radians
        self.trajectory_setpoint_pub.publish(setpoint)
        self.current_setpoint = setpoint

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0,
                            param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Send command to vehicle"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

    def arm(self):
        """Arm the vehicle"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        """Disarm the vehicle"""
        # Stop offboard mode when disarming
        self.offboard_mode_active = False
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def takeoff(self, altitude=5.0):
        """Takeoff to specified altitude using offboard control"""
        # Set target altitude (negative in NED frame)
        self.target_altitude = -abs(altitude)

        # Enable offboard mode with setpoint publishing
        if not self.offboard_mode_active:
            self.offboard_mode_active = True
            self.get_logger().info(f'Offboard control activated, target altitude: {altitude}m')
            # Give PX4 time to receive setpoints (need >2 before mode switch)
            time.sleep(0.5)

        # Switch to offboard mode
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        time.sleep(0.3)

        # Arm the vehicle
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Takeoff sequence initiated')

    def land(self):
        """Land the vehicle"""
        # Gradually descend using offboard setpoints
        if self.offboard_mode_active:
            # Set target to ground level
            self.target_altitude = 0.0
            self.get_logger().info('Landing via offboard setpoints')
        else:
            # Use standard land command
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def set_offboard_mode(self):
        """Set vehicle to offboard mode with continuous setpoint publishing"""
        # Enable continuous setpoint publishing
        self.offboard_mode_active = True
        self.get_logger().info('Starting offboard control mode - publishing setpoints')

        # Give PX4 time to receive setpoints
        time.sleep(0.5)

        # Switch to offboard mode
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def rtl(self):
        """Return to launch"""
        # Disable offboard mode
        self.offboard_mode_active = False
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def goto_position(self, x, y, z, yaw=0.0):
        """Move to specified position in NED frame"""
        if not self.offboard_mode_active:
            self.get_logger().warn('Cannot goto - offboard mode not active. Enable offboard or takeoff first.')
            return False

        # Update target position
        self.target_position_x = float(x)
        self.target_position_y = float(y)
        self.target_altitude = float(z)
        self.target_yaw = float(yaw)

        self.get_logger().info(f'Going to position: N={x:.2f}, E={y:.2f}, D={z:.2f}, Yaw={yaw:.2f}')
        return True

    def execute_mission(self, waypoints):
        """Execute waypoint mission"""
        if not self.offboard_mode_active:
            self.get_logger().warn('Cannot execute mission - offboard mode not active')
            self.socketio.emit('mission_status', {
                'message': 'Error: Offboard mode not active. Takeoff first.',
                'completed': True
            }, namespace='/gcs')
            return False

        if not waypoints or len(waypoints) == 0:
            self.get_logger().warn('No waypoints provided')
            return False

        self.get_logger().info(f'Starting mission with {len(waypoints)} waypoints')

        # Execute waypoints sequentially
        def mission_thread():
            try:
                for i, wp in enumerate(waypoints):
                    if not self.offboard_mode_active:
                        self.socketio.emit('mission_status', {
                            'message': 'Mission aborted - offboard mode disabled',
                            'completed': True
                        }, namespace='/gcs')
                        return

                    # Update status
                    self.socketio.emit('mission_status', {
                        'message': f'Flying to waypoint {i+1}/{len(waypoints)}...',
                        'completed': False
                    }, namespace='/gcs')

                    # Set target position
                    self.target_position_x = float(wp['x'])
                    self.target_position_y = float(wp['y'])
                    self.target_altitude = float(wp['z'])

                    # Wait until close to waypoint (within 1m)
                    while self.offboard_mode_active:
                        if self.telemetry['position']['x'] == 0 and self.telemetry['position']['y'] == 0:
                            # Position not available yet
                            time.sleep(0.5)
                            continue

                        dx = self.telemetry['position']['x'] - self.target_position_x
                        dy = self.telemetry['position']['y'] - self.target_position_y
                        dz = self.telemetry['position']['z'] - self.target_altitude
                        distance = (dx**2 + dy**2 + dz**2)**0.5

                        if distance < 1.0:  # Within 1 meter
                            self.get_logger().info(f'Reached waypoint {i+1}')
                            break

                        time.sleep(0.5)

                    # Hold position for 2 seconds
                    time.sleep(2.0)

                # Mission complete
                self.socketio.emit('mission_status', {
                    'message': f'Mission complete! Flew to {len(waypoints)} waypoints.',
                    'completed': True
                }, namespace='/gcs')
                self.get_logger().info('Mission completed successfully')

            except Exception as e:
                self.get_logger().error(f'Mission error: {str(e)}')
                self.socketio.emit('mission_status', {
                    'message': f'Mission error: {str(e)}',
                    'completed': True
                }, namespace='/gcs')

        # Start mission in background thread
        import threading
        thread = threading.Thread(target=mission_thread, daemon=True)
        thread.start()
        return True


# Flask Application
app = Flask(__name__,
            template_folder='templates',
            static_folder='static')
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global GCS node
gcs_node = None


@app.route('/')
def index():
    """Serve main GCS interface"""
    return render_template('index.html')


@app.route('/api/telemetry')
def get_telemetry():
    """REST API endpoint for telemetry"""
    if gcs_node:
        return jsonify(gcs_node.telemetry)
    return jsonify({'error': 'Node not initialized'}), 500


@app.route('/api/command', methods=['POST'])
def send_command():
    """REST API endpoint for sending commands"""
    if not gcs_node:
        return jsonify({'error': 'Node not initialized'}), 500

    data = request.json
    command = data.get('command')

    try:
        if command == 'arm':
            gcs_node.arm()
        elif command == 'disarm':
            gcs_node.disarm()
        elif command == 'takeoff':
            altitude = data.get('altitude', 5.0)
            gcs_node.takeoff(altitude)
        elif command == 'land':
            gcs_node.land()
        elif command == 'offboard':
            gcs_node.set_offboard_mode()
        elif command == 'rtl':
            gcs_node.rtl()
        else:
            return jsonify({'error': f'Unknown command: {command}'}), 400

        return jsonify({'success': True, 'command': command})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@socketio.on('connect', namespace='/gcs')
def handle_connect():
    """Handle WebSocket connection"""
    print('Client connected')
    emit('status', {'connected': True})


@socketio.on('disconnect', namespace='/gcs')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    print('Client disconnected')


@socketio.on('command', namespace='/gcs')
def handle_command(data):
    """Handle commands via WebSocket"""
    if not gcs_node:
        emit('error', {'message': 'Node not initialized'})
        return

    command = data.get('command')
    print(f'Received command via WebSocket: {command}')

    try:
        if command == 'arm':
            gcs_node.arm()
        elif command == 'disarm':
            gcs_node.disarm()
        elif command == 'takeoff':
            gcs_node.takeoff(data.get('altitude', 5.0))
        elif command == 'land':
            gcs_node.land()
        elif command == 'offboard':
            gcs_node.set_offboard_mode()
        elif command == 'rtl':
            gcs_node.rtl()
        elif command == 'goto':
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            z = data.get('z', -5.0)
            yaw = data.get('yaw', 0.0)
            success = gcs_node.goto_position(x, y, z, yaw)
            emit('command_ack', {'command': command, 'success': success})
            return
        elif command == 'mission':
            waypoints = data.get('waypoints', [])
            success = gcs_node.execute_mission(waypoints)
            emit('command_ack', {'command': command, 'success': success})
            return

        emit('command_ack', {'command': command, 'success': True})
    except Exception as e:
        emit('command_ack', {'command': command, 'success': False, 'error': str(e)})


def ros_spin_thread():
    """Thread for ROS 2 spinning"""
    while rclpy.ok():
        rclpy.spin_once(gcs_node, timeout_sec=0.01)


def main(args=None):
    """Main entry point"""
    global gcs_node

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create GCS node
    gcs_node = WebGCSNode(socketio)

    # Start ROS spinning in separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    # Get template directory
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    static_dir = os.path.join(os.path.dirname(__file__), 'static')

    print('='*60)
    print('Professional Web GCS Starting...')
    print('='*60)
    print(f'Template directory: {template_dir}')
    print(f'Static directory: {static_dir}')
    print('')
    print('Open your browser to: http://localhost:5000')
    print('='*60)

    # Start Flask-SocketIO server
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        gcs_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
