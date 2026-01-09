#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude,
    BatteryStatus,
    VehicleCommandAck
)
from PyQt5.QtCore import QObject, pyqtSignal
import time


class DroneController(Node, QObject):
    """ROS 2 node for controlling PX4 drone with Qt signals for GUI updates"""

    # Qt signals for updating GUI
    position_updated = pyqtSignal(float, float, float)  # x, y, z
    velocity_updated = pyqtSignal(float, float, float)  # vx, vy, vz
    attitude_updated = pyqtSignal(float, float, float)  # roll, pitch, yaw (degrees)
    battery_updated = pyqtSignal(float)  # battery percentage
    status_updated = pyqtSignal(str, bool)  # flight mode, armed status
    command_ack_received = pyqtSignal(int, int)  # command, result

    def __init__(self):
        Node.__init__(self, 'drone_controller')
        QObject.__init__(self)

        # QoS profile for PX4 - use sensor data QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # QoS profile for commands
        cmd_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers - use reliable QoS for commands
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', cmd_qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', cmd_qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', cmd_qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback, qos_profile)
        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self.battery_status_callback, qos_profile)
        self.vehicle_command_ack_subscriber = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback, qos_profile)

        # State variables
        self.offboard_mode_enabled = False
        self.armed = False
        self.nav_state = 0
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.data_received = False

        # Offboard control heartbeat timer (100ms = 10Hz)
        self.offboard_heartbeat_timer = self.create_timer(0.1, self.publish_offboard_control_heartbeat)

        # Velocity setpoint (updated by joystick)
        self.velocity_setpoint = [0.0, 0.0, 0.0, 0.0]  # vx, vy, vz, yaw_rate

        self.get_logger().info('Drone Controller initialized')
        self.get_logger().info('Waiting for data from PX4...')

    def get_timestamp(self):
        """Get current timestamp in microseconds"""
        return int(self.get_clock().now().nanoseconds / 1000)

    # ============ Callbacks ============

    def vehicle_local_position_callback(self, msg):
        """Update position and velocity from PX4"""
        if not self.data_received:
            self.get_logger().info('First position data received from PX4!')
            self.data_received = True

        self.current_position = [msg.x, msg.y, msg.z]
        self.current_velocity = [msg.vx, msg.vy, msg.vz]
        self.position_updated.emit(msg.x, msg.y, msg.z)
        self.velocity_updated.emit(msg.vx, msg.vy, msg.vz)

    def vehicle_status_callback(self, msg):
        """Update vehicle status"""
        self.armed = (msg.arming_state == 2)  # ARMING_STATE_ARMED = 2
        self.nav_state = msg.nav_state

        # Convert nav_state to readable string
        nav_state_map = {
            0: "MANUAL",
            1: "ALTCTL",
            2: "POSCTL",
            14: "OFFBOARD",
            17: "AUTO_TAKEOFF",
            18: "AUTO_LAND"
        }
        mode_str = nav_state_map.get(msg.nav_state, f"UNKNOWN({msg.nav_state})")
        self.status_updated.emit(mode_str, self.armed)

    def vehicle_attitude_callback(self, msg):
        """Update attitude (quaternion to Euler angles)"""
        # Convert quaternion to Euler angles
        import math
        q = msg.q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_yaw = yaw

        # Convert to degrees for display
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.attitude_updated.emit(roll_deg, pitch_deg, yaw_deg)

    def battery_status_callback(self, msg):
        """Update battery status"""
        battery_percent = msg.remaining * 100.0
        self.battery_updated.emit(battery_percent)

    def vehicle_command_ack_callback(self, msg):
        """Handle command acknowledgments"""
        self.command_ack_received.emit(msg.command, msg.result)

    # ============ Control Methods ============

    def publish_offboard_control_heartbeat(self):
        """Publish offboard control mode at 10Hz"""
        if not self.offboard_mode_enabled:
            return

        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True  # Velocity control mode
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_timestamp()
        self.offboard_control_mode_publisher.publish(msg)

        # Also publish velocity setpoint
        self.publish_velocity_setpoint()

    def publish_velocity_setpoint(self):
        """Publish velocity setpoint from joystick input"""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]  # Don't control position
        msg.velocity = [self.velocity_setpoint[0], self.velocity_setpoint[1], self.velocity_setpoint[2]]
        msg.yaw = float('nan')  # Will use yawspeed instead
        msg.yawspeed = self.velocity_setpoint[3]
        msg.timestamp = self.get_timestamp()
        self.trajectory_setpoint_publisher.publish(msg)

    def set_velocity(self, vx, vy, vz, yaw_rate):
        """Set velocity setpoint (called by joystick widget)"""
        self.velocity_setpoint = [vx, vy, vz, yaw_rate]

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_timestamp()
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """Arm the drone"""
        self.get_logger().info('Arming...')
        # Force arm (param2=21196 is the magic number to force arm in simulation)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)

    def disarm(self):
        """Disarm the drone"""
        self.get_logger().info('Disarming...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def enable_offboard_mode(self):
        """Enable offboard control mode"""
        self.get_logger().info('Enabling offboard mode...')
        self.offboard_mode_enabled = True
        # Send offboard mode command
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def disable_offboard_mode(self):
        """Disable offboard control mode"""
        self.get_logger().info('Disabling offboard mode...')
        self.offboard_mode_enabled = False
        self.velocity_setpoint = [0.0, 0.0, 0.0, 0.0]

    def takeoff(self, altitude=5.0):
        """Takeoff to specified altitude"""
        self.get_logger().info(f'Taking off to {altitude}m...')

        # First enable offboard mode
        if not self.offboard_mode_enabled:
            self.enable_offboard_mode()
            time.sleep(0.5)

        # Arm if not armed
        if not self.armed:
            self.arm()
            time.sleep(0.5)

        # Set upward velocity for takeoff
        self.set_velocity(0.0, 0.0, -1.0, 0.0)  # -1.0 m/s upward (NED frame, -Z is up)

        # Create a timer to stop at target altitude (simplified)
        # In production, you'd monitor altitude and stop when reached
        def stop_takeoff():
            if abs(self.current_position[2] + altitude) < 0.5:  # Within 0.5m of target
                self.set_velocity(0.0, 0.0, 0.0, 0.0)  # Stop

        # Note: Proper implementation would use a state machine for takeoff

    def land(self):
        """Land the drone"""
        self.get_logger().info('Landing...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def emergency_stop(self):
        """Emergency stop - set all velocities to zero"""
        self.get_logger().warn('EMERGENCY STOP!')
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
