#!/usr/bin/env python3

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QTableWidget, QTableWidgetItem, QPushButton, QLabel)
from PyQt5.QtCore import Qt, QTimer
import math


class WaypointWidget(QWidget):
    """Widget for waypoint mission planning"""

    def __init__(self, drone_controller, parent=None):
        super().__init__(parent)
        self.drone_controller = drone_controller
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_active = False
        self.waypoint_reached_threshold = 0.5  # meters

        # Setup UI
        self.init_ui()

        # Mission execution timer
        self.mission_timer = QTimer()
        self.mission_timer.timeout.connect(self.execute_mission_step)

    def init_ui(self):
        """Initialize the user interface"""
        main_layout = QVBoxLayout()

        # Group box
        group_box = QGroupBox("Waypoint Mission Planner")
        layout = QVBoxLayout()

        # Waypoint table
        self.waypoint_table = QTableWidget()
        self.waypoint_table.setColumnCount(4)
        self.waypoint_table.setHorizontalHeaderLabels(["X (m)", "Y (m)", "Z (m)", "Yaw (°)"])
        self.waypoint_table.setMinimumHeight(200)
        layout.addWidget(self.waypoint_table)

        # Add waypoint controls
        add_layout = QHBoxLayout()
        add_layout.addWidget(QLabel("Add Waypoint:"))

        add_current_btn = QPushButton("Add Current Position")
        add_current_btn.clicked.connect(self.add_current_position)
        add_layout.addWidget(add_current_btn)

        add_manual_btn = QPushButton("Add Manual (0,0,-5,0)")
        add_manual_btn.clicked.connect(lambda: self.add_waypoint(0.0, 0.0, -5.0, 0.0))
        add_layout.addWidget(add_manual_btn)

        add_layout.addStretch()
        layout.addLayout(add_layout)

        # Mission control buttons
        control_layout = QHBoxLayout()

        remove_btn = QPushButton("Remove Selected")
        remove_btn.clicked.connect(self.remove_selected_waypoint)
        control_layout.addWidget(remove_btn)

        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self.clear_waypoints)
        control_layout.addWidget(clear_btn)

        self.execute_btn = QPushButton("Execute Mission")
        self.execute_btn.setStyleSheet("QPushButton { background-color: green; color: white; font-weight: bold; }")
        self.execute_btn.clicked.connect(self.start_mission)
        control_layout.addWidget(self.execute_btn)

        self.stop_btn = QPushButton("Stop Mission")
        self.stop_btn.setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; }")
        self.stop_btn.clicked.connect(self.stop_mission)
        self.stop_btn.setEnabled(False)
        control_layout.addWidget(self.stop_btn)

        layout.addLayout(control_layout)

        # Status label
        self.mission_status_label = QLabel("Mission Status: Idle")
        self.mission_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.mission_status_label)

        group_box.setLayout(layout)
        main_layout.addWidget(group_box)
        self.setLayout(main_layout)

    def add_waypoint(self, x, y, z, yaw):
        """Add a waypoint to the table"""
        self.waypoints.append((x, y, z, yaw))

        # Add to table
        row = self.waypoint_table.rowCount()
        self.waypoint_table.insertRow(row)
        self.waypoint_table.setItem(row, 0, QTableWidgetItem(f"{x:.2f}"))
        self.waypoint_table.setItem(row, 1, QTableWidgetItem(f"{y:.2f}"))
        self.waypoint_table.setItem(row, 2, QTableWidgetItem(f"{z:.2f}"))
        self.waypoint_table.setItem(row, 3, QTableWidgetItem(f"{yaw:.1f}"))

    def add_current_position(self):
        """Add current drone position as waypoint"""
        x, y, z = self.drone_controller.current_position
        yaw = math.degrees(self.drone_controller.current_yaw)
        self.add_waypoint(x, y, z, yaw)

    def remove_selected_waypoint(self):
        """Remove selected waypoint from table"""
        current_row = self.waypoint_table.currentRow()
        if current_row >= 0:
            self.waypoint_table.removeRow(current_row)
            del self.waypoints[current_row]

    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoint_table.setRowCount(0)
        self.waypoints = []
        self.mission_status_label.setText("Mission Status: Idle")

    def start_mission(self):
        """Start executing the mission"""
        if len(self.waypoints) == 0:
            self.mission_status_label.setText("Mission Status: No waypoints!")
            return

        # Enable offboard mode
        if not self.drone_controller.offboard_mode_enabled:
            self.drone_controller.enable_offboard_mode()

        self.mission_active = True
        self.current_waypoint_index = 0
        self.execute_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        # Start mission timer (10Hz update)
        self.mission_timer.start(100)

        self.mission_status_label.setText(f"Mission Status: Executing waypoint 1/{len(self.waypoints)}")

    def stop_mission(self):
        """Stop mission execution"""
        self.mission_active = False
        self.mission_timer.stop()
        self.execute_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        # Stop the drone
        self.drone_controller.set_velocity(0.0, 0.0, 0.0, 0.0)

        self.mission_status_label.setText("Mission Status: Stopped")

    def execute_mission_step(self):
        """Execute one step of the mission"""
        if not self.mission_active or self.current_waypoint_index >= len(self.waypoints):
            self.stop_mission()
            self.mission_status_label.setText("Mission Status: Completed")
            return

        # Get current waypoint
        target_x, target_y, target_z, target_yaw = self.waypoints[self.current_waypoint_index]

        # Get current position
        current_x, current_y, current_z = self.drone_controller.current_position

        # Calculate distance to waypoint
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Check if waypoint is reached
        if distance < self.waypoint_reached_threshold:
            # Move to next waypoint
            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                # Mission complete
                self.stop_mission()
                self.mission_status_label.setText("Mission Status: Completed!")
                return
            else:
                self.mission_status_label.setText(
                    f"Mission Status: Executing waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
                return

        # Calculate velocity towards waypoint (proportional control)
        max_vel = 2.0  # m/s
        kp = 1.0  # Proportional gain

        vx = kp * dx
        vy = kp * dy
        vz = kp * dz

        # Limit velocity
        vel_magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
        if vel_magnitude > max_vel:
            scale = max_vel / vel_magnitude
            vx *= scale
            vy *= scale
            vz *= scale

        # Calculate yaw rate
        target_yaw_rad = math.radians(target_yaw)
        current_yaw_rad = self.drone_controller.current_yaw
        dyaw = target_yaw_rad - current_yaw_rad

        # Normalize to -pi to pi
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi

        yaw_rate = dyaw * 0.5  # Proportional yaw control

        # Send velocity command
        self.drone_controller.set_velocity(vx, vy, vz, yaw_rate)
