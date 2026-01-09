#!/usr/bin/env python3

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QLabel, QProgressBar, QPushButton, QGridLayout)
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QFont, QColor


class StatusWidget(QWidget):
    """Widget for displaying drone status information"""

    def __init__(self, drone_controller, parent=None):
        super().__init__(parent)
        self.drone_controller = drone_controller

        # Setup UI
        self.init_ui()

        # Connect signals
        self.connect_signals()

    def init_ui(self):
        """Initialize the user interface"""
        main_layout = QVBoxLayout()

        # Position group
        pos_group = QGroupBox("Position (NED)")
        pos_layout = QGridLayout()

        self.pos_x_label = self.create_value_label("0.00")
        self.pos_y_label = self.create_value_label("0.00")
        self.pos_z_label = self.create_value_label("0.00")

        pos_layout.addWidget(QLabel("X:"), 0, 0)
        pos_layout.addWidget(self.pos_x_label, 0, 1)
        pos_layout.addWidget(QLabel("m"), 0, 2)

        pos_layout.addWidget(QLabel("Y:"), 1, 0)
        pos_layout.addWidget(self.pos_y_label, 1, 1)
        pos_layout.addWidget(QLabel("m"), 1, 2)

        pos_layout.addWidget(QLabel("Z:"), 2, 0)
        pos_layout.addWidget(self.pos_z_label, 2, 1)
        pos_layout.addWidget(QLabel("m"), 2, 2)

        pos_group.setLayout(pos_layout)
        main_layout.addWidget(pos_group)

        # Velocity group
        vel_group = QGroupBox("Velocity")
        vel_layout = QGridLayout()

        self.vel_x_label = self.create_value_label("0.00")
        self.vel_y_label = self.create_value_label("0.00")
        self.vel_z_label = self.create_value_label("0.00")

        vel_layout.addWidget(QLabel("VX:"), 0, 0)
        vel_layout.addWidget(self.vel_x_label, 0, 1)
        vel_layout.addWidget(QLabel("m/s"), 0, 2)

        vel_layout.addWidget(QLabel("VY:"), 1, 0)
        vel_layout.addWidget(self.vel_y_label, 1, 1)
        vel_layout.addWidget(QLabel("m/s"), 1, 2)

        vel_layout.addWidget(QLabel("VZ:"), 2, 0)
        vel_layout.addWidget(self.vel_z_label, 2, 1)
        vel_layout.addWidget(QLabel("m/s"), 2, 2)

        vel_group.setLayout(vel_layout)
        main_layout.addWidget(vel_group)

        # Attitude group
        att_group = QGroupBox("Attitude")
        att_layout = QGridLayout()

        self.roll_label = self.create_value_label("0.0")
        self.pitch_label = self.create_value_label("0.0")
        self.yaw_label = self.create_value_label("0.0")

        att_layout.addWidget(QLabel("Roll:"), 0, 0)
        att_layout.addWidget(self.roll_label, 0, 1)
        att_layout.addWidget(QLabel("°"), 0, 2)

        att_layout.addWidget(QLabel("Pitch:"), 1, 0)
        att_layout.addWidget(self.pitch_label, 1, 1)
        att_layout.addWidget(QLabel("°"), 1, 2)

        att_layout.addWidget(QLabel("Yaw:"), 2, 0)
        att_layout.addWidget(self.yaw_label, 2, 1)
        att_layout.addWidget(QLabel("°"), 2, 2)

        att_group.setLayout(att_layout)
        main_layout.addWidget(att_group)

        # Battery group
        battery_group = QGroupBox("Battery")
        battery_layout = QVBoxLayout()

        self.battery_bar = QProgressBar()
        self.battery_bar.setMinimum(0)
        self.battery_bar.setMaximum(100)
        self.battery_bar.setValue(100)
        self.battery_bar.setTextVisible(True)
        self.battery_bar.setFormat("%p%")
        battery_layout.addWidget(self.battery_bar)

        battery_group.setLayout(battery_layout)
        main_layout.addWidget(battery_group)

        # Flight status group
        status_group = QGroupBox("Flight Status")
        status_layout = QVBoxLayout()

        # Mode label
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self.mode_label = QLabel("UNKNOWN")
        self.mode_label.setFont(QFont("Arial", 12, QFont.Bold))
        mode_layout.addWidget(self.mode_label)
        mode_layout.addStretch()
        status_layout.addLayout(mode_layout)

        # Armed label
        armed_layout = QHBoxLayout()
        armed_layout.addWidget(QLabel("Armed:"))
        self.armed_label = QLabel("NO")
        self.armed_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.armed_label.setStyleSheet("QLabel { color: red; }")
        armed_layout.addWidget(self.armed_label)
        armed_layout.addStretch()
        status_layout.addLayout(armed_layout)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # Quick action buttons
        action_group = QGroupBox("Quick Actions")
        action_layout = QVBoxLayout()

        # Arm/Disarm buttons
        arm_layout = QHBoxLayout()
        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setStyleSheet("QPushButton { background-color: green; color: white; font-weight: bold; }")
        self.arm_btn.clicked.connect(self.on_arm_clicked)
        arm_layout.addWidget(self.arm_btn)

        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; }")
        self.disarm_btn.clicked.connect(self.on_disarm_clicked)
        arm_layout.addWidget(self.disarm_btn)
        action_layout.addLayout(arm_layout)

        # Takeoff/Land buttons
        flight_layout = QHBoxLayout()
        self.takeoff_btn = QPushButton("TAKEOFF")
        self.takeoff_btn.clicked.connect(self.on_takeoff_clicked)
        flight_layout.addWidget(self.takeoff_btn)

        self.land_btn = QPushButton("LAND")
        self.land_btn.clicked.connect(self.on_land_clicked)
        flight_layout.addWidget(self.land_btn)
        action_layout.addLayout(flight_layout)

        # Offboard mode button
        self.offboard_btn = QPushButton("ENABLE OFFBOARD")
        self.offboard_btn.setCheckable(True)
        self.offboard_btn.clicked.connect(self.on_offboard_toggled)
        action_layout.addWidget(self.offboard_btn)

        # Emergency stop
        self.estop_btn = QPushButton("EMERGENCY STOP")
        self.estop_btn.setStyleSheet("QPushButton { background-color: darkred; color: white; font-weight: bold; }")
        self.estop_btn.clicked.connect(self.on_emergency_stop)
        action_layout.addWidget(self.estop_btn)

        action_group.setLayout(action_layout)
        main_layout.addWidget(action_group)

        main_layout.addStretch()
        self.setLayout(main_layout)

    def create_value_label(self, text):
        """Create a label for displaying values"""
        label = QLabel(text)
        label.setFont(QFont("Courier", 10))
        label.setAlignment(Qt.AlignRight)
        return label

    def connect_signals(self):
        """Connect drone controller signals to update methods"""
        self.drone_controller.position_updated.connect(self.update_position)
        self.drone_controller.velocity_updated.connect(self.update_velocity)
        self.drone_controller.attitude_updated.connect(self.update_attitude)
        self.drone_controller.battery_updated.connect(self.update_battery)
        self.drone_controller.status_updated.connect(self.update_status)

    @pyqtSlot(float, float, float)
    def update_position(self, x, y, z):
        """Update position display"""
        self.pos_x_label.setText(f"{x:.2f}")
        self.pos_y_label.setText(f"{y:.2f}")
        self.pos_z_label.setText(f"{z:.2f}")

    @pyqtSlot(float, float, float)
    def update_velocity(self, vx, vy, vz):
        """Update velocity display"""
        self.vel_x_label.setText(f"{vx:.2f}")
        self.vel_y_label.setText(f"{vy:.2f}")
        self.vel_z_label.setText(f"{vz:.2f}")

    @pyqtSlot(float, float, float)
    def update_attitude(self, roll, pitch, yaw):
        """Update attitude display"""
        self.roll_label.setText(f"{roll:.1f}")
        self.pitch_label.setText(f"{pitch:.1f}")
        self.yaw_label.setText(f"{yaw:.1f}")

    @pyqtSlot(float)
    def update_battery(self, percent):
        """Update battery display"""
        self.battery_bar.setValue(int(percent))

        # Change color based on level
        if percent > 50:
            self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: green; }")
        elif percent > 20:
            self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: orange; }")
        else:
            self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: red; }")

    @pyqtSlot(str, bool)
    def update_status(self, mode, armed):
        """Update flight status display"""
        self.mode_label.setText(mode)

        if armed:
            self.armed_label.setText("YES")
            self.armed_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
        else:
            self.armed_label.setText("NO")
            self.armed_label.setStyleSheet("QLabel { color: red; font-weight: bold; }")

    # Button callbacks
    def on_arm_clicked(self):
        """Handle ARM button click"""
        self.drone_controller.arm()

    def on_disarm_clicked(self):
        """Handle DISARM button click"""
        self.drone_controller.disarm()

    def on_takeoff_clicked(self):
        """Handle TAKEOFF button click"""
        self.drone_controller.takeoff(altitude=5.0)

    def on_land_clicked(self):
        """Handle LAND button click"""
        self.drone_controller.land()

    def on_offboard_toggled(self, checked):
        """Handle OFFBOARD button toggle"""
        if checked:
            self.drone_controller.enable_offboard_mode()
            self.offboard_btn.setText("DISABLE OFFBOARD")
        else:
            self.drone_controller.disable_offboard_mode()
            self.offboard_btn.setText("ENABLE OFFBOARD")

    def on_emergency_stop(self):
        """Handle EMERGENCY STOP button click"""
        self.drone_controller.emergency_stop()
