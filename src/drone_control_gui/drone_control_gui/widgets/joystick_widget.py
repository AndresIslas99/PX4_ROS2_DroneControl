#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QSlider, QPushButton
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor
import math


class JoystickWidget(QWidget):
    """Virtual joystick widget for velocity control"""

    # Signal emitted when joystick position changes (vx, vy)
    velocity_changed = pyqtSignal(float, float, float, float)  # vx, vy, vz, yaw_rate

    def __init__(self, parent=None):
        super().__init__(parent)

        self.joystick_position = QPoint(0, 0)  # Relative to center
        self.is_pressed = False
        self.max_velocity = 3.0  # m/s
        self.current_vz = 0.0  # Altitude velocity
        self.current_yaw_rate = 0.0  # Yaw rate

        # Setup UI
        self.init_ui()

    def init_ui(self):
        """Initialize the user interface"""
        main_layout = QVBoxLayout()

        # Group box
        group_box = QGroupBox("Velocity Control")
        layout = QVBoxLayout()

        # Joystick canvas
        self.joystick_canvas = JoystickCanvas(self)
        self.joystick_canvas.setMinimumSize(200, 200)
        self.joystick_canvas.position_changed.connect(self.on_joystick_moved)
        layout.addWidget(self.joystick_canvas)

        # Altitude control
        altitude_layout = QHBoxLayout()
        altitude_layout.addWidget(QLabel("Altitude:"))
        self.altitude_slider = QSlider(Qt.Horizontal)
        self.altitude_slider.setMinimum(-100)
        self.altitude_slider.setMaximum(100)
        self.altitude_slider.setValue(0)
        self.altitude_slider.setTickInterval(25)
        self.altitude_slider.setTickPosition(QSlider.TicksBelow)
        self.altitude_slider.valueChanged.connect(self.on_altitude_changed)
        altitude_layout.addWidget(self.altitude_slider)
        self.altitude_label = QLabel("0.0 m/s")
        altitude_layout.addWidget(self.altitude_label)
        layout.addLayout(altitude_layout)

        # Yaw control
        yaw_layout = QHBoxLayout()
        yaw_layout.addWidget(QLabel("Yaw Rate:"))
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setMinimum(-100)
        self.yaw_slider.setMaximum(100)
        self.yaw_slider.setValue(0)
        self.yaw_slider.setTickInterval(25)
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.valueChanged.connect(self.on_yaw_changed)
        yaw_layout.addWidget(self.yaw_slider)
        self.yaw_label = QLabel("0.0 °/s")
        yaw_layout.addWidget(self.yaw_label)
        layout.addLayout(yaw_layout)

        # Center button
        center_btn = QPushButton("Center / Stop")
        center_btn.clicked.connect(self.center_all)
        layout.addWidget(center_btn)

        group_box.setLayout(layout)
        main_layout.addWidget(group_box)
        self.setLayout(main_layout)

    def on_joystick_moved(self, x, y):
        """Handle joystick movement"""
        # Convert to velocity (-max_velocity to +max_velocity)
        vx = (x / 100.0) * self.max_velocity  # Forward/backward
        vy = (y / 100.0) * self.max_velocity  # Left/right

        # Emit velocity signal
        self.velocity_changed.emit(vx, vy, self.current_vz, self.current_yaw_rate)

    def on_altitude_changed(self, value):
        """Handle altitude slider change"""
        # Convert slider value (-100 to 100) to velocity
        self.current_vz = -(value / 100.0) * 2.0  # -2.0 to +2.0 m/s (negative is up in NED)
        self.altitude_label.setText(f"{-self.current_vz:.1f} m/s")  # Show as positive for up
        self.velocity_changed.emit(*self.joystick_canvas.get_velocity(), self.current_vz, self.current_yaw_rate)

    def on_yaw_changed(self, value):
        """Handle yaw slider change"""
        # Convert slider value to deg/s
        self.current_yaw_rate = (value / 100.0) * 90.0  # -90 to +90 deg/s
        yaw_rad = math.radians(self.current_yaw_rate)
        self.yaw_label.setText(f"{self.current_yaw_rate:.1f} °/s")
        self.velocity_changed.emit(*self.joystick_canvas.get_velocity(), self.current_vz, yaw_rad)

    def center_all(self):
        """Center all controls and stop"""
        self.joystick_canvas.center()
        self.altitude_slider.setValue(0)
        self.yaw_slider.setValue(0)
        self.velocity_changed.emit(0.0, 0.0, 0.0, 0.0)


class JoystickCanvas(QWidget):
    """Canvas for drawing the joystick"""

    position_changed = pyqtSignal(float, float)  # x%, y%

    def __init__(self, parent=None):
        super().__init__(parent)
        self.position = QPoint(0, 0)  # Relative to center (-100 to +100)
        self.is_pressed = False
        self.max_velocity = 3.0
        self.setMouseTracking(True)

    def get_velocity(self):
        """Get current velocity from joystick position"""
        vx = (self.position.x() / 100.0) * self.max_velocity
        vy = (self.position.y() / 100.0) * self.max_velocity
        return vx, vy

    def center(self):
        """Center the joystick"""
        self.position = QPoint(0, 0)
        self.update()
        self.position_changed.emit(0.0, 0.0)

    def paintEvent(self, event):
        """Draw the joystick"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Get center
        center_x = self.width() // 2
        center_y = self.height() // 2
        radius = min(center_x, center_y) - 20

        # Draw outer circle (boundary)
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)

        # Draw crosshair
        painter.setPen(QPen(QColor(150, 150, 150), 1, Qt.DashLine))
        painter.drawLine(center_x - radius, center_y, center_x + radius, center_y)
        painter.drawLine(center_x, center_y - radius, center_x, center_y + radius)

        # Draw joystick knob
        knob_x = center_x + (self.position.x() / 100.0) * radius
        knob_y = center_y + (self.position.y() / 100.0) * radius
        knob_radius = 15

        painter.setPen(QPen(QColor(0, 120, 215), 3))
        painter.setBrush(QBrush(QColor(0, 150, 255) if self.is_pressed else QColor(0, 120, 215)))
        painter.drawEllipse(int(knob_x - knob_radius), int(knob_y - knob_radius),
                            knob_radius * 2, knob_radius * 2)

    def mousePressEvent(self, event):
        """Handle mouse press"""
        if event.button() == Qt.LeftButton:
            self.is_pressed = True
            self.update_position(event.pos())

    def mouseMoveEvent(self, event):
        """Handle mouse move"""
        if self.is_pressed:
            self.update_position(event.pos())

    def mouseReleaseEvent(self, event):
        """Handle mouse release"""
        if event.button() == Qt.LeftButton:
            self.is_pressed = False
            self.center()

    def update_position(self, pos):
        """Update joystick position from mouse position"""
        center_x = self.width() // 2
        center_y = self.height() // 2
        radius = min(center_x, center_y) - 20

        # Calculate relative position
        dx = pos.x() - center_x
        dy = pos.y() - center_y

        # Limit to circle
        distance = math.sqrt(dx**2 + dy**2)
        if distance > radius:
            dx = (dx / distance) * radius
            dy = (dy / distance) * radius

        # Convert to percentage (-100 to +100)
        x_percent = (dx / radius) * 100
        y_percent = (dy / radius) * 100

        self.position = QPoint(int(x_percent), int(y_percent))
        self.update()

        # Emit signal
        self.position_changed.emit(x_percent, y_percent)
