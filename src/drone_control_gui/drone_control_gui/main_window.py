#!/usr/bin/env python3

import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from drone_control_gui.drone_controller import DroneController
from drone_control_gui.widgets.camera_widget import CameraWidget
from drone_control_gui.widgets.depth_widget import DepthWidget
from drone_control_gui.widgets.joystick_widget import JoystickWidget
from drone_control_gui.widgets.status_widget import StatusWidget
from drone_control_gui.widgets.waypoint_widget import WaypointWidget


class ROS2Thread(QThread):
    """Thread for running ROS 2 spin"""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self._running = True

    def run(self):
        """Spin the ROS 2 node"""
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def stop(self):
        """Stop the thread"""
        self._running = False


class MainWindow(QMainWindow):
    """Main window for drone control GUI"""

    def __init__(self):
        super().__init__()

        # Initialize ROS 2
        rclpy.init()

        # Create drone controller node
        self.drone_controller = DroneController()

        # Setup UI
        self.init_ui()

        # Start ROS 2 spin thread
        self.ros_thread = ROS2Thread(self.drone_controller)
        self.ros_thread.start()

        self.drone_controller.get_logger().info('Drone Control GUI started')

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("PX4 Drone Control Station")
        self.setGeometry(100, 100, 1600, 900)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Top section: Camera feeds and status
        top_splitter = QSplitter(Qt.Horizontal)

        # Camera widget
        self.camera_widget = CameraWidget(self.drone_controller)
        top_splitter.addWidget(self.camera_widget)

        # Depth widget
        self.depth_widget = DepthWidget(self.drone_controller)
        top_splitter.addWidget(self.depth_widget)

        # Status widget
        self.status_widget = StatusWidget(self.drone_controller)
        top_splitter.addWidget(self.status_widget)

        # Set splitter proportions
        top_splitter.setSizes([400, 400, 300])

        main_layout.addWidget(top_splitter)

        # Middle section: Control panel
        control_layout = QHBoxLayout()

        # Joystick widget
        self.joystick_widget = JoystickWidget()
        self.joystick_widget.velocity_changed.connect(self.on_velocity_changed)
        control_layout.addWidget(self.joystick_widget)

        # Waypoint widget
        self.waypoint_widget = WaypointWidget(self.drone_controller)
        control_layout.addWidget(self.waypoint_widget)

        main_layout.addLayout(control_layout)

        # Status bar
        self.statusBar().showMessage("Ready")

        # Apply dark theme styling
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QGroupBox {
                border: 2px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #3d3d3d;
                border: 1px solid #555555;
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #4d4d4d;
            }
            QPushButton:pressed {
                background-color: #2d2d2d;
            }
            QTableWidget {
                background-color: #3d3d3d;
                alternate-background-color: #353535;
                selection-background-color: #0078d7;
            }
            QHeaderView::section {
                background-color: #2b2b2b;
                padding: 4px;
                border: 1px solid #555555;
            }
            QProgressBar {
                border: 1px solid #555555;
                border-radius: 3px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #0078d7;
            }
            QSlider::groove:horizontal {
                border: 1px solid #555555;
                height: 8px;
                background: #3d3d3d;
                margin: 2px 0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #0078d7;
                border: 1px solid #0078d7;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
        """)

    def on_velocity_changed(self, vx, vy, vz, yaw_rate):
        """Handle velocity changes from joystick"""
        self.drone_controller.set_velocity(vx, vy, vz, yaw_rate)

    def closeEvent(self, event):
        """Handle window close event"""
        self.drone_controller.get_logger().info('Shutting down...')

        # Stop ROS thread
        self.ros_thread.stop()
        self.ros_thread.wait()

        # Shutdown ROS
        self.drone_controller.destroy_node()
        rclpy.shutdown()

        event.accept()


def main(args=None):
    """Main entry point for the application"""
    app = QApplication(sys.argv)

    # Set application style
    app.setStyle('Fusion')

    # Create and show main window
    window = MainWindow()
    window.show()

    # Run application
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
