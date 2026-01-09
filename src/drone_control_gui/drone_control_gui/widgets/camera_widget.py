#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QGroupBox
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraWidget(QWidget):
    """Widget for displaying RGB camera feed from drone"""

    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self.bridge = CvBridge()
        self.frame_count = 0
        self.fps = 0.0

        # Setup UI
        self.init_ui()

        # Subscribe to camera topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.camera_subscriber = node.create_subscription(
            Image,
            '/camera',  # Gazebo camera topic
            self.camera_callback,
            qos_profile
        )

    def init_ui(self):
        """Initialize the user interface"""
        layout = QVBoxLayout()

        # Group box
        group_box = QGroupBox("RGB Camera")
        group_layout = QVBoxLayout()

        # Image display label
        self.image_label = QLabel()
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("QLabel { background-color: black; }")
        self.image_label.setText("Waiting for camera feed...")
        self.image_label.setStyleSheet("QLabel { background-color: black; color: white; }")

        # FPS label
        self.fps_label = QLabel("FPS: 0.0")
        self.fps_label.setAlignment(Qt.AlignCenter)

        group_layout.addWidget(self.image_label)
        group_layout.addWidget(self.fps_label)
        group_box.setLayout(group_layout)

        layout.addWidget(group_box)
        self.setLayout(layout)

        # Timer for FPS calculation
        from PyQt5.QtCore import QTimer
        import time
        self.last_frame_time = time.time()
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)  # Update FPS every second

    def camera_callback(self, msg):
        """Callback for camera image messages"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convert to QImage
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Scale to widget size while maintaining aspect ratio
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.image_label.width(),
                self.image_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            # Update label
            self.image_label.setPixmap(scaled_pixmap)

            # Update frame count for FPS calculation
            self.frame_count += 1

        except Exception as e:
            self.node.get_logger().error(f'Error processing camera image: {str(e)}')

    def update_fps(self):
        """Update FPS display"""
        import time
        current_time = time.time()
        elapsed = current_time - self.last_frame_time
        if elapsed > 0:
            self.fps = self.frame_count / elapsed
            self.fps_label.setText(f"FPS: {self.fps:.1f}")
        self.frame_count = 0
        self.last_frame_time = current_time
