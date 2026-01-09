#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthWidget(QWidget):
    """Widget for displaying depth image from depth camera"""

    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self.bridge = CvBridge()

        # Setup UI
        self.init_ui()

        # Subscribe to depth image topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        self.depth_subscriber = node.create_subscription(
            Image,
            '/depth_camera',  # Gazebo depth camera image topic
            self.depth_callback,
            qos_profile
        )

    def init_ui(self):
        """Initialize the user interface"""
        layout = QVBoxLayout()

        # Group box
        group_box = QGroupBox("Depth Camera")
        group_layout = QVBoxLayout()

        # Depth image display
        self.depth_label = QLabel()
        self.depth_label.setMinimumSize(640, 480)
        self.depth_label.setAlignment(Qt.AlignCenter)
        self.depth_label.setStyleSheet("QLabel { background-color: black; color: white; }")
        self.depth_label.setText("Waiting for depth camera...")

        # Point count label
        self.point_label = QLabel("Depth Range: - ")
        self.point_label.setAlignment(Qt.AlignCenter)

        group_layout.addWidget(self.depth_label)
        group_layout.addWidget(self.point_label)
        group_box.setLayout(group_layout)

        layout.addWidget(group_box)
        self.setLayout(layout)

    def depth_callback(self, msg):
        """Callback for depth image messages"""
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Normalize depth for visualization (0-10m range)
            depth_normalized = np.clip(depth_image, 0, 10) / 10.0
            depth_normalized = (depth_normalized * 255).astype(np.uint8)

            # Apply colormap (COLORMAP_JET: blue=close, red=far)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # Convert BGR to RGB
            depth_rgb = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)

            # Convert to QImage
            h, w, ch = depth_rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(depth_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Scale to widget size
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.depth_label.width(),
                self.depth_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            # Update label
            self.depth_label.setPixmap(scaled_pixmap)

            # Update depth range
            min_depth = np.min(depth_image[depth_image > 0]) if np.any(depth_image > 0) else 0
            max_depth = np.max(depth_image)
            self.point_label.setText(f"Depth Range: {min_depth:.2f}m - {max_depth:.2f}m")

        except Exception as e:
            self.node.get_logger().error(f'Error processing depth image: {str(e)}')
