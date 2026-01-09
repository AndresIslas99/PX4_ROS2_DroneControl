#!/usr/bin/env python3
"""
Camera Bridge: Gazebo to ROS 2
Bridges Gazebo camera topics to standard ROS 2 Image topics for the GUI
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Try to import ros_gz_bridge types
try:
    from ros_gz_interfaces.msg import Image as GzImage
    HAS_GZ_BRIDGE = True
except ImportError:
    HAS_GZ_BRIDGE = False
    print("Warning: ros_gz_interfaces not found. Using direct subscription.")


class CameraBridge(Node):
    """Bridge Gazebo camera topics to ROS 2 standard Image topics"""

    def __init__(self):
        super().__init__('camera_bridge')

        self.bridge = CvBridge()

        # QoS profile for Gazebo topics (best effort)
        qos_gazebo = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # QoS profile for ROS 2 topics (reliable)
        qos_ros = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to Gazebo camera topics
        # Try the bridged topic first
        self.camera_sub = self.create_subscription(
            Image,
            '/camera',  # This should be bridged by ros_gz_bridge
            self.camera_callback,
            qos_gazebo
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera',  # Depth camera
            self.depth_callback,
            qos_gazebo
        )

        # Publishers for standardized topics
        self.rgb_pub = self.create_publisher(
            Image,
            '/drone/camera/rgb/image_raw',
            qos_ros
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/drone/camera/depth/image_raw',
            qos_ros
        )

        self.get_logger().info('Camera bridge started')
        self.get_logger().info('  Subscribing to: /camera and /depth_camera')
        self.get_logger().info('  Publishing to: /drone/camera/rgb/image_raw and /drone/camera/depth/image_raw')

        self.frame_count = 0

    def camera_callback(self, msg):
        """Forward RGB camera images"""
        try:
            # Simply republish on standardized topic
            self.rgb_pub.publish(msg)
            self.frame_count += 1

            if self.frame_count % 30 == 0:  # Log every 30 frames
                self.get_logger().info(f'RGB camera: {self.frame_count} frames bridged')

        except Exception as e:
            self.get_logger().error(f'Error bridging RGB camera: {str(e)}')

    def depth_callback(self, msg):
        """Forward depth camera images"""
        try:
            # Republish depth images
            self.depth_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error bridging depth camera: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        bridge = CameraBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
