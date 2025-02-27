#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publisher for the image topic
        self.publisher_ = self.create_publisher(Image, 'img_raw', 10)
        
        # Timer to publish images at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_image)
        
        # OpenCV bridge to convert images
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # Convert OpenCV image to ROS 2 Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image
        self.publisher_.publish(img_msg)
        self.get_logger().info("Publishing image to img_raw")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
