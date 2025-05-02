#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('frame_width', 640) #640 320
        self.declare_parameter('frame_height', 480) #480  240
        self.declare_parameter('jpeg_quality', 150)  # Higher quality for better detection
        
        camera_device = self.get_parameter('camera_device').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.get_logger().info(f"Using camera device: {camera_device}")
        
        # Ensure correct topic format, should be: '/img_raw/compressed'
        self.publisher_ = self.create_publisher(CompressedImage, '/img_raw/compressed', 10)
        self.bridge = CvBridge()
        
        # Open camera with optimized settings
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Request maximum FPS
        
        time.sleep(1)  # Give camera time to initialize
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera with device {camera_device}")
            return
            
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera resolution: {actual_width}x{actual_height}")
        
        # Flush initial frames which might be corrupt
        for _ in range(5):
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Could not read a frame during buffer flush")
                
        # Set timer for image publishing - 30fps
        self.timer = self.create_timer(0.033, self.publish_image)
        
    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
            
        # Compress the image
        _, jpeg_image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        
        # Create compressed image message
        compressed_msg = CompressedImage()
        compressed_msg.format = "jpeg"
        compressed_msg.data = jpeg_image.tobytes()
        
        # Add header information
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.header.frame_id = "camera_frame"
        
        # Publish the compressed image
        self.publisher_.publish(compressed_msg)
        
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPublisher()
        if not node.cap.isOpened():
            node.get_logger().error("Camera is not initialized properly. Shutting down.")
            return
        rclpy.spin(node)
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()