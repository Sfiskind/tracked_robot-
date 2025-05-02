import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SimpleCamera(Node):
    def __init__(self):
        super().__init__('simple_camera')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.bridge = CvBridge()
        
        # Try using different camera API preferences
        self.get_logger().info('Trying to open camera with direct access')
        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().info('Failed with V4L2, trying default API')
            self.cap = cv2.VideoCapture(1)
            
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return
            
        # Try setting a lower resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # Log success
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f'Camera opened successfully: {width}x{height}')
    
    def timer_callback(self):
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            self.get_logger().error('Camera not available')
            return
            
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to capture frame')
    
    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    simple_camera = SimpleCamera()
    rclpy.spin(simple_camera)
    if hasattr(simple_camera, 'cap'):
        simple_camera.cap.release()
    simple_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()