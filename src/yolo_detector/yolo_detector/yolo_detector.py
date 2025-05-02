#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
from std_msgs.msg import Header

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Declare parameters with Raspberry Pi optimized defaults
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')  # Nano model for speed
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('use_compressed_image', True)
        self.declare_parameter('processing_rate', 5.0)  # Hz - process 5 frames per second max
        self.declare_parameter('input_size', 192)  # Good balance of speed and accuracy
        self.declare_parameter('enable_optimization', True)
        self.declare_parameter('skip_frames', 2)  # RE-ADDED: Process every 2nd frame

        # Get parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        use_compressed = self.get_parameter('use_compressed_image').value
        processing_rate = self.get_parameter('processing_rate').value
        self.input_size = self.get_parameter('input_size').value
        self.enable_optimization = self.get_parameter('enable_optimization').value
        self.skip_frames = self.get_parameter('skip_frames').value # RE-ADDED

        # RE-ADDED: Frame counter for skipping
        self.frame_count = 0

        # Set up frame processing rate limiter
        if processing_rate > 0:
            self.min_processing_interval = 1.0 / processing_rate
        else:
            self.min_processing_interval = 0.0 # Process as fast as possible if rate is 0 or less
        self.last_processing_time = 0.0

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path} on device: {device}')
        try:
            self.model = YOLO(model_path)
            # Optimize model if enabled
            if self.enable_optimization:
                self.get_logger().info('Attempting to fuse YOLO model layers...')
                self.model.fuse()  # Fuse Conv2d+BatchNorm layers for speed
                self.get_logger().info('Model fusion complete.')

            # Warm up the model
            self.get_logger().info('Warming up YOLO model...')
            dummy_input = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            for _ in range(3):  # Run a few times to warm up
                self.model(dummy_input, verbose=False)
            self.get_logger().info('YOLO model warm-up complete.')

            self.get_logger().info('YOLO model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            raise # Reraise exception to prevent node from continuing improperly

        # Create publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        self.annotated_image_pub = self.create_publisher(
            Image,
            '/annotated_image',
            10
        )

        # Initialize subscribers
        if use_compressed:
            self.get_logger().info(f'Subscribing to /img_raw/compressed (Processing rate limit: {processing_rate:.1f} Hz, Skip frames: {self.skip_frames})') # Updated log
            self.image_sub = self.create_subscription(
                CompressedImage,
                '/img_raw/compressed',
                self.compressed_image_callback,
                10 # QoS profile depth
            )
        else:
            self.get_logger().info(f'Subscribing to /img_raw (Processing rate limit: {processing_rate:.1f} Hz, Skip frames: {self.skip_frames})') # Updated log
            self.image_sub = self.create_subscription(
                Image,
                '/img_raw', # Assuming raw image topic name
                self.image_callback,
                10 # QoS profile depth
            )

        self.get_logger().info('YOLO detector node initialized successfully.')

    def image_callback(self, msg):
        """Process raw image messages."""
        # Check if we need to rate limit
        current_time = time.time()
        if self.min_processing_interval > 0.0 and \
           current_time - self.last_processing_time < self.min_processing_interval:
            return  # Skip this frame due to rate limiting

        # RE-ADDED: Apply frame skipping
        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0:
            return  # Skip this frame due to frame skipping

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_image is None:
                self.get_logger().warn("imgmsg_to_cv2 returned None")
                return
            self.process_image(cv_image, msg.header)
            self.last_processing_time = current_time
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {str(e)}')

    def compressed_image_callback(self, msg):
        """Process compressed image messages."""
        # Check if we need to rate limit
        current_time = time.time()
        if self.min_processing_interval > 0.0 and \
           current_time - self.last_processing_time < self.min_processing_interval:
            return  # Skip this frame due to rate limiting

        # RE-ADDED: Apply frame skipping
        self.frame_count += 1
        if self.skip_frames > 1 and self.frame_count % self.skip_frames != 0: # Check skip_frames > 1
            return  # Skip this frame due to frame skipping

        try:
            # Convert compressed image to CV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().error("Failed to decode compressed image")
                return

            self.process_image(cv_image, msg.header)
            self.last_processing_time = current_time
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {str(e)}')

    def process_image(self, cv_image, header):
        """Process the image with YOLO and publish results."""
        start_time = time.time()

        # Keep reference to original image size
        h, w = cv_image.shape[:2]

        # Calculate aspect-preserving resize dimensions
        aspect = w / h
        if aspect > 1:  # Wider than tall
            new_w = self.input_size
            new_h = int(self.input_size / aspect)
        else:  # Taller than wide or square
            new_h = self.input_size
            new_w = int(self.input_size * aspect)

        # Ensure dimensions are valid
        if new_w <= 0 or new_h <= 0:
             self.get_logger().warn(f"Invalid resize dimensions calculated: {new_w}x{new_h}")
             return

        # Resize image with proper interpolation method for downscaling
        try:
            resized_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        except Exception as e:
            self.get_logger().error(f"Error resizing image: {str(e)}")
            return

        # Run YOLO detection
        try:
            results = self.model(resized_image, conf=self.confidence_threshold, device=self.get_parameter('device').value, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {str(e)}")
            return

        # Prepare detections message
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = header # Use header from input message

        # Copy image for annotation BEFORE potential modifications
        annotated_img = cv_image.copy()

        # Scale factor for bounding boxes
        scale_x = w / new_w
        scale_y = h / new_h

        num_detections = 0
        # Process detection results
        if results and len(results) > 0:
            result = results[0]  # Get results for the first (only) image

            if hasattr(result, 'boxes') and result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates, confidence and class
                    # Ensure data is transferred from GPU if necessary and converted
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = result.names.get(class_id, f"ID_{class_id}") # Use .get for safety

                    # Scale coordinates back to original image
                    orig_x1 = x1 * scale_x
                    orig_y1 = y1 * scale_y
                    orig_x2 = x2 * scale_x
                    orig_y2 = y2 * scale_y

                    # Create Detection2D message
                    detection = Detection2D()
                    detection.header = header # Use header from input message

                    # Set bounding box
                    detection.bbox.center.position.x = (orig_x1 + orig_x2) / 2.0
                    detection.bbox.center.position.y = (orig_y1 + orig_y2) / 2.0
                    # Theta defaults to 0 for 2D BBox
                    detection.bbox.size_x = orig_x2 - orig_x1
                    detection.bbox.size_y = orig_y2 - orig_y1

                    # Set hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    # Use modern 'id' field if available in vision_msgs
                    if hasattr(hypothesis, 'id'):
                        hypothesis.id = str(class_id)
                    # Fallback for older vision_msgs with 'class_id'
                    elif hasattr(hypothesis, 'class_id'):
                        hypothesis.class_id = str(class_id) # Deprecated field

                    hypothesis.score = confidence
                    # Pose within hypothesis is typically not filled by 2D detectors
                    detection.results.append(hypothesis)

                    # Append detection to the array
                    detection_array_msg.detections.append(detection)
                    num_detections += 1

                    # Draw on image
                    cv2.rectangle(annotated_img, (int(orig_x1), int(orig_y1)), (int(orig_x2), int(orig_y2)), (0, 255, 0), 2)
                    label = f"{class_name}: {confidence:.2f}"
                    cv2.putText(annotated_img, label,
                                (int(orig_x1), int(orig_y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detection array only if detections exist
        if num_detections > 0:
            self.detections_pub.publish(detection_array_msg)
            # self.get_logger().debug(f"Published {num_detections} detections") # Keep as debug

        # Always publish annotated image
        try:
            # Ensure header timestamp is current for the published annotated image
            annotated_img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            annotated_img_msg.header = header # Use header from input message
            self.annotated_image_pub.publish(annotated_img_msg)
            # Log confirmation at INFO level
            self.get_logger().info("Published annotated image to /annotated_image topic")
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {str(e)}")

        # Log performance at INFO level
        processing_time = time.time() - start_time
        fps = 1.0 / processing_time if processing_time > 0 else 0
        self.get_logger().info(f'FPS: {fps:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = None # Define outside try block for access in finally
    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, shutting down.")
    except Exception as e:
        if node:
             node.get_logger().fatal(f"Unhandled exception: {str(e)}")
        else:
             print(f"Unhandled exception before node initialization: {str(e)}")
    finally:
        # Node destruction happens implicitly via rclpy.shutdown context sometimes
        # but calling explicitly if node exists is safer.
        if node and rclpy.ok(): # Check if node exists and context is valid
             if hasattr(node, 'cap') and node.cap.isOpened(): # Check if camera is open before release
                  node.get_logger().info("Releasing camera resource.")
                  node.cap.release()
             node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS shutdown complete.")

if __name__ == '__main__':
    main()