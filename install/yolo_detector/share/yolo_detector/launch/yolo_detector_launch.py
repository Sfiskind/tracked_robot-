from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments with better defaults for Raspberry Pi
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',  # Stick with nano model for speed
        description='Path to YOLO model file'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device to run inference on (cpu, cuda, etc.)'
    )
    
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed_image',
        default_value='True',  # Always use compressed with your camera node
        description='Whether to subscribe to compressed image topic'
    )
    
    input_size_arg = DeclareLaunchArgument(
        'input_size',
        default_value='192',  # Good balance between speed and accuracy
        description='Input size for detection (smaller = faster)'
    )
    
    skip_frames_arg = DeclareLaunchArgument(
        'skip_frames',
        default_value='2',  # Process every other frame
        description='Process every n frames (higher = faster but more laggy)'
    )
    
    processing_rate_arg = DeclareLaunchArgument(
        'processing_rate', 
        default_value='5.0',  # Back to 5Hz to match original but with skip_frames
        description='Maximum processing rate in Hz'
    )

    # Create the YOLO detector node
    yolo_detector_node = Node(
        package='yolo_detector',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': LaunchConfiguration('device'),
            'use_compressed_image': LaunchConfiguration('use_compressed_image'),
            'input_size': LaunchConfiguration('input_size'),
            'skip_frames': LaunchConfiguration('skip_frames'),
            'processing_rate': LaunchConfiguration('processing_rate'),
            'enable_optimization': True
        }],
        # No remappings - use the exact topic from the camera
        output='screen',  # Add this to see more debugging information
    )
    
    # Create a launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(model_path_arg)
    ld.add_action(confidence_threshold_arg)
    ld.add_action(device_arg)
    ld.add_action(use_compressed_arg)
    ld.add_action(input_size_arg)
    ld.add_action(skip_frames_arg)
    ld.add_action(processing_rate_arg)
    
    # Add the YOLO detector node to the launch description
    ld.add_action(yolo_detector_node)
    
    return ld