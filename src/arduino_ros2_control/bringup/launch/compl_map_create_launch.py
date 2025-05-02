import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Use absolute paths for your arduino_ros2_control launch files.
    mapping_launch = '/home/pi/ros2_main/src/arduino_ros2_control/bringup/launch/online_sync_launch.py'
    diffbot_launch = '/home/pi/ros2_main/src/arduino_ros2_control/bringup/launch/diffbot.launch.py'
    
    # For rplidar_ros, we use get_package_share_directory since it's installed properly.
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_share, 'launch', 'rplidar_c1_launch.py')
    
    # Launch your twist_converter node with a remapping.
    # This replicates running:
    # ros2 run your_twist_converter twist_converter --ros-args -r cmd_vel_stamped:=/diffbot_base_controller/cmd_vel
    twist_converter_node = Node(
        package='your_twist_converter',
        executable='twist_converter',
        output='screen',
        remappings=[('cmd_vel_stamped', '/diffbot_base_controller/cmd_vel')]
    )
    
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(diffbot_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),
        twist_converter_node,
    ])
    
    return ld