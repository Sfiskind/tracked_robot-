import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Use absolute paths for your arduino_ros2_control launch files.
    navigation_launch = '/home/pi/ros2_main/src/arduino_ros2_control/bringup/launch/navigation_launch.py'
    diffbot_launch = '/home/pi/ros2_main/src/arduino_ros2_control/bringup/launch/diffbot.launch.py'
    localization_launch = '/home/pi/ros2_main/src/arduino_ros2_control/bringup/launch/localization_launch.py'

    # For rplidar_ros, we use get_package_share_directory since it's installed properly.
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_share, 'launch', 'rplidar_c1_launch.py')

    # Define launch descriptions for each component
    diffbot_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diffbot_launch)
    )

    # Launch your twist_converter node with parameters and remapping.
    # --- MODIFIED SECTION ---
    twist_converter_node = Node(
        package='your_twist_converter', # Make sure this matches your package name
        executable='twist_converter',
        output='screen',
        remappings=[('cmd_vel_stamped', '/diffbot_base_controller/cmd_vel')],
        parameters=[{ # Pass the desired parameters for Nav2 operation
            'linear_x_scale': -5.0,   # Example: Increase magnitude from -3.0 to -5.0
            'linear_y_scale': 5.0,    # Example: Increase magnitude from 3.0 to 5.0
            'linear_z_scale': -10.0,  # Example: Increase magnitude from -7.0 to -10.0
            'angular_x_scale': 5.0,   # Example: Increase magnitude from 3.0 to 5.0
            'angular_y_scale': 5.0,   # Example: Increase magnitude from 3.0 to 5.0
            'angular_z_scale': -10.0  # Example: Increase magnitude from -7.0 to -10.0
        }]
    )
    # --- END MODIFIED SECTION ---

    rplidar_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch)
    )

    localization_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch)
    )

    navigation_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    # Use TimerAction to sequence the launch files
    # Start with diffbot_launch immediately

    # Start twist_converter_node after 10 seconds
    twist_converter_timer = TimerAction(
        period=10.0,
        actions=[twist_converter_node]
    )

    # Start rplidar_launch after 15 seconds
    rplidar_timer = TimerAction(
        period=15.0,
        actions=[rplidar_launch_desc]
    )

    # Start localization_launch after 20 seconds
    localization_timer = TimerAction(
        period=20.0,
        actions=[localization_launch_desc]
    )

    #Start navigation_launch after 35 seconds
    navigation_timer = TimerAction(
        period=35.0,
         actions=[navigation_launch_desc]
    )

    # Return the launch description with timed execution
    ld = LaunchDescription([
        diffbot_launch_desc,  # Start immediately
        twist_converter_timer,
        rplidar_timer,
        localization_timer,
        navigation_timer
    ])

    return ld