#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')

        # Declare parameters with default values (e.g., your original scaling)
        self.declare_parameter('linear_x_scale', -3.0) 
        self.declare_parameter('linear_y_scale', 3.0)  
        self.declare_parameter('linear_z_scale', -7.0) 
        self.declare_parameter('angular_x_scale', 3.0) 
        self.declare_parameter('angular_y_scale', 3.0) 
        self.declare_parameter('angular_z_scale', -7.0)
        # Get the parameter values
        self.linear_x_scale_ = self.get_parameter('linear_x_scale').get_parameter_value().double_value
        self.linear_y_scale_ = self.get_parameter('linear_y_scale').get_parameter_value().double_value
        self.linear_z_scale_ = self.get_parameter('linear_z_scale').get_parameter_value().double_value
        self.angular_x_scale_ = self.get_parameter('angular_x_scale').get_parameter_value().double_value
        self.angular_y_scale_ = self.get_parameter('angular_y_scale').get_parameter_value().double_value
        self.angular_z_scale_ = self.get_parameter('angular_z_scale').get_parameter_value().double_value

        # Log the scaling factors being used
        self.get_logger().info(f'Using linear scaling: x={self.linear_x_scale_}, y={self.linear_y_scale_}, z={self.linear_z_scale_}')
        self.get_logger().info(f'Using angular scaling: x={self.angular_x_scale_}, y={self.angular_y_scale_}, z={self.angular_z_scale_}')

        # Subscribe to plain Twist messages
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        # Publish TwistStamped messages
        self.publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel_stamped',
            10
        )
        self.get_logger().info('Twist Converter node has been started and parameters loaded.')

    def listener_callback(self, twist_msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'  # Adjust if needed

        # Apply scaling using the parameters retrieved during initialization
        twist_stamped.twist.linear.x = twist_msg.linear.x * self.linear_x_scale_
        twist_stamped.twist.linear.y = twist_msg.linear.y * self.linear_y_scale_
        twist_stamped.twist.linear.z = twist_msg.linear.z * self.linear_z_scale_
        twist_stamped.twist.angular.x = twist_msg.angular.x * self.angular_x_scale_
        twist_stamped.twist.angular.y = twist_msg.angular.y * self.angular_y_scale_
        twist_stamped.twist.angular.z = twist_msg.angular.z * self.angular_z_scale_

        self.publisher.publish(twist_stamped)
        # Removed the hardcoded log message about scaling, node now logs actual scales on startup

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()