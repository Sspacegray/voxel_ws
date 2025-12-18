#!/usr/bin/env python3
"""
Twist to TwistStamped Converter Node

Subscribes to a Twist topic and republishes as TwistStamped.
Used to bridge Nav2's velocity_smoother (Twist output) to
ros2_control's diff_drive_controller (TwistStamped input).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamperNode(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        
        # Parameters
        self.declare_parameter('input_topic', '/cmd_vel_smoothed')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Subscriber and Publisher
        self.sub = self.create_subscription(Twist, input_topic, self.twist_callback, 10)
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)
        
        self.get_logger().info(f'Twist Stamper: {input_topic} (Twist) -> {output_topic} (TwistStamped)')

    def twist_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
