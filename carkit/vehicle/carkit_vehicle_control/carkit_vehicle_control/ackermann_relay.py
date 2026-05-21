#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class AckermannRelay(Node):
    def __init__(self):
        super().__init__('ackermann_relay')
        self.declare_parameter('input_topic', '/ackermann_cmd')
        self.declare_parameter('output_topic', '/ackermann_cmd')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        if self.input_topic == self.output_topic:
            self.get_logger().info(
                f'input_topic and output_topic are both {self.input_topic}; relay is idle'
            )
            self.publisher = None
            self.subscription = None
            return

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.output_topic,
            10,
        )
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            self.input_topic,
            self.command_callback,
            10,
        )
        self.get_logger().info(
            f'Relaying Ackermann commands: {self.input_topic} -> {self.output_topic}'
        )

    def command_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
