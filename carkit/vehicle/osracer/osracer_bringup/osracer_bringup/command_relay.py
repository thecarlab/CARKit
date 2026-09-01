#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
import signal

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CommandRelay(Node):
    def __init__(self):
        super().__init__('osracer_command_relay')
        self.declare_parameter('input_topic', '/teleop')
        self.declare_parameter('output_topic', '/ackermann_cmd')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        if input_topic == output_topic:
            raise ValueError('input_topic and output_topic must be different')

        self.publisher = self.create_publisher(AckermannDriveStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            input_topic,
            self.publisher.publish,
            10,
        )
        self.get_logger().info(f'Relaying {input_topic} -> {output_topic}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
