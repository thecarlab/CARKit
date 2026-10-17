"""Bounded, watchdog-protected relay for standalone manual driving."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ackermann_msgs.msg import AckermannDriveStamped


class CommandRelay(Node):
    def __init__(self):
        super().__init__('manual_command_relay')
        source = self.declare_parameter('input_topic', '/teleop').value
        target = self.declare_parameter('output_topic', '/ackermann_cmd').value
        if source == target:
            raise ValueError('Input and output topics must differ')
        self.max_speed = self.declare_parameter('max_speed', 0.8).value
        self.max_reverse_speed = self.declare_parameter('max_reverse_speed', 0.8).value
        self.max_steering = self.declare_parameter('max_steering_angle', 0.34).value
        self.timeout = self.declare_parameter('command_timeout_sec', 0.3).value
        if not all(math.isfinite(v) and v > 0 for v in (
            self.max_speed, self.max_reverse_speed, self.max_steering, self.timeout
        )):
            raise ValueError('Command limits and timeout must be finite and positive')
        self.command = None
        self.received_at = None
        self.publisher = self.create_publisher(AckermannDriveStamped, target, 10)
        self.subscription = self.create_subscription(AckermannDriveStamped, source, self.receive, 10)
        self.create_timer(0.05, self.publish)

    def receive(self, message):
        self.command = message
        self.received_at = time.monotonic()

    def publish(self):
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        if self.command is not None and time.monotonic() - self.received_at <= self.timeout:
            speed, angle = self.command.drive.speed, self.command.drive.steering_angle
            if math.isfinite(speed) and math.isfinite(angle):
                output.drive.speed = max(-self.max_reverse_speed, min(self.max_speed, speed))
                output.drive.steering_angle = max(-self.max_steering, min(self.max_steering, angle))
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CommandRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
