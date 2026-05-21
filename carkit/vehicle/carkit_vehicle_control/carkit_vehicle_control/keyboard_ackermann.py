#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


HELP = """
Keyboard Ackermann control

w/s : increase/decrease speed
a/d : steer left/right
x   : center steering
space: stop
q   : quit
"""


class KeyboardAckermann(Node):
    def __init__(self):
        super().__init__('keyboard_ackermann')
        self.declare_parameter('output_topic', '/ackermann_cmd')
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('steering_step', 0.05)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.output_topic = self.get_parameter('output_topic').value
        self.speed_step = self.get_parameter('speed_step').value
        self.steering_step = self.get_parameter('steering_step').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        self.speed = 0.0
        self.steering = 0.0
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.output_topic,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_command)
        self.get_logger().info(HELP)
        self.get_logger().info(f'Publishing keyboard commands on {self.output_topic}')

    def handle_key(self, key):
        if key == 'w':
            self.speed = min(self.max_speed, self.speed + self.speed_step)
        elif key == 's':
            self.speed = max(-self.max_speed, self.speed - self.speed_step)
        elif key == 'a':
            self.steering = min(self.max_steering_angle, self.steering + self.steering_step)
        elif key == 'd':
            self.steering = max(-self.max_steering_angle, self.steering - self.steering_step)
        elif key == 'x':
            self.steering = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.steering = 0.0
        elif key == 'q':
            raise KeyboardInterrupt
        self.get_logger().info(f'speed={self.speed:.2f}, steering={self.steering:.2f}')

    def poll_keyboard(self):
        if not sys.stdin.isatty():
            return
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if ready:
            key = sys.stdin.read(1)
            self.handle_key(key)

    def publish_command(self):
        self.poll_keyboard()
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steering)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    settings = None
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    node = KeyboardAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
