#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
import signal

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steering_axis', 2)
        self.declare_parameter('speed_scale', 2.0)
        self.declare_parameter('steering_scale', 0.2)
        self.declare_parameter('mode_toggle_button', 10)
        self.declare_parameter('manual_mode_initial', True)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('teleop_topic', '/teleop')
        self.declare_parameter(
            'autonomy_enable_topic',
            'enable_autonomous_control',
        )
        self.declare_parameter('frame_id', 'base_link')

        self.speed_axis = int(self.get_parameter('speed_axis').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.steering_scale = float(self.get_parameter('steering_scale').value)
        self.mode_toggle_button = int(
            self.get_parameter('mode_toggle_button').value
        )
        self.manual_enabled = bool(
            self.get_parameter('manual_mode_initial').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.last_toggle_pressed = False

        self.teleop_pub = self.create_publisher(
            AckermannDriveStamped,
            str(self.get_parameter('teleop_topic').value),
            10,
        )
        self.mode_pub = self.create_publisher(
            Int8,
            str(self.get_parameter('autonomy_enable_topic').value),
            10,
        )
        self.mode_sub = self.create_subscription(
            Int8,
            str(self.get_parameter('autonomy_enable_topic').value),
            self.mode_callback,
            10,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self.joy_callback,
            10,
        )
        self.publish_mode()
        self.log_mode('startup')

    def joy_callback(self, msg):
        toggle_pressed = self.button_pressed(msg, self.mode_toggle_button)
        if toggle_pressed and not self.last_toggle_pressed:
            self.manual_enabled = not self.manual_enabled
            self.publish_mode()
            self.log_mode('mode toggle')
            if not self.manual_enabled:
                self.teleop_pub.publish(self.command(0.0, 0.0))
        self.last_toggle_pressed = toggle_pressed

        if not self.manual_enabled:
            return

        speed = self.axis_value(msg, self.speed_axis) * self.speed_scale
        steering = (
            self.axis_value(msg, self.steering_axis) * self.steering_scale
        )
        self.teleop_pub.publish(self.command(speed, steering))

    def command(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        return msg

    def mode_callback(self, msg):
        if msg.data not in (0, 1):
            return
        manual_enabled = msg.data == 0
        if manual_enabled == self.manual_enabled:
            return
        self.manual_enabled = manual_enabled
        if not self.manual_enabled:
            self.teleop_pub.publish(self.command(0.0, 0.0))
        self.log_mode('external mode request')

    @staticmethod
    def axis_value(msg, index):
        if index < 0 or index >= len(msg.axes):
            return 0.0
        return float(msg.axes[index])

    @staticmethod
    def button_pressed(msg, index):
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    def publish_mode(self):
        msg = Int8()
        msg.data = 0 if self.manual_enabled else 1
        self.mode_pub.publish(msg)

    def log_mode(self, source):
        owner = (
            'controller'
            if self.manual_enabled
            else 'CARKit autonomous stack'
        )
        self.get_logger().info(f'{source}: {owner} is controlling the vehicle')


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
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
