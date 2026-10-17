#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
import signal
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, String


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steering_axis', 2)
        self.declare_parameter('speed_scale', 2.0)
        self.declare_parameter('steering_scale', 0.2)
        self.declare_parameter('mode_toggle_button', 10)
        self.declare_parameter('manual_mode_initial', True)
        self.declare_parameter('standalone_manual', False)
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
        self.standalone_manual = bool(self.get_parameter('standalone_manual').value)
        self.manual_enabled = self.standalone_manual
        self.confirmed_mode = 'HUMAN_CONTROL' if self.standalone_manual else None
        self.mode_received_at = None
        self.neutral_required = True
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
            String, '/control_center/main_state', self.mode_callback, 10,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self.joy_callback,
            10,
        )
        self.create_timer(0.1, self.check_mode_freshness)
        self.log_mode('startup')

    def check_mode_freshness(self):
        if not self.standalone_manual and (self.mode_received_at is None or
                time.monotonic() - self.mode_received_at > 1.5):
            self.manual_enabled = False
            self.confirmed_mode = None
            self.neutral_required = True
            self.teleop_pub.publish(self.command(0.0, 0.0))

    def joy_callback(self, msg):
        self.check_mode_freshness()
        toggle_pressed = self.button_pressed(msg, self.mode_toggle_button)
        if (toggle_pressed and not self.last_toggle_pressed and not self.standalone_manual
                and self.confirmed_mode in ('HUMAN_CONTROL', 'AUTO_DRIVE')):
            self.mode_pub.publish(Int8(data=1 if self.confirmed_mode == 'HUMAN_CONTROL' else 0))
            self.neutral_required = True
            self.teleop_pub.publish(self.command(0.0, 0.0))
        self.last_toggle_pressed = toggle_pressed
        if not self.manual_enabled:
            return
        speed_axis = self.axis_value(msg, self.speed_axis)
        steering_axis = self.axis_value(msg, self.steering_axis)
        if self.neutral_required:
            if abs(speed_axis) <= 0.05 and abs(steering_axis) <= 0.05:
                self.neutral_required = False
            self.teleop_pub.publish(self.command(0.0, 0.0))
            return
        self.teleop_pub.publish(self.command(
            speed_axis * self.speed_scale, steering_axis * self.steering_scale))

    def command(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        return msg

    def mode_callback(self, msg):
        if self.standalone_manual or msg.data not in ('HUMAN_CONTROL', 'AUTO_DRIVE', 'EMERGENCY_STOP'):
            return
        self.mode_received_at = time.monotonic()
        if msg.data == self.confirmed_mode:
            return
        self.confirmed_mode = msg.data
        self.manual_enabled = msg.data == 'HUMAN_CONTROL'
        self.neutral_required = True
        self.teleop_pub.publish(self.command(0.0, 0.0))
        self.log_mode('confirmed control state')

    @staticmethod
    def axis_value(msg, index):
        if index < 0 or index >= len(msg.axes):
            return 0.0
        value = float(msg.axes[index])
        return max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0

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
