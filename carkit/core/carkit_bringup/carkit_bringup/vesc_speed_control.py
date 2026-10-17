"""Feedback adapter after the command arbiter, before the vendor RPM converter."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped

from .speed_feedback import SpeedFeedback


class VescSpeedControl(Node):
    def __init__(self):
        super().__init__('vesc_speed_control')
        def param(name, default):
            return float(self.declare_parameter(name, default).value)
        self.gain = param('speed_to_erpm_gain', 4614.0)
        self.offset = param('speed_to_erpm_offset', 0.0)
        self.control = SpeedFeedback(param('max_speed', 0.8), param('max_reverse_speed', 0.8),
                                     param('speed_kp', 0.35), param('speed_ki', 0.6),
                                     param('speed_correction_limit', 0.5))
        self.command = None
        self.command_time = self.feedback_time = -math.inf
        self.last_stamp = None
        self.measured = 0.0
        self.previous_time = time.monotonic()
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd/feedback', 1)
        self.create_subscription(AckermannDriveStamped, 'ackermann_cmd', self.on_command, 1)
        self.create_subscription(VescStateStamped, 'sensors/core', self.on_feedback, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick)

    def on_command(self, message):
        if not all(math.isfinite(v) for v in (message.drive.speed, message.drive.steering_angle)):
            self.command = None
            self.control.reset()
            self.tick()
            return
        self.command = message
        self.command_time = time.monotonic()
        if abs(message.drive.speed) < 1e-6:
            self.control.reset()
            # Stops bypass the periodic control delay.
            self.publisher.publish(message)

    def on_feedback(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        speed = (message.state.speed - self.offset) / self.gain
        valid = (math.isfinite(speed) and -0.05 <= age <= 0.25
                 and (self.last_stamp is None or stamp > self.last_stamp)
                 and abs(speed) <= 1.5 * max(self.control.forward, self.control.reverse)
                 and message.state.fault_code == 0)
        if not valid:
            self.feedback_time = -math.inf
            self.control.reset()
            return
        self.last_stamp = stamp
        self.measured = speed
        self.feedback_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = now - self.previous_time
        self.previous_time = now
        message = copy.deepcopy(self.command) if self.command else AckermannDriveStamped()
        if now - self.command_time > 0.25 or now - self.feedback_time > 0.25:
            self.control.reset()
            message.drive.speed = 0.0
            if now - self.command_time > 0.25:
                message.drive.steering_angle = 0.0
        else:
            message.drive.speed = self.control.update(message.drive.speed, self.measured, dt)
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VescSpeedControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
