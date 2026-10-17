"""Publish fresh RealSense measurements at 50 Hz without inventing timestamps."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRateLimit(Node):
    def __init__(self):
        super().__init__('imu_rate_limit')
        self.pending = {}
        self.last_stamp = {}
        self.publishers_by_stream = {}
        for stream in ('gyro', 'accel'):
            self.publishers_by_stream[stream] = self.create_publisher(
                Imu, f'/camera/camera/{stream}/sample', qos_profile_sensor_data)
            self.create_subscription(
                Imu, f'/camera/camera/{stream}/sample_raw',
                lambda message, key=stream: self.receive(key, message), qos_profile_sensor_data)
        self.create_timer(0.02, self.tick)

    def receive(self, stream, message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if not -0.05 <= age <= 0.25 or stamp <= self.last_stamp.get(stream, 0):
            return
        self.last_stamp[stream] = stamp
        self.pending[stream] = (time.monotonic(), message)

    def tick(self):
        pending, self.pending = self.pending, {}
        for stream, (arrival, message) in pending.items():
            age = (self.get_clock().now().nanoseconds - self.last_stamp[stream]) * 1e-9
            if time.monotonic() - arrival <= 0.25 and -0.05 <= age <= 0.25:
                self.publishers_by_stream[stream].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRateLimit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
