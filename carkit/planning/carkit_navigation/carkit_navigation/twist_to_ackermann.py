#!/usr/bin/env python3

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('ackermann_topic', '/drive')
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_reverse_speed', 0.4)
        self.declare_parameter('max_steering_angle', 0.27)
        self.declare_parameter('min_speed_for_steering', 0.05)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_speed = abs(float(self.get_parameter('max_speed').value))
        self.max_reverse_speed = abs(float(self.get_parameter('max_reverse_speed').value))
        self.max_steering_angle = abs(float(self.get_parameter('max_steering_angle').value))
        self.min_speed_for_steering = abs(float(self.get_parameter('min_speed_for_steering').value))

        self.publisher = self.create_publisher(AckermannDriveStamped, self.ackermann_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )

        self.get_logger().info(
            f'Converting {self.cmd_vel_topic} Twist commands to '
            f'{self.ackermann_topic} AckermannDriveStamped commands'
        )

    def cmd_callback(self, msg):
        speed = clamp(msg.linear.x, -self.max_reverse_speed, self.max_speed)
        steering_angle = self.steering_from_twist(speed, msg.angular.z)

        ackermann = AckermannDriveStamped()
        ackermann.header.stamp = self.get_clock().now().to_msg()
        ackermann.header.frame_id = 'base_link'
        ackermann.drive.speed = float(speed)
        ackermann.drive.steering_angle = float(steering_angle)

        self.publisher.publish(ackermann)

    def steering_from_twist(self, speed, yaw_rate):
        if abs(speed) < self.min_speed_for_steering or abs(yaw_rate) < 1e-6:
            return 0.0

        steering_angle = math.atan(self.wheelbase * yaw_rate / speed)
        return clamp(steering_angle, -self.max_steering_angle, self.max_steering_angle)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
