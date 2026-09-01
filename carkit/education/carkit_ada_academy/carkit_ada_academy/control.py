#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""ADA guided control: /plan + /odom -> /drive."""

import math

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .math_utils import pure_pursuit_command


def yaw_from_quaternion(quaternion):
    """Convert a quaternion orientation into planar yaw radians."""
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


class AdaControl(Node):
    def __init__(self):
        super().__init__("carkit_ada_control")
        self.path = []
        self.odom = None
        self.declare_parameter("lookahead", 0.55)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("target_speed", 0.45)
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10,
        )
        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(0.1, self._update)
        self.get_logger().info(
            "ADA guided control: /plan + /odom -> /drive"
        )

    def _on_path(self, message):
        self.path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]

    def _on_odom(self, message):
        self.odom = message

    def _update(self):
        """Run one periodic update and publish the resulting output."""
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if self.odom is not None:
            pose = self.odom.pose.pose
            # ADA exercise: tune these parameters or replace this function.
            speed, steering = pure_pursuit_command(
                (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation),
                self.path,
                float(self.get_parameter("lookahead").value),
                float(self.get_parameter("wheelbase").value),
                float(self.get_parameter("target_speed").value),
            )
            command.drive.speed = speed
            command.drive.steering_angle = steering
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = AdaControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
