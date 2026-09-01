#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Control exercise: /plan + /odom -> /drive (never directly to hardware)."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from .math_utils import guided_command


def yaw_from_quaternion(quaternion):
    """Convert a quaternion orientation into planar yaw radians."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class StudentControl(Node):
    def __init__(self, guided):
        super().__init__("carkit_student_control")
        self.guided = guided
        self.path = []
        self.odom = None
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(0.1, self._update)
        level = "guided baseline" if guided else "safe boilerplate"
        self.get_logger().info(f"CARKit control {level}: /plan + /odom -> /drive")

    def _on_path(self, message):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]

    def _on_odom(self, message):
        self.odom = message

    def _update(self):
        """Run one periodic update and publish the resulting output."""
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if self.guided and self.odom is not None:
            pose = self.odom.pose.pose
            speed, steering = guided_command(
                (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation),
                self.path,
            )
            command.drive.speed = speed
            command.drive.steering_angle = steering
        else:
            # TODO(Intro2AV): compute speed and steering. Zero is intentionally safe.
            command.drive.speed = 0.0
            command.drive.steering_angle = 0.0
        self.publisher.publish(command)


def _main(guided, args=None):
    rclpy.init(args=args)
    node = StudentControl(guided)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def guided_main(args=None):
    """Run the guided student implementation entry point."""
    _main(True, args)


def boilerplate_main(args=None):
    """Run the safe boilerplate student implementation entry point."""
    _main(False, args)
