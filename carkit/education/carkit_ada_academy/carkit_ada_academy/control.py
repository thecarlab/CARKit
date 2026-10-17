#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""ADA guided control: /plan + /odom -> /drive."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException

from .frames import odometry_fresh, transform_points
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
        self.odom_received_at = None
        self.path_frame = ""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("lookahead", 0.55)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("target_speed", 0.45)
        self.declare_parameter("max_steering_angle", 0.34)
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
        self.path_frame = message.header.frame_id
        if not self.path_frame or any(
            pose.header.frame_id and pose.header.frame_id != self.path_frame
            for pose in message.poses
        ):
            self.path = []
            return
        self.path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]

    def _on_odom(self, message):
        self.odom = message
        self.odom_received_at = time.monotonic()

    def _update(self):
        """Run one periodic update and publish the resulting output."""
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if self.path and odometry_fresh(
            self.odom, self.odom_received_at, time.monotonic(),
            float(self.get_parameter("odom_timeout_sec").value),
        ):
            pose = self.odom.pose.pose
            try:
                path = transform_points(
                    self.tf_buffer, self.path, self.path_frame,
                    self.odom.header.frame_id, self.odom.header.stamp,
                )
            except (TransformException, ValueError):
                self.publisher.publish(command)
                return
            # ADA exercise: tune these parameters or replace this function.
            speed, steering = pure_pursuit_command(
                (pose.position.x, pose.position.y),
                yaw_from_quaternion(pose.orientation),
                path,
                float(self.get_parameter("lookahead").value),
                float(self.get_parameter("wheelbase").value),
                float(self.get_parameter("target_speed").value),
                float(self.get_parameter("max_steering_angle").value),
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
