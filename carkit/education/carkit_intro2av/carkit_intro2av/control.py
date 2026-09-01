#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Reference-shaped Intro2AV path-tracking ROS node."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .control_algorithm import ControlConfig, compute_command


class Intro2AvControl(Node):
    """Run student control behind production-style safety boundaries."""

    def __init__(self):
        super().__init__("carkit_intro2av_control")
        self._declare_parameters()
        self.path = None
        self.odometry = None
        self.path_received_at = 0.0
        self.odom_received_at = 0.0
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self._parameter("drive_topic"), 10
        )
        self.path_subscription = self.create_subscription(
            Path, self._parameter("plan_topic"), self._on_path, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self._parameter("odom_topic"),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        rate = max(1.0, float(self._parameter("control_rate_hz")))
        self.timer = self.create_timer(1.0 / rate, self._publish_command)
        self.get_logger().info(
            "Intro2AV controller ready: plan + odom -> drive; "
            "implement control_algorithm.compute_command"
        )

    def _declare_parameters(self):
        """Declare the ROS parameters and their safe default values."""
        defaults = {
            "plan_topic": "/plan",
            "odom_topic": "/odom",
            "drive_topic": "/drive",
            "base_frame": "base_link",
            "control_rate_hz": 10.0,
            "input_timeout_sec": 0.5,
            "wheelbase_m": 0.325,
            "lookahead_m": 0.55,
            "target_speed_mps": 0.45,
            "maximum_speed_mps": 1.0,
            "maximum_steering_rad": 0.34,
            "goal_tolerance_m": 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _parameter(self, name):
        """Return the current value of a declared ROS parameter."""
        return self.get_parameter(name).value

    def _on_path(self, message):
        self.path = message
        self.path_received_at = time.monotonic()

    def _on_odometry(self, message):
        self.odometry = message
        self.odom_received_at = time.monotonic()

    def _configuration(self):
        """Collect current parameters into an algorithm configuration."""
        return ControlConfig(
            wheelbase_m=float(self._parameter("wheelbase_m")),
            lookahead_m=float(self._parameter("lookahead_m")),
            target_speed_mps=float(self._parameter("target_speed_mps")),
            maximum_speed_mps=float(self._parameter("maximum_speed_mps")),
            maximum_steering_rad=float(
                self._parameter("maximum_steering_rad")
            ),
            goal_tolerance_m=float(self._parameter("goal_tolerance_m")),
        )

    def _publish_command(self):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self._parameter("base_frame"))
        speed, steering = 0.0, 0.0
        timeout = float(self._parameter("input_timeout_sec"))
        now = time.monotonic()
        inputs_fresh = (
            self.path is not None
            and self.odometry is not None
            and bool(self.path.poses)
            and now - self.path_received_at <= timeout
            and now - self.odom_received_at <= timeout
        )
        if inputs_fresh:
            try:
                output = compute_command(
                    self.odometry, self.path, self._configuration()
                )
                speed = float(output.speed_mps)
                steering = float(output.steering_angle_rad)
            except Exception as error:
                self.get_logger().error(f"Control algorithm failed: {error}")
        if not math.isfinite(speed) or not math.isfinite(steering):
            self.get_logger().error("Control algorithm returned non-finite output")
            speed, steering = 0.0, 0.0
        speed_limit = abs(float(self._parameter("maximum_speed_mps")))
        steering_limit = abs(float(self._parameter("maximum_steering_rad")))
        message.drive.speed = min(max(speed, -speed_limit), speed_limit)
        message.drive.steering_angle = min(
            max(steering, -steering_limit), steering_limit
        )
        self.drive_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Intro2AvControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
