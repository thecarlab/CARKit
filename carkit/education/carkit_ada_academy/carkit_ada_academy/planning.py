#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""ADA guided planner: /odom + /goal_pose -> /plan."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException

from .frames import odometry_fresh, transform_points
from .math_utils import straight_line_points


class AdaPlanning(Node):
    def __init__(self):
        super().__init__("carkit_ada_planning")
        self.odom = None
        self.odom_received_at = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("point_spacing", 0.15)
        self.publisher = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self._on_goal,
            10,
        )
        self.get_logger().info(
            "ADA guided planning: /odom + /goal_pose -> /plan"
        )

    def _on_odom(self, message):
        self.odom = message
        self.odom_received_at = time.monotonic()

    def _on_goal(self, goal):
        if not odometry_fresh(
            self.odom, self.odom_received_at, time.monotonic(),
            float(self.get_parameter("odom_timeout_sec").value),
        ):
            self.get_logger().warning("Waiting for fresh /odom before planning")
            self.publisher.publish(Path())  # Invalidate any previous plan.
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = goal.header.frame_id
        current = self.odom.pose.pose.position
        target = goal.pose.position
        try:
            start = transform_points(
                self.tf_buffer, [(current.x, current.y)],
                self.odom.header.frame_id, goal.header.frame_id,
                self.odom.header.stamp,
            )[0]
            if not all(math.isfinite(v) for v in (target.x, target.y)):
                raise ValueError("Goal coordinates must be finite")
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f"Cannot plan in goal frame: {error}")
            self.publisher.publish(Path())
            return
        spacing = float(self.get_parameter("point_spacing").value)
        # ADA exercise: change path generation without touching CARKit core.
        points = straight_line_points(
            start,
            (target.x, target.y),
            spacing,
        )
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = AdaPlanning()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
