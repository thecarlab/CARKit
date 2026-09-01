#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""ADA guided planner: /odom + /goal_pose -> /plan."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .math_utils import straight_line_points


class AdaPlanning(Node):
    def __init__(self):
        super().__init__("carkit_ada_planning")
        self.odom = None
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

    def _on_goal(self, goal):
        if self.odom is None:
            self.get_logger().warning("Waiting for /odom before planning")
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = goal.header.frame_id or "map"
        current = self.odom.pose.pose.position
        target = goal.pose.position
        spacing = float(self.get_parameter("point_spacing").value)
        # ADA exercise: change path generation without touching CARKit core.
        points = straight_line_points(
            (current.x, current.y),
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
