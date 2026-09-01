#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Planning exercise: /odom + /goal_pose -> /plan."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from .math_utils import straight_line_points


class StudentPlanning(Node):
    def __init__(self, guided):
        super().__init__("carkit_student_planning")
        self.guided = guided
        self.odom = None
        self.publisher = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        level = "guided baseline" if guided else "boilerplate"
        self.get_logger().info(f"CARKit planning {level}: /odom + /goal_pose -> /plan")

    def _on_odom(self, message):
        self.odom = message

    def _on_goal(self, goal):
        if self.odom is None:
            self.get_logger().warning("Waiting for /odom before planning")
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = goal.header.frame_id or "map"
        if self.guided:
            current = self.odom.pose.pose.position
            target = goal.pose.position
            for x, y in straight_line_points((current.x, current.y), (target.x, target.y)):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        else:
            # TODO(Intro2AV): populate path.poses with a collision-free route.
            pass
        self.publisher.publish(path)


def _main(guided, args=None):
    rclpy.init(args=args)
    node = StudentPlanning(guided)
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
