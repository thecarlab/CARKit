#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Reference-shaped Intro2AV global-planning ROS node."""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from .planning_algorithm import PlanningConfig, compute_path


class Intro2AvPlanning(Node):
    """Own ROS state and delegate only path search to student code."""

    def __init__(self):
        super().__init__("carkit_intro2av_planning")
        self._declare_parameters()
        self.map_message = None
        self.odometry = None
        self.goal = None
        self.goal_revision = 0
        self.planned_revision = -1
        self.last_plan_time = 0.0

        self.plan_publisher = self.create_publisher(
            Path, self._parameter("plan_topic"), 10
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self._parameter("map_topic"),
            self._on_map,
            map_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self._parameter("odom_topic"),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped, self._parameter("goal_topic"), self._on_goal, 10
        )
        rate = max(0.1, float(self._parameter("planning_rate_hz")))
        self.timer = self.create_timer(1.0 / rate, self._update_plan)
        self.get_logger().info(
            "Intro2AV planner ready: map + odom + goal -> plan; "
            "implement planning_algorithm.compute_path"
        )

    def _declare_parameters(self):
        """Declare the ROS parameters and their safe default values."""
        defaults = {
            "map_topic": "/map",
            "odom_topic": "/odom",
            "goal_topic": "/goal_pose",
            "plan_topic": "/plan",
            "global_frame": "map",
            "planning_rate_hz": 2.0,
            "occupancy_threshold": 65,
            "allow_unknown": False,
            "inflation_radius_m": 0.25,
            "waypoint_spacing_m": 0.10,
            "goal_tolerance_m": 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _parameter(self, name):
        """Return the current value of a declared ROS parameter."""
        return self.get_parameter(name).value

    def _on_map(self, message):
        self.map_message = message

    def _on_odometry(self, message):
        self.odometry = message

    def _on_goal(self, message):
        self.goal = message
        self.goal_revision += 1

    def _configuration(self):
        """Collect current parameters into an algorithm configuration."""
        return PlanningConfig(
            occupancy_threshold=int(self._parameter("occupancy_threshold")),
            allow_unknown=bool(self._parameter("allow_unknown")),
            inflation_radius_m=float(self._parameter("inflation_radius_m")),
            waypoint_spacing_m=float(self._parameter("waypoint_spacing_m")),
            goal_tolerance_m=float(self._parameter("goal_tolerance_m")),
        )

    def _update_plan(self):
        if self.map_message is None or self.odometry is None or self.goal is None:
            return
        now = time.monotonic()
        if (
            self.planned_revision == self.goal_revision
            and now - self.last_plan_time < 1.0
        ):
            return
        self.last_plan_time = now
        try:
            poses = compute_path(
                self.map_message,
                self.odometry,
                self.goal,
                self._configuration(),
            )
            path = self._validated_path(poses)
        except Exception as error:
            self.get_logger().error(f"Planning algorithm failed: {error}")
            path = self._empty_path()
        self.planned_revision = self.goal_revision
        self.plan_publisher.publish(path)

    def _empty_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = str(self._parameter("global_frame"))
        return path

    def _validated_path(self, poses):
        path = self._empty_path()
        for pose in poses:
            if not isinstance(pose, PoseStamped):
                raise TypeError("planner must return PoseStamped values")
            position = pose.pose.position
            if not math.isfinite(position.x) or not math.isfinite(position.y):
                raise ValueError("planner returned a non-finite waypoint")
            pose.header = path.header
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = Intro2AvPlanning()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
