#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
"""Launch all Python Intro2AV algorithm nodes with editable configs."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def configured_node(executable, config):
    return Node(
        package="carkit_intro2av",
        executable=executable,
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("carkit_intro2av"), "config", config
        ])],
    )


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    return LaunchDescription([
        configured_node("planning_node", "planning.yaml"),
        configured_node("control_node", "control.yaml"),
        configured_node("perception_node", "perception.yaml"),
    ])
