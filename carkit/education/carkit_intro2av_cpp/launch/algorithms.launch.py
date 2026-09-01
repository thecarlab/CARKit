#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
"""Launch all C++ Intro2AV algorithm nodes with editable configs."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def configured_node(executable, config):
    return Node(
        package="carkit_intro2av_cpp",
        executable=executable,
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("carkit_intro2av_cpp"), "config", config
        ])],
    )


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    return LaunchDescription([
        configured_node("planning_node", "planning.yaml"),
        configured_node("control_node", "control.yaml"),
        configured_node("perception_node", "perception.yaml"),
    ])
