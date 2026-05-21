#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description='Ackermann topic consumed by the low-level vehicle controller'
    )

    command_mux = Node(
        package='carkit_command_mux',
        executable='carkit_command_mux_node',
        name='carkit_manual_command_mux',
        output='screen',
        remappings=[
            ('ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
    )

    return LaunchDescription([
        vehicle_command_topic_arg,
        command_mux,
    ])
