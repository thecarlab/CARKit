#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description='Final Ackermann command topic sent to the vehicle adapter/controller'
    )
    start_cmd_vel_bridge_arg = DeclareLaunchArgument(
        'start_cmd_vel_bridge',
        default_value='false',
        description='Start Twist-to-Ackermann bridge for /cmd_vel teleop tests'
    )

    command_mux = Node(
        package='carkit_command_mux',
        executable='carkit_command_mux_node',
        name='carkit_command_mux_node',
        output='screen',
        remappings=[
            ('ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
    )

    cmd_vel_bridge = Node(
        package='carkit_tools',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        output='screen',
        remappings=[
            ('/ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('start_cmd_vel_bridge')),
    )

    return LaunchDescription([
        vehicle_command_topic_arg,
        start_cmd_vel_bridge_arg,
        command_mux,
        cmd_vel_bridge,
    ])
