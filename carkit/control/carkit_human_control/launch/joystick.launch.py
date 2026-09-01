#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description=(
            'Ackermann topic consumed by the low-level vehicle controller. '
            'Use /manual_command_unused when carkit_control_center owns '
            'the final /ackermann_cmd in autonomous driving.'
        )
    )
    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('osracer_bringup'),
            'config',
            'joy_teleop.yaml',
        ]),
        description='OSRacer joystick config'
    )
    chassis_port_arg = DeclareLaunchArgument(
        'chassis_port',
        default_value='/dev/osrbot_base',
        description='OSRacer chassis serial device inside Docker'
    )
    protocol_mode_arg = DeclareLaunchArgument(
        'protocol_mode',
        default_value='legacy',
        description='OSRacer controller protocol: legacy or modern'
    )
    osracer_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('osracer_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config'),
            'chassis_port': LaunchConfiguration('chassis_port'),
            'protocol_mode': LaunchConfiguration('protocol_mode'),
            'vehicle_command_topic': LaunchConfiguration('vehicle_command_topic'),
        }.items(),
    )

    return LaunchDescription([
        vehicle_command_topic_arg,
        joy_config_arg,
        chassis_port_arg,
        protocol_mode_arg,
        osracer_bringup,
    ])
