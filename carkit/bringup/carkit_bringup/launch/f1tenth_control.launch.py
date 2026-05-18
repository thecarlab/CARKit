#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    f1tenth_package_arg = DeclareLaunchArgument(
        'f1tenth_package',
        default_value='f1tenth_stack',
        description='External F1TENTH ROS 2 package that owns low-level vehicle bringup'
    )
    f1tenth_launch_arg = DeclareLaunchArgument(
        'f1tenth_launch',
        default_value='bringup_launch.py',
        description='Launch file inside the F1TENTH package launch directory'
    )
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/drive',
        description='Ackermann command topic consumed by the F1TENTH controller'
    )
    start_carkit_mux_arg = DeclareLaunchArgument(
        'start_carkit_mux',
        default_value='false',
        description='Start CARKit command mux and remap its output to vehicle_command_topic'
    )
    start_cmd_vel_bridge_arg = DeclareLaunchArgument(
        'start_cmd_vel_bridge',
        default_value='false',
        description='Start CARKit Twist-to-Ackermann bridge for /cmd_vel teleop tests'
    )

    f1tenth_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('f1tenth_package')),
                'launch',
                LaunchConfiguration('f1tenth_launch'),
            ])
        )
    )

    carkit_mux = Node(
        package='carkit_command_mux',
        executable='carkit_command_mux_node',
        name='carkit_command_mux_node',
        output='screen',
        remappings=[
            ('ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('start_carkit_mux')),
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
        f1tenth_package_arg,
        f1tenth_launch_arg,
        vehicle_command_topic_arg,
        start_carkit_mux_arg,
        start_cmd_vel_bridge_arg,
        f1tenth_bringup,
        carkit_mux,
        cmd_vel_bridge,
    ])
