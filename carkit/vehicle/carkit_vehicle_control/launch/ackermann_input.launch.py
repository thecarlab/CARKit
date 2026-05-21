#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/ackermann_cmd',
        description='Ackermann topic published by autonomy or keyboard'
    )
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description='Ackermann topic consumed by the low-level vehicle controller'
    )
    start_keyboard_arg = DeclareLaunchArgument(
        'start_keyboard',
        default_value='false',
        description='Start keyboard Ackermann publisher'
    )
    keyboard_topic_arg = DeclareLaunchArgument(
        'keyboard_topic',
        default_value='/ackermann_cmd',
        description='Topic published by keyboard Ackermann control'
    )

    ackermann_relay = Node(
        package='carkit_vehicle_control',
        executable='ackermann_relay',
        name='ackermann_relay',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('vehicle_command_topic'),
        }],
    )

    keyboard = Node(
        package='carkit_vehicle_control',
        executable='keyboard_ackermann',
        name='keyboard_ackermann',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'output_topic': LaunchConfiguration('keyboard_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('start_keyboard')),
    )

    return LaunchDescription([
        input_topic_arg,
        vehicle_command_topic_arg,
        start_keyboard_arg,
        keyboard_topic_arg,
        ackermann_relay,
        keyboard,
    ])
