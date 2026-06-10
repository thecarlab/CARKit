#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description='Ackermann topic consumed by the low-level vehicle controller'
    )
    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth_stack'),
            'config',
            'joy_teleop.yaml',
        ]),
        description='F1TENTH joystick and joy_teleop config'
    )
    vesc_config_arg = DeclareLaunchArgument(
        'vesc_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth_stack'),
            'config',
            'vesc.yaml',
        ]),
        description='F1TENTH VESC config'
    )
    mux_config_arg = DeclareLaunchArgument(
        'mux_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth_stack'),
            'config',
            'mux.yaml',
        ]),
        description='F1TENTH Ackermann mux config'
    )
    f1tenth_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth_stack'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config'),
            'vesc_config': LaunchConfiguration('vesc_config'),
            'mux_config': LaunchConfiguration('mux_config'),
            'vehicle_command_topic': LaunchConfiguration('vehicle_command_topic'),
        }.items(),
    )

    return LaunchDescription([
        vehicle_command_topic_arg,
        joy_config_arg,
        vesc_config_arg,
        mux_config_arg,
        f1tenth_bringup,
    ])
