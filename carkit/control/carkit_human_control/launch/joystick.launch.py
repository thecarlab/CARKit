#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            'start_ackermann_mux': 'false',
        }.items(),
    )
    control_center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_control_center'),
                'launch',
                'control_center.launch.py',
            ])
        )
    )

    return LaunchDescription([
        joy_config_arg,
        vesc_config_arg,
        f1tenth_bringup,
        control_center,
    ])
