#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
        }.items(),
    )

    command_mux = Node(
        package='carkit_command_mux',
        executable='carkit_command_mux_node',
        name='carkit_controller_command_mux',
        output='screen',
        remappings=[
            ('ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
    )

    return LaunchDescription([
        vehicle_command_topic_arg,
        joy_config_arg,
        vesc_config_arg,
        mux_config_arg,
        f1tenth_bringup,
        command_mux,
    ])
