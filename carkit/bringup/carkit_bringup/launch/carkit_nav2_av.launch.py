#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_av = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_navigation'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'map': LaunchConfiguration('map'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'start_lidar': LaunchConfiguration('start_lidar'),
            'start_static_tf': LaunchConfiguration('start_static_tf'),
            'auto_start_lidar_motor': LaunchConfiguration('auto_start_lidar_motor'),
            'start_odom_tf': LaunchConfiguration('start_odom_tf'),
            'start_cmd_bridge': LaunchConfiguration('start_cmd_bridge'),
            'start_command_mux': LaunchConfiguration('start_command_mux'),
            'start_stop_sign': LaunchConfiguration('start_stop_sign'),
            'vehicle_command_topic': LaunchConfiguration('vehicle_command_topic'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='navigation',
            description='CARKit Nav2 mode: mapping or navigation'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_slam'),
                'maps',
                'map.yaml',
            ]),
            description='Saved 2D occupancy map YAML for navigation mode'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Start the SLLiDAR driver'),
        DeclareLaunchArgument(
            'start_static_tf',
            default_value='true',
            description='Publish base_link to laser static transform'),
        DeclareLaunchArgument(
            'auto_start_lidar_motor',
            default_value='true',
            description='Call /start_motor after launch so the LiDAR publishes scans'),
        DeclareLaunchArgument(
            'start_odom_tf',
            default_value='true',
            description='Republish /odom pose as odom to base_link TF'),
        DeclareLaunchArgument(
            'start_cmd_bridge',
            default_value='true',
            description='Start Twist-to-Ackermann bridge in navigation mode'),
        DeclareLaunchArgument(
            'start_command_mux',
            default_value='true',
            description='Start Ackermann mux in navigation mode'),
        DeclareLaunchArgument(
            'start_stop_sign',
            default_value='true',
            description='Start CARKit stop sign override in navigation mode'),
        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/ackermann_cmd',
            description='Final Ackermann command topic produced by the mux'),
        nav2_av,
    ])
