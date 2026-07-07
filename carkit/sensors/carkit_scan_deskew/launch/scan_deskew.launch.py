#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('carkit_scan_deskew'),
        'config',
        'scan_deskew.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Scan deskew parameter file'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Input LaserScan topic'),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/scan_deskewed',
            description='Deskewed LaserScan output topic'),
        Node(
            package='carkit_scan_deskew',
            executable='scan_deskew_node',
            name='scan_deskew_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                },
            ],
        ),
    ])
