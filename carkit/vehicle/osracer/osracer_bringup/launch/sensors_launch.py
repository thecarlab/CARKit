#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    share = FindPackageShare('osracer_bringup')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'camera_launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'lidar_launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
        launch_arguments={
            'lidar_topic': LaunchConfiguration('lidar_topic'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_topic', default_value='/scan'),
        camera,
        lidar,
    ])
