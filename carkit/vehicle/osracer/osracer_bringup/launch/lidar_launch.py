#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def string_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    arguments = [
        DeclareLaunchArgument('lidar_frame_id', default_value='laser'),
        DeclareLaunchArgument('lidar_topic', default_value='/scan'),
        DeclareLaunchArgument('lidar_host_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('lidar_sensor_ip', default_value='192.168.8.2'),
        DeclareLaunchArgument('lidar_port', default_value='2368'),
        DeclareLaunchArgument('lidar_inverted', default_value='false'),
        DeclareLaunchArgument('lidar_angle_offset', default_value='0'),
        DeclareLaunchArgument('lidar_scan_frequency', default_value='30'),
        DeclareLaunchArgument('lidar_filter', default_value='3'),
    ]

    lidar = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='osracer_lidar',
        output='screen',
        parameters=[{
            'frame_id': string_parameter('lidar_frame_id'),
            'output_topic': string_parameter('lidar_topic'),
            'hostip': string_parameter('lidar_host_ip'),
            'sensorip': string_parameter('lidar_sensor_ip'),
            'port': string_parameter('lidar_port'),
            'inverted': ParameterValue(
                LaunchConfiguration('lidar_inverted'), value_type=bool
            ),
            'angle_offset': ParameterValue(
                LaunchConfiguration('lidar_angle_offset'), value_type=int
            ),
            'scanfreq': string_parameter('lidar_scan_frequency'),
            'filter': string_parameter('lidar_filter'),
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315',
        }],
    )

    return LaunchDescription(arguments + [lidar])
