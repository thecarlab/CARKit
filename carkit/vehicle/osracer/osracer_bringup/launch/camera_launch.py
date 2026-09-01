#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    camera_info = PathJoinSubstitution([
        FindPackageShare('osracer_bringup'),
        'config',
        'camera_info',
        'rgb.yaml',
    ])

    arguments = [
        DeclareLaunchArgument('camera_device', default_value='/dev/osrbot_usb_cam'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_link'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_framerate', default_value='10.0'),
    ]

    camera = Node(
        package='carkit_camera',
        executable='low_latency_camera_node',
        name='osracer_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'frame_id': LaunchConfiguration('camera_frame_id'),
            'image_width': LaunchConfiguration('camera_width'),
            'image_height': LaunchConfiguration('camera_height'),
            'capture_framerate': 30,
            'publish_framerate': LaunchConfiguration('camera_framerate'),
            'buffer_count': 2,
            'camera_name': 'rgb',
            'camera_info_url': ParameterValue(
                ['file://', camera_info], value_type=str
            ),
        }],
    )

    return LaunchDescription(arguments + [camera])
