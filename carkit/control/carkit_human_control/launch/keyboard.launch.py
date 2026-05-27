#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )

    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description='Ackermann topic consumed by the low-level vehicle controller'
    )
    vesc_config_arg = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='F1TENTH VESC config'
    )

    keyboard = Node(
        package='carkit_human_control',
        executable='keyboard_ackermann',
        name='keyboard_ackermann',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'output_topic': LaunchConfiguration('vehicle_command_topic'),
            'max_steering_angle': 0.27,
        }],
    )

    ackermann_to_vesc = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    vesc_to_odom = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '3.141592653589793', '0.0', '0.0', 'base_link', 'laser'],
    )

    return LaunchDescription([
        vehicle_command_topic_arg,
        vesc_config_arg,
        keyboard,
        ackermann_to_vesc,
        vesc_to_odom,
        vesc_driver,
        static_tf,
    ])
