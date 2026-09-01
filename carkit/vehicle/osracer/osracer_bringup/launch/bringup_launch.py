#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    default_joy_config = PathJoinSubstitution([
        FindPackageShare('osracer_bringup'),
        'config',
        'joy_teleop.yaml',
    ])

    arguments = [
        DeclareLaunchArgument('joy_config', default_value=default_joy_config),
        DeclareLaunchArgument('vehicle_command_topic', default_value='/ackermann_cmd'),
        DeclareLaunchArgument('chassis_port', default_value='/dev/osrbot_base'),
        DeclareLaunchArgument('chassis_baudrate', default_value='460800'),
        DeclareLaunchArgument('protocol_mode', default_value='legacy'),
        DeclareLaunchArgument('wheelbase', default_value='0.325'),
        DeclareLaunchArgument('forward_max_speed', default_value='0.8'),
        DeclareLaunchArgument('reverse_max_speed', default_value='0.8'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.5235987756'),
        DeclareLaunchArgument('battery_min_voltage', default_value='10.8'),
        DeclareLaunchArgument('battery_max_voltage', default_value='12.6'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
    ]

    nodes = [
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            parameters=[LaunchConfiguration('joy_config')],
        ),
        Node(
            package='osracer_bringup',
            executable='joystick_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[LaunchConfiguration('joy_config')],
        ),
        Node(
            package='osracer_bringup',
            executable='command_relay',
            name='osracer_command_relay',
            output='screen',
            parameters=[{
                'input_topic': '/teleop',
                'output_topic': LaunchConfiguration('vehicle_command_topic'),
            }],
        ),
        Node(
            package='osracer_base',
            executable='chassis_driver',
            name='osracer_base',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('chassis_port'),
                'baudrate': ParameterValue(
                    LaunchConfiguration('chassis_baudrate'), value_type=int
                ),
                'protocol_mode': LaunchConfiguration('protocol_mode'),
                'legacy_wheelbase': float_parameter('wheelbase'),
                'legacy_forward_max_speed': float_parameter('forward_max_speed'),
                'legacy_reverse_max_speed': float_parameter('reverse_max_speed'),
                'legacy_max_steering_angle': float_parameter('max_steering_angle'),
                'legacy_battery_min_voltage': float_parameter('battery_min_voltage'),
                'legacy_battery_max_voltage': float_parameter('battery_max_voltage'),
                'cmd_timeout': float_parameter('cmd_timeout'),
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'imu_frame_id': 'imu_link',
                'mag_frame_id': 'imu_link',
                'publish_tf': False,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_laser',
            arguments=[
                '--x', '-0.082558', '--y', '-0.017229', '--z', '0.034095',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_imu',
            arguments=[
                '--x', '0.041796', '--y', '-0.017758', '--z', '-0.063599',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_camera',
            arguments=[
                '--x', '0.12323', '--y', '-0.017229', '--z', '-0.053395',
                '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
        ),
    ]

    return LaunchDescription(arguments + nodes)
