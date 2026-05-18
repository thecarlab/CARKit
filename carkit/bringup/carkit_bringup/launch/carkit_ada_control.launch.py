#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def demo_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('ada_demo'), "' == '", name, "'"
    ]))


def generate_launch_description():
    bringup_dir = get_package_share_directory('carkit_bringup')
    pure_pursuit_dir = get_package_share_directory('carkit_pure_pursuit')
    default_waypoints = os.path.join(bringup_dir, 'waypoints', 'waypoints.yaml')

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Waypoint YAML used by CARKit pure pursuit'
    )
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/ackermann_cmd',
        description='Final Ackermann command topic sent to the vehicle adapter/controller'
    )
    start_pure_pursuit_arg = DeclareLaunchArgument(
        'start_pure_pursuit',
        default_value='true',
        description='Start CARKit path loader and pure pursuit controller'
    )
    start_command_mux_arg = DeclareLaunchArgument(
        'start_command_mux',
        default_value='true',
        description='Start CARKit command mux'
    )
    start_stop_sign_arg = DeclareLaunchArgument(
        'start_stop_sign',
        default_value='true',
        description='Start CARKit stop sign behavior node'
    )
    start_emergency_brake_arg = DeclareLaunchArgument(
        'start_emergency_brake',
        default_value='false',
        description='Start CARKit emergency brake node'
    )
    start_cmd_vel_bridge_arg = DeclareLaunchArgument(
        'start_cmd_vel_bridge',
        default_value='false',
        description='Start Twist-to-Ackermann bridge for /cmd_vel tests'
    )
    ada_demo_arg = DeclareLaunchArgument(
        'ada_demo',
        default_value='none',
        description='Optional CARKit/ADA demo node: none, demo1, demo2'
    )

    pure_pursuit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_pure_pursuit'),
                'launch',
                'pure_pursuit_system.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': os.path.join(pure_pursuit_dir, 'config', 'pure_pursuit_params.yaml'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'frame_id': 'map',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_pure_pursuit')),
    )

    command_mux = Node(
        package='carkit_command_mux',
        executable='carkit_command_mux_node',
        name='carkit_command_mux_node',
        output='screen',
        remappings=[
            ('ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('start_command_mux')),
    )

    stop_sign_behavior = Node(
        package='carkit_behaviors',
        executable='stop_sign_behavior_node',
        name='stop_sign_behavior_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_stop_sign')),
    )

    emergency_brake = Node(
        package='carkit_pure_pursuit',
        executable='emergency_braker',
        name='emergency_braker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_emergency_brake')),
    )

    cmd_vel_bridge = Node(
        package='carkit_tools',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        output='screen',
        remappings=[
            ('/ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('start_cmd_vel_bridge')),
    )

    demo1 = Node(
        package='carkit_tools',
        executable='demo1',
        name='ada_demo1',
        output='screen',
        remappings=[
            ('/ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
        condition=demo_is('demo1'),
    )

    demo2 = Node(
        package='carkit_tools',
        executable='demo2',
        name='ada_demo2',
        output='screen',
        remappings=[
            ('/ackermann_cmd', LaunchConfiguration('vehicle_command_topic')),
        ],
        condition=demo_is('demo2'),
    )

    return LaunchDescription([
        waypoints_file_arg,
        vehicle_command_topic_arg,
        start_pure_pursuit_arg,
        start_command_mux_arg,
        start_stop_sign_arg,
        start_emergency_brake_arg,
        start_cmd_vel_bridge_arg,
        ada_demo_arg,
        pure_pursuit,
        command_mux,
        stop_sign_behavior,
        emergency_brake,
        cmd_vel_bridge,
        demo1,
        demo2,
    ])
