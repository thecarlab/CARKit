#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    start_cmd_bridge = LaunchConfiguration('start_cmd_bridge')
    start_command_mux = LaunchConfiguration('start_command_mux')
    start_stop_sign = LaunchConfiguration('start_stop_sign')
    vehicle_command_topic = LaunchConfiguration('vehicle_command_topic')
    mux_config = LaunchConfiguration('mux_config')

    bt_xml_nav_to_pose = PathJoinSubstitution([
        FindPackageShare('carkit_amcl'),
        'behavior_trees',
        'navigate_to_pose_ackermann.xml',
    ])
    bt_xml_nav_through_poses = PathJoinSubstitution([
        FindPackageShare('carkit_amcl'),
        'behavior_trees',
        'navigate_through_poses_ackermann.xml',
    ])

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_nav_to_pose,
            'default_nav_through_poses_bt_xml': bt_xml_nav_through_poses,
        },
        convert_types=True,
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items(),
    )

    cmd_bridge = Node(
        package='carkit_amcl',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'ackermann_topic': '/drive',
            'wheelbase': 0.325,
            'max_speed': 1.5,
            'max_reverse_speed': 0.3,
            'max_steering_angle': 0.27,
            'min_speed_for_steering': 0.05,
        }],
        condition=IfCondition(start_cmd_bridge),
    )

    command_mux = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        output='screen',
        parameters=[mux_config],
        remappings=[
            ('ackermann_cmd', vehicle_command_topic),
        ],
        condition=IfCondition(start_command_mux),
    )

    stop_sign_behavior = Node(
        package='carkit_behaviors',
        executable='stop_sign_behavior_node',
        name='stop_sign_behavior_node',
        output='screen',
        condition=IfCondition(start_stop_sign),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_slam'),
                'maps',
                'map.yaml',
            ]),
            description='Saved 2D occupancy map YAML for Nav2 localization'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_amcl'),
                'config',
                'nav2_params.yaml',
            ]),
            description='CARKit Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use Nav2 composed bringup'),
        DeclareLaunchArgument(
            'start_cmd_bridge',
            default_value='true',
            description='Start Twist-to-Ackermann bridge from /cmd_vel to /drive'),
        DeclareLaunchArgument(
            'start_command_mux',
            default_value='true',
            description='Start Ackermann command mux'),
        DeclareLaunchArgument(
            'start_stop_sign',
            default_value='true',
            description='Start existing CARKit stop sign behavior override'),
        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/ackermann_cmd',
            description='Final Ackermann command topic produced by the mux'),
        DeclareLaunchArgument(
            'mux_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('f1tenth_stack'),
                'config',
                'mux.yaml',
            ]),
            description='Ackermann mux config'),
        nav2_bringup,
        cmd_bridge,
        command_mux,
        stop_sign_behavior,
    ])
