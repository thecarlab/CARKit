#!/usr/bin/env python3

# Copyright 2026 University of Delaware
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
import glob
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def mode_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == '", name, "'"
    ]))


def default_lidar_serial_port():
    for pattern in (
        '/dev/serial/by-id/usb-Silicon_Labs_*',
        '/dev/serial/by-id/*SLLidar*',
        '/dev/serial/by-id/*Slamtec*',
        '/dev/ttyUSB*',
    ):
        matches = sorted(glob.glob(pattern))
        if matches:
            return os.path.realpath(matches[0])
    return '/dev/ttyUSB0'


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_odom_tf = LaunchConfiguration('start_odom_tf')
    lidar_frame = LaunchConfiguration('lidar_frame')
    base_frame = LaunchConfiguration('base_frame')

    lidar = GroupAction(
        actions=[
            SetRemap(src='scan', dst='/scan/raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('sllidar_ros2'),
                        'launch',
                        'sllidar_s2_launch.py',
                    ])
                ),
                launch_arguments={
                    'channel_type': LaunchConfiguration('lidar_channel_type'),
                    'serial_port': LaunchConfiguration('lidar_serial_port'),
                    'serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
                    'frame_id': lidar_frame,
                    'inverted': LaunchConfiguration('lidar_inverted'),
                    'angle_compensate': LaunchConfiguration('lidar_angle_compensate'),
                    'scan_mode': LaunchConfiguration('lidar_scan_mode'),
                }.items(),
            ),
        ],
        condition=IfCondition(start_lidar),
    )
    lidar_filter = Node(
        package='carkit_scan_filter',
        executable='scan_footprint_filter_node',
        name='carkit_scan_footprint_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan/raw',
            'output_topic': '/scan',
            'vehicle_length_m': 0.50,
            'vehicle_width_m': 0.25,
            'padding_m': 0.0,
        }],
        condition=IfCondition(start_lidar),
    )

    start_lidar_motor = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    (
                        'WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"; '
                        'source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" && '
                        'source "${WORKSPACE}/install/setup.bash" 2>/dev/null; '
                        'echo "[carkit_navigation] Calling /start_motor"; '
                        'timeout 8 ros2 service call /start_motor std_srvs/srv/Empty "{}"'
                    ),
                ],
                output='screen',
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", start_lidar, "' == 'true' and '",
            LaunchConfiguration('auto_start_lidar_motor'), "' == 'true'",
        ])),
    )

    odom_tf = Node(
        package='carkit_amcl',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': base_frame,
            'use_message_stamp': True,
        }],
        condition=IfCondition(start_odom_tf),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_slam'),
                'launch',
                'slam.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'start_map_saver': LaunchConfiguration('start_map_saver'),
        }.items(),
        condition=mode_is('mapping'),
    )

    nav2 = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('carkit_amcl'),
                        'launch',
                        'nav2.launch.py',
                    ])
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': use_sim_time,
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': LaunchConfiguration('use_composition'),
                    'start_cmd_bridge': LaunchConfiguration('start_cmd_bridge'),
                    'start_command_mux': LaunchConfiguration('start_command_mux'),
                    'vehicle_command_topic': LaunchConfiguration('vehicle_command_topic'),
                    'mux_config': LaunchConfiguration('mux_config'),
                }.items(),
            ),
        ],
        scoped=True,
        condition=mode_is('navigation'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='navigation',
            description='CARKit Nav2 mode: mapping or navigation'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'map',
            default_value='/workspaces/CARKit/map/map_3f.yaml',
            description='Saved 2D occupancy map YAML for navigation mode'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_amcl'),
                'config',
                'nav2_params.yaml',
            ]),
            description='CARKit Nav2 parameter file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_slam'),
                'config',
                'slam_toolbox_params.yaml',
            ]),
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate lifecycle nodes'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use Nav2 composed bringup'),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Start the SLLiDAR driver'),
        DeclareLaunchArgument(
            'auto_start_lidar_motor',
            default_value='true',
            description='Call /start_motor after launch so the LiDAR publishes scans'),
        DeclareLaunchArgument(
            'start_odom_tf',
            default_value='true',
            description='Republish /odom pose as odom to base_link TF'),
        DeclareLaunchArgument(
            'start_map_saver',
            default_value='true',
            description='Start map saver in mapping mode'),
        DeclareLaunchArgument(
            'start_cmd_bridge',
            default_value='true',
            description='Start Twist-to-Ackermann bridge in navigation mode'),
        DeclareLaunchArgument(
            'start_command_mux',
            default_value='false',
            description='Start the OSRacer command relay in navigation mode'),
        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/ackermann_cmd',
            description='OSRacer relay output topic when start_command_mux is true'),
        DeclareLaunchArgument(
            'mux_config',
            default_value='',
            description='Deprecated compatibility argument; no longer used'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser',
            description='LiDAR frame used by /scan'),
        DeclareLaunchArgument('lidar_channel_type', default_value='serial'),
        DeclareLaunchArgument('lidar_serial_port', default_value=default_lidar_serial_port()),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='1000000'),
        DeclareLaunchArgument('lidar_inverted', default_value='false'),
        DeclareLaunchArgument('lidar_angle_compensate', default_value='true'),
        DeclareLaunchArgument('lidar_scan_mode', default_value='DenseBoost'),
        lidar,
        lidar_filter,
        start_lidar_motor,
        odom_tf,
        slam,
        nav2,
    ])
