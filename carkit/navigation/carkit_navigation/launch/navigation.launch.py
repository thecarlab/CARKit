#!/usr/bin/env python3

import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def mode_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == '", name, "'"
    ]))


def visualization_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('visualization'), "' == '", name, "'"
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
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_odom_tf = LaunchConfiguration('start_odom_tf')
    lidar_frame = LaunchConfiguration('lidar_frame')
    base_frame = LaunchConfiguration('base_frame')

    lidar = IncludeLaunchDescription(
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
        condition=IfCondition(LaunchConfiguration('auto_start_lidar_motor')),
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
                    'visualization': 'none',
                }.items(),
            ),
        ],
        scoped=True,
        condition=mode_is('navigation'),
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            ])
        ),
        launch_arguments={
            'address': LaunchConfiguration('foxglove_address'),
            'port': LaunchConfiguration('foxglove_port'),
            'remote_access': LaunchConfiguration('foxglove_remote_access'),
            'device_token': LaunchConfiguration('foxglove_device_token'),
            'sysinfo': LaunchConfiguration('foxglove_sysinfo'),
            'topic_whitelist': LaunchConfiguration('foxglove_topic_whitelist'),
            'client_topic_whitelist': LaunchConfiguration(
                'foxglove_client_topic_whitelist'
            ),
            'param_whitelist': LaunchConfiguration('foxglove_param_whitelist'),
            'service_whitelist': LaunchConfiguration('foxglove_service_whitelist'),
            'capabilities': LaunchConfiguration('foxglove_capabilities'),
        }.items(),
        condition=visualization_is('foxglove'),
    )

    mapping_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', LaunchConfiguration('mapping_rviz_config')],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('visualization'),
            "' == 'rviz' and '", mode, "' == 'mapping'"
        ])),
    )

    planning_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_planning',
        arguments=['-d', LaunchConfiguration('planning_rviz_config')],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('visualization'),
            "' == 'rviz' and '", mode, "' == 'navigation'"
        ])),
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
            'mapping_rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_slam'),
                'rviz',
                'mapping.rviz',
            ]),
            description='RViz config for mapping mode'),
        DeclareLaunchArgument(
            'planning_rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_navigation'),
                'rviz',
                'navigation.rviz',
            ]),
            description='RViz config for navigation mode (AMCL + planning)'),
        DeclareLaunchArgument(
            'visualization',
            default_value='foxglove',
            description='Visualization mode: foxglove, rviz, or none'),
        DeclareLaunchArgument(
            'foxglove_address',
            default_value='0.0.0.0',
            description='Foxglove Bridge bind address'),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8765',
            description='Foxglove Bridge WebSocket port'),
        DeclareLaunchArgument(
            'foxglove_remote_access',
            default_value='false',
            description='Enable Foxglove remote access'),
        DeclareLaunchArgument(
            'foxglove_device_token',
            default_value='',
            description='Foxglove device token for remote access'),
        DeclareLaunchArgument(
            'foxglove_sysinfo',
            default_value='false',
            description='Publish system info through Foxglove Bridge'),
        DeclareLaunchArgument(
            'foxglove_topic_whitelist',
            default_value=(
                "['^/map$', '^/map_metadata$', '^/tf$', '^/tf_static$', "
                "'^/scan$', '^/amcl_pose$', '^/particle_cloud$', "
                "'^/plan$', '^/plan_smoothed$', '^/received_global_plan$', "
                "'^/local_plan$', '^/goal_pose$', '^/move_base_simple/goal$', "
                "'^/initialpose$', '^/clicked_point$', "
                "'^/behavior/stop_sign_position$', "
                "'^/behavior/traffic_light_position$', "
                "'^/behavior/stop_sign_markers$', "
                "'^/behavior/traffic_light_markers$']"
            ),
            description='Foxglove Bridge topic whitelist'),
        DeclareLaunchArgument(
            'foxglove_client_topic_whitelist',
            default_value=(
                "['^/goal_pose$', '^/move_base_simple/goal$', "
                "'^/initialpose$', '^/clicked_point$']"
            ),
            description='Topics Foxglove clients may publish'),
        DeclareLaunchArgument(
            'foxglove_param_whitelist',
            default_value="['^$']",
            description='Foxglove Bridge parameter whitelist'),
        DeclareLaunchArgument(
            'foxglove_service_whitelist',
            default_value="['^$']",
            description='Foxglove Bridge service whitelist'),
        DeclareLaunchArgument(
            'foxglove_capabilities',
            default_value='[clientPublish,connectionGraph]',
            description='Foxglove Bridge capabilities'),
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
            description='Start legacy Ackermann mux in navigation mode'),
        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/ackermann_cmd',
            description='Legacy mux output topic when start_command_mux is true'),
        DeclareLaunchArgument(
            'mux_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('f1tenth_stack'),
                'config',
                'mux.yaml',
            ]),
            description='Ackermann mux config'),
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
        start_lidar_motor,
        odom_tf,
        foxglove_bridge,
        slam,
        nav2,
        mapping_rviz,
        planning_rviz,
    ])
