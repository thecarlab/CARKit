#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def mode_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == '", name, "'"
    ]))


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_static_tf = LaunchConfiguration('start_static_tf')
    start_odom_tf = LaunchConfiguration('start_odom_tf')
    use_rviz = LaunchConfiguration('use_rviz')
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

    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_to_laser',
        arguments=[
            '--x',
            LaunchConfiguration('lidar_tf_x'),
            '--y',
            LaunchConfiguration('lidar_tf_y'),
            '--z',
            LaunchConfiguration('lidar_tf_z'),
            '--yaw',
            LaunchConfiguration('lidar_tf_yaw'),
            '--pitch',
            LaunchConfiguration('lidar_tf_pitch'),
            '--roll',
            LaunchConfiguration('lidar_tf_roll'),
            '--frame-id',
            base_frame,
            '--child-frame-id',
            lidar_frame,
        ],
        condition=IfCondition(start_static_tf),
    )

    start_lidar_motor = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    'timeout 8 ros2 service call /start_motor std_srvs/srv/Empty "{}" || true',
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
                    'use_rviz': 'false',
                }.items(),
            ),
        ],
        scoped=True,
        condition=mode_is('navigation'),
    )

    mapping_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', LaunchConfiguration('mapping_rviz_config')],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "' == 'true' and '", mode, "' == 'mapping'"
        ])),
    )

    planning_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_planning',
        arguments=['-d', LaunchConfiguration('planning_rviz_config')],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "' == 'true' and '", mode, "' == 'navigation'"
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
            'use_rviz',
            default_value='true',
            description='Start RViz'),
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
            'start_static_tf',
            default_value='true',
            description='Publish base_link to laser static transform'),
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
            default_value='true',
            description='Start Ackermann mux in navigation mode'),
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
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser',
            description='LiDAR frame used by /scan'),
        DeclareLaunchArgument('lidar_channel_type', default_value='serial'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='1000000'),
        DeclareLaunchArgument('lidar_inverted', default_value='false'),
        DeclareLaunchArgument('lidar_angle_compensate', default_value='true'),
        DeclareLaunchArgument('lidar_scan_mode', default_value='DenseBoost'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.27'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.11'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        lidar,
        static_laser_tf,
        start_lidar_motor,
        odom_tf,
        slam,
        nav2,
        mapping_rviz,
        planning_rviz,
    ])
