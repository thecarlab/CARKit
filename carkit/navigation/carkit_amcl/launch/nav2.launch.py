#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def visualization_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('visualization'), "' == '", name, "'"
    ]))


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    start_cmd_bridge = LaunchConfiguration('start_cmd_bridge')
    start_command_mux = LaunchConfiguration('start_command_mux')
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
            'wheelbase': 0.25,
            'max_speed': 3.0,
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_localization',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=visualization_is('rviz'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/workspaces/CARKit/map/map_3f.yaml',
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
            default_value='false',
            description='Start legacy Ackermann command mux'),
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
            'visualization',
            default_value='foxglove',
            description='Visualization mode: foxglove, rviz, or none'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('carkit_navigation'),
                'rviz',
                'navigation.rviz',
            ]),
            description='RViz config for navigation (AMCL + planning)'),
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
        nav2_bringup,
        cmd_bridge,
        command_mux,
        foxglove_bridge,
        rviz,
    ])
