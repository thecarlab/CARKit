#!/usr/bin/env python3

"""Bring up the AMCL-anchored simple loop controller with its related nodes.

This launch starts the ``simple_loop_controller`` node together with the
stacks it depends on to drive the closed straight/turn loop autonomously:

* localization (LiDAR + AMCL + odom->base_link TF) via
  ``carkit_navigation/navigation.launch.py`` so the controller gets the
  ``map -> base_link`` transform it follows,
* ``carkit_control_center`` which consumes the controller's ``/drive`` output
  and owns the final ``/ackermann_cmd`` based on the joystick mode, and
* ``carkit_human_control`` (joystick + F1TENTH vehicle bringup) which provides
  the manual/autonomous mode toggle and actuates the VESC.

Each related stack can be toggled off (``start_localization``,
``start_control_center``, ``start_joystick``) when it is already running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_localization = LaunchConfiguration('start_localization')
    start_control_center = LaunchConfiguration('start_control_center')
    start_joystick = LaunchConfiguration('start_joystick')

    simple_loop_controller = Node(
        package='carkit_amcl',
        executable='simple_loop_controller',
        name='simple_loop_controller',
        output='screen',
        parameters=[{
            'pose_source': 'tf',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'amcl_pose_topic': '/amcl_pose',
            'ackermann_topic': LaunchConfiguration('ackermann_topic'),
            'path_topic': '/simple_loop_path',
            'straight_distance': 3.5,
            'turn_radius': 1.7,
            'wheelbase': 0.25,
            'max_steering_angle': 0.27,
            'turn_direction': LaunchConfiguration('simple_loop_turn_direction'),
            'linear_speed': LaunchConfiguration('simple_loop_speed'),
            'lookahead_distance': LaunchConfiguration('simple_loop_lookahead'),
            'loop_path': LaunchConfiguration('simple_loop_loop_path'),
            'require_autonomous_mode': LaunchConfiguration(
                'simple_loop_require_autonomous'
            ),
            'autonomy_enable_topic': LaunchConfiguration(
                'simple_loop_autonomy_enable_topic'
            ),
        }],
    )

    # Joystick + F1TENTH vehicle bringup (joy, joy_teleop, VESC, actuation).
    # vehicle_command_topic is pushed off /ackermann_cmd so the control center
    # remains the sole publisher of the final command in autonomous mode.
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_human_control'),
                'launch',
                'joystick.launch.py',
            ])
        ),
        launch_arguments={
            'vehicle_command_topic': LaunchConfiguration('vehicle_command_topic'),
        }.items(),
        condition=IfCondition(start_joystick),
    )

    control_center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_control_center'),
                'launch',
                'control_center.launch.py',
            ])
        ),
        condition=IfCondition(start_control_center),
    )

    # Localization stack. The simple loop controller is started explicitly above,
    # so keep it disabled here to avoid a duplicate node. The Nav2 twist bridge is
    # also left off so it does not fight the controller for /drive.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_navigation'),
                'launch',
                'navigation.launch.py',
            ])
        ),
        launch_arguments={
            'mode': 'navigation',
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'visualization': LaunchConfiguration('visualization'),
            'start_cmd_bridge': 'false',
            'start_simple_loop_controller': 'false',
        }.items(),
        condition=IfCondition(start_localization),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='Start LiDAR + AMCL localization (map -> base_link TF)'),
        DeclareLaunchArgument(
            'start_control_center',
            default_value='true',
            description='Start carkit_control_center as the /ackermann_cmd arbiter'),
        DeclareLaunchArgument(
            'start_joystick',
            default_value='true',
            description='Start the joystick + F1TENTH vehicle bringup'),
        DeclareLaunchArgument(
            'ackermann_topic',
            default_value='/drive',
            description=(
                'Topic the simple loop controller publishes to. Use /drive so '
                'the control center arbitrates it; use /ackermann_cmd to drive '
                'the vehicle directly (only without the control center)')),
        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/ackermann_mux_unused',
            description=(
                'Legacy mux output topic for joystick.launch.py. Kept off '
                '/ackermann_cmd so the control center owns the final command')),
        DeclareLaunchArgument(
            'map',
            default_value='/workspaces/CARKit/map/map_3f.yaml',
            description='Saved 2D occupancy map YAML for AMCL localization'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'visualization',
            default_value='foxglove',
            description='Visualization mode for localization: foxglove, rviz, or none'),
        DeclareLaunchArgument(
            'simple_loop_speed',
            default_value='0.35',
            description='Forward speed for the simple loop controller in m/s'),
        DeclareLaunchArgument(
            'simple_loop_lookahead',
            default_value='0.8',
            description='Pure-pursuit lookahead distance for the simple loop controller'),
        DeclareLaunchArgument(
            'simple_loop_turn_direction',
            default_value='left',
            description='Half-circle direction for the simple loop controller: left or right'),
        DeclareLaunchArgument(
            'simple_loop_loop_path',
            default_value='true',
            description='Keep driving the closed simple loop until the node is stopped'),
        DeclareLaunchArgument(
            'simple_loop_require_autonomous',
            default_value='true',
            description=(
                'Only publish loop commands while the joystick reports '
                'autonomous mode on the autonomy enable topic')),
        DeclareLaunchArgument(
            'simple_loop_autonomy_enable_topic',
            default_value='/enable_autonomous_control',
            description='Int8 topic (0=manual, 1=autonomous) that gates loop commands'),
        localization,
        control_center,
        joystick,
        simple_loop_controller,
    ])
