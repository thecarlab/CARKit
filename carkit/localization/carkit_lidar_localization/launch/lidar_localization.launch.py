import os

import launch
import launch.actions
import launch.events
from launch.conditions import IfCondition

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()
    localization_dir = get_package_share_directory('carkit_lidar_localization')
    workspace_map_path = '/workspaces/CARKit/map/map.pcd'
    legacy_workspace_map_path = '/workspaces/CARKit/map.pcd'
    if os.path.exists(workspace_map_path):
        default_map_path = workspace_map_path
    elif os.path.exists(legacy_workspace_map_path):
        default_map_path = legacy_workspace_map_path
    else:
        default_map_path = workspace_map_path

    default_localization_param_dir = os.path.join(
        localization_dir,
        'param',
        'localization.yaml')
    default_rviz_config = os.path.join(localization_dir, 'rviz', 'localization.rviz')

    localization_param_dir = launch.substitutions.LaunchConfiguration('localization_param_dir')
    map_path = launch.substitutions.LaunchConfiguration('map_path')
    rviz_config = launch.substitutions.LaunchConfiguration('rviz_config')
    use_rviz = launch.substitutions.LaunchConfiguration('use_rviz')

    lidar_localization = launch_ros.actions.LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='carkit_lidar_localization',
        executable='lidar_localization_node',
        parameters=[localization_param_dir, {'map_path': map_path}],
        remappings=[('/cloud','/cloud_in')],
        output='screen')

    ld.add_action(launch.actions.DeclareLaunchArgument(
        'localization_param_dir',
        default_value=default_localization_param_dir,
        description='Full path to the localization parameter file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Full path to the PCD map file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file used for localization and initial pose'))
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz so the initial pose can be set interactively'))

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_localization',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(rviz_node)
    ld.add_action(lidar_localization)
    ld.add_action(to_inactive)

    return ld
