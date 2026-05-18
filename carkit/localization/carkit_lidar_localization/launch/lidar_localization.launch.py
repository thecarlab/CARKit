import os

import launch
import launch.actions
import launch.events

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
    bringup_dir = get_package_share_directory('carkit_bringup')

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('carkit_lidar_localization'),
            'param',
            'localization.yaml'))
    map_path = launch.substitutions.LaunchConfiguration(
        'map_path',
        default=os.path.join(bringup_dir, 'map', 'map.pcd'))

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
        default_value=localization_param_dir,
        description='Full path to the localization parameter file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'map_path',
        default_value=map_path,
        description='Full path to the PCD map file'))

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

    ld.add_action(lidar_localization)
    ld.add_action(to_inactive)

    return ld
