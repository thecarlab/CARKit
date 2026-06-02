#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    realsense_ros2_dir = get_package_share_directory('realsense2_camera')
    bringup_dir = get_package_share_directory('carkit_bringup')
    default_mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_av.rviz'),
        description='RViz config file'
    )
    mux_config_arg = DeclareLaunchArgument(
        'mux_config',
        default_value=default_mux_config,
        description='F1TENTH Ackermann mux config'
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_s2_launch.py')
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_ros2_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_rgbd': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true'
        }.items()
    )

    lidar_transform_node = Node(
        package='carkit_sensor_transforms',
        executable='lidar_transformer_node',
        name='lidar_transformer_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        output='screen',
        parameters=[LaunchConfiguration('mux_config')]
    )

    stop_sign_behavior_node = Node(
        package='carkit_behaviors',
        executable='stop_sign_behavior_node',
        name='stop_sign_behavior_node',
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        mux_config_arg,
        rviz_node,
        sllidar_launch,
        realsense_launch,
        lidar_transform_node,
        ackermann_mux_node,
        stop_sign_behavior_node,
    ])
