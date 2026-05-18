#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2') 
    realsense_ros2_dir = get_package_share_directory('realsense2_camera')
    
    bringup_dir = get_package_share_directory('carkit_bringup')
    pure_pursuit_dir = get_package_share_directory('carkit_pure_pursuit')
    default_rviz = os.path.join(bringup_dir, 'rviz', 'localization.rviz')
    default_waypoints = os.path.join(bringup_dir, 'waypoints', 'waypoints.yaml')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz config file'
    )
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Path to the CARKit waypoints YAML'
    )
    
    # Launch sllidar_s1
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_s2_launch.py')
        )
    )
    
    # Launch realsense node
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
    
    # Launch lidar transform node from CARKit sensor transforms.
    lidar_transform_node = Node(
        package='carkit_sensor_transforms',
        executable='lidar_transformer_node',
        name='lidar_transformer_node',
        output='screen',
        parameters=[]
    )
    
    # Launch RViz2 with localization configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    # Launch lidar localization (delayed to start after rviz2)
    lidar_localization_launch = TimerAction(
        period=3.0,  # Wait 3 seconds before starting lidar localization
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('carkit_lidar_localization'),
                        'launch',
                        'lidar_localization.launch.py'
                    ])
                )
            )
        ]
    )
    
    # Launch control center node
    carkit_command_mux_node = Node(
        package='carkit_command_mux',
        executable='carkit_command_mux_node',
        name='carkit_command_mux_node',
        output='screen'
    )
    
    # Launch stop sign behavior node
    stop_sign_behavior_node = Node(
        package='carkit_behaviors',
        executable='stop_sign_behavior_node',
        name='stop_sign_behavior_node',
        output='screen'
    )
    

    # Launch pure pursuit controller
    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carkit_pure_pursuit'),
                'launch',
                'pure_pursuit_system.launch.py'
            ])
        ),
        launch_arguments={
            'config_file': os.path.join(pure_pursuit_dir, 'config', 'pure_pursuit_params.yaml'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'frame_id': 'map'
        }.items()
    )
    
    return LaunchDescription([
        rviz_config_arg,
        waypoints_file_arg,
        rviz_node,
        sllidar_launch,
        realsense_launch,
        carkit_command_mux_node,
        lidar_transform_node,
        lidar_localization_launch, 
        pure_pursuit_launch,
        stop_sign_behavior_node
    ]) 
