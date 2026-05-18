#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the interactive markers'
    )
    
    # Interactive waypoints node
    interactive_waypoints_node = Node(
        package='carkit_tools',
        executable='interactive_waypoints',
        name='interactive_waypoints_node',
        output='screen',
        parameters=[
            {'frame_id': LaunchConfiguration('frame_id')}
        ]
    )
    
    # RViz2 node with basic configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '']
    )
    
    return LaunchDescription([
        frame_id_arg,
        interactive_waypoints_node,
        # Uncomment the line below if you want to automatically launch RViz2
        # rviz_node,
    ]) 
