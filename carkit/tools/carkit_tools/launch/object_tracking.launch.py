from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_object_type_arg = DeclareLaunchArgument(
        "target_object_type",
        default_value="book",
        description="Type of object to track",
    )
    waypoint_distance_arg = DeclareLaunchArgument(
        "waypoint_distance",
        default_value="0.5",
        description="Distance between waypoints in meters",
    )

    object_position_node = Node(
        package="carkit_tools",
        executable="object_position",
        name="object_position_node",
        parameters=[{
            "target_object_type": LaunchConfiguration("target_object_type"),
        }],
        output="screen",
    )
    path_tracker_node = Node(
        package="carkit_tools",
        executable="path_tracker",
        name="path_tracker_node",
        parameters=[{
            "waypoint_distance": LaunchConfiguration("waypoint_distance"),
        }],
        output="screen",
    )

    return LaunchDescription([
        target_object_type_arg,
        waypoint_distance_arg,
        object_position_node,
        path_tracker_node,
    ])
