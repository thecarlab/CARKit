from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("carkit_behavior"), "config", "behavior_center.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="carkit_behavior",
                executable="behavior_center_node",
                name="behavior_center_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
