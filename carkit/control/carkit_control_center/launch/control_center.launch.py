from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("carkit_control_center"),
            "config",
            "control_center.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="carkit_control_center",
                executable="control_center_node",
                name="control_center_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
