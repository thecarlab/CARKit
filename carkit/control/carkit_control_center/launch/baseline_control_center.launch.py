from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="carkit_control_center",
                executable="baseline_control_center_node",
                name="baseline_control_center_node",
                output="screen",
            ),
            Node(
                package="carkit_control_center",
                executable="baseline_behavior_center_node",
                name="baseline_behavior_center_node",
                output="screen",
            ),
        ]
    )
