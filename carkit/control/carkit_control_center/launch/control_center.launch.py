# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    config = PathJoinSubstitution(
        [
            FindPackageShare("carkit_control_center"),
            "config",
            "control_center.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("max_speed", default_value="0.8"),
            DeclareLaunchArgument("allow_autonomy", default_value="true"),
            DeclareLaunchArgument("require_measured_odometry", default_value="false"),
            DeclareLaunchArgument("max_reverse_speed", default_value="0.8"),
            DeclareLaunchArgument("max_steering_angle", default_value="0.34"),
            DeclareLaunchArgument("estop_button", default_value="2"),
            DeclareLaunchArgument("clear_estop_button", default_value="3"),
            Node(
                package="carkit_control_center",
                executable="control_center_node",
                name="control_center_node",
                output="screen",
                parameters=[config, {
                    "allow_autonomy": ParameterValue(LaunchConfiguration("allow_autonomy"), value_type=bool),
                    "require_measured_odometry": ParameterValue(LaunchConfiguration("require_measured_odometry"), value_type=bool),
                    **{key: ParameterValue(LaunchConfiguration(key), value_type=float)
                       for key in ("max_speed", "max_reverse_speed", "max_steering_angle")},
                    **{key: ParameterValue(LaunchConfiguration(key), value_type=int)
                       for key in ("estop_button", "clear_estop_button")},
                }],
            ),
        ]
    )
