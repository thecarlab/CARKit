from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("carkit_perception"),
                "launch",
                "perception.launch.py",
            ])
        ),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "image_size": LaunchConfiguration("image_size"),
            "start_rviz": LaunchConfiguration("start_rviz"),
        }.items(),
    )

    behavior = Node(
        package="carkit_behavior",
        executable="road_rule_behavior",
        name="road_rule_behavior",
        output="screen",
        parameters=[{
            "traffic_light_min_confidence": LaunchConfiguration(
                "traffic_light_min_confidence"
            ),
            "stop_sign_min_confidence": LaunchConfiguration(
                "stop_sign_min_confidence"
            ),
            "max_lateral_offset": LaunchConfiguration("max_lateral_offset"),
            "stop_distance": LaunchConfiguration("stop_distance"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=(
                "/workspaces/CARKit/carkit/perception/"
                "carkit_perception/models/yolo11n_fp16.engine"
            ),
        ),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "traffic_light_min_confidence",
            default_value="0.35",
        ),
        DeclareLaunchArgument("stop_sign_min_confidence", default_value="0.4"),
        DeclareLaunchArgument("max_lateral_offset", default_value="1.0"),
        DeclareLaunchArgument("stop_distance", default_value="1.0"),
        perception,
        behavior,
    ])
