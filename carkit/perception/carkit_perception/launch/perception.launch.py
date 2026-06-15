import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("carkit_perception")

    perception_3d_node = Node(
        package="carkit_perception",
        executable="perception_3d_node",
        name="perception_3d_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "image_size": LaunchConfiguration("image_size"),
            "image_topic": LaunchConfiguration("image_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "inference_image_topic": LaunchConfiguration("inference_image_topic"),
            "detection_3d_topic": LaunchConfiguration("detection_3d_topic"),
            "min_confidence": LaunchConfiguration("min_confidence"),
            "sync_slop": LaunchConfiguration("sync_slop"),
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="perception_rviz",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=(
                "/workspaces/CARKit/carkit/perception/"
                "carkit_perception/models/yolo11n_fp16.engine"
            ),
            description="FP16 TensorRT engine exported on this Jetson.",
        ),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/aligned_depth_to_color/camera_info",
        ),
        DeclareLaunchArgument(
            "inference_image_topic",
            default_value="/yolo/inference_image",
        ),
        DeclareLaunchArgument(
            "detection_3d_topic",
            default_value="/yolo/detections_3d",
        ),
        DeclareLaunchArgument("min_confidence", default_value="0.2"),
        DeclareLaunchArgument("sync_slop", default_value="0.08"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(package_share, "rviz", "perception.rviz"),
        ),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        perception_3d_node,
        rviz_node,
    ])
