# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    package_share = get_package_share_directory("carkit_perception")

    perception_2d_node = Node(
        package="carkit_perception",
        executable="perception_2d_node",
        name="perception_2d_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "traffic_sign_model_path": LaunchConfiguration(
                "traffic_sign_model_path"
            ),
            "model_profile": LaunchConfiguration("model_profile"),
            "custom_model_path": LaunchConfiguration("custom_model_path"),
            "image_size": LaunchConfiguration("image_size"),
            "image_topic": LaunchConfiguration("image_topic"),
            "input_transport": LaunchConfiguration("input_transport"),
            "inference_image_topic": LaunchConfiguration(
                "inference_image_topic"
            ),
            "inference_compressed_topic": LaunchConfiguration(
                "inference_compressed_topic"
            ),
            "detection_2d_topic": LaunchConfiguration("detection_2d_topic"),
            "min_confidence": LaunchConfiguration("min_confidence"),
            "traffic_sign_min_confidence": LaunchConfiguration(
                "traffic_sign_min_confidence"
            ),
            "max_inference_rate_hz": LaunchConfiguration(
                "max_inference_rate_hz"
            ),
            "secondary_inference_interval": LaunchConfiguration(
                "secondary_inference_interval"
            ),
            "inference_jpeg_quality": LaunchConfiguration(
                "inference_jpeg_quality"
            ),
        }],
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
        DeclareLaunchArgument(
            "traffic_sign_model_path",
            default_value=os.path.join(
                package_share,
                "models",
                "traffic_sign_1_fp16.engine",
            ),
            description="Traffic-sign FP16 TensorRT engine path.",
        ),
        DeclareLaunchArgument(
            "model_profile",
            default_value="combined",
            description=(
                "Model selection: generic_coco, traffic_signs, combined, "
                "or custom."
            ),
        ),
        DeclareLaunchArgument("custom_model_path", default_value=""),
        DeclareLaunchArgument("image_size", default_value="448"),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera/color/image_raw/compressed",
        ),
        DeclareLaunchArgument("input_transport", default_value="compressed"),
        DeclareLaunchArgument(
            "inference_image_topic",
            default_value="/yolo/inference_image",
        ),
        DeclareLaunchArgument(
            "inference_compressed_topic",
            default_value="/yolo/inference_image/compressed",
        ),
        DeclareLaunchArgument(
            "detection_2d_topic",
            default_value="/yolo/detections_2d",
        ),
        DeclareLaunchArgument("min_confidence", default_value="0.2"),
        DeclareLaunchArgument(
            "traffic_sign_min_confidence",
            default_value="0.2",
        ),
        DeclareLaunchArgument(
            "max_inference_rate_hz",
            default_value="10.0",
            description="Target perception output rate.",
        ),
        DeclareLaunchArgument(
            "secondary_inference_interval",
            default_value="2",
            description="Run the optional secondary model every N frames.",
        ),
        DeclareLaunchArgument("inference_jpeg_quality", default_value="70"),
        perception_2d_node,
    ])
