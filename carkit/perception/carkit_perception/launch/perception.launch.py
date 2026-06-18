import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("carkit_perception")

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[{
            "enable_color": True,
            "enable_depth": False,
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "enable_motion": False,
            "enable_rgbd": False,
            "enable_sync": False,
            "align_depth.enable": False,
            "pointcloud.enable": False,
        }],
    )

    perception_2d_node = Node(
        package="carkit_perception",
        executable="perception_2d_node",
        name="perception_2d_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "image_size": LaunchConfiguration("image_size"),
            "image_topic": LaunchConfiguration("image_topic"),
            "inference_image_topic": LaunchConfiguration(
                "inference_image_topic"
            ),
            "detection_2d_topic": LaunchConfiguration("detection_2d_topic"),
            "min_confidence": LaunchConfiguration("min_confidence"),
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
            "inference_image_topic",
            default_value="/yolo/inference_image",
        ),
        DeclareLaunchArgument(
            "detection_2d_topic",
            default_value="/yolo/detections_2d",
        ),
        DeclareLaunchArgument("min_confidence", default_value="0.2"),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start a color-only RealSense driver.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                package_share,
                "rviz",
                "perception.rviz",
            ),
        ),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        camera,
        perception_2d_node,
        rviz_node,
    ])
