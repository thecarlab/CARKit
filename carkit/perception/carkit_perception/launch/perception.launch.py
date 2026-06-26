import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def visualization_is(name, legacy_argument):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration("visualization"), "' == '", name, "' or ('",
        LaunchConfiguration("visualization"), "' == 'none' and '",
        LaunchConfiguration(legacy_argument), "' == 'true')",
    ]))


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
            "rgb_camera.color_profile": "640x480x15",
            "rgb_camera.color_format": "RGB8",
            "rgb_camera.global_time_enabled": False,
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
        condition=visualization_is("rviz", "start_rviz"),
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ])
        ),
        launch_arguments={
            "address": LaunchConfiguration("foxglove_address"),
            "port": LaunchConfiguration("foxglove_port"),
            "remote_access": LaunchConfiguration("foxglove_remote_access"),
            "device_token": LaunchConfiguration("foxglove_device_token"),
            "sysinfo": LaunchConfiguration("foxglove_sysinfo"),
            "topic_whitelist": LaunchConfiguration("foxglove_topic_whitelist"),
            "client_topic_whitelist": LaunchConfiguration(
                "foxglove_client_topic_whitelist"
            ),
            "param_whitelist": LaunchConfiguration("foxglove_param_whitelist"),
            "service_whitelist": LaunchConfiguration(
                "foxglove_service_whitelist"
            ),
            "capabilities": LaunchConfiguration("foxglove_capabilities"),
        }.items(),
        condition=visualization_is("foxglove", "start_foxglove_bridge"),
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
            description="RViz config for perception visualization.",
        ),
        DeclareLaunchArgument(
            "visualization",
            default_value="none",
            description=(
                "Perception visualization mode: foxglove, rviz, or none."
            ),
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description=(
                "Deprecated compatibility flag. Use visualization:=rviz."
            ),
        ),
        DeclareLaunchArgument(
            "start_foxglove_bridge",
            default_value="false",
            description=(
                "Deprecated compatibility flag. Use visualization:=foxglove."
            ),
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="0.0.0.0",
            description="Foxglove Bridge bind address.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove Bridge WebSocket port.",
        ),
        DeclareLaunchArgument(
            "foxglove_remote_access",
            default_value="false",
            description="Enable Foxglove remote access.",
        ),
        DeclareLaunchArgument(
            "foxglove_device_token",
            default_value="",
            description="Foxglove device token for remote access.",
        ),
        DeclareLaunchArgument(
            "foxglove_sysinfo",
            default_value="false",
            description="Publish system info through Foxglove Bridge.",
        ),
        DeclareLaunchArgument(
            "foxglove_topic_whitelist",
            default_value=(
                "['^/camera(/.*)?$', '^/yolo/.*$', '^/tf$', '^/tf_static$']"
            ),
            description="Foxglove Bridge topic whitelist.",
        ),
        DeclareLaunchArgument(
            "foxglove_client_topic_whitelist",
            default_value="['^$']",
            description="Topics Foxglove clients may publish.",
        ),
        DeclareLaunchArgument(
            "foxglove_param_whitelist",
            default_value="['^$']",
            description="Foxglove Bridge parameter whitelist.",
        ),
        DeclareLaunchArgument(
            "foxglove_service_whitelist",
            default_value="['^$']",
            description="Foxglove Bridge service whitelist.",
        ),
        DeclareLaunchArgument(
            "foxglove_capabilities",
            default_value="[connectionGraph]",
            description="Foxglove Bridge capabilities.",
        ),
        camera,
        perception_2d_node,
        rviz_node,
        foxglove_bridge,
    ])
