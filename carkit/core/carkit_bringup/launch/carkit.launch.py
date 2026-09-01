#!/usr/bin/env python3

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
"""One hardware-neutral entry point for the complete CARKit stack."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_IMPLEMENTATIONS = {
    "reference", "ada_academy", "intro2av_python", "intro2av_cpp", "off"
}
COURSE_PACKAGES = {
    "ada_academy": "carkit_ada_academy",
    "intro2av_python": "carkit_intro2av",
    "intro2av_cpp": "carkit_intro2av_cpp",
}
VALID_CHASSIS = {"osracer", "f1tenth"}
VALID_PERCEPTION_MODELS = {
    "generic_coco", "traffic_signs", "combined", "custom"
}


def _include(package, launch_file, arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share) / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def _enabled(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _profile(context):
    name = _value(context, "profile")
    profile_path = (
        Path(get_package_share_directory("carkit_bringup"))
        / "config"
        / "profiles"
        / f"{name}.yaml"
    )
    if not profile_path.is_file():
        raise RuntimeError(
            f"Unknown CARKit profile {name!r}. Expected reference, "
            "ada_high_school, or intro2av."
        )
    with profile_path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _resolve(context, profile, name, section):
    requested = _value(context, name)
    if requested:
        return requested
    return str(profile[section][name]).lower()


def _reference_perception(context, detection_topic="/yolo/detections_2d"):
    return _include(
        "carkit_perception",
        "perception.launch.py",
        {
            "model_profile": _value(context, "perception_model"),
            "custom_model_path": _value(
                context, "custom_perception_model_path"
            ),
            "detection_2d_topic": detection_topic,
            "max_inference_rate_hz": "10.0",
        },
    )


def _course_parameters(implementation, component):
    """Load the student package's component config when it provides one."""
    package = COURSE_PACKAGES[implementation]
    path = Path(get_package_share_directory(package)) / "config" / f"{component}.yaml"
    return [str(path)] if path.is_file() else []


def _launch_stack(context):
    profile = _profile(context)
    chassis = _value(context, "chassis")
    if chassis not in VALID_CHASSIS:
        raise RuntimeError(f"Unsupported chassis {chassis!r}; choose osracer or f1tenth")

    components = {
        key: _enabled(
            _value(context, f"start_{key}")
            if _value(context, f"start_{key}") != "profile"
            else str(profile["components"][key]).lower()
        )
        for key in ("chassis", "sensors", "planning", "control", "perception", "behavior")
    }
    implementations = {
        key: _resolve(context, profile, key, "implementations")
        for key in ("planning", "control", "perception")
    }
    invalid = set(implementations.values()) - VALID_IMPLEMENTATIONS
    if invalid:
        raise RuntimeError(f"Invalid implementation selection: {sorted(invalid)}")
    perception_model = _value(context, "perception_model")
    if perception_model not in VALID_PERCEPTION_MODELS:
        raise RuntimeError(
            f"Invalid perception model selection: {perception_model}"
        )

    actions = [LogInfo(msg=(
        f"CARKit profile={profile['name']} chassis={chassis} "
        f"planning={implementations['planning']} "
        f"control={implementations['control']} "
        f"perception={implementations['perception']} "
        f"model={perception_model}"
    ))]

    # Platform-specific details end here. Everything downstream uses the
    # stable topics in config/interfaces.yaml.
    if components["chassis"]:
        # When the control center is active it must be the only publisher to
        # the hardware command topic. Manual /teleop still reaches it through
        # the arbiter; the platform's direct relay is parked on an unused topic.
        platform_command_topic = (
            "/carkit/manual_command_unused"
            if components["control"]
            else "/ackermann_cmd"
        )
        if chassis == "osracer":
            actions.append(_include(
                "osracer_bringup",
                "bringup_launch.py",
                {"vehicle_command_topic": platform_command_topic},
            ))
        else:
            actions.append(_include(
                "f1tenth_stack",
                "bringup_launch.py",
                {"vehicle_command_topic": platform_command_topic},
            ))

    if components["sensors"]:
        if chassis == "osracer":
            actions.append(_include(
                "osracer_bringup",
                "sensors_launch.py",
                {
                    "start_camera": _value(context, "start_camera"),
                    "start_lidar": _value(context, "start_lidar"),
                    "lidar_topic": "/scan/raw",
                },
            ))
        else:
            if _enabled(_value(context, "start_lidar")):
                actions.append(Node(
                    package="urg_node",
                    executable="urg_node_driver",
                    name="carkit_lidar",
                    output="screen",
                    parameters=[{"laser_frame_id": "laser"}],
                    remappings=[("scan", "/scan/raw")],
                ))
            if _enabled(_value(context, "start_camera")):
                actions.append(Node(
                    package="realsense2_camera",
                    executable="realsense2_camera_node",
                    namespace="camera",
                    name="camera",
                    output="screen",
                    parameters=[{
                        "enable_color": True,
                        "enable_depth": False,
                        "enable_infra": False,
                        "enable_gyro": False,
                        "enable_accel": False,
                    }],
                ))

        if _enabled(_value(context, "start_lidar")):
            actions.append(Node(
                package="carkit_scan_filter",
                executable="scan_footprint_filter_node",
                name="carkit_scan_footprint_filter",
                output="screen",
                parameters=[{
                    "input_topic": "/scan/raw",
                    "output_topic": "/scan",
                    "vehicle_length_m": 0.50,
                    "vehicle_width_m": 0.25,
                    "padding_m": 0.0,
                }],
            ))

    if components["planning"] and implementations["planning"] != "off":
        if implementations["planning"] == "reference":
            actions.append(_include(
                "carkit_navigation",
                "navigation.launch.py",
                {
                    "mode": _value(context, "navigation_mode"),
                    "map": _value(context, "map"),
                    "start_lidar": "false",
                    "start_cmd_bridge": (
                        "true" if implementations["control"] == "reference" else "false"
                    ),
                },
            ))
        else:
            actions.append(Node(
                package=COURSE_PACKAGES[implementations["planning"]],
                executable="planning_node",
                output="screen",
                parameters=_course_parameters(
                    implementations["planning"], "planning"
                ),
            ))

    if components["control"]:
        # The tested arbiter remains present for watchdog, clamping, manual
        # takeover, and E-stop even while students replace path tracking.
        actions.append(_include(
            "carkit_control_center", "control_center.launch.py"
        ))
        if implementations["control"] in COURSE_PACKAGES:
            actions.append(Node(
                package=COURSE_PACKAGES[implementations["control"]],
                executable="control_node",
                output="screen",
                parameters=_course_parameters(
                    implementations["control"], "control"
                ),
            ))

    if components["perception"] and implementations["perception"] != "off":
        if implementations["perception"] == "reference":
            actions.append(_reference_perception(context))
        elif implementations["perception"] == "ada_academy":
            # The protected detector supplies a working baseline. ADA students
            # own the package that filters its typed results.
            actions.append(_reference_perception(
                context,
                "/carkit/reference/detections_2d",
            ))
            actions.append(Node(
                package="carkit_ada_academy",
                executable="perception_node",
                output="screen",
            ))
        else:
            actions.append(Node(
                package=COURSE_PACKAGES[implementations["perception"]],
                executable="perception_node",
                output="screen",
                parameters=_course_parameters(
                    implementations["perception"], "perception"
                ),
            ))

    if components["behavior"]:
        actions.append(_include("carkit_behavior", "behavior_center.launch.py"))

    if _enabled(_value(context, "web_bridge")):
        actions.append(Node(
            package="carkit_web_bridge",
            executable="web_bridge_node",
            name="carkit_web_bridge",
            output="screen",
            parameters=[{
                "address": "0.0.0.0",
                "port": int(_value(context, "web_bridge_port")),
                "maximum_clients": 5,
            }],
        ))
    return actions


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    arguments = [
        DeclareLaunchArgument("profile", default_value="ada_high_school"),
        DeclareLaunchArgument("chassis", default_value="osracer"),
        DeclareLaunchArgument("start_chassis", default_value="profile"),
        DeclareLaunchArgument("start_sensors", default_value="profile"),
        DeclareLaunchArgument("start_planning", default_value="profile"),
        DeclareLaunchArgument("start_control", default_value="profile"),
        DeclareLaunchArgument("start_perception", default_value="profile"),
        DeclareLaunchArgument("start_behavior", default_value="profile"),
        DeclareLaunchArgument("planning", default_value=""),
        DeclareLaunchArgument("control", default_value=""),
        DeclareLaunchArgument("perception", default_value=""),
        DeclareLaunchArgument(
            "perception_model", default_value="combined"
        ),
        DeclareLaunchArgument(
            "custom_perception_model_path", default_value=""
        ),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("navigation_mode", default_value="navigation"),
        DeclareLaunchArgument("map", default_value="/workspaces/CARKit/map/map_3f.yaml"),
        DeclareLaunchArgument("web_bridge", default_value="true"),
        DeclareLaunchArgument("web_bridge_port", default_value="9090"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_stack)])
