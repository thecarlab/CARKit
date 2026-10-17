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

from carkit_bringup.hardware import load_hardware, navigation_arguments, scan_parameters
from carkit_bringup.hardware_launch import mounting_nodes, vesc_nodes, vesc_sensor_nodes


VALID_IMPLEMENTATIONS = {
    "reference", "ada_academy", "intro2av_python", "intro2av_cpp", "off"
}
COURSE_PACKAGES = {
    "ada_academy": "carkit_ada_academy",
    "intro2av_python": "carkit_intro2av",
    "intro2av_cpp": "carkit_intro2av_cpp",
}
VALID_CHASSIS = {"osracer", "f1tenth", "vesc_rplidar"}
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
            "image_topic": "/camera/camera/color/image_raw",
            "input_transport": "raw",
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
        raise RuntimeError(f"Unsupported chassis {chassis!r}; choose osracer, f1tenth or vesc_rplidar")

    hardware = load_hardware(chassis, _value(context, "hardware_config"))
    manual_input = _value(context, "manual_input")
    if manual_input not in {"keyboard", "joystick"}:
        raise ValueError("manual_input must be keyboard or joystick")

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

    # One command owner and one hardware profile, independent of vendor launches.
    if components["chassis"] and manual_input == "joystick":
        actions.append(_include("carkit_human_control", "input.launch.py", {
            **{key: str(hardware["joystick"][key]) for key in
               ("device_id", "speed_axis", "steering_axis", "mode_toggle_button")},
            "speed_scale": str(hardware["max_speed"]),
            "steering_scale": str(hardware["max_steering_angle"]),
            "max_reverse_speed": str(hardware["max_reverse_speed"]),
            "direct_relay": "false",
        }))
    if components["chassis"]:
        if chassis == "osracer":
            actions.append(_include("osracer_bringup", "bringup_launch.py", {
                "start_joystick": "false", "publish_sensor_tf": "false",
                "chassis_port": hardware["driver"]["port"],
                "chassis_baudrate": str(hardware["driver"]["baudrate"]),
                "protocol_mode": hardware["driver"]["protocol_mode"],
                "wheelbase": str(hardware["wheelbase"]),
                "forward_max_speed": str(hardware["max_speed"]),
                "reverse_max_speed": str(hardware["max_reverse_speed"]),
                "max_steering_angle": str(hardware["max_steering_angle"]),
            }))
        else:
            actions.extend(vesc_nodes(hardware))
        # Own odom TF independently of the planning implementation. Both VESC
        # drivers are configured not to publish it; the legacy driver defaults off.
        actions.append(Node(package="carkit_amcl", executable="odom_tf_broadcaster",
                            name="odom_tf_broadcaster", parameters=[{
                                "odom_topic": "/odom", "odom_frame": "odom",
                                "base_frame": "base_link", "use_message_stamp": True,
                            }]))

    if components["sensors"]:
        camera = _enabled(_value(context, "start_camera"))
        lidar = _enabled(_value(context, "start_lidar"))
        # VESC needs camera mounting TF for its IMU even with RGB video off.
        actions.extend(mounting_nodes(hardware, camera=camera or chassis != "osracer", lidar=lidar))
        if chassis == "osracer":
            actions.append(_include("osracer_bringup", "sensors_launch.py", {
                "start_camera": str(camera).lower(), "start_lidar": str(lidar).lower(),
                "lidar_topic": "/scan/raw", "camera_device": hardware["camera"]["device"],
                "lidar_sensor_ip": hardware["lidar"]["sensor_ip"],
                "lidar_host_ip": hardware["lidar"]["host_ip"],
                "lidar_port": str(hardware["lidar"]["port"]),
            }))
        else:
            actions.extend(vesc_sensor_nodes(hardware, camera=camera, lidar=lidar))
        if lidar:
            actions.append(Node(
                package="carkit_scan_filter", executable="scan_footprint_filter_node",
                name="carkit_scan_footprint_filter", output="screen", parameters=[{
                    "input_topic": "/scan/raw", "output_topic": "/scan",
                    "padding_m": 0.0, **scan_parameters(hardware),
                }]))

    if components["planning"] and implementations["planning"] != "off":
        if implementations["planning"] == "reference":
            actions.append(_include(
                "carkit_navigation",
                "navigation.launch.py",
                {
                    "mode": _value(context, "navigation_mode"),
                    "map": _value(context, "map"),
                    "start_lidar": "false",
                    "start_odom_tf": "false",
                    **navigation_arguments(hardware),
                    "start_cmd_bridge": (
                        "true" if components["control"] and implementations["control"] == "reference" else "false"
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

    # Even a manual-only session keeps the watchdog and E-stop arbiter.
    if components["control"] or components["chassis"]:
        # The tested arbiter remains present for watchdog, clamping, manual
        # takeover, and E-stop even while students replace path tracking.
        actions.append(_include(
            "carkit_control_center", "control_center.launch.py",
            {key: str(hardware[key]) for key in
             ("max_speed", "max_reverse_speed", "max_steering_angle")} |
            {key: str(hardware["joystick"][key]) for key in
             ("estop_button", "clear_estop_button")}
            | {"allow_autonomy": str(components["control"] and implementations["control"] != "off").lower()}
            | {"require_measured_odometry": str(chassis != "osracer").lower()}
        ))
        if components["control"] and implementations["control"] in COURSE_PACKAGES:
            actions.append(Node(
                package=COURSE_PACKAGES[implementations["control"]],
                executable="control_node",
                output="screen",
                parameters=_course_parameters(
                    implementations["control"], "control"
                ) + [{"wheelbase": float(hardware["wheelbase"]),
                      "wheelbase_m": float(hardware["wheelbase"]),
                      "target_speed": min(0.45, float(hardware["max_speed"])),
                      "target_speed_mps": min(0.45, float(hardware["max_speed"])),
                      "maximum_speed_mps": float(hardware["max_speed"]),
                      "max_steering_angle": float(hardware["max_steering_angle"]),
                      "maximum_steering_rad": float(hardware["max_steering_angle"])}],
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
                "address": _value(context, "web_bridge_address"),
                "port": int(_value(context, "web_bridge_port")),
                "maximum_clients": 5,
                **{f"footprint_{key}": float(value) for key, value in hardware["footprint"].items()},
                "laser_mount": [float(value) for value in hardware["mounts"]["laser"]],
                "manual_input": manual_input,
                "max_speed": float(hardware["max_speed"]),
                "max_reverse_speed": float(hardware["max_reverse_speed"]),
                "max_steering_angle": float(hardware["max_steering_angle"]),
            }],
        ))
    return actions


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    arguments = [
        DeclareLaunchArgument("profile", default_value="ada_high_school"),
        DeclareLaunchArgument("chassis", default_value="osracer"),
        DeclareLaunchArgument("manual_input", default_value="keyboard"),
        DeclareLaunchArgument("hardware_config", default_value="",
                             description="Optional per-vehicle hardware YAML"),
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
        DeclareLaunchArgument("web_bridge_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("web_bridge_port", default_value="9090"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_stack)])
