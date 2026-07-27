#!/usr/bin/env python3
"""Restore saved CARKit waypoints through the Foxglove waypoint interface."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import time
from typing import Any, Optional, Sequence

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


GOAL_TOPIC = "/foxglove/waypoints/goal"
COMMAND_TOPIC = "/foxglove/waypoints/command"
MAXIMUM_POSES = 50


def finite_number(value: Any, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} must be a number") from error
    if not math.isfinite(number):
        raise ValueError(f"{field} must be finite")
    return number


def load_waypoints(input_path: Path) -> list[dict[str, Any]]:
    try:
        document = json.loads(input_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {input_path}: {error}") from error

    if (
        not isinstance(document, dict)
        or document.get("format") != "carkit_foxglove_waypoints"
        or document.get("version") != 1
    ):
        raise ValueError("unsupported waypoint JSON format or version")

    raw_waypoints = document.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoint file contains no waypoints")
    if len(raw_waypoints) > MAXIMUM_POSES:
        raise ValueError(
            f"waypoint file contains {len(raw_waypoints)} poses; "
            f"the current limit is {MAXIMUM_POSES}"
        )

    waypoints: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_waypoints, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"waypoint {index} must be an object")
        frame_id = str(raw.get("frame_id", "")).strip() or "map"
        position = raw.get("position")
        orientation = raw.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            raise ValueError(
                f"waypoint {index} requires position and orientation objects"
            )
        normalized = {
            "frame_id": frame_id,
            "position": {
                axis: finite_number(
                    position.get(axis),
                    f"waypoint {index} position.{axis}",
                )
                for axis in ("x", "y", "z")
            },
            "orientation": {
                axis: finite_number(
                    orientation.get(axis),
                    f"waypoint {index} orientation.{axis}",
                )
                for axis in ("x", "y", "z", "w")
            },
        }
        quaternion = normalized["orientation"]
        norm = math.sqrt(sum(value * value for value in quaternion.values()))
        if norm < 1.0e-9:
            raise ValueError(f"waypoint {index} orientation is a zero quaternion")
        for axis in quaternion:
            quaternion[axis] /= norm
        waypoints.append(normalized)
    return waypoints


def pose_message(
    node: Node,
    waypoint: dict[str, Any],
) -> PoseStamped:
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = waypoint["frame_id"]
    position = waypoint["position"]
    orientation = waypoint["orientation"]
    message.pose.position.x = position["x"]
    message.pose.position.y = position["y"]
    message.pose.position.z = position["z"]
    message.pose.orientation.x = orientation["x"]
    message.pose.orientation.y = orientation["y"]
    message.pose.orientation.z = orientation["z"]
    message.pose.orientation.w = orientation["w"]
    return message


def wait_for_subscriber(
    node: Node,
    publisher,
    topic: str,
    timeout_sec: float,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"No subscriber found on {topic} within {timeout_sec:.1f} seconds.")
    return False


def parse_arguments(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish saved poses to foxglove_waypoints so their numbered "
            "markers reappear without manually setting each pose."
        )
    )
    parser.add_argument("input", type=Path, help="Waypoint JSON file to restore")
    parser.add_argument(
        "--goal-topic",
        default=GOAL_TOPIC,
        help=f"PoseStamped destination (default: {GOAL_TOPIC})",
    )
    parser.add_argument(
        "--command-topic",
        default=COMMAND_TOPIC,
        help=f"Waypoint command topic (default: {COMMAND_TOPIC})",
    )
    publish_mode = parser.add_mutually_exclusive_group()
    publish_mode.add_argument(
        "--clear",
        action="store_true",
        help="Clear existing pending waypoints before restoring (the default)",
    )
    publish_mode.add_argument(
        "--append",
        action="store_true",
        help="Append to existing pending waypoints instead of replacing them",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Delay between poses in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for foxglove_waypoints (default: 5)",
    )
    parsed = parser.parse_args(arguments)
    if parsed.interval < 0.0:
        parser.error("--interval cannot be negative")
    if parsed.wait_timeout <= 0.0:
        parser.error("--wait-timeout must be greater than zero")
    return parsed


def main(arguments: Optional[Sequence[str]] = None) -> int:
    parsed = parse_arguments(arguments)
    try:
        waypoints = load_waypoints(parsed.input)
    except ValueError as error:
        print(f"Failed to load waypoints: {error}")
        return 1

    rclpy.init(args=[])
    node = Node("publish_foxglove_waypoints")
    goal_publisher = node.create_publisher(PoseStamped, parsed.goal_topic, 10)
    command_publisher = node.create_publisher(String, parsed.command_topic, 10)
    try:
        if not wait_for_subscriber(
            node,
            goal_publisher,
            parsed.goal_topic,
            parsed.wait_timeout,
        ):
            return 1

        if not parsed.append:
            if not wait_for_subscriber(
                node,
                command_publisher,
                parsed.command_topic,
                parsed.wait_timeout,
            ):
                return 1
            command = String()
            command.data = "clear"
            command_publisher.publish(command)
            time.sleep(0.3)

        for index, waypoint in enumerate(waypoints, start=1):
            message = pose_message(node, waypoint)
            goal_publisher.publish(deepcopy(message))
            position = waypoint["position"]
            print(
                f"Published {index}/{len(waypoints)}: "
                f"frame={waypoint['frame_id']} "
                f"x={position['x']:.6f} y={position['y']:.6f}"
            )
            if parsed.interval > 0.0 and index < len(waypoints):
                time.sleep(parsed.interval)

        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(
        f"Restored {len(waypoints)} waypoint(s). "
        "They are marked in Foxglove but navigation was not started."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
