#!/usr/bin/env python3
"""Save the currently displayed CARKit Foxglove waypoints to JSON."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import Any, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


MARKER_TOPIC = "/foxglove/waypoints/markers"
WAYPOINT_NAMESPACES = {
    "foxglove_waypoints_active": "active",
    "foxglove_waypoints_pending": "pending",
}


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return repository_root() / "test" / f"foxglove_waypoints_{timestamp}.json"


def yaw_degrees(marker: Marker) -> float:
    orientation = marker.pose.orientation
    siny_cosp = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def extract_waypoints(message: MarkerArray) -> list[dict[str, Any]]:
    arrows = [
        marker
        for marker in message.markers
        if marker.action == Marker.ADD
        and marker.type == Marker.ARROW
        and marker.ns in WAYPOINT_NAMESPACES
    ]
    arrows.sort(key=lambda marker: marker.id)

    waypoints: list[dict[str, Any]] = []
    for index, marker in enumerate(arrows, start=1):
        position = marker.pose.position
        orientation = marker.pose.orientation
        waypoints.append(
            {
                "index": index,
                "state_when_saved": WAYPOINT_NAMESPACES[marker.ns],
                "frame_id": marker.header.frame_id or "map",
                "position": {
                    "x": float(position.x),
                    "y": float(position.y),
                    "z": float(position.z),
                },
                "orientation": {
                    "x": float(orientation.x),
                    "y": float(orientation.y),
                    "z": float(orientation.z),
                    "w": float(orientation.w),
                },
                "yaw_deg": yaw_degrees(marker),
            }
        )
    return waypoints


class WaypointSnapshot(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("save_foxglove_waypoints")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.waypoints: Optional[list[dict[str, Any]]] = None
        self.create_subscription(MarkerArray, topic, self._callback, qos)

    def _callback(self, message: MarkerArray) -> None:
        waypoints = extract_waypoints(message)
        if waypoints:
            self.waypoints = waypoints


def write_snapshot(
    output_path: Path,
    marker_topic: str,
    waypoints: list[dict[str, Any]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "format": "carkit_foxglove_waypoints",
        "version": 1,
        "saved_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "source_topic": marker_topic,
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
    }
    temporary_path = output_path.with_name(f".{output_path.name}.tmp")
    temporary_path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(output_path)


def parse_arguments(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Save active and pending waypoints currently displayed by "
            "foxglove_waypoints."
        )
    )
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=None,
        help="Output JSON path (default: test/foxglove_waypoints_<time>.json)",
    )
    parser.add_argument(
        "--marker-topic",
        default=MARKER_TOPIC,
        help=f"MarkerArray topic (default: {MARKER_TOPIC})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for a non-empty marker snapshot (default: 5)",
    )
    parsed = parser.parse_args(arguments)
    if parsed.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")
    return parsed


def main(arguments: Optional[Sequence[str]] = None) -> int:
    parsed = parse_arguments(arguments)
    output_path = parsed.output or default_output_path()

    rclpy.init(args=[])
    node = WaypointSnapshot(parsed.marker_topic)
    deadline = time.monotonic() + parsed.timeout
    try:
        while (
            rclpy.ok()
            and node.waypoints is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not node.waypoints:
        print(
            f"No Foxglove waypoints received from {parsed.marker_topic} "
            f"within {parsed.timeout:.1f} seconds.",
        )
        return 1

    try:
        write_snapshot(
            output_path,
            parsed.marker_topic,
            node.waypoints,
        )
    except OSError as error:
        print(f"Failed to save waypoints: {error}")
        return 1

    print(f"Saved {len(node.waypoints)} waypoint(s) to {output_path.resolve()}")
    for waypoint in node.waypoints:
        position = waypoint["position"]
        print(
            f"  {waypoint['index']}: frame={waypoint['frame_id']} "
            f"x={position['x']:.6f} y={position['y']:.6f} "
            f"yaw={waypoint['yaw_deg']:.2f} deg "
            f"({waypoint['state_when_saved']})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
