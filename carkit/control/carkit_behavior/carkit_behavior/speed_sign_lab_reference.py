"""Instructor reference for speed_sign_lab.py — do not import in the running stack."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from carkit_perception_msgs.msg import YoloDetection2D, YoloDetection2DArray

from carkit_behavior.speed_sign_lab import parse_speed_limit

if TYPE_CHECKING:
    from carkit_behavior.behavior_center_node import BehaviorCenterNode


def best_speed_sign_detection(
    node: BehaviorCenterNode,
    msg: YoloDetection2DArray,
) -> Optional[tuple[YoloDetection2D, float]]:
    candidates = []
    for detection in msg.detections:
        limit_mps = parse_speed_limit(detection.class_name)
        if limit_mps is None:
            continue
        if detection.confidence < node.speed_sign_min_confidence:
            continue
        candidates.append((detection, limit_mps))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0].confidence)


def speed_sign_boost_active(
    node: BehaviorCenterNode,
    now: float,
) -> tuple[bool, float]:
    from carkit_behavior.behavior_center_node import MapPoint

    track = node.reliable_speed_sign_track()
    if track is None:
        return False, 0.0

    if track.boost_until is not None and now < track.boost_until:
        return True, track.limit_mps

    if track.passed:
        return False, 0.0

    robot_position = node.robot_position_in_map()
    if robot_position is None:
        return False, 0.0

    remaining_distance = node.stop_line_path_distance(
        robot_position,
        MapPoint(track.x, track.y),
    )
    if remaining_distance is None:
        return False, 0.0

    previous_distance = track.last_distance_m
    track.last_distance_m = remaining_distance

    crossed_sign = (
        previous_distance is not None
        and previous_distance > 0.0
        and remaining_distance <= 0.0
    )
    if crossed_sign:
        track.passed = True
        track.boost_until = now + node.speed_sign_boost_duration_sec
        return True, track.limit_mps

    return False, 0.0
