"""Speed sign student lab — fill in the STUDENT LAB sections below.

Rule: when the robot passes a tracked speed sign on the Nav2 path, publish
its speed value on /behavior/speed_limit for speed_sign_boost_duration_sec.

How to work on this file:
  Level A: uncomment the UNCOMMENT ME lines in each STUDENT LAB section.
  Level B: write the logic yourself using the step comments.
  Reference: speed_sign_lab_reference.py

Infrastructure (tracking, LiDAR, map transform) lives in behavior_center_node.py.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from carkit_perception_msgs.msg import YoloDetection2D, YoloDetection2DArray

if TYPE_CHECKING:
    from carkit_behavior.behavior_center_node import BehaviorCenterNode

SPEED_SIGN_PREFIX = "speed sign "


def parse_speed_limit(class_name: str) -> Optional[float]:
    """Return speed in m/s from 'speed sign 3.5', or None if not a speed sign."""
    if not class_name.startswith(SPEED_SIGN_PREFIX):
        return None
    try:
        return float(class_name[len(SPEED_SIGN_PREFIX) :])
    except ValueError:
        return None


def best_speed_sign_detection(
    node: BehaviorCenterNode,
    msg: YoloDetection2DArray,
) -> Optional[tuple[YoloDetection2D, float]]:
    """Pick the highest-confidence speed-sign detection above min confidence."""
    # =========================================================================
    # STUDENT LAB: best_speed_sign_detection
    #
    # 1. Loop msg.detections
    # 2. Parse limit with parse_speed_limit(class_name); skip if None
    # 3. Skip detections below node.speed_sign_min_confidence
    # 4. Return (detection, limit_mps) with highest confidence
    # =========================================================================
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
    # UNCOMMENT ME (one-liner alternative):
    # return max(
    #     (
    #         (d, limit)
    #         for d in msg.detections
    #         if (limit := parse_speed_limit(d.class_name)) is not None
    #         and d.confidence >= node.speed_sign_min_confidence
    #     ),
    #     key=lambda item: item[0].confidence,
    #     default=None,
    # )


def speed_sign_boost_active(
    node: BehaviorCenterNode,
    now: float,
) -> tuple[bool, float]:
    """Return (active, limit_mps) while the post-pass speed boost is running."""
    # =========================================================================
    # STUDENT LAB: speed_sign_boost_active
    #
    # 1. Get reliable track; return (False, 0.0) if missing
    # 2. If now < track.boost_until: still boosting → return (True, limit_mps)
    # 3. If track.passed and boost expired: return (False, 0.0)
    # 4. Get robot map position and remaining_distance along /plan to the sign
    # 5. Save previous_distance = track.last_distance_m, then update track
    # 6. Detect PASS: previous_distance > 0 and remaining_distance <= 0
    # 7. On pass: set track.passed, track.boost_until = now + duration,
    #    return (True, track.limit_mps)
    # =========================================================================
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
        node.get_logger().info(
            f"Speed sign passed -> boost to {track.limit_mps:.2f} m/s "
            f"for {node.speed_sign_boost_duration_sec:.1f} s"
        )
        return True, track.limit_mps

    return False, 0.0
