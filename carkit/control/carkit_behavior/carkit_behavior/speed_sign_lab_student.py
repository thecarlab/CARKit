"""Student starter — replace speed_sign_lab.py with this file for the lab assignment.

Only fill in the STUDENT LAB sections. Do not edit behavior_center_node.py.
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
    # =========================================================================
    # STUDENT LAB: best_speed_sign_detection
    # =========================================================================
    # TODO: loop detections, parse limit, filter by confidence, return best
    pass
    return None


def speed_sign_boost_active(
    node: BehaviorCenterNode,
    now: float,
) -> tuple[bool, float]:
    # =========================================================================
    # STUDENT LAB: speed_sign_boost_active
    # =========================================================================
    # TODO: detect path crossing, start 3-second boost, return (active, limit_mps)
    pass
    return False, 0.0
