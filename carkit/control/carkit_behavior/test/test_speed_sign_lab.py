import math
from types import SimpleNamespace

from carkit_perception_msgs.msg import YoloDetection2D, YoloDetection2DArray

from carkit_behavior.behavior_center_node import MapPoint, SpeedSignTrack
from carkit_behavior.speed_sign_lab import (
    best_speed_sign_detection,
    parse_speed_limit,
    speed_sign_boost_active,
)


def make_node():
    node = SimpleNamespace()
    node.speed_sign_min_confidence = 0.6
    node.speed_sign_boost_duration_sec = 3.0
    node.speed_sign_tracks = []
    node.get_logger = lambda: SimpleNamespace(info=lambda *args, **kwargs: None)
    return node


def detection(class_name, confidence=0.9):
    return YoloDetection2D(
        class_name=class_name,
        confidence=confidence,
        bbox_x_min=300.0,
        bbox_y_min=100.0,
        bbox_x_max=340.0,
        bbox_y_max=160.0,
    )


def detection_array(*items):
    msg = YoloDetection2DArray()
    msg.image_width = 640
    msg.image_height = 480
    msg.detections = list(items)
    return msg


def reliable_speed_sign_track(x, y, limit_mps=3.5):
    track = SpeedSignTrack(x, y, 0.9, limit_mps)
    track.observations = 3
    return track


def test_parse_speed_limit():
    assert parse_speed_limit("speed sign 3.5") == 3.5
    assert parse_speed_limit("stop sign") is None
    assert parse_speed_limit("speed sign bad") is None


def test_best_speed_sign_detection_picks_highest_confidence():
    node = make_node()
    msg = detection_array(
        detection("speed sign 2.0", confidence=0.7),
        detection("speed sign 4.0", confidence=0.95),
    )
    result = best_speed_sign_detection(node, msg)
    assert result is not None
    chosen, limit_mps = result
    assert chosen.confidence == 0.95
    assert limit_mps == 4.0


def test_speed_sign_boost_starts_when_sign_is_passed():
    node = make_node()
    track = reliable_speed_sign_track(5.0, 0.0, limit_mps=3.5)
    track.last_distance_m = 0.5
    node.speed_sign_tracks = [track]
    node.reliable_speed_sign_track = lambda: track
    node.robot_position_in_map = lambda: MapPoint(5.1, 0.0)
    node.stop_line_path_distance = lambda robot, sign: sign.x - robot.x

    active, limit_mps = speed_sign_boost_active(node, now=10.0)
    assert active
    assert limit_mps == 3.5
    assert track.passed
    assert track.boost_until == 13.0


def test_speed_sign_boost_holds_for_duration_after_pass():
    node = make_node()
    track = reliable_speed_sign_track(5.0, 0.0, limit_mps=3.5)
    track.passed = True
    track.boost_until = 13.0
    node.speed_sign_tracks = [track]
    node.reliable_speed_sign_track = lambda: track

    active, limit_mps = speed_sign_boost_active(node, now=12.0)
    assert active
    assert limit_mps == 3.5


def test_speed_sign_boost_expires_after_duration():
    node = make_node()
    track = reliable_speed_sign_track(5.0, 0.0, limit_mps=3.5)
    track.passed = True
    track.boost_until = 13.0
    node.speed_sign_tracks = [track]
    node.reliable_speed_sign_track = lambda: track

    active, limit_mps = speed_sign_boost_active(node, now=13.0)
    assert not active
    assert limit_mps == 0.0


def test_speed_sign_does_not_retrigger_before_pass():
    node = make_node()
    track = reliable_speed_sign_track(5.0, 0.0, limit_mps=3.5)
    track.last_distance_m = 2.0
    node.speed_sign_tracks = [track]
    node.reliable_speed_sign_track = lambda: track
    node.robot_position_in_map = lambda: MapPoint(3.0, 0.0)
    node.stop_line_path_distance = lambda robot, sign: sign.x - robot.x

    active, limit_mps = speed_sign_boost_active(node, now=1.0)
    assert not active
    assert limit_mps == 0.0
    assert not track.passed
