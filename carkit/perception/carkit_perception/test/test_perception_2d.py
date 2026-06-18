from types import SimpleNamespace

import numpy as np

from carkit_perception.perception_2d_node import Perception2DNode
from carkit_perception.perception_math import Detection2D, TRAFFIC_LIGHT_RED


def test_builds_color_only_detection_message():
    node = object.__new__(Perception2DNode)
    node.light_classifier = SimpleNamespace(
        classify=lambda image, bbox: TRAFFIC_LIGHT_RED
    )
    detection = Detection2D(
        class_id=9,
        class_name="traffic light",
        bbox=(10.0, 20.0, 30.0, 60.0),
        confidence=0.8,
    )
    message = node.to_traffic_light_message(
        detection,
        np.zeros((80, 80, 3), dtype=np.uint8),
    )
    assert message.detection.class_name == "traffic light"
    assert message.detection.bbox_y_max == 60.0
    assert message.traffic_light_color == TRAFFIC_LIGHT_RED


def test_regular_detection_has_no_traffic_light_color():
    node = object.__new__(Perception2DNode)
    detection = Detection2D(
        class_id=0,
        class_name="person",
        bbox=(10.0, 20.0, 30.0, 60.0),
        confidence=0.8,
    )
    message = node.to_detection_message(detection)
    assert not hasattr(message, "traffic_light_color")
