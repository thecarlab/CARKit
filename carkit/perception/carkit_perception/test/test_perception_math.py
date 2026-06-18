import numpy as np

from carkit_perception.perception_math import (
    TRAFFIC_LIGHT_GREEN,
    TRAFFIC_LIGHT_RED,
    TRAFFIC_LIGHT_UNKNOWN,
    TRAFFIC_LIGHT_YELLOW,
    TrafficLightClassifier,
)


def make_light(color, third):
    image = np.zeros((90, 30, 3), dtype=np.uint8)
    center_y = (15, 45, 75)[third]
    image[center_y - 8:center_y + 8, 7:23] = color
    return image


def test_classifies_vertical_traffic_light_colors():
    classifier = TrafficLightClassifier()
    assert classifier.classify(
        make_light((0, 0, 255), 0),
        (0, 0, 30, 90),
    ) == TRAFFIC_LIGHT_RED
    assert classifier.classify(
        make_light((0, 255, 255), 1),
        (0, 0, 30, 90),
    ) == TRAFFIC_LIGHT_YELLOW
    assert classifier.classify(
        make_light((0, 255, 0), 2),
        (0, 0, 30, 90),
    ) == TRAFFIC_LIGHT_GREEN


def test_returns_unknown_for_dark_crop():
    classifier = TrafficLightClassifier()
    image = np.zeros((90, 30, 3), dtype=np.uint8)
    assert classifier.classify(image, (0, 0, 30, 90)) == TRAFFIC_LIGHT_UNKNOWN
