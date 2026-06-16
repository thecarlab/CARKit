import numpy as np

from carkit_perception.perception_math import (
    TRAFFIC_LIGHT_GREEN,
    TRAFFIC_LIGHT_RED,
    TRAFFIC_LIGHT_UNKNOWN,
    TRAFFIC_LIGHT_YELLOW,
    TrafficLightClassifier,
    median_detection_depth,
    project_pixel_to_camera,
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


def test_depth_uses_center_median_and_millimeter_conversion():
    depth = np.zeros((20, 20), dtype=np.uint16)
    depth[8:12, 8:12] = 1500
    result = median_detection_depth(
        depth,
        "16UC1",
        (5, 5, 15, 15),
        0.1,
        10.0,
    )
    assert result == 1.5


def test_invalid_depth_returns_none():
    depth = np.zeros((20, 20), dtype=np.uint16)
    assert median_detection_depth(
        depth,
        "16UC1",
        (5, 5, 15, 15),
        0.1,
        10.0,
    ) is None


def test_projects_to_optical_camera_frame():
    x, y, z = project_pixel_to_camera(
        (60.0, 40.0, 80.0, 60.0),
        2.0,
        100.0,
        100.0,
        50.0,
        50.0,
    )
    assert (x, y, z) == (0.4, 0.0, 2.0)
