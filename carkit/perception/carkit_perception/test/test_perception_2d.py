from types import SimpleNamespace

import numpy as np
import torch

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


def test_extract_detections_preserves_stop_sign_from_single_table_transfer():
    node = object.__new__(Perception2DNode)
    node.model = SimpleNamespace(names={11: "stop sign"})
    boxes = SimpleNamespace(
        data=torch.tensor([[10.0, 20.0, 30.0, 60.0, 0.8, 11.0]])
    )

    detections = node.extract_detections([SimpleNamespace(boxes=boxes)])

    assert len(detections) == 1
    assert detections[0].class_id == 11
    assert detections[0].class_name == "stop sign"
    assert detections[0].bbox == (10.0, 20.0, 30.0, 60.0)
    assert abs(detections[0].confidence - 0.8) < 1.0e-6


def test_inference_image_is_not_rendered_without_subscribers():
    node = object.__new__(Perception2DNode)
    node.image_pub = SimpleNamespace(
        get_subscription_count=lambda: 0,
        publish=lambda message: (_ for _ in ()).throw(
            AssertionError("image should not be published")
        ),
    )
    node.bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("image should not be converted")
        )
    )
    result = SimpleNamespace(
        plot=lambda: (_ for _ in ()).throw(
            AssertionError("inference image should not be rendered")
        )
    )

    node.publish_inference_image(
        [result],
        SimpleNamespace(header=SimpleNamespace()),
    )


def test_inference_image_is_still_published_with_a_subscriber():
    published = []
    annotated_message = SimpleNamespace(header=None)
    node = object.__new__(Perception2DNode)
    node.image_pub = SimpleNamespace(
        get_subscription_count=lambda: 1,
        publish=published.append,
    )
    node.bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda image, encoding: annotated_message
    )
    header = SimpleNamespace(frame_id="camera")
    result = SimpleNamespace(plot=lambda: np.zeros((2, 2, 3), dtype=np.uint8))

    node.publish_inference_image([result], SimpleNamespace(header=header))

    assert published == [annotated_message]
    assert annotated_message.header is header
