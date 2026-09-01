from types import SimpleNamespace
from unittest import mock

import numpy as np
from std_msgs.msg import Header
import torch

from carkit_perception.perception_2d_node import Perception2DNode
from carkit_perception.perception_2d_node import TRAFFIC_SIGN_CLASS_NAMES
from carkit_perception.perception_math import Detection2D, TRAFFIC_LIGHT_RED


def test_inference_rate_limit_drops_intermediate_camera_frames():
    node = object.__new__(Perception2DNode)
    node.max_inference_rate_hz = 8.0
    node.last_inference_monotonic = None

    with mock.patch(
        "carkit_perception.perception_2d_node.time.monotonic",
        side_effect=[1.0, 1.05, 1.13],
    ):
        assert node.inference_due()
        assert not node.inference_due()
        assert node.inference_due()


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


def test_extract_detections_uses_traffic_sign_class_overrides():
    node = object.__new__(Perception2DNode)
    traffic_sign_model = SimpleNamespace(names={0: "class_0"})
    boxes = SimpleNamespace(
        data=torch.tensor([[10.0, 20.0, 30.0, 60.0, 0.8, 0.0]])
    )

    detections = node.extract_detections(
        [SimpleNamespace(boxes=boxes)],
        traffic_sign_model,
        TRAFFIC_SIGN_CLASS_NAMES,
    )

    assert len(detections) == 1
    assert detections[0].class_id == 0
    assert detections[0].class_name == "speed_sign"


def test_inference_image_is_not_rendered_without_subscribers():
    node = object.__new__(Perception2DNode)
    node.image_pub = SimpleNamespace(
        get_subscription_count=lambda: 0,
        publish=lambda message: (_ for _ in ()).throw(
            AssertionError("image should not be published")
        ),
    )
    node.compressed_image_pub = SimpleNamespace(
        get_subscription_count=lambda: 0,
    )
    node.latest_raw_inference_image = None
    node.latest_compressed_inference_image = None
    node.bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("image should not be converted")
        )
    )
    node.publish_inference_image(
        np.zeros((2, 2, 3), dtype=np.uint8),
        [],
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
    node.compressed_image_pub = SimpleNamespace(
        get_subscription_count=lambda: 0,
    )
    node.latest_detection_output = None
    node.latest_raw_inference_image = None
    node.latest_compressed_inference_image = None
    node.bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda image, encoding: annotated_message
    )
    header = Header(frame_id="camera")
    node.draw_detections = lambda image, detections, color: None

    node.publish_inference_image(
        np.zeros((2, 2, 3), dtype=np.uint8),
        [],
        SimpleNamespace(header=header),
    )
    node.publish_latest_results()

    assert published == [annotated_message]
    assert annotated_message.header is header


def test_inference_image_can_be_published_as_browser_ready_jpeg():
    published = []
    node = object.__new__(Perception2DNode)
    node.image_pub = SimpleNamespace(get_subscription_count=lambda: 0)
    node.compressed_image_pub = SimpleNamespace(
        get_subscription_count=lambda: 1,
        publish=published.append,
    )
    node.latest_detection_output = None
    node.latest_raw_inference_image = None
    node.latest_compressed_inference_image = None
    node.inference_jpeg_quality = 70
    node.draw_detections = lambda image, detections, color: None
    header = Header(frame_id="camera")

    node.publish_inference_image(
        np.zeros((16, 16, 3), dtype=np.uint8),
        [],
        SimpleNamespace(header=header),
    )
    node.publish_latest_results()

    assert len(published) == 1
    assert published[0].header is header
    assert published[0].format == "jpeg"
    assert bytes(published[0].data).startswith(b"\xff\xd8")
