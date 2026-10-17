import time
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
    node.output_received_at = time.monotonic()
    node.output_timeout_sec = 0.5
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
    node.output_received_at = time.monotonic()
    node.output_timeout_sec = 0.5
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


def test_camera_loss_does_not_republish_cached_detections():
    node = object.__new__(Perception2DNode)
    published = []
    node.detection_pub = SimpleNamespace(publish=published.append)
    node.latest_detection_output = object()
    node.latest_raw_inference_image = None
    node.latest_compressed_inference_image = None
    node.output_received_at = time.monotonic()
    node.output_timeout_sec = .5
    for _ in range(100):
        node.publish_latest_results()
    assert len(published) == 1
    node.latest_detection_output = object()
    node.output_received_at = time.monotonic() - 1
    node.publish_latest_results()
    assert len(published) == 1


def test_rate_limit_preserves_ten_hz_from_fifteen_fps_camera():
    node = object.__new__(Perception2DNode)
    node.max_inference_rate_hz = 10.0
    node.last_inference_monotonic = None
    with mock.patch("carkit_perception.perception_2d_node.time.monotonic",
                    side_effect=[1.0 + index / 15 for index in range(90)]):
        assert sum(node.inference_due() for _ in range(90)) == 60


def test_rate_limit_skips_missed_slots_without_catchup_burst():
    node = object.__new__(Perception2DNode)
    node.max_inference_rate_hz = 10.0
    node.last_inference_monotonic = None
    with mock.patch("carkit_perception.perception_2d_node.time.monotonic",
                    side_effect=[1.0, 3.04, 3.05, 3.1]):
        assert [node.inference_due() for _ in range(4)] == [True, True, False, True]


def _callback_node(published):
    node = object.__new__(Perception2DNode)
    node.output_timeout_sec = .5
    node.last_source_stamp = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    node.inference_due = lambda: True
    node.input_transport = "raw"
    node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: np.zeros((2, 2, 3), dtype=np.uint8))
    node.model = SimpleNamespace(predict=lambda *a, **k: [])
    node.image_size = 448
    node.min_confidence = .2
    node.primary_class_names = {}
    node.extract_detections = lambda *a: []
    node.frame_sequence = 0
    node.secondary_model_enabled = False
    node.publish_inference_image = lambda *a: None
    node.latest_raw_inference_image = None
    node.latest_compressed_inference_image = None
    node.detection_pub = SimpleNamespace(publish=published.append)
    return node


def test_completed_inference_publishes_without_waiting_for_another_timer():
    published = []
    node = _callback_node(published)
    header = Header()
    header.stamp.sec = 10
    with mock.patch("carkit_perception.perception_2d_node.time.monotonic", side_effect=[1.0, 1.02]):
        node.image_callback(SimpleNamespace(header=header))
    assert len(published) == 1
    assert published[0].header.stamp.sec == 10
    assert node.latest_detection_output is None


def test_slow_inference_cannot_publish_an_expired_result():
    published = []
    node = _callback_node(published)
    header = Header()
    header.stamp.sec = 10
    with mock.patch("carkit_perception.perception_2d_node.time.monotonic", side_effect=[1.0, 1.6]):
        node.image_callback(SimpleNamespace(header=header))
    assert published == []


def test_out_of_order_camera_frame_does_not_run_inference():
    node = _callback_node([])
    node.last_source_stamp = 9_900_000_000
    node.inference_due = lambda: (_ for _ in ()).throw(AssertionError("old frame reached inference"))
    header = Header()
    header.stamp.sec = 9
    header.stamp.nanosec = 800_000_000
    node.image_callback(SimpleNamespace(header=header))
