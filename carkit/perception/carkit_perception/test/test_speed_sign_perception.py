import importlib
import sys
import types
from types import SimpleNamespace

import numpy as np


class FakeYoloDetection2D:
    pass


class FakeYoloDetection2DArray:
    pass


def install_runtime_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.ok = lambda: False
    sys.modules["rclpy"] = rclpy

    executors = types.ModuleType("rclpy.executors")
    executors.ExternalShutdownException = RuntimeError
    sys.modules["rclpy.executors"] = executors

    node = types.ModuleType("rclpy.node")
    node.Node = object
    sys.modules["rclpy.node"] = node

    qos = types.ModuleType("rclpy.qos")
    qos.HistoryPolicy = SimpleNamespace(KEEP_LAST=1)
    qos.QoSProfile = object
    qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
    sys.modules["rclpy.qos"] = qos

    cv_bridge = types.ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    sys.modules["cv_bridge"] = cv_bridge

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = object
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    ultralytics = types.ModuleType("ultralytics")
    ultralytics.YOLO = object
    sys.modules["ultralytics"] = ultralytics

    perception_msgs = types.ModuleType("carkit_perception_msgs")
    perception_msgs_msg = types.ModuleType("carkit_perception_msgs.msg")
    perception_msgs_msg.YoloDetection2D = FakeYoloDetection2D
    perception_msgs_msg.YoloDetection2DArray = FakeYoloDetection2DArray
    sys.modules["carkit_perception_msgs"] = perception_msgs
    sys.modules["carkit_perception_msgs.msg"] = perception_msgs_msg


install_runtime_stubs()

perception_math = importlib.import_module("carkit_perception.perception_math")
speed_sign_node = importlib.import_module(
    "carkit_perception.speed_sign_perception_node"
)
Detection2D = perception_math.Detection2D
SpeedSignPerceptionNode = speed_sign_node.SpeedSignPerceptionNode


def test_default_model_path_uses_installed_traffic_sign_weight(
    monkeypatch,
    tmp_path,
):
    installed_weight = tmp_path / "share" / "models" / "traffic_sign.pt"
    installed_weight.parent.mkdir(parents=True)
    installed_weight.touch()

    ament_index = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")
    packages.PackageNotFoundError = RuntimeError
    packages.get_package_share_directory = lambda package_name: str(
        installed_weight.parents[1]
    )
    monkeypatch.setitem(sys.modules, "ament_index_python", ament_index)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)

    assert speed_sign_node.default_model_path() == str(installed_weight)


def test_speed_sign_detection_message_uses_existing_message_type():
    node = object.__new__(SpeedSignPerceptionNode)
    detection = Detection2D(
        class_id=0,
        class_name="speed_sign",
        bbox=(10.0, 20.0, 30.0, 60.0),
        confidence=0.8,
    )

    message = node.to_detection_message(detection)

    assert message.class_id == 0
    assert message.class_name == "speed_sign"
    assert message.bbox_y_max == 60.0


def test_extract_detections_reads_speed_sign_model_names():
    node = object.__new__(SpeedSignPerceptionNode)
    node.model = SimpleNamespace(names={0: "speed_sign", 1: "traffic_cone"})
    rows = np.array([
        [10.0, 20.0, 30.0, 60.0, 0.8, 0.0],
        [100.0, 120.0, 130.0, 160.0, 0.7, 1.0],
    ])
    data = SimpleNamespace(
        numel=lambda: rows.size,
        detach=lambda: SimpleNamespace(
            cpu=lambda: SimpleNamespace(numpy=lambda: rows)
        ),
    )
    boxes = SimpleNamespace(data=data)

    detections = node.extract_detections([SimpleNamespace(boxes=boxes)])

    assert len(detections) == 2
    assert detections[0].class_id == 0
    assert detections[0].class_name == "speed_sign"
    assert detections[0].bbox == (10.0, 20.0, 30.0, 60.0)
    assert abs(detections[0].confidence - 0.8) < 1.0e-6
    assert detections[1].class_id == 1
    assert detections[1].class_name == "traffic_cone"
    assert detections[1].bbox == (100.0, 120.0, 130.0, 160.0)


def test_annotate_image_draws_class_id_bbox():
    node = object.__new__(SpeedSignPerceptionNode)
    detection = Detection2D(
        class_id=1,
        class_name="traffic_cone",
        bbox=(10.0, 20.0, 30.0, 60.0),
        confidence=0.7,
    )
    image = np.zeros((80, 100, 3), dtype=np.uint8)

    annotated = node.annotate_image(image, [detection])

    assert annotated[20, 10].tolist() == [0, 170, 255]
