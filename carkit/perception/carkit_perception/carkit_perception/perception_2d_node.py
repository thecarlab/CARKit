#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
# Licensed under the Apache License, Version 2.0 (the "License");

import hashlib
import json
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from ultralytics import YOLO
import ultralytics

from carkit_perception.perception_math import (
    Detection2D,
    TrafficLightClassifier,
)
from carkit_perception_msgs.msg import (
    PerceptionLatencyTrace,
    YoloDetection2D,
    YoloDetection2DArray,
    YoloTrafficLightDetection2D,
)


# Retained for legacy extract_detections callers; the standard node does not
# load or run a traffic-sign model.
TRAFFIC_SIGN_CLASS_NAMES = {
    0: "speed_sign",
    1: "traffic_cone",
}

# COCO class IDs used by the single general-purpose YOLO model.
TRAFFIC_LIGHT_CLASS_ID = 9
STOP_SIGN_CLASS_ID = 11
TARGET_CLASS_IDS = (TRAFFIC_LIGHT_CLASS_ID, STOP_SIGN_CLASS_ID)
AUTO_DRIVE = "AUTO_DRIVE"


class Perception2DNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_2d_node")

        self.declare_parameter(
            "model_path",
            (
                "/workspaces/CARKit/carkit/perception/"
                "carkit_perception/models/yolo11n_fp16.engine"
            ),
        )
        self.declare_parameter("image_size", 640)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "inference_image_topic",
            "/yolo/inference_image",
        )
        self.declare_parameter("detection_2d_topic", "/yolo/detections_2d")
        self.declare_parameter("min_confidence", 0.2)
        self.declare_parameter("require_engine_metadata", True)

        self.model_path = str(self.get_parameter("model_path").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        self._validate_fp16_engine()
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")
        self._validate_target_classes()
        self.light_classifier = TrafficLightClassifier()
        self.main_state = ""

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        control_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.control_state_sub = self.create_subscription(
            String,
            "/control_center/main_state",
            self.main_state_callback,
            control_state_qos,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            sensor_qos,
        )
        self.image_rate_pulse_pub = self.create_publisher(
            Empty,
            "/monitor/rate/image_raw_rx",
            sensor_qos,
        )
        self.rate_pulse_msg = Empty()
        self.detection_pub = self.create_publisher(
            YoloDetection2DArray,
            str(self.get_parameter("detection_2d_topic").value),
            10,
        )
        self.latency_trace_pub = self.create_publisher(
            PerceptionLatencyTrace,
            "/perception/latency_trace",
            10,
        )
        self.detection_sequence = 0
        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("inference_image_topic").value),
            sensor_qos,
        )

        self.get_logger().info(
            f"Loaded FP16 TensorRT model {self.model_path}; "
            "detecting stop signs and traffic lights in AUTO_DRIVE only; "
            "publishing "
            f"{self.get_parameter('detection_2d_topic').value}"
        )

    def main_state_callback(self, msg: String) -> None:
        self.main_state = msg.data

    def _validate_target_classes(self) -> None:
        expected_names = {
            TRAFFIC_LIGHT_CLASS_ID: "traffic light",
            STOP_SIGN_CLASS_ID: "stop sign",
        }
        for class_id, expected_name in expected_names.items():
            actual_name = self.class_name(self.model, class_id, {})
            if actual_name != expected_name:
                raise RuntimeError(
                    "YOLO model does not provide the expected COCO class: "
                    f"id {class_id} must be '{expected_name}', got "
                    f"'{actual_name}'"
                )

    def _validate_fp16_engine(self) -> None:
        engine_path = Path(self.model_path)
        if engine_path.suffix != ".engine":
            raise RuntimeError(
                "model_path must point to an FP16 TensorRT .engine file"
            )
        if not engine_path.is_file():
            raise RuntimeError(
                f"TensorRT engine not found: {engine_path}. "
                "Run export_fp16_engine on this Jetson first."
            )

        try:
            import tensorrt
            import torch
        except ImportError as exc:
            raise RuntimeError(
                "TensorRT and CUDA-enabled PyTorch are required"
            ) from exc

        if not torch.cuda.is_available():
            raise RuntimeError(
                "CUDA is unavailable; FP16 TensorRT inference cannot start"
            )

        metadata_path = engine_path.with_suffix(".json")
        require_metadata = bool(
            self.get_parameter("require_engine_metadata").value
        )
        if not metadata_path.is_file():
            if require_metadata:
                raise RuntimeError(
                    f"Engine metadata not found: {metadata_path}"
                )
            self.get_logger().warning(
                f"Engine metadata not found: {metadata_path}"
            )
            return

        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        engine_digest = hashlib.sha256(engine_path.read_bytes()).hexdigest()
        if metadata.get("engine_sha256") != engine_digest:
            raise RuntimeError(
                "TensorRT engine hash does not match its metadata"
            )
        if metadata.get("precision") != "FP16":
            raise RuntimeError(
                "Engine metadata does not declare FP16 precision"
            )
        if metadata.get("batch") != 1:
            raise RuntimeError(
                "Only batch-one TensorRT engines are supported"
            )
        if metadata.get("image_size") != self.image_size:
            raise RuntimeError("Engine image size does not match image_size")
        exported_tensorrt = metadata.get("versions", {}).get("tensorrt")
        if exported_tensorrt and exported_tensorrt != tensorrt.__version__:
            raise RuntimeError(
                "TensorRT runtime differs from engine export runtime"
            )
        expected_versions = {
            "cuda": torch.version.cuda,
            "pytorch": torch.__version__,
            "ultralytics": ultralytics.__version__,
        }
        for name, runtime_version in expected_versions.items():
            exported_version = metadata.get("versions", {}).get(name)
            if exported_version and exported_version != runtime_version:
                raise RuntimeError(
                    f"{name} runtime differs from engine metadata"
                )

    def image_callback(self, image_msg: Image) -> None:
        self.publish_rate_pulse()
        if self.main_state != AUTO_DRIVE:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to convert color image: {exc}")
            return

        results = self.model.predict(
            color_image,
            imgsz=self.image_size,
            conf=self.min_confidence,
            classes=list(TARGET_CLASS_IDS),
            batch=1,
            verbose=False,
        )
        detections = self.extract_detections(results)

        output = YoloDetection2DArray()
        output.header = image_msg.header
        output.image_height, output.image_width = color_image.shape[:2]
        output.detections = [
            self.to_detection_message(detection)
            for detection in detections
            if detection.class_id == STOP_SIGN_CLASS_ID
        ]
        output.traffic_lights = [
            self.to_traffic_light_message(detection, color_image)
            for detection in detections
            if detection.class_id == TRAFFIC_LIGHT_CLASS_ID
        ]
        self.publish_detection_with_trace(output)

        self.publish_inference_image(
            results,
            [],
            image_msg,
        )

    def publish_rate_pulse(self) -> None:
        """Report image delivery without adding another Image subscriber."""
        publisher = getattr(self, "image_rate_pulse_pub", None)
        if publisher is None or publisher.get_subscription_count() == 0:
            return
        publisher.publish(self.rate_pulse_msg)

    def publish_detection_with_trace(
        self,
        output: YoloDetection2DArray,
    ) -> None:
        """Publish one detection result and its one-to-one timing trace."""
        trace = PerceptionLatencyTrace()
        trace.header = output.header
        trace.detection_sequence = self.detection_sequence
        trace.detection_publish_stamp = self.get_clock().now().to_msg()

        self.detection_pub.publish(output)
        self.latency_trace_pub.publish(trace)
        self.detection_sequence += 1

    def publish_inference_image(
        self,
        results,
        _additional_detections: list[Detection2D],
        image_msg: Image,
    ) -> None:
        if not results or self.image_pub.get_subscription_count() == 0:
            return

        annotated = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8",
        )
        annotated_msg.header = image_msg.header
        self.image_pub.publish(annotated_msg)

    def extract_detections(
        self,
        results,
        model=None,
        class_name_overrides: dict[int, str] | None = None,
    ) -> list[Detection2D]:
        model = self.model if model is None else model
        class_name_overrides = class_name_overrides or {}
        detections = []
        for result in results:
            if result.boxes is None or result.boxes.data.numel() == 0:
                continue
            # A detection row is [x1, y1, x2, y2, confidence, class_id].
            # Copy the table once to avoid three CUDA synchronizations.
            rows = result.boxes.data.detach().cpu().numpy()
            for row in rows:
                x1, y1, x2, y2, confidence, class_id = row[:6]
                class_id = int(class_id)
                detections.append(
                    Detection2D(
                        class_id=class_id,
                        class_name=self.class_name(
                            model,
                            class_id,
                            class_name_overrides,
                        ),
                        bbox=(float(x1), float(y1), float(x2), float(y2)),
                        confidence=float(confidence),
                    )
                )
        return detections

    def class_name(
        self,
        model,
        class_id: int,
        class_name_overrides: dict[int, str],
    ) -> str:
        if class_id in class_name_overrides:
            return class_name_overrides[class_id]
        names = getattr(model, "names", {})
        if isinstance(names, dict):
            return str(names.get(class_id, f"class_{class_id}"))
        if 0 <= class_id < len(names):
            return str(names[class_id])
        return f"class_{class_id}"

    def to_detection_message(
        self,
        detection: Detection2D,
    ) -> YoloDetection2D:
        message = YoloDetection2D()
        message.class_id = detection.class_id
        message.class_name = detection.class_name
        message.confidence = detection.confidence
        (
            message.bbox_x_min,
            message.bbox_y_min,
            message.bbox_x_max,
            message.bbox_y_max,
        ) = detection.bbox
        return message

    def to_traffic_light_message(
        self,
        detection: Detection2D,
        color_image,
    ) -> YoloTrafficLightDetection2D:
        message = YoloTrafficLightDetection2D()
        message.detection = self.to_detection_message(detection)
        message.traffic_light_color = self.light_classifier.classify(
            color_image,
            detection.bbox,
        )
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Perception2DNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
