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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
import ultralytics

from carkit_perception.perception_math import (
    Detection2D,
    TrafficLightClassifier,
)
from carkit_perception_msgs.msg import (
    YoloDetection2D,
    YoloDetection2DArray,
    YoloTrafficLightDetection2D,
)


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
        self.light_classifier = TrafficLightClassifier()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            sensor_qos,
        )
        self.detection_pub = self.create_publisher(
            YoloDetection2DArray,
            str(self.get_parameter("detection_2d_topic").value),
            10,
        )
        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("inference_image_topic").value),
            sensor_qos,
        )

        self.get_logger().info(
            f"Loaded FP16 TensorRT model {self.model_path}; "
            "using color images only and publishing "
            f"{self.get_parameter('detection_2d_topic').value}"
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
            if detection.class_name != "traffic light"
        ]
        output.traffic_lights = [
            self.to_traffic_light_message(detection, color_image)
            for detection in detections
            if detection.class_name == "traffic light"
        ]
        self.detection_pub.publish(output)

        if results:
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding="bgr8",
            )
            annotated_msg.header = image_msg.header
            self.image_pub.publish(annotated_msg)

    def extract_detections(self, results) -> list[Detection2D]:
        detections = []
        for result in results:
            if result.boxes is None or result.boxes.cls is None:
                continue
            for class_id, bbox, confidence in zip(
                result.boxes.cls.cpu().numpy(),
                result.boxes.xyxy.cpu().numpy(),
                result.boxes.conf.cpu().numpy(),
            ):
                x1, y1, x2, y2 = bbox
                detections.append(
                    Detection2D(
                        class_id=int(class_id),
                        class_name=str(self.model.names[int(class_id)]),
                        bbox=(float(x1), float(y1), float(x2), float(y2)),
                        confidence=float(confidence),
                    )
                )
        return detections

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
