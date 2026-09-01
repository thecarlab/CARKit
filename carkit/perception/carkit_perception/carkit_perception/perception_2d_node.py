#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
# Licensed under the Apache License, Version 2.0 (the "License");

# CARKit learning annotation: implements the behavior described by this file's package and module.
import hashlib
import json
from pathlib import Path
import time

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
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


TRAFFIC_SIGN_CLASS_NAMES = {
    0: "speed_sign",
    1: "traffic_cone",
}
TRAFFIC_SIGN_WEIGHT = Path("models/traffic_sign_1_fp16.engine")


def default_traffic_sign_model_path() -> str:
    try:
        share_path = Path(get_package_share_directory("carkit_perception"))
        return str(share_path / TRAFFIC_SIGN_WEIGHT)
    except PackageNotFoundError:
        pass

    source_path = Path(__file__).resolve().parents[1] / TRAFFIC_SIGN_WEIGHT
    return str(source_path)


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
        self.declare_parameter(
            "traffic_sign_model_path",
            default_traffic_sign_model_path(),
        )
        self.declare_parameter("model_profile", "combined")
        self.declare_parameter("custom_model_path", "")
        self.declare_parameter("image_size", 448)
        self.declare_parameter(
            "image_topic",
            "/camera/camera/color/image_raw/compressed",
        )
        self.declare_parameter("input_transport", "compressed")
        self.declare_parameter(
            "inference_image_topic",
            "/yolo/inference_image",
        )
        self.declare_parameter(
            "inference_compressed_topic",
            "/yolo/inference_image/compressed",
        )
        self.declare_parameter("detection_2d_topic", "/yolo/detections_2d")
        self.declare_parameter("min_confidence", 0.2)
        self.declare_parameter("traffic_sign_min_confidence", 0.2)
        self.declare_parameter("max_inference_rate_hz", 10.0)
        self.declare_parameter("secondary_inference_interval", 2)
        self.declare_parameter("inference_jpeg_quality", 70)
        self.declare_parameter("require_engine_metadata", True)

        self.model_profile = str(
            self.get_parameter("model_profile").value
        ).strip().lower()
        configured_model_path = str(self.get_parameter("model_path").value)
        self.traffic_sign_model_path = str(
            self.get_parameter("traffic_sign_model_path").value
        )
        custom_model_path = str(
            self.get_parameter("custom_model_path").value
        ).strip()
        if self.model_profile == "generic_coco":
            self.model_path = configured_model_path
            self.secondary_model_enabled = False
            self.primary_class_names = {}
        elif self.model_profile == "traffic_signs":
            self.model_path = self.traffic_sign_model_path
            self.secondary_model_enabled = False
            self.primary_class_names = TRAFFIC_SIGN_CLASS_NAMES
        elif self.model_profile == "combined":
            self.model_path = configured_model_path
            self.secondary_model_enabled = True
            self.primary_class_names = {}
        elif self.model_profile == "custom":
            if not custom_model_path:
                raise RuntimeError(
                    "custom_model_path is required for model_profile=custom"
                )
            self.model_path = custom_model_path
            self.secondary_model_enabled = False
            self.primary_class_names = {}
        else:
            raise RuntimeError(
                "model_profile must be generic_coco, traffic_signs, "
                "combined, or custom"
            )
        self.image_size = int(self.get_parameter("image_size").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.traffic_sign_min_confidence = float(
            self.get_parameter("traffic_sign_min_confidence").value
        )
        self.max_inference_rate_hz = max(
            0.1,
            float(self.get_parameter("max_inference_rate_hz").value),
        )
        self.secondary_inference_interval = max(
            1,
            int(self.get_parameter("secondary_inference_interval").value),
        )
        self.inference_jpeg_quality = min(
            95,
            max(30, int(self.get_parameter("inference_jpeg_quality").value)),
        )
        self.last_inference_monotonic = None
        self.frame_sequence = 0
        self.cached_secondary_detections = []
        self.input_transport = str(
            self.get_parameter("input_transport").value
        ).strip().lower()
        if self.input_transport not in {"raw", "compressed"}:
            raise RuntimeError("input_transport must be raw or compressed")

        self._validate_fp16_engine(self.model_path, "model_path")
        if self.secondary_model_enabled:
            self._validate_fp16_engine(
                self.traffic_sign_model_path,
                "traffic_sign_model_path",
            )
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")
        self.traffic_sign_model = (
            YOLO(self.traffic_sign_model_path, task="detect")
            if self.secondary_model_enabled
            else None
        )
        self.light_classifier = TrafficLightClassifier()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            CompressedImage if self.input_transport == "compressed" else Image,
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
        self.compressed_image_pub = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("inference_compressed_topic").value),
            sensor_qos,
        )
        self.latest_detection_output = None
        self.latest_raw_inference_image = None
        self.latest_compressed_inference_image = None
        self.output_timer = self.create_timer(
            1.0 / self.max_inference_rate_hz,
            self.publish_latest_results,
        )

        self.get_logger().info(
            f"Loaded {self.model_profile} FP16 TensorRT model "
            f"{self.model_path}; secondary model "
            f"{'enabled' if self.secondary_model_enabled else 'disabled'}; "
            f"target rate {self.max_inference_rate_hz:.1f} Hz; publishing "
            f"{self.get_parameter('detection_2d_topic').value}"
        )

    def _validate_fp16_engine(self, model_path: str, parameter_name: str) -> None:
        engine_path = Path(model_path)
        if engine_path.suffix != ".engine":
            raise RuntimeError(
                f"{parameter_name} must point to an FP16 TensorRT .engine file"
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
            raise RuntimeError(
                f"{parameter_name} engine image size does not match image_size"
            )
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

    def image_callback(self, image_msg) -> None:
        if not self.inference_due():
            return
        try:
            if self.input_transport == "compressed":
                encoded = np.frombuffer(image_msg.data, dtype=np.uint8)
                color_image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if color_image is None:
                    raise ValueError("OpenCV could not decode the image")
            else:
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
        detections = self.extract_detections(
            results,
            self.model,
            self.primary_class_names,
        )
        self.frame_sequence += 1
        if (
            self.secondary_model_enabled
            and (
                self.frame_sequence == 1
                or self.frame_sequence % self.secondary_inference_interval == 0
            )
        ):
            traffic_sign_results = self.traffic_sign_model.predict(
                color_image,
                imgsz=self.image_size,
                conf=self.traffic_sign_min_confidence,
                batch=1,
                verbose=False,
            )
            self.cached_secondary_detections = self.extract_detections(
                traffic_sign_results,
                self.traffic_sign_model,
                TRAFFIC_SIGN_CLASS_NAMES,
            )
        traffic_sign_detections = self.cached_secondary_detections

        output = YoloDetection2DArray()
        output.header = image_msg.header
        output.image_height, output.image_width = color_image.shape[:2]
        output.detections = [
            self.to_detection_message(detection)
            for detection in detections + traffic_sign_detections
            if detection.class_name != "traffic light"
        ]
        output.traffic_lights = [
            self.to_traffic_light_message(detection, color_image)
            for detection in detections
            if detection.class_name == "traffic light"
        ]
        self.latest_detection_output = output

        self.publish_inference_image(
            color_image,
            detections + traffic_sign_detections,
            image_msg,
        )

    def inference_due(self) -> bool:
        """Return whether enough time has elapsed to run another inference."""
        now = time.monotonic()
        if (
            self.last_inference_monotonic is not None
            and now - self.last_inference_monotonic
            + 0.002 < 1.0 / self.max_inference_rate_hz
        ):
            return False
        self.last_inference_monotonic = now
        return True

    def publish_inference_image(
        self,
        color_image,
        detections: list[Detection2D],
        image_msg: Image,
    ) -> None:
        raw_subscribers = self.image_pub.get_subscription_count()
        compressed_subscribers = (
            self.compressed_image_pub.get_subscription_count()
        )
        if raw_subscribers == 0 and compressed_subscribers == 0:
            return

        # Drawing the already-extracted boxes avoids the relatively expensive
        # Ultralytics plot pipeline and keeps web visualization off the GPU.
        annotated = color_image.copy()
        self.draw_detections(
            annotated,
            detections,
            color=(36, 255, 12),
        )
        if raw_subscribers:
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding="bgr8",
            )
            annotated_msg.header = image_msg.header
            self.latest_raw_inference_image = annotated_msg
        if compressed_subscribers:
            success, encoded = cv2.imencode(
                ".jpg",
                annotated,
                [cv2.IMWRITE_JPEG_QUALITY, self.inference_jpeg_quality],
            )
            if not success:
                self.get_logger().warning("Failed to encode inference image")
                return
            compressed = CompressedImage()
            compressed.header = image_msg.header
            compressed.format = "jpeg"
            compressed.data = encoded.tobytes()
            self.latest_compressed_inference_image = compressed

    def publish_latest_results(self) -> None:
        """Publish a stable-rate view of the newest completed inference."""
        if self.latest_detection_output is not None:
            self.detection_pub.publish(self.latest_detection_output)
        if (
            self.latest_raw_inference_image is not None
            and self.image_pub.get_subscription_count() > 0
        ):
            self.image_pub.publish(self.latest_raw_inference_image)
        if (
            self.latest_compressed_inference_image is not None
            and self.compressed_image_pub.get_subscription_count() > 0
        ):
            self.compressed_image_pub.publish(
                self.latest_compressed_inference_image
            )

    def draw_detections(
        self,
        image,
        detections: list[Detection2D],
        color: tuple[int, int, int],
    ) -> None:
        for detection in detections:
            x1, y1, x2, y2 = (int(value) for value in detection.bbox)
            label = f"{detection.class_name} {detection.confidence:.2f}"
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                image,
                label,
                (x1, max(12, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )

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
