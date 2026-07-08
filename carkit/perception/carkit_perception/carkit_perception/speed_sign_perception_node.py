#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
# Licensed under the Apache License, Version 2.0 (the "License");

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

from carkit_perception.perception_math import Detection2D, clipped_bbox
from carkit_perception_msgs.msg import YoloDetection2D, YoloDetection2DArray


SPEED_SIGN_CLASS_NAMES = {
    0: "speed_sign",
    1: "traffic_cone",
}
TRAFFIC_SIGN_WEIGHT = Path("models/traffic_sign.pt")


def default_model_path() -> str:
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )

        installed_path = (
            Path(get_package_share_directory("carkit_perception"))
            / TRAFFIC_SIGN_WEIGHT
        )
        if installed_path.is_file():
            return str(installed_path)
    except (ImportError, PackageNotFoundError):
        pass

    source_path = Path(__file__).resolve().parents[1] / TRAFFIC_SIGN_WEIGHT
    return str(source_path)


class SpeedSignPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("speed_sign_perception_node")

        self.declare_parameter("model_path", default_model_path())
        self.declare_parameter("image_size", 640)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detection_topic", "/speed_sign")
        self.declare_parameter("visualization_topic", "/traffic_sign")
        self.declare_parameter("min_confidence", 0.2)

        self.model_path = str(self.get_parameter("model_path").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        self._validate_model_path()
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")

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
            str(self.get_parameter("detection_topic").value),
            10,
        )
        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("visualization_topic").value),
            sensor_qos,
        )

        self.get_logger().info(
            f"Loaded speed sign model {self.model_path}; publishing "
            f"{self.get_parameter('detection_topic').value} and "
            f"{self.get_parameter('visualization_topic').value}"
        )

    def _validate_model_path(self) -> None:
        model_path = Path(self.model_path)
        if not model_path.is_file():
            raise RuntimeError(f"Speed sign model not found: {model_path}")

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
            self.to_detection_message(detection) for detection in detections
        ]
        output.traffic_lights = []
        self.detection_pub.publish(output)

        if detections:
            self.log_detections(detections)

        self.publish_visualization_image(color_image, detections, image_msg)

    def extract_detections(self, results) -> list[Detection2D]:
        detections = []
        for result in results:
            if result.boxes is None or result.boxes.data.numel() == 0:
                continue
            rows = result.boxes.data.detach().cpu().numpy()
            for row in rows:
                x1, y1, x2, y2, confidence, class_id = row[:6]
                class_id = int(class_id)
                detections.append(
                    Detection2D(
                        class_id=class_id,
                        class_name=self.class_name(class_id),
                        bbox=(float(x1), float(y1), float(x2), float(y2)),
                        confidence=float(confidence),
                    )
                )
        return detections

    def class_name(self, class_id: int) -> str:
        if class_id in SPEED_SIGN_CLASS_NAMES:
            return SPEED_SIGN_CLASS_NAMES[class_id]

        names = self.model.names
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def log_detections(self, detections: list[Detection2D]) -> None:
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            self.get_logger().info(
                "Detected "
                f"class_id={detection.class_id} "
                f"class_name={detection.class_name} "
                f"confidence={detection.confidence:.3f} "
                f"bbox=[{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]"
            )

    def publish_visualization_image(
        self,
        color_image,
        detections: list[Detection2D],
        image_msg: Image,
    ) -> None:
        if self.image_pub.get_subscription_count() == 0:
            return

        annotated = self.annotate_image(color_image, detections)
        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8",
        )
        annotated_msg.header = image_msg.header
        self.image_pub.publish(annotated_msg)

    def annotate_image(self, color_image, detections: list[Detection2D]):
        annotated = color_image.copy()
        height, width = annotated.shape[:2]
        for detection in detections:
            bounds = clipped_bbox(detection.bbox, width, height)
            if bounds is None:
                continue

            left, top, right, bottom = bounds
            color = self.detection_color(detection.class_id)
            cv2.rectangle(annotated, (left, top), (right, bottom), color, 2)
            label = (
                f"id={detection.class_id} "
                f"{detection.class_name} "
                f"{detection.confidence:.2f}"
            )
            self.draw_label(annotated, label, left, top, color)
        return annotated

    def detection_color(self, class_id: int) -> tuple[int, int, int]:
        if class_id == 0:
            return (40, 220, 40)
        if class_id == 1:
            return (0, 170, 255)
        return (255, 255, 255)

    def draw_label(self, image, label: str, left: int, top: int, color) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size, baseline = cv2.getTextSize(
            label,
            font,
            font_scale,
            thickness,
        )
        text_width, text_height = text_size
        label_top = max(0, top - text_height - baseline - 4)
        label_bottom = label_top + text_height + baseline + 4
        label_right = min(image.shape[1], left + text_width + 6)
        cv2.rectangle(
            image,
            (left, label_top),
            (label_right, label_bottom),
            color,
            cv2.FILLED,
        )
        cv2.putText(
            image,
            label,
            (left + 3, label_bottom - baseline - 2),
            font,
            font_scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedSignPerceptionNode()
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
