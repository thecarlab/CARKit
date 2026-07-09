#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
# Licensed under the Apache License, Version 2.0 (the "License");

from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

from carkit_perception.perception_math import Detection2D
from carkit_perception_msgs.msg import YoloDetection2D, YoloDetection2DArray

# TODO: change the model path to your own trained model.
SPEED_SIGN_CLASS_NAMES = {
    0: "speed_sign",
    1: "traffic_cone",
}
TRAFFIC_SIGN_WEIGHT = Path("models/traffic_sign.pt")


def default_model_path() -> str:
    try:
        share_path = Path(get_package_share_directory("carkit_perception"))
        return str(share_path / TRAFFIC_SIGN_WEIGHT)
    except PackageNotFoundError:
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
        """Handle one camera frame and publish speed-sign detections."""
        color_image = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding="bgr8",
        )

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

        self.log_detections(detections)

        self.publish_visualization_image(results, image_msg)

    def extract_detections(self, results) -> list[Detection2D]:
        """Convert raw YOLO result boxes into CARKit Detection2D objects."""
        detections = []
        for result in results:
            if result.boxes is None or result.boxes.data.numel() == 0:
                continue

            rows = result.boxes.data.detach().cpu().numpy()
            for row in rows:
                x1, y1, x2, y2, confidence, class_id = row[:6]
                if confidence < self.min_confidence:
                    continue

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
        """Return the readable label for a detected class id."""
        return SPEED_SIGN_CLASS_NAMES.get(class_id, f"class_{class_id}")

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
        results,
        image_msg: Image,
    ) -> None:
        """Publish YOLO's annotated speed-sign image for visualization."""
        if not results:
            return

        annotated = results[0].plot()
        output = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        output.header = image_msg.header
        self.image_pub.publish(output)

    def to_detection_message(
        self,
        detection: Detection2D,
    ) -> YoloDetection2D:
        """Convert one Detection2D into the ROS detection message type."""
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
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
