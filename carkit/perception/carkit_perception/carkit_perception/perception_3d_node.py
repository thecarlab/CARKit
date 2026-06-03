#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO
import os


@dataclass
class Detection2D:
    label: str
    bbox: tuple[float, float, float, float]
    confidence: float


class Perception3DNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_3d_node")

        default_model = os.path.join(
            get_package_share_directory("carkit_perception"),
            "models",
            "yolo11n.pt",
        )
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detection_topic", "/yolo/detection")
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("detection_3d_topic", "/yolo/detections_3d")
        self.declare_parameter("min_confidence", 0.2)

        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        detection_3d_topic = self.get_parameter("detection_3d_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        self.bridge = CvBridge()
        self.model = YOLO(model_path, task="detect")
        self.depth_image: Optional[np.ndarray] = None
        self.depth_encoding: Optional[str] = None
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.camera_frame = "camera_color_optical_frame"

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.detection_pub = self.create_publisher(String, detection_topic, 10)
        self.detection_3d_pub = self.create_publisher(String, detection_3d_topic, 10)

        self.get_logger().info(
            "Perception3DNode started. "
            f"Running YOLO on {image_topic}; listening to {depth_topic} and "
            f"{camera_info_topic}; publishing detections to {detection_topic} "
            f"and camera-frame detections to {detection_3d_topic}"
        )

    def depth_callback(self, msg: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            self.depth_encoding = msg.encoding
        except Exception as exc:
            self.get_logger().error(f"Failed to convert depth image: {exc}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id or self.camera_frame

    def image_callback(self, msg: Image) -> None:
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert color image: {exc}")
            return

        results = self.model(color_image)
        detections = self.extract_detections(results)

        self.publish_detections(detections)
        self.publish_3d_detections(detections)

    def publish_detections(self, detections: list[Detection2D]) -> None:
        detection_text = []
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            detection_text.append(
                f"{detection.label} [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}] "
                f"({detection.confidence:.2f})"
            )

        if detection_text:
            self.detection_pub.publish(String(data="; ".join(detection_text)))
        else:
            self.detection_pub.publish(String(data="no detections"))

    def publish_3d_detections(self, detections: list[Detection2D]) -> None:
        if self.depth_image is None:
            self.get_logger().debug("Waiting for aligned depth image.")
            return

        if not self.has_camera_intrinsics():
            self.get_logger().debug("Waiting for aligned depth camera_info.")
            return

        detections_3d = []

        for detection in detections:
            if detection.confidence < self.min_confidence:
                continue

            position = self.project_detection(detection)
            if position is None:
                continue

            x, y, z = position
            x1, y1, x2, y2 = detection.bbox
            detections_3d.append(
                f"{detection.label} {self.camera_frame} "
                f"x={x:.3f} y={y:.3f} z={z:.3f} "
                f"conf={detection.confidence:.2f} "
                f"bbox=[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}]"
            )

        if detections_3d:
            self.detection_3d_pub.publish(String(data="; ".join(detections_3d)))
        elif not detections:
            self.detection_3d_pub.publish(String(data="no detections"))
        else:
            self.detection_3d_pub.publish(String(data="no valid 3d detections"))

    def has_camera_intrinsics(self) -> bool:
        return (
            self.fx is not None
            and self.fy is not None
            and self.cx is not None
            and self.cy is not None
            and self.fx > 0.0
            and self.fy > 0.0
        )

    def extract_detections(self, results) -> list[Detection2D]:
        detections = []
        for result in results:
            if not result.boxes or result.boxes.cls is None:
                continue

            cls_ids = result.boxes.cls.cpu().numpy()
            bboxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()

            for cls_id, bbox, confidence in zip(cls_ids, bboxes, confidences):
                class_name = self.model.names[int(cls_id)]
                x1, y1, x2, y2 = bbox
                detections.append(
                    Detection2D(
                        label=class_name,
                        bbox=(float(x1), float(y1), float(x2), float(y2)),
                        confidence=float(confidence),
                    )
                )

        return detections

    def project_detection(
        self,
        detection: Detection2D,
    ) -> Optional[tuple[float, float, float]]:
        if self.depth_image is None:
            return None

        x1, y1, x2, y2 = detection.bbox
        height, width = self.depth_image.shape[:2]
        left = max(0, min(width, int(math.floor(x1))))
        top = max(0, min(height, int(math.floor(y1))))
        right = max(0, min(width, int(math.ceil(x2))))
        bottom = max(0, min(height, int(math.ceil(y2))))

        if left >= right or top >= bottom:
            return None

        depth_region = self.depth_image[top:bottom, left:right]
        depth_meters = self.depth_to_meters(depth_region)
        valid_depths = depth_meters[np.isfinite(depth_meters) & (depth_meters > 0.0)]
        if valid_depths.size == 0:
            return None

        z = float(np.median(valid_depths))
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return float(x), float(y), z

    def depth_to_meters(self, depth: np.ndarray) -> np.ndarray:
        depth_float = depth.astype(np.float32)
        if self.depth_encoding in ("16UC1", "mono16") or depth.dtype == np.uint16:
            return depth_float / 1000.0
        return depth_float


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Perception3DNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
