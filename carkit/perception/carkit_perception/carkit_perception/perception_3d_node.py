#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
import ultralytics

from carkit_perception.perception_math import (
    Detection2D,
    TRAFFIC_LIGHT_UNKNOWN,
    TrafficLightClassifier,
    median_detection_depth,
    project_pixel_to_camera,
)
from carkit_perception_msgs.msg import YoloDetection3D, YoloDetection3DArray


class Perception3DNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_3d_node")

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
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("inference_image_topic", "/yolo/inference_image")
        self.declare_parameter("detection_3d_topic", "/yolo/detections_3d")
        self.declare_parameter("min_confidence", 0.2)
        self.declare_parameter("min_depth", 0.1)
        self.declare_parameter("max_depth", 10.0)
        self.declare_parameter("sync_queue_size", 2)
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("require_engine_metadata", True)

        self.model_path = str(self.get_parameter("model_path").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)

        self._validate_fp16_engine()

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")
        self.light_classifier = TrafficLightClassifier()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = Subscriber(
            self,
            Image,
            str(self.get_parameter("image_topic").value),
            qos_profile=sensor_qos,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile=sensor_qos,
        )
        self.camera_info_sub = Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile=sensor_qos,
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.camera_info_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.synchronizer.registerCallback(self.synchronized_callback)

        self.detection_pub = self.create_publisher(
            YoloDetection3DArray,
            str(self.get_parameter("detection_3d_topic").value),
            10,
        )
        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("inference_image_topic").value),
            sensor_qos,
        )

        self.get_logger().info(
            f"Loaded FP16 TensorRT model {self.model_path}; "
            "publishing typed detections on "
            f"{self.get_parameter('detection_3d_topic').value}"
        )

    def _validate_fp16_engine(self) -> None:
        engine_path = Path(self.model_path)
        if engine_path.suffix != ".engine":
            raise RuntimeError("model_path must point to an FP16 TensorRT .engine file")
        if not engine_path.is_file():
            raise RuntimeError(
                f"TensorRT engine not found: {engine_path}. "
                "Run export_fp16_engine on this Jetson first."
            )

        try:
            import tensorrt
            import torch
        except ImportError as exc:
            raise RuntimeError("TensorRT and CUDA-enabled PyTorch are required") from exc

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable; FP16 TensorRT inference cannot start")

        metadata_path = engine_path.with_suffix(".json")
        require_metadata = bool(
            self.get_parameter("require_engine_metadata").value
        )
        if not metadata_path.is_file():
            if require_metadata:
                raise RuntimeError(f"Engine metadata not found: {metadata_path}")
            self.get_logger().warning(f"Engine metadata not found: {metadata_path}")
            return

        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        engine_digest = hashlib.sha256(engine_path.read_bytes()).hexdigest()
        if metadata.get("engine_sha256") != engine_digest:
            raise RuntimeError("TensorRT engine hash does not match its metadata")
        if metadata.get("precision") != "FP16":
            raise RuntimeError("Engine metadata does not declare FP16 precision")
        if metadata.get("batch") != 1:
            raise RuntimeError("Only batch-one TensorRT engines are supported")
        if metadata.get("image_size") != self.image_size:
            raise RuntimeError(
                "Engine image size does not match the image_size parameter"
            )
        exported_tensorrt = metadata.get("versions", {}).get("tensorrt")
        if exported_tensorrt and exported_tensorrt != tensorrt.__version__:
            raise RuntimeError(
                "TensorRT runtime differs from the engine export runtime: "
                f"{tensorrt.__version__} != {exported_tensorrt}"
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
                    f"{name} runtime differs from engine export metadata: "
                    f"{runtime_version} != {exported_version}"
                )

    def synchronized_callback(
        self,
        image_msg: Image,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
    ) -> None:
        try:
            color_image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to convert synchronized images: {exc}")
            return

        results = self.model.predict(
            color_image,
            imgsz=self.image_size,
            conf=self.min_confidence,
            batch=1,
            verbose=False,
        )
        detections = self.extract_detections(results)

        output = YoloDetection3DArray()
        output.header.stamp = image_msg.header.stamp
        output.header.frame_id = (
            camera_info_msg.header.frame_id
            or image_msg.header.frame_id
            or "camera_color_optical_frame"
        )

        fx = float(camera_info_msg.k[0])
        fy = float(camera_info_msg.k[4])
        cx = float(camera_info_msg.k[2])
        cy = float(camera_info_msg.k[5])
        intrinsics_valid = fx > 0.0 and fy > 0.0

        for detection in detections:
            output.detections.append(
                self.to_detection_message(
                    detection,
                    color_image,
                    depth_image,
                    depth_msg.encoding,
                    intrinsics_valid,
                    fx,
                    fy,
                    cx,
                    cy,
                )
            )

        self.detection_pub.publish(output)

        if results:
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = image_msg.header
            self.image_pub.publish(annotated_msg)

    def extract_detections(self, results) -> list[Detection2D]:
        detections = []
        for result in results:
            if result.boxes is None or result.boxes.cls is None:
                continue

            class_ids = result.boxes.cls.cpu().numpy()
            bboxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            for class_id, bbox, confidence in zip(
                class_ids,
                bboxes,
                confidences,
            ):
                x1, y1, x2, y2 = bbox
                detections.append(
                    Detection2D(
                        class_id=int(class_id),
                        class_name=str(self.model.names[int(class_id)]),
                        bbox=(
                            float(x1),
                            float(y1),
                            float(x2),
                            float(y2),
                        ),
                        confidence=float(confidence),
                    )
                )
        return detections

    def to_detection_message(
        self,
        detection: Detection2D,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        depth_encoding: str,
        intrinsics_valid: bool,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> YoloDetection3D:
        message = YoloDetection3D()
        message.class_id = detection.class_id
        message.class_name = detection.class_name
        message.confidence = detection.confidence
        (
            message.bbox_x_min,
            message.bbox_y_min,
            message.bbox_x_max,
            message.bbox_y_max,
        ) = detection.bbox
        message.traffic_light_color = TRAFFIC_LIGHT_UNKNOWN

        if "traffic light" in detection.class_name.lower():
            message.traffic_light_color = self.light_classifier.classify(
                color_image,
                detection.bbox,
            )

        depth = median_detection_depth(
            depth_image,
            depth_encoding,
            detection.bbox,
            self.min_depth,
            self.max_depth,
        )
        if depth is not None and intrinsics_valid:
            message.x, message.y, message.z = project_pixel_to_camera(
                detection.bbox,
                depth,
                fx,
                fy,
                cx,
                cy,
            )
            message.position_valid = True
        else:
            message.x = math.nan
            message.y = math.nan
            message.z = math.nan
            message.position_valid = False
        return message


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
