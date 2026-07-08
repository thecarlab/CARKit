#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
# Licensed under the Apache License, Version 2.0 (the "License");

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

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

        self.model_path = str(self.get_parameter("model_path").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")
        self.light_classifier = TrafficLightClassifier()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            ___,  # Hint: ROS Image message type
            str(self.get_parameter(___).value),  # Hint: camera topic parameter
            ___,  # Hint: callback function for each camera frame
            sensor_qos,
        )
        self.detection_pub = self.create_publisher(
            ___,  # Hint: detection-array message type
            str(self.get_parameter(___).value),  # Hint: detection topic param
            10,
        )
        self.image_pub = self.create_publisher(
            ___,  # Hint: ROS Image message type
            str(self.get_parameter(___).value),  # Hint: visualization topic
            sensor_qos,
        )

        self.get_logger().info(
            f"Loaded FP16 TensorRT model {self.model_path}; "
            "using color images only and publishing "
            f"{self.get_parameter('detection_2d_topic').value}"
        )

    def image_callback(self, image_msg: Image) -> None:
        """Handle one camera frame and publish YOLO perception results."""
        color_image = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding="bgr8",
        )

        # TODO(student): Run YOLO on the camera image.
        results = self.model.predict(
            ___,  # Hint: the OpenCV image variable created above
            imgsz=self.image_size,
            batch=1,
            verbose=False,
        )
        detections = self.extract_detections(results)

        output = ___()  # Hint: create the detection-array message
        output.header = ___.header  # Hint: copy the incoming image header
        output.image_height, output.image_width = ___.shape[:2]

        output.detections = []
        output.traffic_lights = []
        for detection in detections:
            # TODO(student): Sort normal detections and traffic lights.
            if detection.class_name == ___:  # Hint: COCO traffic-light label
                output.traffic_lights.append(
                    self.to_traffic_light_message(detection, color_image)
                )
            else:
                output.detections.append(
                    self.to_detection_message(detection)
                )
        self.detection_pub.publish(___)  # Hint: publish the output message

        # TODO: Publish YOLO's annotated image for visualization.

    def publish_inference_image(self, results, image_msg: Image) -> None:
        """Publish YOLO's annotated image for visualization."""
        # TODO(student): In the visualization lab, use YOLO's plot() result,
        # convert it to a ROS Image, copy the header, and publish it.
        pass

    def extract_detections(self, results) -> list[Detection2D]:
        """Convert raw YOLO result boxes into CARKit Detection2D objects."""
        detections = []
        for result in results:
            if result.boxes is None or result.boxes.data.numel() == 0:
                continue

            # A detection row is [x1, y1, x2, y2, confidence, class_id].
            rows = result.boxes.data.detach().cpu().numpy()
            for row in rows:
                x1, y1, x2, y2, confidence, class_id = row[:6]

                class_id = int(___)  # Hint: convert YOLO's class id to int
                detections.append(
                    Detection2D(
                        class_id=class_id,
                        class_name=str(self.model.names[class_id]),
                        bbox=(float(x1), float(y1), float(x2), float(y2)),
                        confidence=float(___),  # Hint: YOLO confidence score
                    )
                )
        return detections

    def to_detection_message(
        self,
        detection: Detection2D,
    ) -> YoloDetection2D:
        """Convert one Detection2D into the ROS detection message type."""
        message = ___()  # Hint: create a YoloDetection2D message
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
        """Convert a traffic-light Detection2D and classify its light color."""
        message = ___()  # Hint: create a traffic-light detection message
        message.detection = self.to_detection_message(detection)
        message.traffic_light_color = self.light_classifier.classify(
            color_image,
            detection.bbox,
        )
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Perception2DNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
