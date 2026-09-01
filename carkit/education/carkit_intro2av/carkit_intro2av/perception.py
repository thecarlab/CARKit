#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Reference-shaped Intro2AV perception ROS node."""

import math

from carkit_perception_msgs.msg import YoloDetection2DArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

from .perception_algorithm import PerceptionConfig, process_image


class Intro2AvPerception(Node):
    """Keep ROS transport/rate handling while students own detection."""

    def __init__(self):
        super().__init__("carkit_intro2av_perception")
        self._declare_parameters()
        self.latest_image = None
        self.camera_info = None
        self.image_generation = 0
        self.processed_generation = -1
        self.latest_output = None
        self.latest_preview = None

        self.detection_publisher = self.create_publisher(
            YoloDetection2DArray,
            self._parameter("detection_2d_topic"),
            10,
        )
        self.preview_publisher = self.create_publisher(
            CompressedImage,
            self._parameter("inference_compressed_topic"),
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self._parameter("image_topic"),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._parameter("camera_info_topic"),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        rate = max(0.1, float(self._parameter("max_inference_rate_hz")))
        self.timer = self.create_timer(1.0 / rate, self._publish_result)
        self.get_logger().info(
            "Intro2AV perception ready: compressed camera -> typed YOLO "
            "topics; implement perception_algorithm.process_image"
        )

    def _declare_parameters(self):
        """Declare the ROS parameters and their safe default values."""
        defaults = {
            "image_topic": "/camera/camera/color/image_raw/compressed",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "detection_2d_topic": "/yolo/detections_2d",
            "inference_compressed_topic": "/yolo/inference_image/compressed",
            "max_inference_rate_hz": 10.0,
            "minimum_confidence": 0.20,
            "image_size": 448,
            "model_path": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _parameter(self, name):
        """Return the current value of a declared ROS parameter."""
        return self.get_parameter(name).value

    def _on_image(self, message):
        self.latest_image = message
        self.image_generation += 1

    def _on_camera_info(self, message):
        self.camera_info = message

    def _configuration(self):
        """Collect current parameters into an algorithm configuration."""
        return PerceptionConfig(
            minimum_confidence=float(self._parameter("minimum_confidence")),
            image_size=int(self._parameter("image_size")),
            model_path=str(self._parameter("model_path")),
        )

    def _publish_result(self):
        if self.latest_image is None:
            return
        if self.processed_generation != self.image_generation:
            self._process_latest_image()
        if self.latest_output is not None:
            self.detection_publisher.publish(self.latest_output)
        if (
            self.latest_preview is not None
            and self.preview_publisher.get_subscription_count() > 0
        ):
            self.preview_publisher.publish(self.latest_preview)

    def _process_latest_image(self):
        image = self.latest_image
        generation = self.image_generation
        try:
            result = process_image(
                image, self.camera_info, self._configuration()
            )
            self._validate_result(result)
            output = YoloDetection2DArray()
            output.header = image.header
            if self.camera_info is not None:
                output.image_width = self.camera_info.width
                output.image_height = self.camera_info.height
            output.detections = list(result.detections)
            output.traffic_lights = list(result.traffic_lights)
            self.latest_output = output
            self.latest_preview = result.preview or image
        except Exception as error:
            self.get_logger().error(f"Perception algorithm failed: {error}")
            self.latest_output = self._empty_output(image)
            self.latest_preview = image
        self.processed_generation = generation

    def _empty_output(self, image):
        output = YoloDetection2DArray()
        output.header = image.header
        if self.camera_info is not None:
            output.image_width = self.camera_info.width
            output.image_height = self.camera_info.height
        return output

    @staticmethod
    def _validate_result(result):
        for detection in result.detections:
            values = (
                detection.confidence,
                detection.bbox_x_min,
                detection.bbox_y_min,
                detection.bbox_x_max,
                detection.bbox_y_max,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("detector returned a non-finite value")
            if not 0.0 <= detection.confidence <= 1.0:
                raise ValueError("detection confidence must be in [0, 1]")


def main(args=None):
    rclpy.init(args=args)
    node = Intro2AvPerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
