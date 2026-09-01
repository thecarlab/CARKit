#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""ADA guided filtering around the protected reference detector."""

from copy import deepcopy

from carkit_perception_msgs.msg import YoloDetection2DArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .math_utils import keep_detection


class AdaPerception(Node):
    def __init__(self):
        super().__init__("carkit_ada_perception")
        self.declare_parameter("minimum_confidence", 0.35)
        self.publisher = self.create_publisher(
            YoloDetection2DArray,
            "/yolo/detections_2d",
            10,
        )
        self.create_subscription(
            YoloDetection2DArray,
            "/carkit/reference/detections_2d",
            self._on_detections,
            10,
        )
        self.get_logger().info(
            "ADA guided perception filter: reference detections -> /yolo"
        )

    def _on_detections(self, message):
        output = deepcopy(message)
        threshold = float(
            self.get_parameter("minimum_confidence").value
        )
        # ADA exercise: modify keep_detection for a class-specific lesson.
        output.detections = [
            detection
            for detection in message.detections
            if keep_detection(
                detection.class_name,
                detection.confidence,
                threshold,
            )
        ]
        output.traffic_lights = [
            light
            for light in message.traffic_lights
            if keep_detection(
                light.detection.class_name,
                light.detection.confidence,
                threshold,
            )
        ]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = AdaPerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
