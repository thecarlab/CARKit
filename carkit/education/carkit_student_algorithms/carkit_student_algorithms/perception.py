#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Perception exercise: camera images -> typed detections and display image."""

import rclpy
from carkit_perception_msgs.msg import YoloDetection2DArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class StudentPerception(Node):
    def __init__(self, guided):
        super().__init__("carkit_student_perception")
        self.guided = guided
        self.detections = self.create_publisher(
            YoloDetection2DArray, "/yolo/detections_2d", 10
        )
        self.preview = self.create_publisher(Image, "/yolo/inference_image", 10)
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self._on_image, 10
        )
        level = "guided interface" if guided else "boilerplate"
        self.get_logger().info(f"CARKit perception {level}: camera -> /yolo/*")

    def _on_image(self, image):
        result = YoloDetection2DArray()
        result.header = image.header
        result.image_width = image.width
        result.image_height = image.height
        if self.guided:
            # TODO(ADA): add a simple detector and append typed detections here.
            # Keeping the camera pass-through makes experiments visible immediately.
            pass
        else:
            # TODO(Intro2AV): run your model and fill result.detections.
            pass
        self.detections.publish(result)
        self.preview.publish(image)


def _main(guided, args=None):
    rclpy.init(args=args)
    node = StudentPerception(guided)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def guided_main(args=None):
    """Run the guided student implementation entry point."""
    _main(True, args)


def boilerplate_main(args=None):
    """Run the safe boilerplate student implementation entry point."""
    _main(False, args)
