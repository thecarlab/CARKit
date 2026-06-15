#!/usr/bin/env python3

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

from carkit_perception_msgs.msg import YoloDetection3DArray


class Demo1Node(Node):
    def __init__(self):
        super().__init__("demo1_node")
        self.declare_parameter("target_object_type", "apple")
        self.declare_parameter("target_distance", 0.5)
        self.declare_parameter("speed_ratio", 1.5)
        self.declare_parameter("min_confidence", 0.45)
        self.target_object_type = self.get_parameter("target_object_type").value
        self.target_distance = float(self.get_parameter("target_distance").value)
        self.speed_ratio = float(self.get_parameter("speed_ratio").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            "/ackermann_cmd",
            10,
        )
        self.yolo_sub = self.create_subscription(
            YoloDetection3DArray,
            "/yolo/detections_3d",
            self.yolo_callback,
            10,
        )
        self.object_distance = None
        self.cmd_timer = self.create_timer(0.1, self.publish_command)

    def parameter_callback(self, params):
        for param in params:
            if param.name == "target_object_type":
                self.target_object_type = param.value
            elif param.name == "target_distance":
                self.target_distance = float(param.value)
            elif param.name == "speed_ratio":
                self.speed_ratio = float(param.value)
            elif param.name == "min_confidence":
                self.min_confidence = float(param.value)
        return True

    def yolo_callback(self, message):
        candidates = [
            detection
            for detection in message.detections
            if detection.class_name == self.target_object_type
            and detection.position_valid
            and detection.confidence >= self.min_confidence
        ]
        self.object_distance = (
            max(candidates, key=lambda detection: detection.confidence).z
            if candidates
            else None
        )

    def publish_command(self):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if self.object_distance is not None:
            error = self.object_distance - self.target_distance
            if abs(error) >= 0.05:
                speed = error * self.speed_ratio
                if abs(speed) >= 0.25:
                    command.drive.speed = float(
                        np.clip(abs(speed), 0.25, 1.0) * np.sign(speed)
                    )
        self.cmd_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = Demo1Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
