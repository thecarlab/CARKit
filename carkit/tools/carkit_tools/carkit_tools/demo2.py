#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

from carkit_perception_msgs.msg import YoloDetection3DArray


class Demo2Node(Node):
    def __init__(self):
        super().__init__("demo2_node")
        self.declare_parameter("target_object_type", "apple")
        self.declare_parameter("target_distance", 0.5)
        self.declare_parameter("speed_ratio", 1.5)
        self.declare_parameter("steering_ratio", 0.5)
        self.declare_parameter("max_steering_angle", 0.4)
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("center_angle_threshold", 0.1)
        self.target_object_type = self.get_parameter("target_object_type").value
        self.target_distance = float(self.get_parameter("target_distance").value)
        self.speed_ratio = float(self.get_parameter("speed_ratio").value)
        self.steering_ratio = float(self.get_parameter("steering_ratio").value)
        self.max_steering_angle = float(
            self.get_parameter("max_steering_angle").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.center_angle_threshold = float(
            self.get_parameter("center_angle_threshold").value
        )
        self.target_detection = None

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
        self.cmd_timer = self.create_timer(0.033, self.publish_command)

    def yolo_callback(self, message):
        candidates = [
            detection
            for detection in message.detections
            if detection.class_name == self.target_object_type
            and detection.position_valid
            and detection.confidence >= self.min_confidence
        ]
        self.target_detection = (
            max(candidates, key=lambda detection: detection.confidence)
            if candidates
            else None
        )

    def publish_command(self):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if self.target_detection is None:
            self.cmd_pub.publish(command)
            return

        angle = math.atan2(self.target_detection.x, self.target_detection.z)
        if abs(angle) > self.center_angle_threshold:
            command.drive.speed = 0.5
            command.drive.steering_angle = float(
                np.clip(
                    -angle * self.steering_ratio,
                    -self.max_steering_angle,
                    self.max_steering_angle,
                )
            )
        else:
            error = self.target_detection.z - self.target_distance
            if abs(error) >= 0.05:
                command.drive.speed = float(
                    np.clip(abs(error * self.speed_ratio), 0.5, 0.75)
                    * np.sign(error)
                )
        self.cmd_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = Demo2Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
