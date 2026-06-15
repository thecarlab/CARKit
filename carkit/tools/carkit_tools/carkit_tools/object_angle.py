#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from carkit_perception_msgs.msg import YoloDetection3DArray


class ObjectAngleNode(Node):
    def __init__(self):
        super().__init__("object_angle_node")
        self.declare_parameter("target_object_type", "bottle")
        self.declare_parameter("min_confidence", 0.2)
        self.target_object_type = self.get_parameter("target_object_type").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.angle_pub = self.create_publisher(Float32, "/object_angle", 10)
        self.yolo_sub = self.create_subscription(
            YoloDetection3DArray,
            "/yolo/detections_3d",
            self.yolo_callback,
            10,
        )
        self.get_logger().info(
            f"Object angle node tracking {self.target_object_type}"
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == "target_object_type":
                self.target_object_type = param.value
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
        if not candidates:
            return

        best = max(candidates, key=lambda detection: detection.confidence)
        angle = math.degrees(math.atan2(best.x, best.z))
        self.angle_pub.publish(Float32(data=float(angle)))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAngleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
