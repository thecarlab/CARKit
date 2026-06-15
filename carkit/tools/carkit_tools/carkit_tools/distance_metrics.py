#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from carkit_perception_msgs.msg import YoloDetection3DArray


class DistanceMetricsNode(Node):
    def __init__(self):
        super().__init__("distance_metrics_node")
        self.declare_parameter("target_object_type", "bottle")
        self.declare_parameter("min_confidence", 0.4)
        self.target_object_type = self.get_parameter("target_object_type").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.avg_dist_pub = self.create_publisher(
            Float32,
            "/average_distance",
            10,
        )
        self.min_dist_pub = self.create_publisher(Float32, "/min_distance", 10)
        self.max_dist_pub = self.create_publisher(Float32, "/max_distance", 10)
        self.median_dist_pub = self.create_publisher(
            Float32,
            "/median_distance",
            10,
        )
        self.yolo_sub = self.create_subscription(
            YoloDetection3DArray,
            "/yolo/detections_3d",
            self.yolo_callback,
            10,
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == "target_object_type":
                self.target_object_type = param.value
            elif param.name == "min_confidence":
                self.min_confidence = float(param.value)
        return True

    def yolo_callback(self, message):
        distances = [
            detection.z
            for detection in message.detections
            if detection.class_name == self.target_object_type
            and detection.position_valid
            and detection.confidence >= self.min_confidence
        ]
        if not distances:
            return

        average = sum(distances) / len(distances)
        ordered = sorted(distances)
        midpoint = len(ordered) // 2
        if len(ordered) % 2:
            median = ordered[midpoint]
        else:
            median = (ordered[midpoint - 1] + ordered[midpoint]) / 2.0

        self.avg_dist_pub.publish(Float32(data=float(average)))
        self.min_dist_pub.publish(Float32(data=float(min(distances))))
        self.max_dist_pub.publish(Float32(data=float(max(distances))))
        self.median_dist_pub.publish(Float32(data=float(median)))


def main(args=None):
    rclpy.init(args=args)
    node = DistanceMetricsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
