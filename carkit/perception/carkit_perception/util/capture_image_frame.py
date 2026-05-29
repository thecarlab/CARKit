#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameCapture(Node):
    def __init__(self, topic: str, output_path: Path) -> None:
        super().__init__("frame_capture")
        self.bridge = CvBridge()
        self.output_path = output_path
        self.subscription = self.create_subscription(Image, topic, self.on_image, 10)
        self.get_logger().info(f"Waiting for one frame on {topic}")

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_path), frame)
        self.get_logger().info(f"Saved frame to {self.output_path}")
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Save one ROS image topic frame to disk.")
    parser.add_argument(
        "--topic",
        default="/camera/camera/color/image_raw",
        help="ROS image topic to subscribe to.",
    )
    parser.add_argument(
        "--output",
        default="realsense_frame.jpg",
        help="Output image path.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = FrameCapture(args.topic, Path(args.output).expanduser())
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
