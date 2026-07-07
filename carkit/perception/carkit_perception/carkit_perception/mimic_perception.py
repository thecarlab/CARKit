#!/usr/bin/env python3
"""Interactive publisher that mimics /yolo/detections_2d for testing."""

import sys
import threading
import time

import rclpy
from carkit_perception_msgs.msg import (
    YoloDetection2D,
    YoloDetection2DArray,
    YoloTrafficLightDetection2D,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

DETECTION_TOPIC = "/yolo/detections_2d"
PUBLISH_HZ = 10.0
VEHICLE_STREAM_SEC = 3.0

HELP = """
Commands:
  1       publish stop sign once
  2       publish stop sign + vehicle for 3 s
  3 <spd> publish speed sign (e.g. 3 30)
  4       stop red light, continuously publish green light
  5       stop green light, continuously publish red light
  6       stop publishing traffic light
  q       quit
"""


def make_detection(class_name: str, confidence: float = 0.99) -> YoloDetection2D:
    msg = YoloDetection2D()
    msg.class_name = class_name
    msg.confidence = confidence
    msg.bbox_x_min = 280.0
    msg.bbox_y_min = 100.0
    msg.bbox_x_max = 360.0
    msg.bbox_y_max = 200.0
    return msg


def make_traffic_light(color: int) -> YoloTrafficLightDetection2D:
    msg = YoloTrafficLightDetection2D()
    msg.detection = make_detection("traffic light", 0.95)
    msg.traffic_light_color = color
    return msg


class MimicPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("mimic_perception")
        self.lock = threading.Lock()
        self.traffic_light_color = None
        self.vehicle_until = 0.0
        self.pending_once: list[YoloDetection2D] = []

        self.pub = self.create_publisher(YoloDetection2DArray, DETECTION_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_tick)
        self.get_logger().info(f"Publishing {DETECTION_TOPIC}")
        print(HELP.strip())

    def publish_tick(self) -> None:
        now = time.monotonic()
        detections: list[YoloDetection2D] = []
        traffic_lights: list[YoloTrafficLightDetection2D] = []

        with self.lock:
            detections.extend(self.pending_once)
            self.pending_once.clear()
            if now < self.vehicle_until:
                detections.append(make_detection("car"))
            color = self.traffic_light_color

        if color is not None:
            traffic_lights.append(make_traffic_light(color))
        if not detections and not traffic_lights:
            return

        msg = YoloDetection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.image_width = 640
        msg.image_height = 480
        msg.detections = detections
        msg.traffic_lights = traffic_lights
        self.pub.publish(msg)

    def handle_command(self, line: str) -> bool:
        parts = line.strip().split()
        if not parts:
            return True
        cmd = parts[0].lower()
        if cmd in ("q", "quit", "exit"):
            return False

        with self.lock:
            if cmd == "1":
                self.pending_once.append(make_detection("stop sign"))
                print("-> stop sign")
            elif cmd == "2":
                self.pending_once.append(make_detection("stop sign"))
                self.vehicle_until = time.monotonic() + VEHICLE_STREAM_SEC
                print(f"-> stop sign + car for {VEHICLE_STREAM_SEC:.0f} s")
            elif cmd == "3":
                if len(parts) < 2:
                    print("usage: 3 <speed>")
                    return True
                speed = parts[1]
                self.pending_once.append(make_detection(f"speed sign {speed}"))
                print(f"-> speed sign {speed}")
            elif cmd == "4":
                self.traffic_light_color = (
                    YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN
                )
                print("-> green traffic light (continuous)")
            elif cmd == "5":
                self.traffic_light_color = (
                    YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED
                )
                print("-> red traffic light (continuous)")
            elif cmd == "6":
                self.traffic_light_color = None
                print("-> traffic light stopped")
            else:
                print(f"unknown command: {line!r}")
        return True


def stdin_loop(node: MimicPerceptionNode) -> None:
    for line in sys.stdin:
        if not node.handle_command(line):
            rclpy.shutdown()
            break


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MimicPerceptionNode()
    thread = threading.Thread(target=stdin_loop, args=(node,), daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
