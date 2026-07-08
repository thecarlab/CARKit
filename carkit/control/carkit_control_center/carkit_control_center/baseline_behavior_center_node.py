#!/usr/bin/env python3
"""Behavior center: beginner stop sign, speed sign, and traffic light behaviors.

Each behavior is a small function that takes simple inputs:
  - stop_sign_behavior(sees_stop_sign, now)   -> True while stopping
  - traffic_light_behavior(color, now)        -> True while stopping
  - speed_sign_behavior(speed_limit, now)     -> (active, limit_mps)

The YOLO message parsing happens in the helper functions at the top,
so the behaviors themselves only deal with booleans and numbers.
"""

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from carkit_perception_msgs.msg import (
    YoloDetection2DArray,
    YoloTrafficLightDetection2D,
)
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

PERCEPTION_TOPIC = "/yolo/detections_2d"
STOP_SIGN_CLASS = "stop sign"
SPEED_SIGN_PREFIX = "speed sign "
STOP_DURATION_SEC = 3.0
SPEED_SIGN_DURATION_SEC = 3.0


def sees_stop_sign(msg: YoloDetection2DArray) -> bool:
    """Return True if the YOLO message contains a stop sign."""
    for detection in msg.detections:
        if detection.class_name == STOP_SIGN_CLASS:
            return True
    return False


def get_traffic_light_color(msg: YoloDetection2DArray):
    """Return the color of the highest-confidence traffic light, or None."""
    if not msg.traffic_lights:
        return None
    best = msg.traffic_lights[0]
    for traffic_light in msg.traffic_lights:
        if traffic_light.detection.confidence > best.detection.confidence:
            best = traffic_light
    return best.traffic_light_color


def get_speed_limit(msg: YoloDetection2DArray):
    """Return the speed limit in m/s from a 'speed sign X' detection, or None."""
    for detection in msg.detections:
        if not detection.class_name.startswith(SPEED_SIGN_PREFIX):
            continue
        try:
            return float(detection.class_name[len(SPEED_SIGN_PREFIX) :])
        except ValueError:
            return None
    return None


class BehaviorCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("baseline_behavior_center_node")

        self.stop_until = None
        self.traffic_light_stop = False
        self.speed_limit_mps = 0.0
        self.speed_limit_until = None

        self.create_subscription(
            YoloDetection2DArray, PERCEPTION_TOPIC, self.detections_callback, 10
        )
        self.active_pub = self.create_publisher(Bool, "/behavior/override_active", 10)
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, "/behavior/override_cmd", 10
        )
        self.speed_limit_pub = self.create_publisher(
            Float32, "/behavior/speed_limit", 10
        )
        self.state_pub = self.create_publisher(String, "/behavior/state", 10)
        self.create_timer(0.05, self.publish_override)
        self.get_logger().info(
            "baseline_behavior_center_node started; publishes "
            "/behavior/state, /behavior/override_active, /behavior/speed_limit"
        )

    def detections_callback(self, msg: YoloDetection2DArray) -> None:
        """A new camera frame arrived: feed simple facts to each behavior."""
        now = self.get_clock().now()
        self.stop_sign_behavior(sees_stop_sign(msg), now)
        self.traffic_light_behavior(get_traffic_light_color(msg), now)
        self.speed_sign_behavior(get_speed_limit(msg), now)

    def publish_override(self) -> None:
        """Timer tick (20x per second): ask each behavior if it is active."""
        now = self.get_clock().now()
        stop_sign_active = self.stop_sign_behavior(False, now)
        traffic_light_active = self.traffic_light_behavior(None, now)
        speed_active, speed_limit_mps = self.speed_sign_behavior(None, now)

        stop_active = traffic_light_active or stop_sign_active
        if stop_active:
            state = "TRAFFIC_LIGHT" if traffic_light_active else "STOP_SIGN"
        elif speed_active:
            state = "SPEED_SIGN"
        else:
            state = "NORMAL"

        self.active_pub.publish(Bool(data=stop_active))
        if stop_active:
            stop = AckermannDriveStamped()
            stop.header.stamp = now.to_msg()
            self.cmd_pub.publish(stop)

        limit = Float32()
        limit.data = float(speed_limit_mps)
        self.speed_limit_pub.publish(limit)
        self.state_pub.publish(String(data=state))

    def stop_sign_behavior(self, sees_stop_sign: bool, now) -> bool:
        """See a stop sign -> stop for 3 seconds -> go."""
        if sees_stop_sign and self.stop_until is None:
            self.stop_until = now + Duration(seconds=STOP_DURATION_SEC)

        if self.stop_until is not None and now >= self.stop_until:
            self.stop_until = None

        if self.stop_until is not None:
            return True
        else:
            return False

    def traffic_light_behavior(self, color, now) -> bool:
        """Red or yellow -> stop. Green -> go. No light -> keep last answer."""
        if color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED:
            self.traffic_light_stop = True
        if color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_YELLOW:
            self.traffic_light_stop = True
        if color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN:
            self.traffic_light_stop = False

        return self.traffic_light_stop

    def speed_sign_behavior(self, speed_limit, now):
        """See a speed sign -> limit speed for 3 seconds -> back to normal."""
        if speed_limit is not None:
            self.speed_limit_mps = speed_limit
            self.speed_limit_until = now + Duration(seconds=SPEED_SIGN_DURATION_SEC)

        if self.speed_limit_until is not None and now >= self.speed_limit_until:
            self.speed_limit_mps = 0.0
            self.speed_limit_until = None

        if self.speed_limit_until is not None:
            return True, self.speed_limit_mps
        else:
            return False, 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BehaviorCenterNode()
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
