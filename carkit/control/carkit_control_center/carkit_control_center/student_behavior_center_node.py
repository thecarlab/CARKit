#!/usr/bin/env python3
"""Student behavior center — fill in the TODO lines below.

Each behavior is a small function that takes simple inputs:
  - stop_sign_behavior(sees_stop_sign, now)   -> True while stopping
  - traffic_light_behavior(color, now)        -> True while stopping
  - speed_sign_behavior(speed_limit, now)     -> (active, limit_mps)

The YOLO message parsing is already done for you by the helper functions
at the top of this file, so the behaviors only deal with booleans and numbers.

How to work on this file (pick one level):
  Level A (no coding): uncomment the answer lines marked UNCOMMENT ME.
  Level B (beginner):  write the TODO lines yourself.
  Level C (practice):  delete each behavior body and write it from scratch.

Reference solution: baseline_behavior_center_node.py
Test publisher:     ros2 run carkit_perception mimic_perception
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


class StudentBehaviorCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("student_behavior_center_node")

        # State variables for your behaviors (do not remove).
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
            "student_behavior_center_node started; fill in the TODO lines below"
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

    # =========================================================================
    # STOP SIGN
    #
    # Inputs:  sees_stop_sign - True if the camera sees a stop sign right now
    #          now            - the current time
    # Goal:    see a stop sign -> stop for 3 seconds -> go.
    #
    # Steps:
    #   1. If we see a stop sign and no stop is running, start one:
    #        self.stop_until = now + Duration(seconds=STOP_DURATION_SEC)
    #   2. If the stop timer has finished, clear it:
    #        self.stop_until = None
    #   3. Return True while a stop is running, otherwise False.
    # =========================================================================
    def stop_sign_behavior(self, sees_stop_sign: bool, now) -> bool:
        if sees_stop_sign and self.stop_until is None:
            # TODO step 1: start a 3-second stop timer
            # UNCOMMENT ME: self.stop_until = now + Duration(seconds=STOP_DURATION_SEC)
            pass

        if self.stop_until is not None and now >= self.stop_until:
            # TODO step 2: the stop timer finished — clear it
            # UNCOMMENT ME: self.stop_until = None
            pass

        # TODO step 3: return True while stopping (hint: is self.stop_until set?)
        return False

    # =========================================================================
    # TRAFFIC LIGHT
    #
    # Inputs:  color - TRAFFIC_LIGHT_RED / YELLOW / GREEN, or None if no light
    #          now   - the current time
    # Goal:    red or yellow -> stop. Green -> go. No light -> keep last answer.
    # =========================================================================
    def traffic_light_behavior(self, color, now) -> bool:
        if color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED:
            # TODO: red means stop
            # UNCOMMENT ME: self.traffic_light_stop = True
            pass
        if color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_YELLOW:
            # TODO: yellow also means stop
            # UNCOMMENT ME: self.traffic_light_stop = True
            pass
        if color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN:
            # TODO: green means go
            # UNCOMMENT ME: self.traffic_light_stop = False
            pass

        return self.traffic_light_stop

    # =========================================================================
    # SPEED SIGN
    #
    # Inputs:  speed_limit - the limit in m/s from the sign, or None if no sign
    #          now         - the current time
    # Goal:    see a speed sign -> limit speed for 3 seconds -> back to normal.
    # =========================================================================
    def speed_sign_behavior(self, speed_limit, now):
        if speed_limit is not None:
            # TODO: save the limit and start a 3-second timer
            # UNCOMMENT ME: self.speed_limit_mps = speed_limit
            # UNCOMMENT ME: self.speed_limit_until = now + Duration(seconds=SPEED_SIGN_DURATION_SEC)
            pass

        if self.speed_limit_until is not None and now >= self.speed_limit_until:
            # TODO: the timer finished — clear the limit
            # UNCOMMENT ME: self.speed_limit_mps = 0.0
            # UNCOMMENT ME: self.speed_limit_until = None
            pass

        if self.speed_limit_until is not None:
            return True, self.speed_limit_mps
        else:
            return False, 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StudentBehaviorCenterNode()
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
