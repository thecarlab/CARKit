#!/usr/bin/env python3
"""Behavior center: stop sign, speed sign, and traffic light behaviors."""

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
VEHICLE_CLASSES = {"car", "truck", "bus", "motorcycle"}
STOP_DURATION_SEC = 3.0
POST_VEHICLE_WAIT_SEC = 3.0
SPEED_SIGN_DURATION_SEC = 3.0
COOLDOWN_SEC = 5.0
DETECTION_TIMEOUT_SEC = 0.5


class BehaviorCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("baseline_behavior_center_node")
        self.latest_msg = None
        self.latest_msg_time = None

        # Stop sign: idle | waiting_vehicle | post_vehicle_wait | timed_stop
        self.stop_mode = "idle"
        self.stop_until = None
        self.cooldown_until = None

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
        self.latest_msg = msg
        self.latest_msg_time = self.get_clock().now()
        self._update_speed_sign(msg)
        self._update_traffic_light(msg)
        self._arm_stop_sign(msg)

    def fresh_msg(self):
        if self.latest_msg is None or self.latest_msg_time is None:
            return None
        age = self.get_clock().now() - self.latest_msg_time
        if age > Duration(seconds=DETECTION_TIMEOUT_SEC):
            return None
        return self.latest_msg

    def has_vehicle(self, msg: YoloDetection2DArray) -> bool:
        return any(d.class_name in VEHICLE_CLASSES for d in msg.detections)

    def has_stop_sign(self, msg: YoloDetection2DArray) -> bool:
        return any(d.class_name == STOP_SIGN_CLASS for d in msg.detections)

    def _update_speed_sign(self, msg: YoloDetection2DArray) -> None:
        for detection in msg.detections:
            if not detection.class_name.startswith(SPEED_SIGN_PREFIX):
                continue
            try:
                limit = float(detection.class_name[len(SPEED_SIGN_PREFIX) :])
            except ValueError:
                return
            now = self.get_clock().now()
            self.speed_limit_mps = limit
            self.speed_limit_until = now + Duration(seconds=SPEED_SIGN_DURATION_SEC)
            self.get_logger().info(
                f"Speed sign -> {limit:.2f} m/s for {SPEED_SIGN_DURATION_SEC:.0f} s"
            )
            return

    def _update_speed_limit(self, now) -> None:
        if self.speed_limit_until is None:
            return
        if now >= self.speed_limit_until:
            self.speed_limit_mps = 0.0
            self.speed_limit_until = None
            self.get_logger().info("Speed sign override expired")

    def speed_sign_active(self, now) -> bool:
        return self.speed_limit_until is not None and now < self.speed_limit_until

    def _update_traffic_light(self, msg: YoloDetection2DArray) -> None:
        if not msg.traffic_lights:
            return
        color = max(
            msg.traffic_lights,
            key=lambda tl: tl.detection.confidence,
        ).traffic_light_color
        if color in (
            YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
            YoloTrafficLightDetection2D.TRAFFIC_LIGHT_YELLOW,
        ):
            if not self.traffic_light_stop:
                self.get_logger().info("Traffic light red/yellow -> stop")
            self.traffic_light_stop = True
        elif color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN:
            if self.traffic_light_stop:
                self.get_logger().info("Traffic light green -> go")
            self.traffic_light_stop = False

    def _arm_stop_sign(self, msg: YoloDetection2DArray) -> None:
        if not self.has_stop_sign(msg):
            return
        now = self.get_clock().now()
        if self.stop_mode != "idle":
            return
        if self.cooldown_until is not None and now < self.cooldown_until:
            return

        if self.has_vehicle(msg):
            self.stop_mode = "waiting_vehicle"
            self.get_logger().info(
                "Stop sign + vehicle -> stop until vehicle clears, then wait "
                f"{POST_VEHICLE_WAIT_SEC:.0f} s"
            )
        else:
            self.stop_mode = "timed_stop"
            self.stop_until = now + Duration(seconds=STOP_DURATION_SEC)
            self.get_logger().info(
                f"Stop sign -> stop {STOP_DURATION_SEC:.0f} s"
            )

    def _update_stop_sign(self, now) -> None:
        msg = self.fresh_msg()
        vehicle_present = self.has_vehicle(msg) if msg else False

        if self.stop_mode == "waiting_vehicle":
            if not vehicle_present:
                self.stop_mode = "post_vehicle_wait"
                self.stop_until = now + Duration(seconds=POST_VEHICLE_WAIT_SEC)
                self.get_logger().info(
                    f"Vehicle cleared -> wait {POST_VEHICLE_WAIT_SEC:.0f} s"
                )
        elif self.stop_mode == "post_vehicle_wait":
            if self.stop_until is not None and now >= self.stop_until:
                self._finish_stop_sign(now)
        elif self.stop_mode == "timed_stop":
            if self.stop_until is not None and now >= self.stop_until:
                self._finish_stop_sign(now)

    def _finish_stop_sign(self, now) -> None:
        self.stop_mode = "idle"
        self.stop_until = None
        self.cooldown_until = now + Duration(seconds=COOLDOWN_SEC)
        self.get_logger().info("Stop sign complete -> go")

    def stop_sign_active(self, now) -> bool:
        return self.stop_mode != "idle"

    def publish_override(self) -> None:
        now = self.get_clock().now()
        self._update_stop_sign(now)
        self._update_speed_limit(now)

        stop_active = self.traffic_light_stop or self.stop_sign_active(now)
        speed_active = self.speed_sign_active(now)
        if stop_active:
            if self.traffic_light_stop:
                state = "TRAFFIC_LIGHT"
            else:
                state = "STOP_SIGN"
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
        limit.data = float(self.speed_limit_mps) if speed_active else 0.0
        self.speed_limit_pub.publish(limit)

        self.state_pub.publish(String(data=state))


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
