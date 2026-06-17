#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from carkit_perception_msgs.msg import YoloDetection3D, YoloDetection3DArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String


NORMAL_NAV2 = "NORMAL_NAV2"
STOP_SIGN = "STOP_SIGN"
TRAFFIC_LIGHT = "TRAFFIC_LIGHT"
CONE = "CONE"
AUTO_DRIVE = "AUTO_DRIVE"


class BehaviorCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("behavior_center_node")

        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("stop_sign_trigger_distance_m", 2.0)
        self.declare_parameter("stop_sign_stop_duration_sec", 5.0)
        self.declare_parameter("stop_sign_cooldown_sec", 10.0)
        self.declare_parameter("traffic_light_trigger_distance_m", 4.0)
        self.declare_parameter("traffic_light_hold_timeout_sec", 0.5)
        self.declare_parameter("cone_trigger_distance_m", 3.0)
        self.declare_parameter("cone_speed_limit_mps", 0.3)
        self.declare_parameter("cone_obstacle_radius_m", 0.25)
        self.declare_parameter("cone_ttl_sec", 1.0)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )
        self.stop_sign_trigger_distance_m = float(
            self.get_parameter("stop_sign_trigger_distance_m").value
        )
        self.stop_sign_stop_duration_sec = float(
            self.get_parameter("stop_sign_stop_duration_sec").value
        )
        self.stop_sign_cooldown_sec = float(
            self.get_parameter("stop_sign_cooldown_sec").value
        )
        self.traffic_light_trigger_distance_m = float(
            self.get_parameter("traffic_light_trigger_distance_m").value
        )
        self.traffic_light_hold_timeout_sec = float(
            self.get_parameter("traffic_light_hold_timeout_sec").value
        )
        self.cone_trigger_distance_m = float(
            self.get_parameter("cone_trigger_distance_m").value
        )
        self.cone_speed_limit_mps = float(
            self.get_parameter("cone_speed_limit_mps").value
        )
        self.cone_obstacle_radius_m = float(
            self.get_parameter("cone_obstacle_radius_m").value
        )
        self.cone_ttl_sec = float(self.get_parameter("cone_ttl_sec").value)

        self.main_state = ""
        self.latest_detections: Optional[YoloDetection3DArray] = None
        self.latest_detection_time = None
        self.latest_odom: Optional[Odometry] = None
        self.stop_until = 0.0
        self.stop_cooldown_until = 0.0
        self.traffic_light_hold_until = 0.0

        self.create_subscription(
            String,
            "/control_center/main_state",
            self.main_state_callback,
            10,
        )
        self.create_subscription(
            YoloDetection3DArray,
            "/yolo/detections_3d",
            self.detections_callback,
            10,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.state_pub = self.create_publisher(String, "/behavior/state", 10)
        self.override_active_pub = self.create_publisher(
            Bool,
            "/behavior/override_active",
            10,
        )
        self.override_cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            "/behavior/override_cmd",
            10,
        )
        self.speed_limit_pub = self.create_publisher(
            Float32,
            "/behavior/speed_limit",
            10,
        )
        self.cone_pub = self.create_publisher(
            PointCloud2,
            "/behavior/cone_obstacles",
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("behavior_center_node started")

    def main_state_callback(self, msg: String) -> None:
        self.main_state = msg.data

    def detections_callback(self, msg: YoloDetection3DArray) -> None:
        self.latest_detections = msg
        self.latest_detection_time = self.now_sec()

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def timer_callback(self) -> None:
        now = self.now_sec()
        state = NORMAL_NAV2
        override_active = False
        publish_override_cmd = False

        if self.main_state != AUTO_DRIVE:
            self.publish_state(state, override_active)
            self.publish_cones([], Header())
            return

        if now < self.stop_until:
            state = STOP_SIGN
            override_active = True
            publish_override_cmd = True
            detections = self.fresh_detections(now)
            if detections is not None:
                self.publish_cones([], detections.header)
            else:
                self.publish_cones([], Header())
        else:
            detections = self.fresh_detections(now)
            if detections is None:
                if now < self.traffic_light_hold_until:
                    state = TRAFFIC_LIGHT
                    override_active = True
                    publish_override_cmd = True
                self.publish_state(state, override_active)
                self.publish_cones([], Header())
                if publish_override_cmd:
                    self.override_cmd_pub.publish(self.zero_command())
                return

            if self.stop_sign_triggered(detections, now):
                state = STOP_SIGN
                override_active = True
                publish_override_cmd = True
                self.stop_until = now + self.stop_sign_stop_duration_sec
                self.stop_cooldown_until = (
                    self.stop_until + self.stop_sign_cooldown_sec
                )
            elif self.traffic_light_stop_active(detections, now):
                state = TRAFFIC_LIGHT
                override_active = True
                publish_override_cmd = True
            else:
                cone_points = self.cone_points(detections)
                if cone_points:
                    state = CONE
                    self.publish_cones(cone_points, detections.header)
                    speed_limit = Float32()
                    speed_limit.data = float(self.cone_speed_limit_mps)
                    self.speed_limit_pub.publish(speed_limit)
                else:
                    self.publish_cones([], detections.header)

        self.publish_state(state, override_active)
        if publish_override_cmd:
            self.override_cmd_pub.publish(self.zero_command())

    def fresh_detections(self, now: float) -> Optional[YoloDetection3DArray]:
        if self.latest_detections is None or self.latest_detection_time is None:
            return None
        if now - self.latest_detection_time > self.detection_timeout_sec:
            return None
        return self.latest_detections

    def stop_sign_triggered(self, msg: YoloDetection3DArray, now: float) -> bool:
        if now < self.stop_cooldown_until:
            return False
        for detection in msg.detections:
            if "stop" not in detection.class_name.lower():
                continue
            if not self.detection_in_front_within(
                detection,
                self.stop_sign_trigger_distance_m,
            ):
                continue
            if detection.confidence >= self.min_confidence:
                return True
        return False

    def traffic_light_stop_active(
        self,
        msg: YoloDetection3DArray,
        now: float,
    ) -> bool:
        saw_green = False
        for detection in msg.detections:
            if "traffic light" not in detection.class_name.lower():
                continue
            if detection.confidence < self.min_confidence:
                continue
            if not self.detection_in_front_within(
                detection,
                self.traffic_light_trigger_distance_m,
            ):
                continue
            if detection.traffic_light_color in (
                YoloDetection3D.TRAFFIC_LIGHT_RED,
                YoloDetection3D.TRAFFIC_LIGHT_YELLOW,
            ):
                self.traffic_light_hold_until = (
                    now + self.traffic_light_hold_timeout_sec
                )
                return True
            if detection.traffic_light_color == YoloDetection3D.TRAFFIC_LIGHT_GREEN:
                saw_green = True

        if saw_green:
            self.traffic_light_hold_until = 0.0
            return False
        return now < self.traffic_light_hold_until

    def cone_points(self, msg: YoloDetection3DArray) -> list[list[float]]:
        points = []
        for detection in msg.detections:
            if "cone" not in detection.class_name.lower():
                continue
            if detection.confidence < self.min_confidence:
                continue
            if not self.detection_in_front_within(
                detection,
                self.cone_trigger_distance_m,
            ):
                continue
            points.extend(self.cone_obstacle_points(detection))
        return points

    def cone_obstacle_points(self, detection) -> list[list[float]]:
        x = float(detection.x)
        y = float(detection.y)
        z = float(detection.z)
        radius = max(0.0, self.cone_obstacle_radius_m)
        if radius == 0.0:
            return [[x, y, z]]
        return [
            [x, y, z],
            [x - radius, y, z],
            [x + radius, y, z],
            [x, y, z - radius],
            [x, y, z + radius],
        ]

    def detection_in_front_within(self, detection, max_distance: float) -> bool:
        if not detection.position_valid:
            return False
        if not all(
            math.isfinite(value)
            for value in (detection.x, detection.y, detection.z)
        ):
            return False
        if detection.z <= 0.0:
            return False
        distance = math.hypot(float(detection.x), float(detection.z))
        return distance <= max_distance

    def publish_state(self, state: str, override_active: bool) -> None:
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        active_msg = Bool()
        active_msg.data = bool(override_active)
        self.override_active_pub.publish(active_msg)

    def publish_cones(self, points: list[list[float]], header: Header) -> None:
        cloud_header = Header()
        cloud_header.stamp = self.get_clock().now().to_msg()
        cloud_header.frame_id = header.frame_id or "camera_color_optical_frame"
        self.cone_pub.publish(point_cloud2.create_cloud_xyz32(cloud_header, points))

    def zero_command(self) -> AckermannDriveStamped:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        return msg

    def now_sec(self) -> float:
        return stamp_to_sec(self.get_clock().now().to_msg())


def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


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
