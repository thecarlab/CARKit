#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from carkit_perception_msgs.msg import (
    YoloDetection2D,
    YoloDetection2DArray,
    YoloTrafficLightDetection2D,
)
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String


NORMAL_NAV2 = "NORMAL_NAV2"
STOP_SIGN = "STOP_SIGN"
TRAFFIC_LIGHT = "TRAFFIC_LIGHT"
CONE = "CONE"
AUTO_DRIVE = "AUTO_DRIVE"


class ObjectLocation:
    __slots__ = ("range_m", "scan_angle_rad", "x", "y")

    def __init__(
        self,
        range_m: float,
        scan_angle_rad: float,
        x: float,
        y: float,
    ) -> None:
        self.range_m = range_m
        self.scan_angle_rad = scan_angle_rad
        self.x = x
        self.y = y


class BehaviorCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("behavior_center_node")

        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.declare_parameter("camera_horizontal_fov_deg", 69.4)
        self.declare_parameter("camera_to_scan_yaw_offset_rad", 0.0)
        self.declare_parameter("camera_forward_offset_m", 0.08)
        self.declare_parameter("camera_lateral_offset_m", 0.0)
        self.declare_parameter("stop_sign_trigger_distance_m", 2.0)
        self.declare_parameter("stop_sign_lidar_angle_window_deg", 8.0)
        self.declare_parameter("stop_sign_lidar_min_range_m", 0.15)
        self.declare_parameter("stop_sign_lidar_max_range_m", 10.0)
        self.declare_parameter("stop_sign_stop_duration_sec", 5.0)
        self.declare_parameter("stop_sign_cooldown_sec", 10.0)
        self.declare_parameter("traffic_light_min_bbox_height_ratio", 0.06)
        self.declare_parameter("traffic_light_hold_timeout_sec", 0.5)
        self.declare_parameter("cone_trigger_distance_m", 3.0)
        self.declare_parameter("cone_lidar_angle_window_deg", 6.0)
        self.declare_parameter("cone_lidar_min_range_m", 0.15)
        self.declare_parameter("cone_lidar_max_range_m", 10.0)
        self.declare_parameter("cone_speed_limit_mps", 0.3)
        self.declare_parameter("cone_obstacle_radius_m", 0.25)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )
        self.scan_timeout_sec = float(
            self.get_parameter("scan_timeout_sec").value
        )
        self.camera_horizontal_fov_rad = math.radians(
            float(self.get_parameter("camera_horizontal_fov_deg").value)
        )
        self.camera_to_scan_yaw_offset_rad = float(
            self.get_parameter("camera_to_scan_yaw_offset_rad").value
        )
        self.camera_forward_offset_m = float(
            self.get_parameter("camera_forward_offset_m").value
        )
        self.camera_lateral_offset_m = float(
            self.get_parameter("camera_lateral_offset_m").value
        )
        self.stop_sign_trigger_distance_m = float(
            self.get_parameter("stop_sign_trigger_distance_m").value
        )
        self.stop_sign_lidar_angle_window_rad = math.radians(
            float(self.get_parameter("stop_sign_lidar_angle_window_deg").value)
        )
        self.stop_sign_lidar_min_range_m = float(
            self.get_parameter("stop_sign_lidar_min_range_m").value
        )
        self.stop_sign_lidar_max_range_m = float(
            self.get_parameter("stop_sign_lidar_max_range_m").value
        )
        self.stop_sign_stop_duration_sec = float(
            self.get_parameter("stop_sign_stop_duration_sec").value
        )
        self.stop_sign_cooldown_sec = float(
            self.get_parameter("stop_sign_cooldown_sec").value
        )
        self.traffic_light_min_bbox_height_ratio = float(
            self.get_parameter("traffic_light_min_bbox_height_ratio").value
        )
        self.traffic_light_hold_timeout_sec = float(
            self.get_parameter("traffic_light_hold_timeout_sec").value
        )
        self.cone_trigger_distance_m = float(
            self.get_parameter("cone_trigger_distance_m").value
        )
        self.cone_lidar_angle_window_rad = math.radians(
            float(self.get_parameter("cone_lidar_angle_window_deg").value)
        )
        self.cone_lidar_min_range_m = float(
            self.get_parameter("cone_lidar_min_range_m").value
        )
        self.cone_lidar_max_range_m = float(
            self.get_parameter("cone_lidar_max_range_m").value
        )
        self.cone_speed_limit_mps = float(
            self.get_parameter("cone_speed_limit_mps").value
        )
        self.cone_obstacle_radius_m = float(
            self.get_parameter("cone_obstacle_radius_m").value
        )

        self.main_state = ""
        self.latest_detections: Optional[YoloDetection2DArray] = None
        self.latest_detection_time = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = None
        self.camera_image_width: Optional[float] = None
        self.camera_image_height: Optional[float] = None
        self.camera_cx: Optional[float] = None
        self.camera_fx: Optional[float] = None
        self.stop_until = 0.0
        self.stop_cooldown_until = 0.0
        self.traffic_light_hold_until = 0.0
        self.last_stop_sign_debug_sec = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String,
            "/control_center/main_state",
            self.main_state_callback,
            10,
        )
        self.create_subscription(
            YoloDetection2DArray,
            "/yolo/detections_2d",
            self.detections_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            sensor_qos,
        )

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
        self.stop_sign_position_pub = self.create_publisher(
            PointStamped,
            "/behavior/stop_sign_position",
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            "behavior_center_node started with 2D detections"
        )

    def main_state_callback(self, msg: String) -> None:
        self.main_state = msg.data

    def detections_callback(self, msg: YoloDetection2DArray) -> None:
        self.latest_detections = msg
        self.latest_detection_time = self.now_sec()

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = self.now_sec()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if msg.width <= 0 or msg.height <= 0:
            return
        self.camera_image_width = float(msg.width)
        self.camera_image_height = float(msg.height)
        self.camera_cx = float(msg.k[2])
        self.camera_fx = float(msg.k[0])

    def timer_callback(self) -> None:
        now = self.now_sec()
        state = NORMAL_NAV2
        override_active = False
        publish_override_cmd = False

        if self.main_state != AUTO_DRIVE:
            self.publish_state(state, override_active)
            self.publish_cones([], None)
            return

        scan = self.fresh_scan(now)
        detections = self.fresh_detections(now)

        if now < self.stop_until:
            state = STOP_SIGN
            override_active = True
            publish_override_cmd = True
            if detections is not None:
                self.publish_stop_sign_from_detections(detections, scan)
            self.publish_cones([], scan)
        elif detections is None:
            if now < self.traffic_light_hold_until:
                state = TRAFFIC_LIGHT
                override_active = True
                publish_override_cmd = True
            self.publish_cones([], scan)
        else:
            self.publish_stop_sign_from_detections(detections, scan)
            if self.stop_sign_triggered(detections, now, scan):
                state = STOP_SIGN
                override_active = True
                publish_override_cmd = True
                self.stop_until = now + self.stop_sign_stop_duration_sec
                self.stop_cooldown_until = (
                    self.stop_until + self.stop_sign_cooldown_sec
                )
                self.publish_cones([], scan)
            elif self.traffic_light_stop_active(detections, now):
                state = TRAFFIC_LIGHT
                override_active = True
                publish_override_cmd = True
                self.publish_cones([], scan)
            else:
                cone_points = self.cone_points(detections, scan)
                if cone_points:
                    state = CONE
                    self.publish_cones(cone_points, scan)
                    speed_limit = Float32()
                    speed_limit.data = float(self.cone_speed_limit_mps)
                    self.speed_limit_pub.publish(speed_limit)
                else:
                    self.publish_cones([], scan)

        self.publish_state(state, override_active)
        if publish_override_cmd:
            self.override_cmd_pub.publish(self.zero_command())

    def fresh_detections(self, now: float) -> Optional[YoloDetection2DArray]:
        if (
            self.latest_detections is None
            or self.latest_detection_time is None
        ):
            return None
        if now - self.latest_detection_time > self.detection_timeout_sec:
            return None
        return self.latest_detections

    def fresh_scan(self, now: float) -> Optional[LaserScan]:
        if self.latest_scan is None or self.latest_scan_time is None:
            return None
        if now - self.latest_scan_time > self.scan_timeout_sec:
            return None
        return self.latest_scan

    def best_stop_sign_detection(
        self,
        msg: YoloDetection2DArray,
    ) -> Optional[YoloDetection2D]:
        candidates = [
            detection
            for detection in msg.detections
            if "stop" in detection.class_name.lower()
            and detection.confidence >= self.min_confidence
        ]
        return max(candidates, key=lambda item: item.confidence, default=None)

    def detection_horizontal_bearing_rad(
        self,
        detection: YoloDetection2D,
        image_width: float = 0.0,
    ) -> Optional[float]:
        center_x = (detection.bbox_x_min + detection.bbox_x_max) / 2.0
        if self.camera_fx is not None and self.camera_cx is not None:
            if self.camera_fx > 0.0:
                return math.atan2(center_x - self.camera_cx, self.camera_fx)

        width = image_width or self.camera_image_width
        if width is None or width <= 0.0:
            return None
        normalized_pos = (center_x / width) - 0.5
        return normalized_pos * self.camera_horizontal_fov_rad

    def camera_bearing_to_scan_angle(self, camera_bearing_rad: float) -> float:
        return normalize_angle(
            math.pi
            - camera_bearing_rad
            + self.camera_to_scan_yaw_offset_rad
        )

    def localize_detection(
        self,
        detection: YoloDetection2D,
        image_width: float,
        scan: LaserScan,
        angle_window_rad: float,
        min_range_m: float,
        max_range_m: float,
    ) -> Optional[ObjectLocation]:
        camera_bearing = self.detection_horizontal_bearing_rad(
            detection,
            image_width,
        )
        if camera_bearing is None:
            return None
        target_angle = self.camera_bearing_to_scan_angle(camera_bearing)
        lidar_hit = self.find_lidar_hit_at_bearing(
            scan,
            target_angle,
            angle_window_rad,
            min_range_m,
            max_range_m,
        )
        if lidar_hit is None:
            return None
        range_m, scan_angle_rad = lidar_hit
        return ObjectLocation(
            range_m=range_m,
            scan_angle_rad=scan_angle_rad,
            x=range_m * math.cos(scan_angle_rad),
            y=range_m * math.sin(scan_angle_rad),
        )

    def find_lidar_hit_at_bearing(
        self,
        scan: LaserScan,
        target_angle: float,
        angle_window_rad: float,
        min_range_m: float,
        max_range_m: float,
    ) -> Optional[tuple[float, float]]:
        half_window = angle_window_rad / 2.0
        best_range = None
        best_angle = target_angle
        front_angle = math.pi + self.camera_to_scan_yaw_offset_rad
        target_camera_bearing = normalize_angle(front_angle - target_angle)
        camera_x = (
            self.camera_forward_offset_m * math.cos(front_angle)
            + self.camera_lateral_offset_m
            * math.cos(front_angle + math.pi / 2.0)
        )
        camera_y = (
            self.camera_forward_offset_m * math.sin(front_angle)
            + self.camera_lateral_offset_m
            * math.sin(front_angle + math.pi / 2.0)
        )
        for index, raw_range in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if not lidar_range_valid(
                raw_range,
                scan,
                min_range_m,
                max_range_m,
            ):
                continue
            point_x = raw_range * math.cos(angle)
            point_y = raw_range * math.sin(angle)
            angle_from_camera = math.atan2(
                point_y - camera_y,
                point_x - camera_x,
            )
            candidate_camera_bearing = normalize_angle(
                front_angle - angle_from_camera
            )
            if (
                abs(
                    angle_diff(
                        candidate_camera_bearing,
                        target_camera_bearing,
                    )
                )
                > half_window
            ):
                continue
            if best_range is None or raw_range < best_range:
                best_range = float(raw_range)
                best_angle = angle
        if best_range is None:
            return None
        return best_range, best_angle

    def localize_stop_sign(
        self,
        detection: YoloDetection2D,
        image_width: float,
        scan: LaserScan,
    ) -> Optional[ObjectLocation]:
        return self.localize_detection(
            detection,
            image_width,
            scan,
            self.stop_sign_lidar_angle_window_rad,
            self.stop_sign_lidar_min_range_m,
            self.stop_sign_lidar_max_range_m,
        )

    def publish_stop_sign_from_detections(
        self,
        detections: YoloDetection2DArray,
        scan: Optional[LaserScan],
    ) -> None:
        if scan is None:
            return
        detection = self.best_stop_sign_detection(detections)
        if detection is None:
            return
        location = self.localize_stop_sign(
            detection,
            float(detections.image_width),
            scan,
        )
        if location is None:
            return
        msg = PointStamped()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = scan.header.frame_id or "laser"
        msg.point.x = float(location.x)
        msg.point.y = float(location.y)
        self.stop_sign_position_pub.publish(msg)

    def stop_sign_triggered(
        self,
        msg: YoloDetection2DArray,
        now: float,
        scan: Optional[LaserScan],
    ) -> bool:
        if now < self.stop_cooldown_until or scan is None:
            return False
        detection = self.best_stop_sign_detection(msg)
        if detection is None:
            return False
        location = self.localize_stop_sign(
            detection,
            float(msg.image_width),
            scan,
        )
        if location is None:
            return False
        if now - self.last_stop_sign_debug_sec >= 1.0:
            self.last_stop_sign_debug_sec = now
            self.get_logger().info(
                "Stop sign localized at "
                f"{location.range_m:.2f} m "
                f"(trigger <= {self.stop_sign_trigger_distance_m:.2f} m)"
            )
        return location.range_m <= self.stop_sign_trigger_distance_m

    def traffic_light_is_near(
        self,
        detection: YoloDetection2D,
        image_height: float,
    ) -> bool:
        height = image_height or self.camera_image_height
        if height is None or height <= 0.0:
            return False
        bbox_height = max(0.0, detection.bbox_y_max - detection.bbox_y_min)
        return bbox_height / height >= self.traffic_light_min_bbox_height_ratio

    def traffic_light_stop_active(
        self,
        msg: YoloDetection2DArray,
        now: float,
    ) -> bool:
        saw_green = False
        for traffic_light in msg.traffic_lights:
            detection = traffic_light.detection
            if detection.confidence < self.min_confidence:
                continue
            if not self.traffic_light_is_near(
                detection,
                float(msg.image_height),
            ):
                continue
            if traffic_light.traffic_light_color in (
                YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
                YoloTrafficLightDetection2D.TRAFFIC_LIGHT_YELLOW,
            ):
                self.traffic_light_hold_until = (
                    now + self.traffic_light_hold_timeout_sec
                )
                return True
            if (
                traffic_light.traffic_light_color
                == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN
            ):
                saw_green = True
        if saw_green:
            self.traffic_light_hold_until = 0.0
            return False
        return now < self.traffic_light_hold_until

    def cone_points(
        self,
        msg: YoloDetection2DArray,
        scan: Optional[LaserScan],
    ) -> list[list[float]]:
        if scan is None:
            return []
        points = []
        for detection in msg.detections:
            if "cone" not in detection.class_name.lower():
                continue
            if detection.confidence < self.min_confidence:
                continue
            location = self.localize_detection(
                detection,
                float(msg.image_width),
                scan,
                self.cone_lidar_angle_window_rad,
                self.cone_lidar_min_range_m,
                self.cone_lidar_max_range_m,
            )
            if (
                location is None
                or location.range_m > self.cone_trigger_distance_m
            ):
                continue
            points.extend(self.cone_obstacle_points(location))
        return points

    def cone_obstacle_points(
        self,
        location: ObjectLocation,
    ) -> list[list[float]]:
        x = float(location.x)
        y = float(location.y)
        radius = max(0.0, self.cone_obstacle_radius_m)
        if radius == 0.0:
            return [[x, y, 0.0]]
        return [
            [x, y, 0.0],
            [x - radius, y, 0.0],
            [x + radius, y, 0.0],
            [x, y - radius, 0.0],
            [x, y + radius, 0.0],
        ]

    def publish_state(self, state: str, override_active: bool) -> None:
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)
        active_msg = Bool()
        active_msg.data = bool(override_active)
        self.override_active_pub.publish(active_msg)

    def publish_cones(
        self,
        points: list[list[float]],
        scan: Optional[LaserScan],
    ) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "laser"
        if scan is not None and scan.header.frame_id:
            header.frame_id = scan.header.frame_id
        self.cone_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def zero_command(self) -> AckermannDriveStamped:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        return msg

    def now_sec(self) -> float:
        return stamp_to_sec(self.get_clock().now().to_msg())


def lidar_range_valid(
    raw_range: float,
    scan: LaserScan,
    min_range_m: float,
    max_range_m: float,
) -> bool:
    if not math.isfinite(raw_range):
        return False
    if raw_range <= scan.range_min or raw_range >= scan.range_max:
        return False
    return min_range_m <= raw_range <= max_range_m


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_diff(left: float, right: float) -> float:
    return normalize_angle(left - right)


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
