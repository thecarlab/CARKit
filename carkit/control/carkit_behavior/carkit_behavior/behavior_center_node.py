#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from carkit_perception_msgs.msg import (
    YoloDetection2D,
    YoloDetection2DArray,
)
from carkit_behavior.path_geometry import (
    distance_along_path,
    should_stop_before_line,
)
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String
from tf2_ros import Buffer, TransformException, TransformListener


NORMAL_NAV2 = "NORMAL_NAV2"
STOP_SIGN = "STOP_SIGN"
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


class MapPoint:
    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class StopSignTrack:
    __slots__ = (
        "x",
        "y",
        "weight",
        "observations",
        "stopped",
        "rearm_for_new_plan",
        "last_distance_m",
        "min_distance_m",
    )

    def __init__(self, x: float, y: float, confidence: float) -> None:
        self.x = x
        self.y = y
        self.weight = max(0.01, confidence)
        self.observations = 1
        self.stopped = False
        self.rearm_for_new_plan = False
        self.last_distance_m: Optional[float] = None
        self.min_distance_m: Optional[float] = None

    def update(self, x: float, y: float, confidence: float) -> None:
        weight = max(0.01, confidence)
        total_weight = self.weight + weight
        self.x = (self.x * self.weight + x * weight) / total_weight
        self.y = (self.y * self.weight + y * weight) / total_weight
        self.weight = total_weight
        self.observations += 1


class BehaviorCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("behavior_center_node")

        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.declare_parameter("camera_horizontal_fov_deg", 69.4)
        self.declare_parameter("camera_to_scan_yaw_offset_rad", 0.0)
        self.declare_parameter("camera_forward_offset_m", 0.08)
        self.declare_parameter("camera_lateral_offset_m", 0.0)
        self.declare_parameter("global_plan_topic", "/plan")
        self.declare_parameter("stop_sign_stop_before_distance_m", 0.5)
        self.declare_parameter("stop_sign_stop_line_tolerance_m", 0.25)
        self.declare_parameter("stop_sign_rearm_distance_m", 1.0)
        self.declare_parameter("stop_sign_lidar_angle_window_deg", 8.0)
        self.declare_parameter("stop_sign_lidar_min_range_m", 0.15)
        self.declare_parameter("stop_sign_lidar_max_range_m", 10.0)
        self.declare_parameter("stop_sign_stop_duration_sec", 5.0)
        self.declare_parameter("stop_sign_cooldown_sec", 10.0)
        self.declare_parameter("stop_sign_min_confidence", 0.75)
        self.declare_parameter("stop_sign_map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("stop_sign_required_observations", 3)
        self.declare_parameter("stop_sign_track_match_distance_m", 1.0)
        self.declare_parameter("stop_sign_clear_distance_m", 10.0)
        self.declare_parameter("plan_goal_change_distance_m", 0.25)
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
        self.stop_sign_stop_before_distance_m = float(
            self.get_parameter("stop_sign_stop_before_distance_m").value
        )
        self.stop_sign_stop_line_tolerance_m = float(
            self.get_parameter("stop_sign_stop_line_tolerance_m").value
        )
        self.stop_sign_rearm_distance_m = float(
            self.get_parameter("stop_sign_rearm_distance_m").value
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
        self.stop_sign_min_confidence = float(
            self.get_parameter("stop_sign_min_confidence").value
        )
        self.stop_sign_map_frame = str(
            self.get_parameter("stop_sign_map_frame").value
        )
        self.robot_base_frame = str(
            self.get_parameter("robot_base_frame").value
        )
        self.stop_sign_required_observations = int(
            self.get_parameter("stop_sign_required_observations").value
        )
        self.stop_sign_track_match_distance_m = float(
            self.get_parameter("stop_sign_track_match_distance_m").value
        )
        self.stop_sign_clear_distance_m = float(
            self.get_parameter("stop_sign_clear_distance_m").value
        )
        self.plan_goal_change_distance_m = float(
            self.get_parameter("plan_goal_change_distance_m").value
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
        self.last_stop_sign_debug_sec = 0.0
        self.current_velocity_mps: Optional[float] = None
        self.current_pose_frame: Optional[str] = None
        self.current_robot_x: Optional[float] = None
        self.current_robot_y: Optional[float] = None
        self.current_robot_yaw: Optional[float] = None
        self.latest_global_plan: Optional[Path] = None
        self.active_plan_goal: Optional[MapPoint] = None
        self.stop_sign_tracks: list[StopSignTrack] = []
        self.last_behavior_state = NORMAL_NAV2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("global_plan_topic").value),
            self.global_plan_callback,
            10,
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

    def odom_callback(self, msg: Odometry) -> None:
        self.current_velocity_mps = float(msg.twist.twist.linear.x)
        self.current_pose_frame = msg.header.frame_id
        self.current_robot_x = float(msg.pose.pose.position.x)
        self.current_robot_y = float(msg.pose.pose.position.y)
        self.current_robot_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def global_plan_callback(self, msg: Path) -> None:
        self.latest_global_plan = msg
        if not msg.poses:
            return

        goal_pose = msg.poses[-1].pose.position
        new_goal = MapPoint(float(goal_pose.x), float(goal_pose.y))
        if self.active_plan_goal is None:
            self.active_plan_goal = new_goal
            return

        goal_change = math.hypot(
            new_goal.x - self.active_plan_goal.x,
            new_goal.y - self.active_plan_goal.y,
        )
        if goal_change <= self.plan_goal_change_distance_m:
            return

        self.active_plan_goal = new_goal
        for track in self.stop_sign_tracks:
            if track.stopped:
                track.rearm_for_new_plan = True

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
            self.last_behavior_state = NORMAL_NAV2
            self.publish_state(state, override_active)
            self.publish_cones([], None)
            return

        scan = self.fresh_scan(now)
        detections = self.fresh_detections(now)

        if detections is not None:
            self.publish_stop_sign_from_detections(detections, scan)
        else:
            self.publish_stop_sign_tracks()

        if now < self.stop_until:
            state = STOP_SIGN
            override_active = True
            publish_override_cmd = True
            self.publish_cones([], scan)
        else:
            if self.stop_sign_triggered(now):
                state = STOP_SIGN
                override_active = True
                publish_override_cmd = True
                self.stop_until = now + self.stop_sign_stop_duration_sec
                self.stop_cooldown_until = (
                    self.stop_until + self.stop_sign_cooldown_sec
                )
                self.publish_cones([], scan)
            elif detections is None:
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

        self.log_behavior_transition(state)
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
            and detection.confidence >= self.stop_sign_min_confidence
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
            self.publish_stop_sign_tracks()
            return
        detection = self.best_stop_sign_detection(detections)
        if detection is None:
            self.publish_stop_sign_tracks()
            return
        location = self.localize_stop_sign(
            detection,
            float(detections.image_width),
            scan,
        )
        if location is None:
            self.publish_stop_sign_tracks()
            return
        point = self.transform_location_to_map(
            location,
            scan.header.frame_id or "laser",
            scan.header.stamp,
        )
        if point is None:
            self.publish_stop_sign_tracks()
            return
        self.record_stop_sign_observation(
            point.x,
            point.y,
            float(detection.confidence),
        )
        self.publish_stop_sign_tracks()

    def transform_location_to_map(
        self,
        location: ObjectLocation,
        source_frame: str,
        stamp: Time,
    ) -> Optional[MapPoint]:
        if source_frame == self.stop_sign_map_frame:
            return MapPoint(location.x, location.y)

        transform = self.lookup_transform(
            self.stop_sign_map_frame,
            source_frame,
            stamp,
        )
        if transform is None:
            return None
        return transform_xy(location.x, location.y, transform)

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ):
        if (
            not target_frame
            or not source_frame
            or not hasattr(self, "tf_buffer")
        ):
            return None
        lookup_times = (RclpyTime.from_msg(stamp), RclpyTime())
        for lookup_time in lookup_times:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    lookup_time,
                )
            except TransformException:
                continue
        return None

    def record_stop_sign_observation(
        self,
        x: float,
        y: float,
        confidence: float,
    ) -> StopSignTrack:
        reliable_track = self.reliable_stop_sign_track()
        if reliable_track is not None:
            distance = math.hypot(reliable_track.x - x, reliable_track.y - y)
            if distance > self.stop_sign_clear_distance_m:
                self.stop_sign_tracks = []
            else:
                if distance <= self.stop_sign_track_match_distance_m:
                    reliable_track.update(x, y, confidence)
                return reliable_track

        nearest_track = None
        nearest_distance = math.inf
        for track in self.stop_sign_tracks:
            distance = math.hypot(track.x - x, track.y - y)
            if distance < nearest_distance:
                nearest_track = track
                nearest_distance = distance

        if (
            nearest_track is not None
            and nearest_distance <= self.stop_sign_track_match_distance_m
        ):
            nearest_track.update(x, y, confidence)
            return nearest_track

        track = StopSignTrack(x, y, confidence)
        self.stop_sign_tracks.append(track)
        return track

    def publish_stop_sign_tracks(self) -> None:
        track = self.reliable_stop_sign_track()
        if track is None:
            return

        stamp = self.get_clock().now().to_msg()
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.stop_sign_map_frame
        msg.point.x = float(track.x)
        msg.point.y = float(track.y)
        self.stop_sign_position_pub.publish(msg)

    def stop_sign_track_reliable(self, track: StopSignTrack) -> bool:
        return (
            track.observations
            >= max(1, self.stop_sign_required_observations)
        )

    def reliable_stop_sign_track(self) -> Optional[StopSignTrack]:
        for track in self.stop_sign_tracks:
            if self.stop_sign_track_reliable(track):
                return track
        return None

    def robot_position_in_map(self) -> Optional[MapPoint]:
        if hasattr(self, "tf_buffer"):
            transform = self.lookup_transform(
                self.stop_sign_map_frame,
                self.robot_base_frame,
                self.get_clock().now().to_msg(),
            )
            if transform is not None:
                translation = transform.transform.translation
                return MapPoint(float(translation.x), float(translation.y))

        if (
            self.current_pose_frame == self.stop_sign_map_frame
            and self.current_robot_x is not None
            and self.current_robot_y is not None
        ):
            return MapPoint(self.current_robot_x, self.current_robot_y)
        return None

    def stop_sign_triggered(self, now: float) -> bool:
        robot_position = self.robot_position_in_map()
        if robot_position is None:
            return False

        track = self.reliable_stop_sign_track()
        if track is None:
            return False

        remaining_distance = self.stop_line_path_distance(
            robot_position,
            MapPoint(track.x, track.y),
        )
        if remaining_distance is None:
            return False

        if track.stopped:
            rearm_distance = (
                self.stop_sign_stop_before_distance_m
                + self.stop_sign_rearm_distance_m
            )
            if (
                track.rearm_for_new_plan
                and remaining_distance > rearm_distance
            ):
                track.stopped = False
                track.rearm_for_new_plan = False
                track.last_distance_m = remaining_distance
                track.min_distance_m = remaining_distance
            return False

        previous_distance = track.last_distance_m
        track.last_distance_m = remaining_distance
        if track.min_distance_m is None:
            track.min_distance_m = remaining_distance
        else:
            track.min_distance_m = min(
                track.min_distance_m,
                remaining_distance,
            )

        in_stop_region = should_stop_before_line(
            remaining_distance,
            self.stop_sign_stop_before_distance_m,
            self.stop_sign_stop_line_tolerance_m,
        )
        crossed_stop_region = (
            previous_distance is not None
            and previous_distance > self.stop_sign_stop_before_distance_m
            and remaining_distance < -self.stop_sign_stop_line_tolerance_m
        )
        if now - self.last_stop_sign_debug_sec >= 1.0:
            self.last_stop_sign_debug_sec = now
            self.get_logger().info(
                "Stop sign track at "
                f"({track.x:.2f}, {track.y:.2f}) "
                f"is {remaining_distance:.2f} m ahead on the path "
                f"({track.observations} observations)"
            )
        if in_stop_region or crossed_stop_region:
            track.stopped = True
            track.rearm_for_new_plan = False
            return True
        return False

    def stop_line_path_distance(
        self,
        robot_position: MapPoint,
        stop_sign_position: MapPoint,
    ) -> Optional[float]:
        plan = self.latest_global_plan
        if plan is None or plan.header.frame_id != self.stop_sign_map_frame:
            return None

        path_points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in plan.poses
        ]
        robot_path_distance = distance_along_path(
            (robot_position.x, robot_position.y),
            path_points,
        )
        stop_line_path_distance = distance_along_path(
            (stop_sign_position.x, stop_sign_position.y),
            path_points,
        )
        if robot_path_distance is None or stop_line_path_distance is None:
            return None
        return stop_line_path_distance - robot_path_distance

    def log_behavior_transition(self, state: str) -> None:
        """Log behavior activation/release once, rather than every timer tick."""
        previous_state = self.last_behavior_state
        if state == previous_state:
            return

        velocity = self.velocity_for_log()
        logger = self.get_logger()
        if state == STOP_SIGN:
            logger.info(
                "[BEHAVIOR] STOP_SIGN called | "
                f"current velocity: {velocity} | "
                f"stopping for {self.stop_sign_stop_duration_sec:.1f} s"
            )
        elif state == CONE:
            logger.info(
                "[BEHAVIOR] CONE called | "
                f"current velocity: {velocity} | continuing Nav2 with "
                f"speed limit {self.cone_speed_limit_mps:.2f} m/s"
            )
        elif state == NORMAL_NAV2:
            if previous_state == STOP_SIGN:
                logger.info(
                    "[BEHAVIOR] STOP_SIGN complete | "
                    f"current velocity: {velocity} | returning to Nav2"
                )
            elif previous_state == CONE:
                logger.info(
                    "[BEHAVIOR] CONE cleared | "
                    f"current velocity: {velocity} | continuing Nav2"
                )

        self.last_behavior_state = state

    def velocity_for_log(self) -> str:
        if self.current_velocity_mps is None:
            return "unavailable"
        return f"{self.current_velocity_mps:.2f} m/s"

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


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_xy(x: float, y: float, transform) -> MapPoint:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = yaw_from_quaternion(
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return MapPoint(
        float(translation.x) + cos_yaw * x - sin_yaw * y,
        float(translation.y) + sin_yaw * x + cos_yaw * y,
    )


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
