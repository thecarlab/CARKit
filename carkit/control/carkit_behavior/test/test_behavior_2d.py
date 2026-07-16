import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from carkit_perception_msgs.msg import YoloTrafficLightDetection2D
from visualization_msgs.msg import Marker

from carkit_behavior.behavior_center_node import (
    BehaviorCenterNode,
    ConeTrack,
    SpeedSignTrack,
    StopSignTrack,
    TrafficLightTrack,
    delete_object_marker_array,
    object_marker_array,
)


def make_node():
    node = object.__new__(BehaviorCenterNode)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    node.detection_timeout_sec = 0.5
    node.scan_timeout_sec = 0.25
    node.min_confidence = 0.55
    node.camera_fx = None
    node.camera_cx = None
    node.camera_image_width = 640.0
    node.camera_image_height = 480.0
    node.camera_horizontal_fov_rad = math.radians(69.4)
    node.camera_to_scan_yaw_offset_rad = 0.0
    node.camera_forward_offset_m = 0.08
    node.camera_lateral_offset_m = 0.0
    node.stop_sign_lidar_angle_window_rad = math.radians(8.0)
    node.stop_sign_lidar_min_range_m = 0.15
    node.stop_sign_lidar_max_range_m = 10.0
    node.stop_sign_stop_before_distance_m = 0.5
    node.stop_sign_stop_line_tolerance_m = 0.25
    node.stop_sign_rearm_distance_m = 1.0
    node.stop_sign_min_confidence = 0.75
    node.stop_sign_map_frame = "map"
    node.robot_base_frame = "base_link"
    node.stop_sign_required_observations = 3
    node.stop_sign_track_match_distance_m = 1.0
    node.stop_sign_clear_distance_m = 10.0
    node.traffic_light_min_confidence = 0.6
    node.traffic_light_lidar_angle_window_rad = math.radians(8.0)
    node.traffic_light_lidar_min_range_m = 0.15
    node.traffic_light_lidar_max_range_m = 10.0
    node.traffic_light_stop_ahead_distance_m = 2.0
    node.traffic_light_required_observations = 3
    node.traffic_light_stop_required_frames = 3
    node.traffic_light_green_required_frames = 3
    node.traffic_light_track_match_distance_m = 1.0
    node.traffic_light_clear_distance_m = 10.0
    node.plan_goal_change_distance_m = 0.25
    node.speed_sign_min_confidence = 0.2
    node.speed_sign_lidar_angle_window_rad = math.radians(20.0)
    node.speed_sign_lidar_min_range_m = 0.15
    node.speed_sign_lidar_max_range_m = 10.0
    node.speed_sign_required_observations = 2
    node.speed_sign_track_match_distance_m = 1.0
    node.speed_sign_clear_distance_m = 10.0
    node.speed_sign_pass_tolerance_m = 0.25
    node.speed_sign_pass_near_distance_m = 1.5
    node.speed_sign_pass_range_rearm_m = 0.4
    node.speed_sign_fallback_range_m = 3.0
    node.speed_sign_override_min_speed_mps = 1.0
    node.speed_sign_override_multiplier = 1.5
    node.speed_sign_override_max_speed_mps = 3.0
    node.speed_sign_override_duration_sec = 3.0
    node.cone_min_confidence = 0.2
    node.cone_lidar_angle_window_rad = math.radians(20.0)
    node.cone_lidar_min_range_m = 0.15
    node.cone_lidar_max_range_m = 10.0
    node.cone_required_observations = 2
    node.cone_track_match_distance_m = 1.0
    node.cone_clear_distance_m = 10.0
    node.cone_override_speed_mps = 0.8
    node.cone_fallback_range_m = 2.0
    node.cone_visible = False
    node.last_cone_seen_time = None
    node.pending_detections = None
    node.pending_detection_time = None
    node.last_detection_received_time = None
    node.last_detection_processed_time = None
    node.detection_state_expired = True
    node.latest_scan = None
    node.latest_scan_time = None
    node.speed_sign_override_until = 0.0
    node.last_speed_sign_debug_sec = 10.0
    node.last_speed_sign_localize_debug_sec = 10.0
    node.last_cone_debug_sec = 10.0
    node.latest_nav2_drive = None
    node.current_velocity_mps = None
    node.stop_cooldown_until = 0.0
    node.last_stop_sign_debug_sec = 10.0
    node.last_traffic_light_debug_sec = 10.0
    node.traffic_light_stop_engaged = False
    node.latest_traffic_light_color = (
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_UNKNOWN
    )
    node.traffic_light_stop_color_frames = 0
    node.traffic_light_green_frames = 0
    node.current_pose_frame = "map"
    node.current_robot_x = 0.0
    node.current_robot_y = 0.0
    node.latest_global_plan = None
    node.active_plan_goal = None
    node.stop_sign_tracks = []
    node.traffic_light_tracks = []
    node.speed_sign_tracks = []
    node.cone_tracks = []
    no_op_publisher = SimpleNamespace(publish=lambda msg: None)
    node.cone_position_pub = no_op_publisher
    node.cone_markers_pub = no_op_publisher
    return node


def detection(
    class_name,
    box=(300.0, 100.0, 340.0, 160.0),
    confidence=0.9,
):
    return SimpleNamespace(
        class_name=class_name,
        confidence=confidence,
        bbox_x_min=box[0],
        bbox_y_min=box[1],
        bbox_x_max=box[2],
        bbox_y_max=box[3],
    )


def centered_scan(distance):
    scan = LaserScan()
    scan.header.frame_id = "laser"
    scan.angle_min = -math.pi
    scan.angle_increment = math.radians(1.0)
    scan.range_min = 0.05
    scan.range_max = 20.0
    scan.ranges = [math.inf] * 361
    scan.ranges[360] = distance
    return scan


def detection_array(item):
    return SimpleNamespace(
        image_width=640,
        image_height=480,
        detections=[item],
    )


def test_detection_callback_processes_a_fresh_frame_once():
    node = make_node()
    scan = centered_scan(2.0)
    node.latest_scan = scan
    node.latest_scan_time = 9.9
    node.now_sec = lambda: 10.0
    calls = []
    node.process_detection_frame = (
        lambda detections, used_scan, now: calls.append(
            (detections, used_scan, now)
        )
    )
    msg = detection_array(detection("stop sign"))

    node.detections_callback(msg)

    assert calls == [(msg, scan, 10.0)]
    assert node.pending_detections is None
    assert node.last_detection_received_time == 10.0
    assert not node.detection_state_expired


def test_latest_pending_detection_is_processed_by_one_scan_once():
    node = make_node()
    now = [10.0]
    node.now_sec = lambda: now[0]
    calls = []
    node.process_detection_frame = (
        lambda detections, scan, timestamp: calls.append(
            (detections, scan, timestamp)
        )
    )
    first = detection_array(detection("stop sign"))
    second = detection_array(detection("traffic_cone"))

    node.detections_callback(first)
    now[0] = 10.1
    node.detections_callback(second)
    assert node.pending_detections is second

    now[0] = 10.2
    scan = centered_scan(2.0)
    node.scan_callback(scan)
    assert calls == [(second, scan, 10.2)]
    assert node.pending_detections is None

    now[0] = 10.21
    node.scan_callback(centered_scan(2.0))
    assert len(calls) == 1


def test_expired_pending_detection_is_not_processed():
    node = make_node()
    now = [10.0]
    node.now_sec = lambda: now[0]
    calls = []
    node.process_detection_frame = lambda *args: calls.append(args)
    msg = detection_array(detection("stop sign"))

    node.detections_callback(msg)
    now[0] = 10.6
    node.scan_callback(centered_scan(2.0))

    assert calls == []
    assert node.pending_detections is None


def test_detection_state_expires_only_once():
    node = make_node()
    node.last_detection_received_time = 10.0
    node.detection_state_expired = False
    node.pending_detections = detection_array(detection("traffic_cone"))
    node.pending_detection_time = 10.0
    node.cone_visible = True
    calls = []
    node.clear_traffic_light_track = lambda: calls.append("traffic_light")
    node.publish_stop_sign_tracks = lambda: calls.append("stop_sign")
    node.publish_speed_sign_tracks = lambda: calls.append("speed_sign")
    node.publish_cone_tracks = lambda: calls.append("cone")

    assert not node.expire_detection_state_if_needed(10.5)
    assert node.cone_visible
    assert node.expire_detection_state_if_needed(10.51)
    assert not node.cone_visible
    assert node.pending_detections is None
    assert calls == ["traffic_light", "stop_sign", "speed_sign", "cone"]

    assert not node.expire_detection_state_if_needed(11.0)
    assert calls == ["traffic_light", "stop_sign", "speed_sign", "cone"]


def test_detection_frame_dispatches_each_behavior_once():
    node = make_node()
    calls = []
    node.publish_traffic_light_from_detections = (
        lambda detections, scan: calls.append("traffic_light")
    )
    node.publish_stop_sign_from_detections = (
        lambda detections, scan: calls.append("stop_sign")
    )
    node.publish_speed_sign_from_detections = (
        lambda detections, scan: calls.append("speed_sign")
    )
    node.publish_cone_from_detections = (
        lambda detections, scan, now: calls.append("cone")
    )

    node.process_detection_frame(
        detection_array(detection("stop sign")),
        centered_scan(2.0),
        12.0,
    )

    assert calls == ["traffic_light", "stop_sign", "speed_sign", "cone"]
    assert node.last_detection_processed_time == 12.0


def reliable_track(x, y):
    track = StopSignTrack(x, y, 0.9)
    track.observations = 3
    return track


def traffic_light_track(x, y, color):
    track = TrafficLightTrack(x, y, 0.9, color)
    track.observations = 3
    return track


def global_plan(*points):
    plan = Path()
    plan.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        plan.poses.append(pose)
    return plan


def test_map_object_markers_are_supported_visualization_messages():
    markers = object_marker_array(
        "map",
        Time(sec=12),
        3.0,
        4.0,
        "stop_sign",
        "STOP SIGN",
        (1.0, 0.05, 0.05),
    )

    assert len(markers.markers) == 2
    symbol, label = markers.markers
    assert symbol.type == Marker.SPHERE
    assert symbol.header.frame_id == "map"
    assert symbol.pose.position.x == 3.0
    assert symbol.pose.position.y == 4.0
    assert symbol.color.a == 1.0
    assert label.type == Marker.TEXT_VIEW_FACING
    assert label.text == "STOP SIGN"

    deleted = delete_object_marker_array("map", Time(sec=13), "stop_sign")
    assert [marker.action for marker in deleted.markers] == [
        Marker.DELETE,
        Marker.DELETE,
    ]


def test_stop_sign_tracking_requires_confidence_and_observations():
    node = make_node()
    low_confidence = detection("stop sign", confidence=0.7)
    assert node.best_stop_sign_detection(detection_array(low_confidence)) is None

    track = node.record_stop_sign_observation(4.0, 0.0, 0.9)
    assert not node.stop_sign_track_reliable(track)
    node.record_stop_sign_observation(4.2, 0.1, 0.9)
    node.record_stop_sign_observation(3.8, -0.1, 0.9)

    assert len(node.stop_sign_tracks) == 1
    assert node.stop_sign_track_reliable(track)
    assert abs(track.x - 4.0) < 1.0e-6
    assert abs(track.y) < 1.0e-6


def test_stop_sign_triggers_at_configured_distance_before_projected_line():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    node.stop_sign_tracks = [reliable_track(5.0, 1.0)]

    node.current_robot_x = 4.49
    assert not node.stop_sign_triggered(0.9)
    node.current_robot_x = 4.5
    assert node.stop_sign_triggered(1.0)
    assert node.stop_sign_tracks[0].stopped
    assert not node.stop_sign_triggered(1.1)


def test_stop_sign_uses_distance_along_curved_path():
    node = make_node()
    node.latest_global_plan = global_plan(
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 4.0),
    )
    node.stop_sign_tracks = [reliable_track(5.0, 2.0)]

    node.current_robot_x = 4.0
    node.current_robot_y = 1.49
    assert not node.stop_sign_triggered(1.0)
    node.current_robot_y = 1.5
    assert node.stop_sign_triggered(1.1)


def test_stop_sign_triggers_within_small_overshoot_region():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    node.stop_sign_tracks = [reliable_track(5.0, 1.0)]

    node.current_robot_x = 5.1
    assert node.stop_sign_triggered(1.0)


def test_stop_sign_does_not_trigger_when_first_seen_well_behind():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    node.stop_sign_tracks = [reliable_track(5.0, 1.0)]

    node.current_robot_x = 5.3
    assert not node.stop_sign_triggered(1.0)


def test_stop_sign_triggers_if_vehicle_crosses_entire_region_between_updates():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    node.stop_sign_tracks = [reliable_track(5.0, 1.0)]

    node.current_robot_x = 4.0
    assert not node.stop_sign_triggered(1.0)
    node.current_robot_x = 5.5
    assert node.stop_sign_triggered(1.1)


def test_same_goal_plan_update_does_not_reset_stop_latch():
    node = make_node()
    track = reliable_track(5.0, 0.0)
    track.stopped = True
    node.stop_sign_tracks = [track]

    node.global_plan_callback(global_plan((0.0, 0.0), (10.0, 0.0)))
    node.global_plan_callback(global_plan((3.0, 0.1), (10.1, 0.0)))

    assert track.stopped


def test_new_goal_rearms_when_sign_is_safely_ahead_on_new_path():
    node = make_node()
    track = reliable_track(5.0, 2.0)
    track.stopped = True
    node.stop_sign_tracks = [track]

    node.global_plan_callback(global_plan((0.0, 0.0), (10.0, 0.0)))
    node.global_plan_callback(
        global_plan((0.0, 0.0), (4.0, 0.0), (4.0, 6.0))
    )

    assert track.stopped
    assert track.rearm_for_new_plan
    node.current_robot_x = 0.0
    node.current_robot_y = 0.0
    assert not node.stop_sign_triggered(1.9)
    assert not track.stopped
    node.current_robot_x = 4.0
    node.current_robot_y = 1.5
    assert node.stop_sign_triggered(2.0)
    assert track.stopped
    assert not node.stop_sign_triggered(2.1)


def test_replan_near_stopped_sign_does_not_cause_second_stop():
    node = make_node()
    track = reliable_track(5.0, 0.0)
    track.stopped = True
    node.stop_sign_tracks = [track]
    node.current_robot_x = 4.5

    node.global_plan_callback(global_plan((0.0, 0.0), (10.0, 0.0)))
    node.global_plan_callback(global_plan((4.5, 0.0), (11.0, 0.0)))

    assert track.stopped
    assert track.rearm_for_new_plan
    assert not node.stop_sign_triggered(2.0)
    assert track.stopped
    assert not node.stop_sign_triggered(2.1)


def test_reliable_stop_sign_latches_until_far_new_sign():
    node = make_node()
    latched = reliable_track(4.0, 0.0)
    node.stop_sign_tracks = [latched]

    assert node.record_stop_sign_observation(7.0, 0.0, 0.9) is latched
    assert node.stop_sign_tracks == [latched]
    assert latched.x == 4.0

    new_track = node.record_stop_sign_observation(15.0, 0.0, 0.9)
    assert node.stop_sign_tracks == [new_track]
    assert new_track is not latched
    assert new_track.x == 15.0
    assert not node.stop_sign_track_reliable(new_track)


def test_lidar_matching_accounts_for_forward_camera_offset():
    node = make_node()
    scan = centered_scan(math.inf)
    scan.ranges[350] = 1.0
    scan_angle = math.radians(170.0)
    point_x = math.cos(scan_angle)
    point_y = math.sin(scan_angle)
    angle_from_camera = math.atan2(point_y, point_x + 0.08)
    camera_bearing = math.pi - angle_from_camera
    target_angle = node.camera_bearing_to_scan_angle(camera_bearing)
    hit = node.find_lidar_hit_at_bearing(
        scan,
        target_angle,
        math.radians(0.5),
        0.15,
        10.0,
    )
    assert hit is not None
    assert hit[0] == 1.0


def test_traffic_light_detection_ignores_non_light_detections():
    node = make_node()
    msg = SimpleNamespace(
        image_width=640,
        image_height=480,
        detections=[
            detection("stop sign", confidence=0.99),
            detection("traffic light", confidence=0.8),
        ],
        traffic_lights=[],
    )

    assert node.best_traffic_light_detection(msg) is None


def test_traffic_light_detection_uses_classified_perception_output():
    node = make_node()
    light = SimpleNamespace(
        detection=detection("traffic light", confidence=0.8),
        traffic_light_color=YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )
    msg = SimpleNamespace(
        image_width=640,
        image_height=480,
        detections=[],
        traffic_lights=[light],
    )

    best = node.best_traffic_light_detection(msg)
    assert best is not None
    assert best[0].class_name == "traffic light"
    assert best[1] == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED


def test_empty_traffic_light_output_clears_stale_track():
    node = make_node()
    node.traffic_light_tracks = [
        traffic_light_track(
            5.0,
            0.0,
            YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
        )
    ]
    msg = SimpleNamespace(
        image_width=640,
        image_height=480,
        detections=[],
        traffic_lights=[],
    )

    node.publish_traffic_light_from_detections(msg, centered_scan(2.0))

    assert node.traffic_light_tracks == []
    assert not node.traffic_light_stop_engaged


def test_empty_traffic_light_output_does_not_release_active_stop():
    node = make_node()
    track = traffic_light_track(
        5.0,
        0.0,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )
    node.traffic_light_tracks = [track]
    node.traffic_light_stop_engaged = True
    msg = SimpleNamespace(
        image_width=640,
        image_height=480,
        detections=[],
        traffic_lights=[],
    )

    node.publish_traffic_light_from_detections(msg, centered_scan(2.0))

    assert node.traffic_light_tracks == [track]
    assert node.traffic_light_stop_engaged


def test_traffic_light_tracking_matches_stop_sign_style():
    node = make_node()
    track = node.record_traffic_light_observation(
        4.0,
        0.0,
        0.9,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )
    assert not node.traffic_light_track_reliable(track)
    node.record_traffic_light_observation(
        4.2,
        0.1,
        0.9,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )
    node.record_traffic_light_observation(
        3.8,
        -0.1,
        0.9,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )

    assert len(node.traffic_light_tracks) == 1
    assert node.traffic_light_track_reliable(track)
    assert abs(track.x - 4.0) < 1.0e-6
    assert abs(track.y) < 1.0e-6


def test_traffic_light_uses_same_localization_as_stop_sign():
    node = make_node()
    item = detection("traffic light")
    scan = centered_scan(2.0)

    stop_location = node.localize_stop_sign(item, 640.0, scan)
    traffic_light_location = node.localize_traffic_light(item, 640.0, scan)

    assert stop_location is not None
    assert traffic_light_location is not None
    assert traffic_light_location.range_m == stop_location.range_m
    assert traffic_light_location.scan_angle_rad == stop_location.scan_angle_rad
    assert traffic_light_location.x == stop_location.x
    assert traffic_light_location.y == stop_location.y


def test_traffic_light_localization_has_independent_lidar_window():
    node = make_node()
    item = detection("traffic light", box=(280.0, 100.0, 320.0, 160.0))
    scan = centered_scan(math.inf)
    scan.ranges[358] = 1.0
    node.stop_sign_lidar_angle_window_rad = math.radians(1.0)
    node.traffic_light_lidar_angle_window_rad = math.radians(12.0)

    assert node.localize_stop_sign(item, 640.0, scan) is None
    assert node.localize_traffic_light(item, 640.0, scan) is not None


def test_red_traffic_light_stops_when_close_on_path():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    node.traffic_light_tracks = [
        traffic_light_track(
            5.0,
            0.0,
            YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
        )
    ]
    node.latest_traffic_light_color = YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED
    node.traffic_light_stop_color_frames = 2

    node.current_robot_x = 2.9
    assert not node.traffic_light_stop_active(1.0)
    node.current_robot_x = 3.0
    assert not node.traffic_light_stop_active(1.1)
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED
    )
    assert node.traffic_light_stop_active(1.1)
    assert node.traffic_light_stop_engaged


def test_green_traffic_light_releases_stop_after_three_frames():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    track = traffic_light_track(
        5.0,
        0.0,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )
    node.traffic_light_tracks = [track]
    node.latest_traffic_light_color = YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED
    node.current_robot_x = 4.0
    node.traffic_light_stop_color_frames = 3

    assert node.traffic_light_stop_active(1.0)
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN
    )
    assert node.traffic_light_stop_active(1.1)
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN
    )
    assert node.traffic_light_stop_active(1.2)
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN
    )
    assert not node.traffic_light_stop_active(1.3)
    assert not node.traffic_light_stop_engaged


def test_green_yolo_state_releases_even_with_location_jitter():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    track = traffic_light_track(
        5.0,
        0.0,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
    )
    node.traffic_light_tracks = [track]
    node.latest_traffic_light_color = YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED
    node.current_robot_x = 4.0
    node.traffic_light_stop_color_frames = 3

    assert node.traffic_light_stop_active(1.0)
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN,
    )
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN,
    )
    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN,
    )

    assert track.color == YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED
    assert not node.traffic_light_stop_active(1.1)
    assert not node.traffic_light_stop_engaged


def test_single_green_yolo_update_does_not_release_active_traffic_light_stop():
    node = make_node()
    node.traffic_light_stop_engaged = True
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    node.traffic_light_tracks = [
        traffic_light_track(
            5.0,
            0.0,
            YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
        )
    ]
    node.current_robot_x = 4.0

    node.update_traffic_light_color_state(
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN,
    )

    assert node.traffic_light_stop_active(1.0)
    assert node.traffic_light_stop_engaged


def speed_sign_track(x, y):
    track = SpeedSignTrack(x, y, 0.9)
    track.observations = 2
    return track


def cone_track(x, y):
    track = ConeTrack(x, y, 0.9)
    track.observations = 2
    return track


def test_speed_sign_tracking_requires_confidence_and_observations():
    node = make_node()
    low_confidence = detection("speed_sign", confidence=0.1)
    assert node.best_speed_sign_detection(detection_array(low_confidence)) is None

    track = node.record_speed_sign_observation(4.0, 0.0, 0.9)
    assert not node.speed_sign_track_reliable(track)
    node.record_speed_sign_observation(4.2, 0.1, 0.9)

    assert len(node.speed_sign_tracks) == 1
    assert node.speed_sign_track_reliable(track)


def test_speed_sign_triggers_after_passing_projected_line():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    track = speed_sign_track(5.0, 1.0)
    track.last_range_m = 2.0
    track.min_range_m = 2.0
    node.speed_sign_tracks = [track]

    node.current_robot_x = 4.5
    assert not node.speed_sign_pass_triggered(1.0)
    node.current_robot_x = 5.3
    assert node.speed_sign_pass_triggered(1.1)
    assert node.speed_sign_tracks[0].passed
    assert not node.speed_sign_pass_triggered(1.2)


def test_speed_sign_triggers_after_closest_range_then_receding():
    node = make_node()
    track = speed_sign_track(5.0, 0.0)
    track.last_range_m = 1.21
    track.min_range_m = 0.8
    node.speed_sign_tracks = [track]

    assert node.speed_sign_pass_triggered(1.0)
    assert track.passed


def test_speed_sign_does_not_retrigger_after_pass():
    node = make_node()
    node.latest_global_plan = global_plan((0.0, 0.0), (10.0, 0.0))
    track = speed_sign_track(5.0, 0.0)
    track.passed = True
    node.speed_sign_tracks = [track]
    node.current_robot_x = 6.0

    assert not node.speed_sign_pass_triggered(1.0)


def test_speed_sign_override_multiplies_current_speed_with_cap():
    node = make_node()
    node.current_velocity_mps = 1.2
    assert math.isclose(node.speed_sign_override_speed(), 1.8)
    node.current_velocity_mps = 2.0
    assert node.speed_sign_override_speed() == 3.0
    node.current_velocity_mps = 2.5
    assert node.speed_sign_override_speed() == 3.0


def test_speed_sign_does_not_override_at_or_below_min_speed():
    node = make_node()
    node.current_velocity_mps = 1.0
    assert node.speed_sign_override_speed() is None
    node.current_velocity_mps = 0.5
    assert node.speed_sign_override_speed() is None


def test_cone_tracking_requires_confidence_and_observations():
    node = make_node()
    low_confidence = detection("traffic_cone", confidence=0.1)
    assert node.best_cone_detection(detection_array(low_confidence)) is None

    track = node.record_cone_observation(3.0, 0.0, 0.9)
    assert not node.cone_track_reliable(track)
    node.record_cone_observation(3.1, 0.1, 0.9)

    assert len(node.cone_tracks) == 1
    assert node.cone_track_reliable(track)


def test_cone_speed_override_requires_stable_visible_track():
    node = make_node()
    node.cone_tracks = [cone_track(3.0, 0.0)]

    assert not node.cone_present()
    node.cone_visible = True
    assert node.cone_present()
    assert not node.cone_speed_override_active()
    node.current_velocity_mps = 0.5
    assert not node.cone_speed_override_active()
    node.current_velocity_mps = 1.0
    assert node.cone_speed_override_active()
    node.cone_visible = False
    assert not node.cone_present()
    assert not node.cone_speed_override_active()


def test_cone_visibility_expires_from_last_seen_time():
    node = make_node()
    node.cone_tracks = [cone_track(3.0, 0.0)]
    node.cone_visible = True
    node.last_cone_seen_time = 10.0
    node.current_velocity_mps = 1.2

    assert node.cone_present(10.5)
    assert node.cone_speed_override_active(10.5)
    assert not node.cone_present(10.51)
    assert not node.cone_speed_override_active(10.51)


def test_cone_does_not_override_at_or_below_limit_speed():
    node = make_node()
    node.cone_tracks = [cone_track(3.0, 0.0)]
    node.cone_visible = True

    node.current_velocity_mps = 0.8
    assert not node.cone_speed_override_active()
    node.current_velocity_mps = 0.3
    assert not node.cone_speed_override_active()


def test_cone_detection_visibility_controls_override_release():
    node = make_node()
    node.cone_tracks = [cone_track(3.0, 0.0)]
    node.current_velocity_mps = 1.2
    cone = detection("traffic_cone", confidence=0.9)
    msg = detection_array(cone)

    node.publish_cone_from_detections(msg, centered_scan(2.0))
    assert node.cone_visible
    assert node.cone_speed_override_active()

    node.publish_cone_from_detections(msg, None)
    assert not node.cone_visible
    assert not node.cone_speed_override_active()


def test_speed_override_command_uses_nav2_steering():
    node = make_node()
    nav2_cmd = SimpleNamespace(
        drive=SimpleNamespace(speed=2.0, steering_angle=0.12)
    )
    node.latest_nav2_drive = nav2_cmd

    command = node.speed_override_command(3.5)
    assert command.drive.speed == 3.5
    assert command.drive.steering_angle == 0.12
