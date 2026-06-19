import math
from types import SimpleNamespace

from carkit_perception_msgs.msg import YoloTrafficLightDetection2D
from sensor_msgs.msg import LaserScan

from carkit_behavior.behavior_center_node import (
    BehaviorCenterNode,
    StopSignTrack,
)


def make_node():
    node = object.__new__(BehaviorCenterNode)
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
    node.stop_sign_trigger_distance_m = 2.0
    node.stop_sign_min_confidence = 0.75
    node.stop_sign_map_frame = "map"
    node.robot_base_frame = "base_link"
    node.stop_sign_required_observations = 3
    node.stop_sign_track_match_distance_m = 1.0
    node.stop_sign_clear_distance_m = 10.0
    node.stop_sign_passed_distance_increase_m = 0.2
    node.stop_cooldown_until = 0.0
    node.last_stop_sign_debug_sec = 10.0
    node.current_pose_frame = "map"
    node.current_robot_x = 0.0
    node.current_robot_y = 0.0
    node.stop_sign_tracks = []
    node.traffic_light_min_bbox_height_ratio = 0.06
    node.traffic_light_hold_timeout_sec = 0.5
    node.traffic_light_hold_until = 0.0
    node.cone_lidar_angle_window_rad = math.radians(6.0)
    node.cone_lidar_min_range_m = 0.15
    node.cone_lidar_max_range_m = 10.0
    node.cone_trigger_distance_m = 3.0
    node.cone_obstacle_radius_m = 0.25
    return node


def detection(
    class_name,
    color=0,
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
        traffic_light_color=color,
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
        traffic_lights=[],
    )


def traffic_light_array(item):
    return SimpleNamespace(
        image_width=640,
        image_height=480,
        detections=[],
        traffic_lights=[
            SimpleNamespace(
                detection=item,
                traffic_light_color=item.traffic_light_color,
            )
        ],
    )


def test_bbox_height_rejects_distant_traffic_light():
    node = make_node()
    far = detection("traffic light", box=(300.0, 100.0, 340.0, 120.0))
    near = detection("traffic light", box=(300.0, 100.0, 340.0, 140.0))
    assert not node.traffic_light_is_near(far, 480.0)
    assert node.traffic_light_is_near(near, 480.0)


def test_red_and_yellow_stop_and_green_releases():
    node = make_node()
    for color in (
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_RED,
        YoloTrafficLightDetection2D.TRAFFIC_LIGHT_YELLOW,
    ):
        light = detection("traffic light", color=color)
        assert node.traffic_light_stop_active(traffic_light_array(light), 2.0)

    green = detection(
        "traffic light",
        color=YoloTrafficLightDetection2D.TRAFFIC_LIGHT_GREEN,
    )
    assert not node.traffic_light_stop_active(traffic_light_array(green), 2.1)
    assert node.traffic_light_hold_until == 0.0


def reliable_track(x, y):
    track = StopSignTrack(x, y, 0.9)
    track.observations = 3
    return track


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


def test_stop_sign_triggers_once_when_robot_is_near_map_track():
    node = make_node()
    node.stop_sign_tracks = [reliable_track(1.0, 0.0)]

    assert node.stop_sign_triggered(1.0)
    assert node.stop_sign_tracks[0].stopped
    assert not node.stop_sign_triggered(1.1)


def test_stop_sign_triggers_when_robot_passes_unreached_track():
    node = make_node()
    node.stop_sign_trigger_distance_m = 1.0
    node.stop_sign_tracks = [reliable_track(5.0, 0.0)]

    node.current_robot_x = 0.0
    assert not node.stop_sign_triggered(1.0)
    node.current_robot_x = 2.5
    assert not node.stop_sign_triggered(1.1)
    node.current_robot_x = 2.1
    assert node.stop_sign_triggered(1.2)


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


def test_cone_is_localized_in_laser_frame_and_inflated():
    node = make_node()
    msg = detection_array(detection("traffic cone"))
    points = node.cone_points(msg, centered_scan(2.0))
    assert len(points) == 5
    assert points[0][0] == -2.0
    assert abs(points[0][1]) < 1.0e-6
    assert node.cone_points(msg, centered_scan(4.0)) == []
