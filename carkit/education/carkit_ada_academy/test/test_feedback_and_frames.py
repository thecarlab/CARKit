"""Regression tests for feedback loss and nonidentity map/odom transforms."""
import math
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import Buffer, TransformException

from carkit_ada_academy.control import AdaControl
from carkit_ada_academy.planning import AdaPlanning
from carkit_ada_academy.frames import transform_points


@pytest.fixture
def context():
    rclpy.init()
    yield
    rclpy.shutdown()


def odom(stamp=10):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = stamp
    message.pose.pose.orientation.w = 1.0
    return message


def map_to_odom(stamp=10):
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.header.stamp.sec = stamp
    transform.transform.translation.x = 5.0
    transform.transform.translation.y = 2.0
    # Odom +X points toward map +Y.
    transform.transform.rotation.z = math.sin(math.pi/4)
    transform.transform.rotation.w = math.cos(math.pi/4)
    return transform


def path_in(frame, points):
    path = Path()
    path.header.frame_id = frame
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def test_controller_stops_after_odometry_loss_and_recovers(context, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr('carkit_ada_academy.control.time.monotonic', lambda: clock[0])
    node = AdaControl()
    node.publisher = Mock()
    try:
        node._on_path(path_in('odom', [(0, 0), (2, 0)]))
        node._on_odom(odom())
        node._update()
        assert node.publisher.publish.call_args[0][0].drive.speed > 0
        clock[0] += 0.51
        for _ in range(100):
            node._update()
            assert node.publisher.publish.call_args[0][0].drive.speed == 0
        node._on_odom(odom(11))
        node._update()
        assert node.publisher.publish.call_args[0][0].drive.speed > 0
    finally:
        node.destroy_node()


def test_planner_transforms_start_and_controller_tracks_in_odom(context):
    planner, controller = AdaPlanning(), AdaControl()
    planner.publisher, controller.publisher = Mock(), Mock()
    try:
        for node in (planner, controller):
            node.tf_buffer.set_transform(map_to_odom(), 'test')
            node._on_odom(odom())
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = 5.0, 4.0
        planner._on_goal(goal)
        path = planner.publisher.publish.call_args[0][0]
        assert path.header.frame_id == 'map'
        assert path.poses[0].pose.position.x == pytest.approx(5)
        assert path.poses[0].pose.position.y == pytest.approx(2)
        controller._on_path(path)
        controller._update()
        command = controller.publisher.publish.call_args[0][0]
        assert command.drive.speed > 0
        assert command.drive.steering_angle == pytest.approx(0, abs=1e-6)
    finally:
        planner.destroy_node()
        controller.destroy_node()


def test_controller_stops_when_transform_missing_or_too_old(context):
    node = AdaControl()
    node.publisher = Mock()
    try:
        node._on_odom(odom())
        node._on_path(path_in('map', [(5, 2), (5, 4)]))
        node._update()
        assert node.publisher.publish.call_args[0][0].drive.speed == 0
        node.tf_buffer.set_transform(map_to_odom(1), 'test')
        node._update()
        assert node.publisher.publish.call_args[0][0].drive.speed == 0
    finally:
        node.destroy_node()


def test_invalid_goal_or_stale_feedback_clears_old_plan(context, monkeypatch):
    node = AdaPlanning()
    node.publisher = Mock()
    try:
        node._on_odom(odom())
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        node._on_goal(goal)
        assert not node.publisher.publish.call_args[0][0].poses
        monkeypatch.setattr('carkit_ada_academy.planning.time.monotonic',
                            lambda: node.odom_received_at + 1)
        goal.header.frame_id = 'odom'
        node._on_goal(goal)
        assert not node.publisher.publish.call_args[0][0].poses
    finally:
        node.destroy_node()


def test_empty_or_inconsistent_path_frame_invalidates_path(context):
    node = AdaControl()
    try:
        path = path_in('map', [(1, 2)])
        path.poses[0].header.frame_id = 'odom'
        node._on_path(path)
        assert not node.path
        node._on_path(path_in('', [(1, 2)]))
        assert not node.path
    finally:
        node.destroy_node()


def test_transform_never_substitutes_identity():
    buffer = Buffer()
    with pytest.raises(TransformException):
        transform_points(buffer, [(0, 0)], 'odom', 'map', odom().header.stamp)
    with pytest.raises(ValueError):
        transform_points(buffer, [(0, 0)], '', 'odom', odom().header.stamp)
