#!/usr/bin/env python3

from copy import deepcopy
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class FoxgloveWaypoints(Node):
    """Accumulate Foxglove pose clicks and submit them to Nav2."""

    def __init__(self) -> None:
        super().__init__('foxglove_waypoints')

        self.declare_parameter('goal_topic', '/foxglove/waypoints/goal')
        self.declare_parameter('command_topic', '/foxglove/waypoints/command')
        self.declare_parameter('marker_topic', '/foxglove/waypoints/markers')
        self.declare_parameter('status_topic', '/foxglove/waypoints/status')
        self.declare_parameter('action_name', '/navigate_through_poses')
        self.declare_parameter('default_frame', 'map')
        self.declare_parameter('maximum_poses', 50)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._pending: List[PoseStamped] = []
        self._active: List[PoseStamped] = []
        self._submitted: List[PoseStamped] = []
        self._sending = False
        self._goal_handle = None

        self._marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            status_qos,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            status_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._pose_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('command_topic').value),
            self._command_callback,
            10,
        )
        self._action_client = ActionClient(
            self,
            NavigateThroughPoses,
            str(self.get_parameter('action_name').value),
        )

        self._publish_markers()
        self._publish_status('Ready: publish poses to accumulate a route')

    def _pose_callback(self, msg: PoseStamped) -> None:
        maximum_poses = int(self.get_parameter('maximum_poses').value)
        if len(self._pending) >= maximum_poses:
            self._publish_status(f'Pose limit reached ({maximum_poses})')
            return

        pose = deepcopy(msg)
        if not pose.header.frame_id:
            pose.header.frame_id = str(self.get_parameter('default_frame').value)
        orientation = pose.pose.orientation
        if all(value == 0.0 for value in (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )):
            pose.pose.orientation.w = 1.0

        self._pending.append(pose)
        self._publish_markers()
        self._publish_status(f'Accumulated {len(self._pending)} pose(s)')

    def _command_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ('start', 'run', 'navigate'):
            self._start()
        elif command in ('clear', 'reset'):
            self._pending.clear()
            self._publish_markers()
            self._publish_status('Pending poses cleared')
        elif command in ('cancel', 'stop'):
            self._cancel()
        else:
            self._publish_status(
                f'Unknown command "{msg.data}"; use start, clear, or cancel'
            )

    def _start(self) -> None:
        if self._sending or self._goal_handle is not None:
            self._publish_status('A route is already starting or running')
            return
        if not self._pending:
            self._publish_status('No poses accumulated')
            return
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self._publish_status('Nav2 /navigate_through_poses is unavailable')
            return

        self._submitted = self._pending
        self._pending = []
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._submitted

        self._sending = True
        self._publish_markers()
        self._publish_status(f'Starting route with {len(goal.poses)} pose(s)')
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        self._sending = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy action transport failure
            self._pending = self._submitted + self._pending
            self._submitted = []
            self._publish_markers()
            self._publish_status(f'Failed to send route: {exc}')
            return

        if not goal_handle.accepted:
            self._pending = self._submitted + self._pending
            self._submitted = []
            self._publish_markers()
            self._publish_status('Nav2 rejected the route')
            return

        self._goal_handle = goal_handle
        self._active = self._submitted
        self._submitted = []
        self._publish_markers()
        self._publish_status(f'Navigating through {len(self._active)} pose(s)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        try:
            status = future.result().status
            status_text = {
                GoalStatus.STATUS_SUCCEEDED: 'Route completed',
                GoalStatus.STATUS_CANCELED: 'Route canceled',
                GoalStatus.STATUS_ABORTED: 'Route aborted by Nav2',
            }.get(status, f'Route ended with status {status}')
        except Exception as exc:  # rclpy action transport failure
            status_text = f'Failed to receive route result: {exc}'

        self._goal_handle = None
        self._active = []
        self._publish_markers()
        self._publish_status(status_text)

    def _cancel(self) -> None:
        if self._goal_handle is None:
            self._publish_status('No active route to cancel')
            return
        self._publish_status('Canceling active route')
        self._goal_handle.cancel_goal_async()

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        marker_id = 0
        for state, poses, color in (
            ('active', self._active, (0.1, 0.55, 1.0)),
            ('pending', self._pending, (0.1, 1.0, 0.35)),
        ):
            for index, pose in enumerate(poses, start=1):
                arrow = Marker()
                arrow.header = pose.header
                arrow.ns = f'foxglove_waypoints_{state}'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose = deepcopy(pose.pose)
                arrow.scale.x = 0.65
                arrow.scale.y = 0.14
                arrow.scale.z = 0.14
                arrow.color.r, arrow.color.g, arrow.color.b = color
                arrow.color.a = 1.0
                marker_array.markers.append(arrow)

                label = Marker()
                label.header = pose.header
                label.ns = f'foxglove_waypoints_{state}_labels'
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = deepcopy(pose.pose)
                label.pose.position.z += 0.45
                label.scale.z = 0.35
                label.color.r, label.color.g, label.color.b = color
                label.color.a = 1.0
                label.text = str(index)
                marker_array.markers.append(label)

        self._marker_pub.publish(marker_array)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FoxgloveWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
