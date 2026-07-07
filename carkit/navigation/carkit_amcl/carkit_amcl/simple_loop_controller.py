#!/usr/bin/env python3

import atexit
import math
import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float
    s: float


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    pose = PoseStamped()
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose.pose.orientation


class SimpleLoopController(Node):
    """Drive a fixed straight-arc-straight-arc route using localization feedback."""

    def __init__(self):
        super().__init__('simple_loop_controller')

        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('ackermann_topic', '/ackermann_cmd')
        self.declare_parameter('path_topic', '/simple_loop_path')
        self.declare_parameter('straight_distance', 3.6)
        self.declare_parameter('turn_radius', 1.75)
        self.declare_parameter('turn_direction', 'left')
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.12)
        self.declare_parameter('max_steering_angle', 0.27)
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('progress_search_distance', 3.0)
        self.declare_parameter('slowdown_distance', 0.8)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('amcl_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('loop_path', True)
        self.declare_parameter('require_autonomous_mode', True)
        self.declare_parameter('autonomy_enable_topic', '/enable_autonomous_control')
        self.declare_parameter('keyboard_control', True)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('min_speed_limit', 0.0)
        self.declare_parameter('max_speed_limit', 2.0)
        self.declare_parameter('keyboard_rate', 20.0)

        self.pose_source = self._pose_source(self.get_parameter('pose_source').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.straight_distance = abs(float(self.get_parameter('straight_distance').value))
        self.turn_radius = abs(float(self.get_parameter('turn_radius').value))
        self.turn_direction = self._turn_direction_sign(
            self.get_parameter('turn_direction').value
        )
        self.wheelbase = abs(float(self.get_parameter('wheelbase').value))
        self.linear_speed = abs(float(self.get_parameter('linear_speed').value))
        self.min_linear_speed = abs(float(self.get_parameter('min_linear_speed').value))
        self.max_steering_angle = abs(float(self.get_parameter('max_steering_angle').value))
        self.lookahead_distance = abs(float(self.get_parameter('lookahead_distance').value))
        self.path_resolution = abs(float(self.get_parameter('path_resolution').value))
        self.progress_search_distance = abs(
            float(self.get_parameter('progress_search_distance').value)
        )
        self.slowdown_distance = abs(float(self.get_parameter('slowdown_distance').value))
        self.goal_tolerance = abs(float(self.get_parameter('goal_tolerance').value))
        self.amcl_timeout = Duration(seconds=float(self.get_parameter('amcl_timeout').value))
        control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.loop_path = bool(self.get_parameter('loop_path').value)
        self.require_autonomous_mode = bool(
            self.get_parameter('require_autonomous_mode').value
        )
        self.autonomy_enable_topic = self.get_parameter('autonomy_enable_topic').value
        self.keyboard_control = bool(self.get_parameter('keyboard_control').value)
        self.speed_step = abs(float(self.get_parameter('speed_step').value))
        self.min_speed_limit = abs(float(self.get_parameter('min_speed_limit').value))
        self.max_speed_limit = abs(float(self.get_parameter('max_speed_limit').value))
        keyboard_rate = max(1.0, float(self.get_parameter('keyboard_rate').value))
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.path_points = []
        self.current_index = 0
        self.lap_count = 0
        self.last_pose = None
        self.last_pose_time = None
        self.last_tf_warning_time = None
        self.stdin_fd = None
        self.original_terminal_settings = None
        self.finished = False
        self.autonomous_enabled = not self.require_autonomous_mode

        self.command_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.ackermann_topic,
            10,
        )
        self.path_publisher = self.create_publisher(Path, self.path_topic, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.pose_callback,
            10,
        )
        self.autonomy_subscription = None
        if self.require_autonomous_mode:
            self.autonomy_subscription = self.create_subscription(
                Int8,
                self.autonomy_enable_topic,
                self.autonomy_enable_callback,
                10,
            )
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.keyboard_timer = None
        if self.keyboard_control:
            self._start_keyboard_control(keyboard_rate)

        if self.pose_source == 'tf':
            pose_description = f'{self.map_frame} -> {self.base_frame} TF'
        else:
            pose_description = self.amcl_pose_topic

        self.get_logger().info(
            f'Waiting for {pose_description}; will publish commands on '
            f'{self.ackermann_topic}'
        )
        if self.require_autonomous_mode:
            self.get_logger().info(
                'Autonomous-mode gating enabled: commands are published only while '
                f'{self.autonomy_enable_topic} reports autonomous (1). Currently silent '
                'until autonomous mode is engaged.'
            )

    def parameter_callback(self, params):
        next_linear_speed = self.linear_speed
        next_min_linear_speed = self.min_linear_speed
        next_speed_step = self.speed_step
        next_min_speed_limit = self.min_speed_limit
        next_max_speed_limit = self.max_speed_limit

        for param in params:
            if param.name == 'linear_speed':
                value = self._numeric_parameter(param.value, param.name)
                if isinstance(value, SetParametersResult):
                    return value
                next_linear_speed = abs(value)
            elif param.name == 'min_linear_speed':
                value = self._numeric_parameter(param.value, param.name)
                if isinstance(value, SetParametersResult):
                    return value
                next_min_linear_speed = abs(value)
            elif param.name == 'speed_step':
                value = self._numeric_parameter(param.value, param.name)
                if isinstance(value, SetParametersResult):
                    return value
                next_speed_step = abs(value)
            elif param.name == 'min_speed_limit':
                value = self._numeric_parameter(param.value, param.name)
                if isinstance(value, SetParametersResult):
                    return value
                next_min_speed_limit = abs(value)
            elif param.name == 'max_speed_limit':
                value = self._numeric_parameter(param.value, param.name)
                if isinstance(value, SetParametersResult):
                    return value
                next_max_speed_limit = abs(value)

        if next_min_speed_limit > next_max_speed_limit:
            return SetParametersResult(
                successful=False,
                reason='min_speed_limit must be <= max_speed_limit',
            )

        next_linear_speed = clamp(
            next_linear_speed,
            next_min_speed_limit,
            next_max_speed_limit,
        )
        if next_linear_speed > 0.0 and next_min_linear_speed > next_linear_speed:
            next_min_linear_speed = next_linear_speed

        speed_changed = (
            next_linear_speed != self.linear_speed
            or next_min_linear_speed != self.min_linear_speed
            or next_speed_step != self.speed_step
            or next_min_speed_limit != self.min_speed_limit
            or next_max_speed_limit != self.max_speed_limit
        )

        self.linear_speed = next_linear_speed
        self.min_linear_speed = next_min_linear_speed
        self.speed_step = next_speed_step
        self.min_speed_limit = next_min_speed_limit
        self.max_speed_limit = next_max_speed_limit

        if speed_changed:
            self._log_speed()

        return SetParametersResult(successful=True)

    def _numeric_parameter(self, value, name):
        try:
            return float(value)
        except (TypeError, ValueError):
            return SetParametersResult(
                successful=False,
                reason=f'{name} must be numeric',
            )

    def _start_keyboard_control(self, keyboard_rate):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'Keyboard speed control disabled because stdin is not a TTY'
            )
            return

        self.stdin_fd = sys.stdin.fileno()
        self.original_terminal_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)
        atexit.register(self._restore_terminal)
        self.keyboard_timer = self.create_timer(
            1.0 / keyboard_rate,
            self._poll_keyboard,
        )
        self.get_logger().info(
            'Keyboard speed control enabled: press w to increase speed, '
            's to decrease speed'
        )

    def _poll_keyboard(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return

        key = sys.stdin.read(1).lower()
        if key == 'w':
            self._adjust_speed(self.speed_step)
        elif key == 's':
            self._adjust_speed(-self.speed_step)

    def _adjust_speed(self, delta):
        speed = clamp(
            self.linear_speed + delta,
            self.min_speed_limit,
            self.max_speed_limit,
        )
        self.set_parameters([
            Parameter('linear_speed', Parameter.Type.DOUBLE, float(speed)),
        ])

    def _log_speed(self):
        self.get_logger().info(
            'linear_speed=%.3f m/s, min_linear_speed=%.3f m/s'
            % (self.linear_speed, self.min_linear_speed)
        )

    def _restore_terminal(self):
        if self.stdin_fd is not None and self.original_terminal_settings is not None:
            termios.tcsetattr(
                self.stdin_fd,
                termios.TCSADRAIN,
                self.original_terminal_settings,
            )
            self.original_terminal_settings = None

    def destroy_node(self):
        self._restore_terminal()
        return super().destroy_node()

    def _pose_source(self, value):
        source = str(value).lower()
        if source in ('tf', 'transform'):
            return 'tf'
        if source in ('amcl', 'amcl_pose', 'topic'):
            return 'amcl_pose'
        self.get_logger().warn(f'Unknown pose_source "{value}", defaulting to TF')
        return 'tf'

    def _turn_direction_sign(self, value):
        direction = str(value).lower()
        if direction in ('left', 'ccw', 'counterclockwise', '1'):
            return 1.0
        if direction in ('right', 'cw', 'clockwise', '-1'):
            return -1.0
        self.get_logger().warn(
            f'Unknown turn_direction "{value}", defaulting to left'
        )
        return 1.0

    def pose_callback(self, msg):
        if self.pose_source != 'amcl_pose':
            return

        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.last_pose = (pose.position.x, pose.position.y, yaw)
        self.last_pose_time = self.get_clock().now()

    def autonomy_enable_callback(self, msg):
        enabled = msg.data == 1
        if enabled == self.autonomous_enabled:
            return

        self.autonomous_enabled = enabled
        if enabled:
            self.get_logger().info(
                'Autonomous mode engaged; publishing simple loop commands'
            )
        else:
            self.get_logger().info(
                'Manual mode engaged; simple loop controller is silent'
            )

    def _build_path(self, x, y, yaw):
        points = [PathPoint(x, y, yaw, 0.0)]
        x, y, yaw = self._append_straight(points, x, y, yaw, self.straight_distance)
        x, y, yaw = self._append_half_circle(points, x, y, yaw, self.turn_radius)
        x, y, yaw = self._append_straight(points, x, y, yaw, self.straight_distance)
        self._append_half_circle(points, x, y, yaw, self.turn_radius)
        return points

    def _append_straight(self, points, start_x, start_y, yaw, distance):
        steps = max(1, int(math.ceil(distance / self.path_resolution)))
        start_s = points[-1].s
        for step in range(1, steps + 1):
            d = min(step * self.path_resolution, distance)
            points.append(
                PathPoint(
                    start_x + d * math.cos(yaw),
                    start_y + d * math.sin(yaw),
                    yaw,
                    start_s + d,
                )
            )
        return points[-1].x, points[-1].y, yaw

    def _append_half_circle(self, points, start_x, start_y, yaw, radius):
        direction = self.turn_direction
        center_x = start_x - direction * radius * math.sin(yaw)
        center_y = start_y + direction * radius * math.cos(yaw)
        start_angle = math.atan2(start_y - center_y, start_x - center_x)
        arc_length = math.pi * radius
        steps = max(1, int(math.ceil(arc_length / self.path_resolution)))
        start_s = points[-1].s

        for step in range(1, steps + 1):
            distance = min(step * self.path_resolution, arc_length)
            angle = start_angle + direction * distance / radius
            point_yaw = normalize_angle(angle + direction * math.pi * 0.5)
            points.append(
                PathPoint(
                    center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                    point_yaw,
                    start_s + distance,
                )
            )

        return points[-1].x, points[-1].y, points[-1].yaw

    def _publish_path(self, frame_id):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for point in self.path_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation = quaternion_from_yaw(point.yaw)
            path.poses.append(pose)

        self.path_publisher.publish(path)

    def control_loop(self):
        if self.require_autonomous_mode and not self.autonomous_enabled:
            return

        if self.finished:
            return

        pose = self._current_pose()
        if pose is None:
            self._publish_stop()
            return

        x, y, yaw = pose
        if not self.path_points:
            self.path_points = self._build_path(x, y, yaw)
            self._publish_path(self.map_frame)
            self.get_logger().info(
                'Started simple loop path: straight %.2fm, half circle %.2fm radius, '
                'straight %.2fm, half circle %.2fm radius'
                % (
                    self.straight_distance,
                    self.turn_radius,
                    self.straight_distance,
                    self.turn_radius,
                )
            )

        previous_index = self.current_index
        nearest_index = self._find_nearest_index(x, y)
        if self.loop_path:
            self.current_index = nearest_index
            self._update_lap_count(previous_index, nearest_index)
        else:
            self.current_index = max(self.current_index, nearest_index)
        remaining = self.path_points[-1].s - self.path_points[self.current_index].s

        if not self.loop_path and remaining <= self.goal_tolerance:
            self.finished = True
            self._publish_stop()
            self.get_logger().info('Simple loop path complete')
            return

        target = self._target_point(self.current_index)
        speed = self.linear_speed
        if not self.loop_path:
            speed = self._speed_for_remaining_distance(remaining)
        if speed <= 0.0:
            self._publish_stop()
            return
        cmd = self._command_to_target(x, y, yaw, target, speed)
        self.command_publisher.publish(cmd)

    def _current_pose(self):
        if self.pose_source == 'tf':
            return self._pose_from_tf()
        if self.last_pose is None or self._amcl_pose_is_stale():
            return None
        return self.last_pose

    def _pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self._warn_tf_throttled(exc)
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            translation.x,
            translation.y,
            yaw_from_quaternion(rotation),
        )

    def _warn_tf_throttled(self, exc):
        now = self.get_clock().now()
        if (
            self.last_tf_warning_time is None
            or (now - self.last_tf_warning_time) > Duration(seconds=2.0)
        ):
            self.get_logger().warn(
                f'Waiting for {self.map_frame} -> {self.base_frame} TF: {exc}'
            )
            self.last_tf_warning_time = now

    def _amcl_pose_is_stale(self):
        return self.get_clock().now() - self.last_pose_time > self.amcl_timeout

    def _find_nearest_index(self, x, y):
        if self.loop_path:
            return self._find_nearest_index_on_loop(x, y)

        search_start = self.current_index
        current_s = self.path_points[self.current_index].s
        search_end = search_start

        while (
            search_end + 1 < len(self.path_points)
            and self.path_points[search_end + 1].s
            <= current_s + self.progress_search_distance
        ):
            search_end += 1

        best_index = search_start
        best_distance_sq = float('inf')
        for index in range(search_start, search_end + 1):
            point = self.path_points[index]
            distance_sq = (point.x - x) ** 2 + (point.y - y) ** 2
            if distance_sq < best_distance_sq:
                best_index = index
                best_distance_sq = distance_sq

        return best_index

    def _find_nearest_index_on_loop(self, x, y):
        best_index = 0
        best_distance_sq = float('inf')
        for index, point in enumerate(self.path_points):
            distance_sq = (point.x - x) ** 2 + (point.y - y) ** 2
            if distance_sq < best_distance_sq:
                best_index = index
                best_distance_sq = distance_sq
        return best_index

    def _update_lap_count(self, previous_index, nearest_index):
        if len(self.path_points) < 4:
            return

        path_length = len(self.path_points)
        if previous_index > path_length * 0.75 and nearest_index < path_length * 0.25:
            self.lap_count += 1
            self.get_logger().info(f'Simple loop lap {self.lap_count} complete')

    def _target_point(self, nearest_index):
        target_s = self.path_points[nearest_index].s + self.lookahead_distance
        if self.loop_path and target_s > self.path_points[-1].s:
            target_s -= self.path_points[-1].s
            nearest_index = 0

        for point in self.path_points[nearest_index:]:
            if point.s >= target_s:
                return point
        return self.path_points[-1]

    def _speed_for_remaining_distance(self, remaining):
        if self.linear_speed <= 0.0:
            return 0.0
        if self.slowdown_distance <= 1e-6:
            return self.linear_speed
        scale = clamp(remaining / self.slowdown_distance, 0.0, 1.0)
        return max(self.min_linear_speed, self.linear_speed * scale)

    def _command_to_target(self, x, y, yaw, target, speed):
        dx = target.x - x
        dy = target.y - y
        distance_sq = max(dx * dx + dy * dy, 1e-6)

        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        local_y = -sin_yaw * dx + cos_yaw * dy
        curvature = 2.0 * local_y / distance_sq

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(
            clamp(
                math.atan(self.wheelbase * curvature),
                -self.max_steering_angle,
                self.max_steering_angle,
            )
        )
        return cmd

    def _publish_stop(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        self.command_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLoopController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A Ctrl-C / shutdown race can surface as a RuntimeError from the
        # executor's message-take path; only propagate if the context is still
        # valid (i.e. this is a genuine runtime error, not a teardown race).
        if rclpy.ok():
            raise
    finally:
        # Restore the terminal first and unconditionally. Any ROS cleanup below
        # can fail after a Ctrl-C (e.g. publishing on an already-invalid
        # context), and such a failure must never leave the shell in no-echo
        # cbreak mode.
        node._restore_terminal()
        try:
            node._publish_stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
