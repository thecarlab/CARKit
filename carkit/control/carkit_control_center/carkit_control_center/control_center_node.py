#!/usr/bin/env python3

import math
from copy import deepcopy
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int8, String


HUMAN_CONTROL = "HUMAN_CONTROL"
AUTO_DRIVE = "AUTO_DRIVE"
EMERGENCY_STOP = "EMERGENCY_STOP"
VALID_STATES = {HUMAN_CONTROL, AUTO_DRIVE, EMERGENCY_STOP}


class TimedMessage:
    def __init__(self) -> None:
        self.msg = None
        self.time: Optional[float] = None

    def update(self, msg, stamp: float) -> None:
        self.msg = msg
        self.time = stamp

    def fresh(self, now: float, timeout: float) -> bool:
        return (
            self.msg is not None
            and self.time is not None
            and now - self.time <= timeout
        )


class ControlCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("control_center_node")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("auto_button", 0)
        self.declare_parameter("human_button", 1)
        self.declare_parameter("estop_button", 2)
        self.declare_parameter("clear_estop_button", 3)
        self.declare_parameter("teleop_timeout_sec", 0.30)
        self.declare_parameter("nav2_timeout_sec", 0.50)
        self.declare_parameter("behavior_timeout_sec", 0.50)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_steering_angle", 0.34)
        self.declare_parameter("initial_state", HUMAN_CONTROL)
        self.declare_parameter("use_autonomy_enable_topic", True)
        self.declare_parameter("autonomy_enable_topic", "enable_autonomous_control")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.auto_button = int(self.get_parameter("auto_button").value)
        self.human_button = int(self.get_parameter("human_button").value)
        self.estop_button = int(self.get_parameter("estop_button").value)
        self.clear_estop_button = int(
            self.get_parameter("clear_estop_button").value
        )
        self.teleop_timeout_sec = float(
            self.get_parameter("teleop_timeout_sec").value
        )
        self.nav2_timeout_sec = float(self.get_parameter("nav2_timeout_sec").value)
        self.behavior_timeout_sec = float(
            self.get_parameter("behavior_timeout_sec").value
        )
        self.max_speed = abs(float(self.get_parameter("max_speed").value))
        self.max_steering_angle = abs(
            float(self.get_parameter("max_steering_angle").value)
        )
        self.use_autonomy_enable_topic = bool(
            self.get_parameter("use_autonomy_enable_topic").value
        )
        self.autonomy_enable_topic = str(
            self.get_parameter("autonomy_enable_topic").value
        )
        self.main_state = str(self.get_parameter("initial_state").value)
        if self.main_state not in VALID_STATES:
            self.get_logger().warning(
                f"Invalid initial_state={self.main_state}; using {HUMAN_CONTROL}"
            )
            self.main_state = HUMAN_CONTROL

        self.last_autonomy_enable = 0
        self.teleop = TimedMessage()
        self.nav2 = TimedMessage()
        self.behavior_override = TimedMessage()
        self.behavior_override_active = TimedMessage()
        self.speed_limit = TimedMessage()
        self.previous_buttons: list[int] = []
        self.debug_counter = 0

        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.create_subscription(
            AckermannDriveStamped,
            "/teleop",
            self.teleop_callback,
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.nav2_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/behavior/override_active",
            self.behavior_active_callback,
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            "/behavior/override_cmd",
            self.behavior_cmd_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/behavior/speed_limit",
            self.speed_limit_callback,
            10,
        )
        if self.use_autonomy_enable_topic:
            self.create_subscription(
                Int8,
                self.autonomy_enable_topic,
                self.autonomy_enable_callback,
                10,
            )

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            "/ackermann_cmd",
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            "/control_center/main_state",
            10,
        )
        self.selected_pub = self.create_publisher(
            String,
            "/control_center/selected_cmd",
            10,
        )
        self.debug_pub = self.create_publisher(
            String,
            "/control_center/debug",
            10,
        )

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.timer_callback)
        mode_source = (
            f"joy_teleop topic {self.autonomy_enable_topic}"
            if self.use_autonomy_enable_topic
            else "joystick auto/human buttons"
        )
        self.get_logger().info(
            f"control_center_node started in {self.main_state}; "
            f"mode source: {mode_source}; "
            f"publishing /ackermann_cmd at {self.publish_rate_hz:.1f} Hz"
        )

    def joy_callback(self, msg: Joy) -> None:
        for button_index, action in (
            (self.estop_button, self.enter_emergency_stop),
            (self.clear_estop_button, self.clear_emergency_stop),
        ):
            if self.rising_edge(msg.buttons, button_index):
                action()
        if not self.use_autonomy_enable_topic:
            for button_index, action in (
                (self.human_button, self.enter_human_control),
                (self.auto_button, self.enter_auto_drive),
            ):
                if self.rising_edge(msg.buttons, button_index):
                    action()
        self.previous_buttons = list(msg.buttons)

    def autonomy_enable_callback(self, msg: Int8) -> None:
        if msg.data not in (0, 1):
            return
        self.last_autonomy_enable = int(msg.data)
        if self.main_state == EMERGENCY_STOP:
            return
        previous_state = self.main_state
        self.apply_autonomy_enable(self.last_autonomy_enable)
        if self.main_state != previous_state:
            mode = "AV stack" if self.main_state == AUTO_DRIVE else "manual"
            self.get_logger().info(f"Mode switched to {mode} ({self.main_state})")

    def teleop_callback(self, msg: AckermannDriveStamped) -> None:
        self.teleop.update(msg, self.now_sec())

    def nav2_callback(self, msg: AckermannDriveStamped) -> None:
        self.nav2.update(msg, self.now_sec())

    def behavior_active_callback(self, msg: Bool) -> None:
        self.behavior_override_active.update(msg, self.now_sec())

    def behavior_cmd_callback(self, msg: AckermannDriveStamped) -> None:
        self.behavior_override.update(msg, self.now_sec())

    def speed_limit_callback(self, msg: Float32) -> None:
        self.speed_limit.update(msg, self.now_sec())

    def timer_callback(self) -> None:
        now = self.now_sec()
        selected = "zero"

        if self.main_state == EMERGENCY_STOP:
            command = self.zero_command()
            selected = "emergency_stop"
        elif self.main_state == HUMAN_CONTROL:
            if self.teleop.fresh(now, self.teleop_timeout_sec):
                command = self.copy_command(self.teleop.msg)
                selected = "teleop"
            else:
                command = self.zero_command()
                selected = "teleop_stale_zero"
        elif self.main_state == AUTO_DRIVE:
            behavior_active = (
                self.behavior_override_active.fresh(now, self.behavior_timeout_sec)
                and bool(self.behavior_override_active.msg.data)
            )
            if behavior_active and self.behavior_override.fresh(
                now,
                self.behavior_timeout_sec,
            ):
                command = self.copy_command(self.behavior_override.msg)
                selected = "behavior_override"
            elif self.nav2.fresh(now, self.nav2_timeout_sec):
                command = self.copy_command(self.nav2.msg)
                self.apply_speed_limit(command, now)
                selected = "nav2_drive"
            else:
                command = self.zero_command()
                selected = "nav2_stale_zero"
        else:
            command = self.zero_command()
            selected = "invalid_state_zero"

        self.clamp_command(command)
        command.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(command)
        self.publish_text(self.state_pub, self.main_state)
        self.publish_text(self.selected_pub, selected)
        self.publish_debug(selected, now)

    def rising_edge(self, buttons: list[int], index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False
        previous = 0
        if index < len(self.previous_buttons):
            previous = self.previous_buttons[index]
        return bool(buttons[index]) and not bool(previous)

    def enter_emergency_stop(self) -> None:
        self.main_state = EMERGENCY_STOP

    def clear_emergency_stop(self) -> None:
        if self.main_state == EMERGENCY_STOP:
            if self.use_autonomy_enable_topic:
                self.apply_autonomy_enable(self.last_autonomy_enable)
            else:
                self.main_state = HUMAN_CONTROL

    def enter_human_control(self) -> None:
        if self.main_state != EMERGENCY_STOP:
            self.main_state = HUMAN_CONTROL
            self.last_autonomy_enable = 0

    def enter_auto_drive(self) -> None:
        if self.main_state != EMERGENCY_STOP:
            self.main_state = AUTO_DRIVE
            self.last_autonomy_enable = 1

    def apply_autonomy_enable(self, enabled: int) -> None:
        self.main_state = AUTO_DRIVE if enabled == 1 else HUMAN_CONTROL

    def apply_speed_limit(self, command: AckermannDriveStamped, now: float) -> None:
        if not self.speed_limit.fresh(now, self.behavior_timeout_sec):
            return
        limit = abs(float(self.speed_limit.msg.data))
        if not math.isfinite(limit):
            return
        command.drive.speed = clamp(command.drive.speed, -limit, limit)

    def clamp_command(self, command: AckermannDriveStamped) -> None:
        command.drive.speed = clamp(command.drive.speed, -self.max_speed, self.max_speed)
        command.drive.steering_angle = clamp(
            command.drive.steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

    def copy_command(self, msg: AckermannDriveStamped) -> AckermannDriveStamped:
        command = AckermannDriveStamped()
        command.header = deepcopy(msg.header)
        command.drive = deepcopy(msg.drive)
        return command

    def zero_command(self) -> AckermannDriveStamped:
        command = AckermannDriveStamped()
        command.drive.speed = 0.0
        command.drive.steering_angle = 0.0
        return command

    def publish_text(self, publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def publish_debug(self, selected: str, now: float) -> None:
        self.debug_counter += 1
        debug_rate_divisor = max(1, int(self.publish_rate_hz))
        if self.debug_counter % debug_rate_divisor != 0:
            return
        fields = {
            "state": self.main_state,
            "selected": selected,
            "teleop_fresh": self.teleop.fresh(now, self.teleop_timeout_sec),
            "nav2_fresh": self.nav2.fresh(now, self.nav2_timeout_sec),
            "behavior_active_fresh": self.behavior_override_active.fresh(
                now,
                self.behavior_timeout_sec,
            ),
            "behavior_cmd_fresh": self.behavior_override.fresh(
                now,
                self.behavior_timeout_sec,
            ),
            "speed_limit_fresh": self.speed_limit.fresh(
                now,
                self.behavior_timeout_sec,
            ),
        }
        self.publish_text(
            self.debug_pub,
            " ".join(f"{key}={value}" for key, value in fields.items()),
        )

    def now_sec(self) -> float:
        return stamp_to_sec(self.get_clock().now().to_msg())


def clamp(value: float, lower: float, upper: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return min(max(float(value), lower), upper)


def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlCenterNode()
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
