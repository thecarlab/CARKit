#!/usr/bin/env python3
"""Baseline control center: joystick manual/AV switch + drive relay with behavior stop."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, String

TELEOP_TOPIC = "/teleop"
DRIVE_TOPIC = "/drive"
OUTPUT_TOPIC = "/ackermann_cmd"
AUTONOMY_ENABLE_TOPIC = "enable_autonomous_control"


class BaselineControlCenterNode(Node):
    def __init__(self) -> None:
        super().__init__("baseline_control_center_node")
        self.teleop = None
        self.drive = None
        self.override_cmd = None
        self.override_active = False
        self.speed_limit_mps = 0.0
        self.auto_mode = False

        self.create_subscription(
            Int8, AUTONOMY_ENABLE_TOPIC, self.autonomy_callback, 10
        )
        self.create_subscription(
            AckermannDriveStamped, TELEOP_TOPIC, self.teleop_callback, 10
        )
        self.create_subscription(
            AckermannDriveStamped, DRIVE_TOPIC, self.drive_callback, 10
        )
        self.create_subscription(
            Bool, "/behavior/override_active", self.active_callback, 10
        )
        self.create_subscription(
            AckermannDriveStamped, "/behavior/override_cmd", self.override_callback, 10
        )
        self.create_subscription(
            Float32, "/behavior/speed_limit", self.speed_limit_callback, 10
        )
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, OUTPUT_TOPIC, 10)
        self.state_pub = self.create_publisher(String, "/control_center/main_state", 10)
        self.create_timer(0.02, self.publish_cmd)
        self.get_logger().info(
            f"baseline_control_center_node started in HUMAN_CONTROL -> {OUTPUT_TOPIC}"
        )

    def autonomy_callback(self, msg: Int8) -> None:
        auto_mode = msg.data == 1
        if auto_mode != self.auto_mode:
            self.auto_mode = auto_mode
            self.get_logger().info(
                f"Mode -> {'AUTO_DRIVE' if auto_mode else 'HUMAN_CONTROL'}"
            )

    def teleop_callback(self, msg: AckermannDriveStamped) -> None:
        self.teleop = msg

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        self.drive = msg

    def active_callback(self, msg: Bool) -> None:
        self.override_active = bool(msg.data)

    def override_callback(self, msg: AckermannDriveStamped) -> None:
        self.override_cmd = msg

    def speed_limit_callback(self, msg: Float32) -> None:
        self.speed_limit_mps = max(0.0, float(msg.data))

    def publish_cmd(self) -> None:
        if not self.auto_mode:
            source = self.teleop
            source_name = "teleop / HUMAN_CONTROL"
        elif self.override_active:
            source = self.override_cmd
            source_name = "behavior override"
        else:
            source = self.drive
            source_name = "autonomous driving"

        cmd = AckermannDriveStamped()
        if source is not None:
            cmd.drive = source.drive
        if (
            self.auto_mode
            and not self.override_active
            and self.speed_limit_mps > 0.0
            and cmd.drive.speed != 0.0
        ):
            cmd.drive.speed = math.copysign(
                self.speed_limit_mps, cmd.drive.speed
            )
            source_name = f"{source_name} (speed {self.speed_limit_mps:.2f} m/s)"

        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"Current cmd source: {source_name}",
            throttle_duration_sec=1.0,
        )
        if self.override_active and not self.auto_mode:
            self.get_logger().warning(
                "Behavior override active but mode is HUMAN_CONTROL; "
                "press L1 to enable AUTO_DRIVE for stop/limit to apply",
                throttle_duration_sec=3.0,
            )
        state = "AUTO_DRIVE" if self.auto_mode else "HUMAN_CONTROL"
        self.state_pub.publish(String(data=state))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaselineControlCenterNode()
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
