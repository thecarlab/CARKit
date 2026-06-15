#!/usr/bin/env python3

# Copyright 2026 CARKit maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from carkit_behavior.behavior_logic import (
    RoadRuleStateMachine,
    in_trigger_zone,
)
from carkit_perception_msgs.msg import YoloDetection3D, YoloDetection3DArray


class RoadRuleBehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("road_rule_behavior")

        self.declare_parameter("detection_topic", "/yolo/detections_3d")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("navigation_topic", "/drive")
        self.declare_parameter("behavior_topic", "/behavior")
        self.declare_parameter("state_topic", "/carkit_behavior/state")
        self.declare_parameter("traffic_light_min_confidence", 0.5)
        self.declare_parameter("stop_sign_min_confidence", 0.4)
        self.declare_parameter("max_lateral_offset", 1.0)
        self.declare_parameter("stop_distance", 1.0)
        self.declare_parameter("stop_speed_threshold", 0.05)
        self.declare_parameter("stop_hold_seconds", 3.0)
        self.declare_parameter("stop_cooldown_seconds", 5.0)
        self.declare_parameter("stop_rearm_absence_seconds", 1.0)
        self.declare_parameter("command_rate", 20.0)

        self.traffic_light_min_confidence = float(
            self.get_parameter("traffic_light_min_confidence").value
        )
        self.stop_sign_min_confidence = float(
            self.get_parameter("stop_sign_min_confidence").value
        )
        self.max_lateral_offset = float(
            self.get_parameter("max_lateral_offset").value
        )
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        if self.stop_distance <= 0.0:
            raise ValueError("stop_distance must be greater than zero")

        self.state_machine = RoadRuleStateMachine(
            stop_speed_threshold=float(
                self.get_parameter("stop_speed_threshold").value
            ),
            stop_hold_seconds=float(
                self.get_parameter("stop_hold_seconds").value
            ),
            stop_cooldown_seconds=float(
                self.get_parameter("stop_cooldown_seconds").value
            ),
            stop_rearm_absence_seconds=float(
                self.get_parameter("stop_rearm_absence_seconds").value
            ),
        )
        self.current_speed = 0.0
        self.latest_navigation_command = AckermannDriveStamped()
        self.last_state_name = None

        self.detection_sub = self.create_subscription(
            YoloDetection3DArray,
            str(self.get_parameter("detection_topic").value),
            self.detection_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.navigation_sub = self.create_subscription(
            AckermannDriveStamped,
            str(self.get_parameter("navigation_topic").value),
            self.navigation_callback,
            10,
        )
        self.behavior_pub = self.create_publisher(
            AckermannDriveStamped,
            str(self.get_parameter("behavior_topic").value),
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.reset_service = self.create_service(
            Trigger,
            "/carkit_behavior/reset",
            self.reset_callback,
        )
        command_rate = float(self.get_parameter("command_rate").value)
        if command_rate <= 0.0:
            raise ValueError("command_rate must be greater than zero")
        self.timer = self.create_timer(1.0 / command_rate, self.timer_callback)

        self.get_logger().info(
            "Road-rule behavior started; stop override publishes on "
            f"{self.get_parameter('behavior_topic').value}"
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def odom_callback(self, message: Odometry) -> None:
        self.current_speed = float(message.twist.twist.linear.x)

    def navigation_callback(self, message: AckermannDriveStamped) -> None:
        self.latest_navigation_command = message

    def detection_callback(self, message: YoloDetection3DArray) -> None:
        red_light = False
        green_light = False
        stop_sign = False
        stop_sign_visible = False

        for detection in message.detections:
            if detection.class_name == "stop sign":
                if detection.confidence >= self.stop_sign_min_confidence:
                    stop_sign_visible = True
                    if self.in_stop_zone(detection):
                        stop_sign = True
                continue

            if (
                detection.class_name != "traffic light"
                or detection.confidence < self.traffic_light_min_confidence
            ):
                continue

            if (
                detection.traffic_light_color
                == YoloDetection3D.TRAFFIC_LIGHT_RED
                and self.in_stop_zone(detection)
            ):
                red_light = True
            elif (
                detection.traffic_light_color
                == YoloDetection3D.TRAFFIC_LIGHT_GREEN
                and self.in_stop_zone(detection)
            ):
                green_light = True

        self.state_machine.observe(
            red_light=red_light,
            green_light=green_light,
            stop_sign=stop_sign,
            stop_sign_visible=stop_sign_visible,
            now=self.now_seconds(),
        )

    def in_stop_zone(self, detection: YoloDetection3D) -> bool:
        return in_trigger_zone(
            position_valid=detection.position_valid,
            x=detection.x,
            z=detection.z,
            max_lateral_offset=self.max_lateral_offset,
            trigger_distance=self.stop_distance,
        )

    def timer_callback(self) -> None:
        self.state_machine.update(self.current_speed, self.now_seconds())

        state_name = self.state_machine.state_name
        if state_name != self.last_state_name:
            self.get_logger().info(f"Behavior state: {state_name}")
            self.last_state_name = state_name
        self.state_pub.publish(String(data=state_name))

        if not self.state_machine.stop_active:
            return

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = (
            self.latest_navigation_command.header.frame_id or "base_link"
        )
        command.drive.steering_angle = (
            self.latest_navigation_command.drive.steering_angle
        )
        command.drive.steering_angle_velocity = (
            self.latest_navigation_command.drive.steering_angle_velocity
        )
        command.drive.speed = 0.0
        command.drive.acceleration = 0.0
        command.drive.jerk = 0.0
        self.behavior_pub.publish(command)

    def reset_callback(self, request, response):
        del request
        self.state_machine.reset()
        response.success = True
        response.message = "Road-rule stop state reset"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadRuleBehaviorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
