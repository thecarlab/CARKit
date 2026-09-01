from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from std_msgs.msg import Int8

from osracer_bringup.joystick_teleop import JoystickTeleop


class JoystickTeleopModeTests(unittest.TestCase):
    def test_external_autonomous_request_disables_manual_and_stops_teleop(self):
        zero_command = object()
        node = SimpleNamespace(
            manual_enabled=True,
            teleop_pub=Mock(),
            command=Mock(return_value=zero_command),
            log_mode=Mock(),
        )

        JoystickTeleop.mode_callback(node, Int8(data=1))

        self.assertFalse(node.manual_enabled)
        node.command.assert_called_once_with(0.0, 0.0)
        node.teleop_pub.publish.assert_called_once_with(zero_command)
        node.log_mode.assert_called_once_with('external mode request')

    def test_external_human_request_restores_remote_without_motion(self):
        node = SimpleNamespace(
            manual_enabled=False,
            teleop_pub=Mock(),
            command=Mock(),
            log_mode=Mock(),
        )

        JoystickTeleop.mode_callback(node, Int8(data=0))

        self.assertTrue(node.manual_enabled)
        node.teleop_pub.publish.assert_not_called()
        node.log_mode.assert_called_once_with('external mode request')

    def test_invalid_or_repeated_mode_request_is_ignored(self):
        node = SimpleNamespace(
            manual_enabled=True,
            teleop_pub=Mock(),
            command=Mock(),
            log_mode=Mock(),
        )

        JoystickTeleop.mode_callback(node, Int8(data=0))
        JoystickTeleop.mode_callback(node, Int8(data=7))

        self.assertTrue(node.manual_enabled)
        node.command.assert_not_called()
        node.teleop_pub.publish.assert_not_called()
        node.log_mode.assert_not_called()
