"""Confirmed mode and neutral-rearm regression checks for the shared adapter."""
import time
from unittest.mock import Mock

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from osracer_bringup.joystick_teleop import JoystickTeleop


def adapter():
    node = object.__new__(JoystickTeleop)
    node.standalone_manual = False
    node.manual_enabled = False
    node.confirmed_mode = None
    node.mode_received_at = None
    node.neutral_required = True
    node.last_toggle_pressed = False
    node.mode_toggle_button = 0
    node.speed_axis = 0
    node.steering_axis = 1
    node.speed_scale = .8
    node.steering_scale = .34
    node.teleop_pub = Mock()
    node.mode_pub = Mock()
    node.command = lambda speed, steer: (speed, steer)
    node.log_mode = Mock()
    return node


def test_estop_reset_requires_confirmed_manual_and_neutral_stick():
    node = adapter()
    for mode in ('AUTO_DRIVE', 'EMERGENCY_STOP', 'HUMAN_CONTROL'):
        node.mode_callback(String(data=mode))
    assert node.manual_enabled
    node.joy_callback(Joy(axes=[1., 0.], buttons=[0]))
    assert node.teleop_pub.publish.call_args.args[0] == (0, 0)
    node.joy_callback(Joy(axes=[0., 0.], buttons=[0]))
    node.joy_callback(Joy(axes=[1., 0.], buttons=[0]))
    assert node.teleop_pub.publish.call_args.args[0] == (.8, 0)


def test_mode_request_does_not_speculatively_change_confirmed_mode():
    node = adapter()
    node.mode_callback(String(data='HUMAN_CONTROL'))
    node.joy_callback(Joy(axes=[0., 0.], buttons=[1]))
    assert node.confirmed_mode == 'HUMAN_CONTROL'
    assert node.mode_pub.publish.call_args.args[0].data == 1
    node.mode_callback(String(data='AUTO_DRIVE'))
    assert not node.manual_enabled


def test_lost_mode_heartbeat_disarms_and_requires_neutral_on_recovery():
    node = adapter()
    node.mode_callback(String(data='HUMAN_CONTROL'))
    node.mode_received_at = time.monotonic() - 2
    node.check_mode_freshness()
    assert not node.manual_enabled
    node.mode_callback(String(data='HUMAN_CONTROL'))
    assert node.manual_enabled and node.neutral_required
