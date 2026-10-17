"""Exercise the production adapter and vendor converter without hardware."""
import os
import time
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '199' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='Requires isolated ROS domain 199 localhost',
)


def test_feedback_stops_and_reverse(runtime_nodes):
    import subprocess
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from ackermann_msgs.msg import AckermannDriveStamped
    from vesc_msgs.msg import VescStateStamped
    from std_msgs.msg import Float64
    from carkit_bringup.vesc_speed_control import VescSpeedControl
    rclpy.init(args=['--ros-args', '-p', 'max_speed:=4.0', '-p', 'max_reverse_speed:=2.0'])
    peer = rclpy.create_node('speed_feedback_test')
    executor = SingleThreadedExecutor()
    runtime = runtime_nodes
    runtime.create("vesc_speed_control", VescSpeedControl, executor,
                   ("--ros-args", "-p", "max_speed:=4.0", "-p", "max_reverse_speed:=2.0"))
    executor.add_node(peer)
    command = peer.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 1)
    wheel = peer.create_publisher(VescStateStamped, 'sensors/core', 10)
    rpms = []
    sub = peer.create_subscription(Float64, 'commands/motor/speed', lambda m: rpms.append(m.data), 10)
    from ament_index_python.packages import get_package_prefix
    converter = subprocess.Popen([get_package_prefix('vesc_ackermann') + '/lib/vesc_ackermann/ackermann_to_vesc_node',
        '--ros-args', '-r', 'ackermann_cmd:=ackermann_cmd/feedback',
        '-p', 'speed_to_erpm_gain:=4614.0', '-p', 'steering_angle_to_servo_gain:=-0.8',
        '-p', 'steering_angle_to_servo_offset:=0.48'], stdout=subprocess.DEVNULL)
    target, measured, feedback, commands, frozen, fault = 1.0, .6, True, True, None, 0

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if commands:
                message = AckermannDriveStamped()
                message.drive.speed = target
                command.publish(message)
            if feedback:
                message = VescStateStamped()
                message.header.stamp = frozen or peer.get_clock().now().to_msg()
                message.state.speed = measured * 4614.0
                message.state.fault_code = fault
                wheel.publish(message)
            for _ in range(5):
                executor.spin_once(timeout_sec=.004)

    try:
        pump(2)
        assert len(rpms) > 10
        assert 4614 < rpms[-1] <= 1.5 * 4614
        target = 0.0
        pump(.15)
        assert rpms[-1] == 0
        target, measured = -1.0, -.6
        pump(.5)
        assert -1.5 * 4614 <= rpms[-1] < -4614
        feedback = False
        pump(.4)
        assert all(v == 0 for v in rpms[-3:])
        feedback = True
        frozen = peer.get_clock().now().to_msg()
        pump(.4)
        assert rpms[-1] == 0
        frozen = None
        pump(.3)
        assert rpms[-1] < -4614
        fault = 1
        pump(.1)
        assert rpms[-1] == 0
        fault = 0
        commands = False
        pump(.4)
        assert all(v == 0 for v in rpms[-3:])
    finally:
        converter.terminate()
        converter.wait(timeout=10)
        executor.shutdown()
        runtime.close()
        peer.destroy_node()
        rclpy.shutdown()
