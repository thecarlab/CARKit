"""Hardware-free ROS integration: real arbiter + real worker + simulated feedback.

Run only in an isolated ROS domain. No motor driver is started by this test.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('vesc_msgs.msg')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, Int8, String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from vesc_msgs.msg import VescStateStamped
import yaml

from carkit_webui.calibration import STAGES, atomic_json, fingerprint, read_json

ARBITER = os.environ.get('CARKIT_ARBITER_EXE')
pytestmark = pytest.mark.skipif(not ARBITER, reason='Set CARKIT_ARBITER_EXE to run isolated ROS integration')
WORKER = Path(__file__).parents[1] / 'carkit_webui/calibration_worker.py'
PROFILE = Path(__file__).parents[3] / 'core/carkit_bringup/config/hardware/vesc_rplidar.yaml'


class SimulatedVehicle(Node):
    def __init__(self, config, fault):
        super().__init__('calibration_simulated_vehicle')
        self.config = config
        self.fault = fault
        self.command = AckermannDriveStamped()
        self.mode = None
        self.moving_since = None
        self.max_speed = 0
        self.v = 0.0
        self.x = 0.0
        self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', self.on_command, 10)
        self.create_subscription(String, '/control_center/main_state', lambda m: setattr(self, 'mode', m.data), 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.gyro = self.create_publisher(Imu, '/camera/camera/gyro/sample', 10)
        self.accel = self.create_publisher(Imu, '/camera/camera/accel/sample', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.vesc = self.create_publisher(VescStateStamped, '/sensors/core', 10)
        self.servo = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.competing = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.auto = self.create_publisher(Int8, '/enable_autonomous_control', 10)
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'; transform.child_frame_id = 'calibration_sensor'
        transform.header.stamp = self.get_clock().now().to_msg(); transform.transform.rotation.w = 1.0
        self.mount_transform = transform
        self.mount_ready_at = time.monotonic()+5.0 if fault == 'delayed_tf_resume' else 0.0
        self.mount_sent = False
        if not self.mount_ready_at:
            self.tf.sendTransform(transform)
            self.mount_sent = True
        self.timer = self.create_timer(.02, self.tick)

    def on_command(self, message):
        self.command = message
        self.max_speed = max(self.max_speed, abs(message.drive.speed))
        if abs(message.drive.speed) > .05 and self.moving_since is None:
            self.moving_since = time.monotonic()

    def tick(self):
        if not self.mount_sent and time.monotonic() >= self.mount_ready_at:
            self.tf.sendTransform(self.mount_transform)
            self.mount_sent = True
        stamp = self.get_clock().now().to_msg()
        # Simulate the low measured speed of the current drivetrain at a higher command.
        self.v += (self.command.drive.speed*(1.0 if self.fault in ('long_straight', 'long_reverse', 'long_turn') else .3)-self.v)*.4
        self.x += self.v*.02
        servo = self.config['driver']['steering_angle_to_servo_offset'] + self.command.drive.steering_angle*self.config['driver']['steering_angle_to_servo_gain']
        # A known physical center/gain different from the configured conversion.
        physical_angle = (servo-.475)/-1.35
        yaw = self.v*math.tan(physical_angle)/self.config['wheelbase']
        odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'odom'
        odom.pose.pose.orientation.w = 1.; odom.pose.pose.position.x = self.x
        odom.twist.twist.linear.x = self.v
        self.odom.publish(odom)
        imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = 'calibration_sensor'
        imu.angular_velocity.z = yaw+.003; imu.linear_acceleration.z = 9.81
        bad = self.moving_since is not None and time.monotonic()-self.moving_since > 1
        if not (self.fault == 'lost_imu' and bad):
            self.gyro.publish(imu)
        self.accel.publish(imu)
        scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = 'calibration_sensor'
        scan.angle_min = -math.pi; scan.angle_increment = math.pi/180
        scan.range_min = .01; scan.range_max = 20.
        scan.ranges = [10.] * 360
        if self.fault == 'obstacle' and bad:
            scan.ranges = [(.4 if 175 <= i < 186 else 10.) for i in range(360)]
        if self.fault == 'return_obstacle' and self.command.drive.speed < -.1:
            scan.ranges = [(.4 if i < 6 or i > 354 else 10.) for i in range(360)]
        self.scan.publish(scan)
        state = VescStateStamped(); state.header.stamp = stamp
        state.state.speed = self.v*self.config['driver']['speed_to_erpm_gain']
        state.state.voltage_input = 12.1; state.state.fault_code = 0
        self.vesc.publish(state)
        self.servo.publish(Float64(data=servo))
        if self.mode == 'CALIBRATING':
            competing = AckermannDriveStamped(); competing.drive.speed = 1.5
            self.competing.publish(competing)
            self.auto.publish(Int8(data=1))


@pytest.mark.parametrize('fault', ['all', 'normal', 'long_straight', 'long_reverse', 'long_turn', 'resume', 'delayed_tf_resume', 'lost_imu', 'lost_lease', 'cancelled', 'obstacle', 'return_obstacle'])
def test_worker_sequence_and_fail_closed(tmp_path, fault):
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated test ROS domain'
    config = yaml.safe_load(PROFILE.read_text())
    request = {'config':config, 'speed':1.0, 'fingerprint':fingerprint(config), 'mode':'all' if fault == 'all' else 'single'}
    atomic_json(tmp_path/'request.json', request)
    seed_stages = STAGES[:2] if fault == 'long_turn' else STAGES[:1] if fault == 'long_reverse' else STAGES[:-1] if fault in ('resume','delayed_tf_resume') else ()
    if seed_stages:
        rows = []
        for index, (key, _, direction, scale) in enumerate(seed_stages):
            servo = config['driver']['steering_angle_to_servo_offset'] + scale*config['max_steering_angle']*config['driver']['steering_angle_to_servo_gain']
            for j in range(45):
                speed = direction*.5
                rows.append({'stage':key, 't':index*8+j*.05, 'settled':True, 'speed':speed,
                             'yaw_rate':speed*math.tan((servo-.475)/-1.35)/config['wheelbase']+.02,
                             'servo':servo, 'gyro_bias':.02})
        atomic_json(tmp_path/'checkpoint.json', {'fingerprint':fingerprint(config),'speed':1.0,
                    'completed':[s[0] for s in seed_stages], 'samples':rows,
                    'returns':[{'stage':s[0],'position_error_m':0} for s in seed_stages]})
    atomic_json(tmp_path/'lease.json', {'deadline':time.monotonic()+30})
    rclpy.init()
    vehicle = SimulatedVehicle(config, fault)
    executor = SingleThreadedExecutor(); executor.add_node(vehicle)
    thread = threading.Thread(target=executor.spin); thread.start()
    log = (tmp_path/'arbiter.log').open('w')
    arbiter = subprocess.Popen([ARBITER, '--ros-args', '-p', 'max_speed:=4.0', '-p', 'max_reverse_speed:=2.0',
                                '-p', f'max_steering_angle:={config["max_steering_angle"]}'], stdout=log, stderr=subprocess.STDOUT)
    worker = None
    try:
        deadline = time.monotonic()+12
        while vehicle.mode is None and time.monotonic() < deadline:
            time.sleep(.05)
        assert vehicle.mode == 'HUMAN_CONTROL'
        worker = subprocess.Popen([sys.executable, str(WORKER), str(tmp_path)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic()+240
        while worker.poll() is None and time.monotonic() < deadline:
            bad = vehicle.moving_since is not None and time.monotonic()-vehicle.moving_since > 1
            if not (fault == 'lost_lease' and bad):
                atomic_json(tmp_path/'lease.json', {'deadline':time.monotonic()+1.5})
            if fault == 'cancelled' and bad:
                (tmp_path/'cancel').touch()
            time.sleep(.1)
        assert worker.poll() is not None, 'Worker did not finish within its bounded sequence'
        output = worker.communicate(timeout=2)[0]
        status = read_json(tmp_path/'status.json', {})
        assert vehicle.mode == 'EMERGENCY_STOP', (status, output)
        assert abs(vehicle.v) < .01
        assert vehicle.max_speed <= 1.00001  # Competing 1.5 m/s input was ignored.
        if fault in ('normal', 'lost_lease', 'long_straight', 'long_reverse', 'long_turn'):
            assert status['phase'] == 'awaiting_confirmation', (status, output)
            expected_stage = 'left_full' if fault == 'long_turn' else 'reverse' if fault == 'long_reverse' else 'forward'
            checkpoint = read_json(tmp_path/'checkpoint.json')
            assert checkpoint['completed'] == [s[0] for s in seed_stages] + [expected_stage]
            recorded = [json.loads(line) for line in (tmp_path/'samples.jsonl').read_text().splitlines()]
            assert {s['stage'] for s in recorded} == {expected_stage}
            assert not (tmp_path/'result.json').exists()
            assert vehicle.max_speed >= .99
            if fault.startswith('long_'):
                assert checkpoint['returns'][-1]['outbound_m'] > 1.0, checkpoint['returns']
                assert checkpoint['returns'][-1]['return_m'] > 1.0
            # The worker has exited and latched E-stop; it cannot advance itself.
            assert status['completed_stages'] == checkpoint['completed']
        elif fault in ('all', 'resume', 'delayed_tf_resume'):
            assert status['phase'] == 'complete', (status, output)
            assert vehicle.max_speed >= .99, 'Worker must reach the new 1.0 m/s command'
            result = read_json(tmp_path/'result.json')
            assert len(result['stages']) == len(STAGES)
            assert len(result['returns']) == 6
            assert all(r['position_error_m'] <= .15 for r in result['returns'])
            assert result['returns'][-1]['from_initial_position_m'] <= .20
            checkpoint = read_json(tmp_path/'checkpoint.json')
            assert checkpoint['completed'] == [s[0] for s in STAGES]
            if fault in ('resume', 'delayed_tf_resume'):
                recorded = [json.loads(line) for line in (tmp_path/'samples.jsonl').read_text().splitlines()]
                assert {s['stage'] for s in recorded} == {'right_half'}
                assert all(s['gyro_bias'] == pytest.approx(.003, abs=.001) for s in recorded)
            assert not result['good']
            assert result['recommendation']['steering_angle_to_servo_offset'] == pytest.approx(.475, abs=.003)
            assert result['recommendation']['steering_angle_to_servo_gain'] == pytest.approx(-1.35, abs=.02)
        else:
            assert status['phase'] == 'failed', (status, output)
            expected = {'lost_imu':'gyro', 'lost_lease':'heartbeat', 'cancelled':'cancelled', 'obstacle':'obstacle', 'return_obstacle':'obstacle'}[fault]
            assert expected in status['message'], status
    finally:
        if worker and worker.poll() is None:
            worker.send_signal(signal.SIGINT)
            worker.wait(timeout=5)
        arbiter.send_signal(signal.SIGINT); arbiter.wait(timeout=5); log.close()
        executor.shutdown(); thread.join(timeout=2); vehicle.destroy_node(); rclpy.shutdown()


def test_obstacle_threshold_is_20cm_beyond_vehicle_front_and_rear():
    from carkit_webui.calibration_worker import obstacle_in_sweep
    c = yaml.safe_load(PROFILE.read_text())
    front = c['footprint']['center_x'] + c['footprint']['length']/2
    rear = c['footprint']['center_x'] - c['footprint']['length']/2
    for direction, edge in [(1, front), (-1, rear)]:
        near = [(edge+direction*.19, y) for y in (-.01, 0, .01)]
        far = [(edge+direction*.21, y) for y in (-.01, 0, .01)]
        assert obstacle_in_sweep(near, direction*.5, 0, c)
        assert not obstacle_in_sweep(far, direction*.5, 0, c)


def test_return_pose_error_is_diagnostic_only(tmp_path):
    from types import SimpleNamespace
    from carkit_webui.calibration_worker import Worker
    worker = SimpleNamespace(travel=0., speed=.5, pose=[1., 1., .5], stage='forward',
                             returns=[], run_dir=tmp_path, last_status=time.monotonic())
    worker.status = lambda *args, **kwargs: None
    worker.safety = lambda *args: None
    worker.publish = lambda *args: None
    worker.spin_for = lambda *args: setattr(worker, 'travel', worker.travel+1.)
    worker.stationary = lambda *args: None
    worker.value = lambda *args: SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.))))
    Worker.return_to_origin(worker, 'Forward', -1, 0, 1., (0.,0.,0.), .02)
    assert worker.returns[0]['position_error_m'] > 1.
    assert worker.returns[0]['heading_error_degrees'] > 20.


@pytest.mark.parametrize('scenario', ['delayed', 'missing', 'cancelled'])
def test_mount_wait_spins_without_command_or_arm(monkeypatch, scenario):
    from types import SimpleNamespace
    from carkit_webui import calibration_worker as module
    clock = [10.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    expected = {'gyro_frame', 'accel_frame', 'scan_frame'}
    def lookup(target, source, stamp):
        assert target == 'base_link'
        if clock[0] < 10.2 or (scenario == 'missing' and source == 'gyro_frame'):
            raise module.TransformException('not discovered yet')
        return SimpleNamespace(transform=source)
    def check_cancel():
        if scenario == 'cancelled' and clock[0] >= 10.1:
            raise RuntimeError('cancelled')
    worker = SimpleNamespace(buffer=SimpleNamespace(lookup_transform=lookup), transforms={},
                             fresh=lambda key: True,
                             value=lambda key: SimpleNamespace(header=SimpleNamespace(frame_id=key+'_frame')),
                             check_cancel=check_cancel, status=lambda *args: None,
                             spin_for=lambda duration: clock.__setitem__(0, clock[0]+duration))
    # No publisher or arm method exists on this fake: discovery must be passive.
    if scenario == 'delayed':
        module.Worker.wait_for_mounts(worker, timeout=.5)
        assert set(worker.transforms) == expected
        assert clock[0] >= 10.2
    else:
        with pytest.raises(RuntimeError, match='Timed out.*gyro_frame' if scenario == 'missing' else 'cancelled'):
            module.Worker.wait_for_mounts(worker, timeout=.5)
        assert worker.transforms == {}
        assert clock[0] <= 10.5
