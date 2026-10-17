"""Bounded six-stage steering test. Never launched without an attended-area confirmation.

The worker uses a dedicated command path, not /teleop or motor topics. The C++
control center provides a second watchdog and exclusive control while CALIBRATING.
"""
from collections import deque
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformListener, TransformException
from vesc_msgs.msg import VescStateStamped

try:
    from .calibration import STAGES, COMPACT_SPEED, STRAIGHT_DISTANCE_LIMIT, STRAIGHT_SECONDS_LIMIT, TURN_DISTANCE_LIMIT, TURN_SECONDS_LIMIT, enough_steady_motion, measurement_checkpoint, analyze, atomic_json, read_json
except ImportError:
    from calibration import STAGES, COMPACT_SPEED, STRAIGHT_DISTANCE_LIMIT, STRAIGHT_SECONDS_LIMIT, TURN_DISTANCE_LIMIT, TURN_SECONDS_LIMIT, enough_steady_motion, measurement_checkpoint, analyze, atomic_json, read_json


def rotate(q, value):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = value
    return ((1-2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz,
            2*(x*z-y*w)*vx + 2*(y*z+x*w)*vy + (1-2*(x*x+y*y))*vz)


def obstacle_in_sweep(points, speed, steering, config):
    """Check the footprint and the next 20 cm of travel along the commanded arc."""
    if abs(speed) < 0.01:
        return False
    length = 0.20
    direction = 1 if speed > 0 else -1
    curvature = math.tan(steering) / config['wheelbase']
    footprint = config['footprint']
    hits = 0
    for px, py in points:
        for step in range(0, 9):
            distance = direction * length * step / 8
            angle = curvature * distance
            x = math.sin(angle)/curvature if abs(curvature) > 1e-6 else distance
            y = (1-math.cos(angle))/curvature if abs(curvature) > 1e-6 else 0.0
            dx, dy = px-x, py-y
            local_x = math.cos(angle)*dx + math.sin(angle)*dy - footprint['center_x']
            local_y = -math.sin(angle)*dx + math.cos(angle)*dy - footprint['center_y']
            if abs(local_x) < footprint['length']/2 and abs(local_y) < footprint['width']/2 + 0.20:
                hits += 1
                break
        if hits >= 3:
            return True
    return False


class Worker(Node):
    def __init__(self, run_dir):
        super().__init__('carkit_steering_calibration')
        self.run_dir = Path(run_dir)
        self.request = read_json(self.run_dir / 'request.json')
        self.config = self.request['config']
        self.speed = self.request['speed']
        if self.speed != COMPACT_SPEED:
            raise ValueError('Compact calibration requires 1.0 m/s')
        self.latest = {}
        self.started = time.monotonic()
        self.cancelled = False
        self.armed = False
        self.stage = 'preflight'
        self.stage_index = -1
        self.last_status = 0.0
        checkpoint = measurement_checkpoint(read_json(self.run_dir / 'checkpoint.json', {}), self.run_dir.name)
        self.completed = list(checkpoint.get('completed', []))
        self.samples = list(checkpoint.get('samples', []))
        self.legacy_without_bias = bool(checkpoint.get('legacy_without_bias'))
        self.pose = [0.0, 0.0, 0.0]
        self.travel = 0.0
        self.gyro_bias = 0.0
        self.track_pose = False
        self.pose_time = time.monotonic()
        self.returns = list(checkpoint.get('returns', []))
        self.transforms = {}
        self.buffer = Buffer(cache_time=Duration(seconds=5))
        self.listener = TransformListener(self.buffer, self)
        self.command_pub = self.create_publisher(AckermannDriveStamped, '/calibration/command', 10)
        self.active_pub = self.create_publisher(Bool, '/calibration/active', 10)
        self.estop_pub = self.create_publisher(Bool, '/control_center/emergency_stop', 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.store('odom', m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/camera/camera/gyro/sample', lambda m: self.store('gyro', m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/camera/camera/accel/sample', lambda m: self.store('accel', m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', lambda m: self.store('scan', m), qos_profile_sensor_data)
        self.create_subscription(VescStateStamped, '/sensors/core', lambda m: self.store('vesc', m), qos_profile_sensor_data)
        self.create_subscription(Float64, '/commands/servo/position', lambda m: self.store('servo', m), 10)
        self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', lambda m: self.store('command', m), 10)
        self.create_subscription(String, '/control_center/main_state', lambda m: self.store('mode', m), 10)
        self.log = (self.run_dir / 'samples.jsonl').open('w', encoding='utf-8')
        signal.signal(signal.SIGINT, lambda *_: setattr(self, 'cancelled', True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, 'cancelled', True))

    def store(self, key, message):
        # Receipt time is insufficient: a queue of stale frames cannot keep a
        # motion test alive. Bound source age where a stamped message exists.
        if hasattr(message, 'header'):
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if stamp.nanoseconds <= 0 or not -0.05 <= age <= 0.4:
                return
        self.latest[key] = (time.monotonic(), message)

    def fresh(self, key):
        return key in self.latest and time.monotonic() - self.latest[key][0] < (1.2 if key == 'mode' else 0.4)

    def value(self, key):
        return self.latest[key][1]

    def check_cancel(self):
        if self.cancelled or (self.run_dir / 'cancel').exists():
            reason = read_json(self.run_dir / 'cancel-reason.json', {}).get('reason', 'Calibration cancelled')
            raise RuntimeError(f'{reason}; vehicle stopped')

    def status(self, phase, message, **extra):
        atomic_json(self.run_dir / 'status.json', {'phase': phase, 'message': message,
                    'stage_index': self.stage_index, 'stage': self.stage, 'completed_stages': self.completed,
                    'elapsed_seconds': round(time.monotonic()-self.started, 1), **extra})

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, deadline-time.monotonic())))
            now = time.monotonic()
            dt = now-self.pose_time
            self.pose_time = now
            if self.track_pose and self.fresh('odom') and self.fresh('gyro'):
                speed = self.value('odom').twist.twist.linear.x
                yaw = self.yaw_rate()-self.gyro_bias
                if not math.isfinite(speed) or not math.isfinite(yaw) or dt > 0.4:
                    raise RuntimeError('Return-pose feedback is invalid; vehicle stopped')
                heading = self.pose[2] + yaw*dt/2
                self.pose[0] += speed*dt*math.cos(heading)
                self.pose[1] += speed*dt*math.sin(heading)
                self.pose[2] += yaw*dt
                self.travel += abs(speed)*dt

    def publish(self, speed, steering=0.0):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.drive.speed = float(speed)
        command.drive.steering_angle = float(steering)
        self.command_pub.publish(command)
        self.active_pub.publish(Bool(data=self.armed))

    def transformed_vector(self, key, vector):
        message = self.value(key)
        transform = self.transforms[message.header.frame_id]
        return rotate(transform.rotation, (vector.x, vector.y, vector.z))

    def yaw_rate(self):
        return self.transformed_vector('gyro', self.value('gyro').angular_velocity)[2]

    def safety(self, speed=0.0, steering=0.0):
        self.check_cancel()
        for key in ('odom', 'gyro', 'accel', 'scan', 'vesc', 'servo', 'mode', 'command'):
            if not self.fresh(key):
                raise RuntimeError(f'{key} feedback is missing or stale; vehicle stopped')
        if self.armed and self.value('mode').data != 'CALIBRATING':
            raise RuntimeError('Calibration control was interrupted; vehicle stopped')
        state = self.value('vesc').state
        measured = self.value('odom').twist.twist.linear.x
        if state.fault_code != 0 or state.voltage_input < 10.5:
            raise RuntimeError('Motor-controller fault or low battery; vehicle stopped')
        if not math.isfinite(measured) or abs(measured) > self.speed + 0.35:
            raise RuntimeError('Measured speed exceeded the calibration limit; vehicle stopped')
        if abs(state.speed) > abs(self.config['driver']['speed_to_erpm_gain']) * (self.speed + 0.4):
            raise RuntimeError('Motor RPM exceeded the calibration limit; vehicle stopped')
        if not math.isfinite(self.yaw_rate()) or abs(self.yaw_rate()) > 1.5:
            raise RuntimeError('Unexpected rotation; vehicle stopped')
        if abs(speed) > 0.01:
            scan = self.value('scan')
            transform = self.transforms[scan.header.frame_id]
            points = []
            for index, distance in enumerate(scan.ranges):
                if not math.isfinite(distance) or not scan.range_min < distance < min(scan.range_max, 3.0):
                    continue
                angle = scan.angle_min + index * scan.angle_increment
                x, y, _ = rotate(transform.rotation, (distance*math.cos(angle), distance*math.sin(angle), 0.0))
                points.append((x+transform.translation.x, y+transform.translation.y))
            if obstacle_in_sweep(points, speed, steering, self.config):
                raise RuntimeError('LiDAR detected an obstacle in the test path; vehicle stopped')

    def wait_for_mounts(self, timeout=15.0):
        """Allow late-joining TF discovery without arming or publishing commands.

        Sensor streams may arrive before the transient-local /tf_static history.
        Keep spinning this executor: a blocking TF lookup would prevent its own
        listener callbacks from filling the buffer.
        """
        deadline = time.monotonic() + timeout
        next_status = 0.0
        missing = []
        while time.monotonic() < deadline:
            self.check_cancel()
            missing = []
            resolved = {}
            for key in ('gyro', 'accel', 'scan'):
                if not self.fresh(key):
                    missing.append(f'{key} sensor feedback')
                    continue
                frame = self.value(key).header.frame_id
                if not frame:
                    missing.append(f'{key} sensor frame ID')
                    continue
                try:
                    resolved[frame] = self.buffer.lookup_transform('base_link', frame, Time()).transform
                except TransformException:
                    missing.append(f'{frame} to base_link')
            if not missing:
                self.transforms.update(resolved)
                return
            if time.monotonic() >= next_status:
                self.status('preflight', 'Waiting for mounting transforms: ' + ', '.join(missing))
                next_status = time.monotonic() + 1.0
            self.spin_for(min(.05, max(0.0, deadline-time.monotonic())))
        raise RuntimeError('Timed out waiting for mounting transforms: ' + ', '.join(missing)
                           + '; check camera/LiDAR TF publishers and retry')

    def preflight(self):
        deadline = time.monotonic() + 25
        required = ('odom', 'gyro', 'accel', 'scan', 'vesc', 'servo', 'mode', 'command')
        while time.monotonic() < deadline:
            self.check_cancel()
            self.spin_for(0.05)
            if all(self.fresh(key) for key in required):
                break
        if not all(self.fresh(key) for key in required):
            missing = ', '.join(key for key in required if not self.fresh(key))
            raise RuntimeError(f'Preflight needs live {missing}; enable the sensors and updated control center')
        self.wait_for_mounts()
        if self.value('mode').data != 'HUMAN_CONTROL':
            raise RuntimeError('Select Human mode and reset E-stop before starting calibration')
        if abs(self.value('odom').twist.twist.linear.x) > 0.02 or abs(self.value('vesc').state.speed) > 30:
            raise RuntimeError('The vehicle must be stationary before calibration')
        center = self.config['driver']['steering_angle_to_servo_offset']
        if abs(self.value('servo').data-center) > 0.003:
            raise RuntimeError('Live servo center differs from the saved profile; release controls and relaunch before testing')
        acceleration = self.transformed_vector('accel', self.value('accel').linear_acceleration)
        magnitude = math.sqrt(sum(v*v for v in acceleration))
        if not 7 < magnitude < 12 or abs(acceleration[2]) < 0.8*magnitude:
            raise RuntimeError('IMU gravity direction does not match the vehicle frame; check camera mounting')
        self.safety()
        # Publish stationary commands before requesting calibration ownership.
        until = time.monotonic()+0.4
        while time.monotonic() < until:
            self.publish(0)
            self.spin_for(0.05)
        self.armed = True
        until = time.monotonic()+2
        while time.monotonic() < until:
            self.check_cancel()
            self.publish(0)
            self.spin_for(0.05)
            if self.fresh('mode') and self.value('mode').data == 'CALIBRATING':
                return
        raise RuntimeError('Control center did not grant calibration control; install the updated arbiter and relaunch')

    def stationary(self, duration, steering=0.0, phase='stopping'):
        self.status(phase, 'Holding stationary' if phase == 'baseline' else 'Stopping between tests')
        values = []
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            self.safety()
            self.publish(0, steering)
            self.spin_for(0.05)
            values.append(self.yaw_rate())
        if abs(self.value('odom').twist.twist.linear.x) > 0.025 or abs(self.value('vesc').state.speed) > 40:
            raise RuntimeError('Motor did not settle to a stop')
        return values

    def run_stage(self, key, label, direction, scale):
        self.stage = key
        steering = scale * self.config['max_steering_angle']
        self.stationary(0.8, steering)
        origin = tuple(self.pose)
        start_travel = self.travel
        expected_servo = self.config['driver']['steering_angle_to_servo_offset'] + steering*self.config['driver']['steering_angle_to_servo_gain']
        if abs(self.value('servo').data-expected_servo) > 0.004:
            raise RuntimeError(f'{label}: live steering conversion differs from the saved profile')
        began = time.monotonic()
        last = began
        distance = 0.0
        history = deque()
        straight = key in ('forward', 'reverse')
        distance_limit = STRAIGHT_DISTANCE_LIMIT if straight else TURN_DISTANCE_LIMIT
        seconds_limit = STRAIGHT_SECONDS_LIMIT if straight else TURN_SECONDS_LIMIT
        self.status('running', label, distance_m=0, speed_mps=0)
        while True:
            now = time.monotonic()
            elapsed = now-began
            measured = self.value('odom').twist.twist.linear.x
            distance += abs(measured)*(now-last)
            last = now
            if elapsed >= seconds_limit or distance >= distance_limit:
                break
            if enough_steady_motion(self.samples, key):
                break
            command_speed = direction*min(self.speed, elapsed*0.35)
            self.safety(command_speed, steering)
            if elapsed > 0.5:
                actual = self.value('command').drive
                if abs(actual.steering_angle-steering) > 0.01 or abs(actual.speed-command_speed) > 0.10:
                    raise RuntimeError('Another command or runtime limit changed the calibration output')
            self.publish(command_speed, steering)
            self.spin_for(0.05)
            history.append((elapsed, measured))
            while history and elapsed-history[0][0] > 0.3:
                history.popleft()
            acceleration = (measured-history[0][1])/max(0.05, elapsed-history[0][0])
            sample = {'stage': key, 'segment': f'{self.run_dir.name}:{key}', 't': time.monotonic()-self.started,
                      'speed': measured, 'yaw_rate': self.yaw_rate(), 'gyro_bias': self.gyro_bias,
                      'servo': self.value('servo').data, 'command_speed': command_speed,
                      'command_steering': steering,
                      'settled': elapsed > max(1.0, self.speed/0.35 + 0.2) and abs(acceleration) < 0.5}
            self.samples.append(sample)
            self.log.write(json.dumps(sample, allow_nan=False)+'\n')
            if now-self.last_status > 0.25:
                self.log.flush()
                self.last_status = now
                self.status('running', label, distance_m=round(distance, 2), speed_mps=round(measured, 2))
        before_stop = self.travel
        self.stationary(1.8, steering)
        stopping_travel = self.travel-before_stop
        outbound = self.travel-start_travel
        self.return_to_origin(label, -direction, steering, outbound, origin, stopping_travel)
        self.status('stopping', f'{label} and return finished', distance_m=round(outbound, 2))

    def return_to_origin(self, label, direction, steering, target, origin, stopping_travel):
        """Retrace the measured arc; record gyro/odometry dead-reckoned endpoint.

        This is not external position localization. Wheel slip or an inaccurate
        speed scale can affect physical accuracy; endpoint errors are diagnostic only.
        """
        straight = self.stage in ('forward', 'reverse')
        distance_limit = STRAIGHT_DISTANCE_LIMIT if straight else TURN_DISTANCE_LIMIT
        if not 0.05 <= target <= distance_limit + 0.25:
            raise RuntimeError('Outbound travel is invalid for a bounded return; vehicle stopped')
        began = time.monotonic()
        start_travel = self.travel
        self.status('returning', f'{label}: returning to starting pose', distance_m=0, speed_mps=0)
        # Measured outbound coasting plus half a 50 ms command cycle compensates
        # the return stop without requiring an unsupported crawling speed.
        stop_allowance = min(target*.2, max(0.0, stopping_travel) + self.speed*.025)
        while self.travel-start_travel < target-stop_allowance:
            elapsed = time.monotonic()-began
            if elapsed > (STRAIGHT_SECONDS_LIMIT + 8.0 if straight else TURN_SECONDS_LIMIT + 8.0):
                raise RuntimeError('Return timed out before reaching the starting pose; vehicle stopped')
            speed = direction*min(self.speed, elapsed*0.35)
            self.safety(speed, steering)
            self.publish(speed, steering)
            self.spin_for(0.05)
            if time.monotonic()-self.last_status > .25:
                self.last_status = time.monotonic()
                self.status('returning', f'{label}: returning to starting pose',
                            distance_m=round(self.travel-start_travel, 2),
                            speed_mps=round(self.value('odom').twist.twist.linear.x, 2))
        self.stationary(1.8, steering)
        position_error = math.hypot(self.pose[0]-origin[0], self.pose[1]-origin[1])
        heading_error = abs(math.atan2(math.sin(self.pose[2]-origin[2]), math.cos(self.pose[2]-origin[2])))
        total_position_error = math.hypot(self.pose[0], self.pose[1])
        total_heading_error = abs(math.atan2(math.sin(self.pose[2]), math.cos(self.pose[2])))
        record = {'stage': self.stage, 'outbound_m': target,
                  'return_m': self.travel-start_travel, 'stop_allowance_m': stop_allowance,
                  'position_error_m': position_error,
                  'heading_error_degrees': math.degrees(heading_error),
                  'from_initial_position_m': total_position_error,
                  'from_initial_heading_degrees': math.degrees(total_heading_error)}
        self.returns.append(record)
        atomic_json(self.run_dir / 'returns.json', self.returns)
        self.stationary(.5)

    def save_checkpoint(self):
        self.log.flush()
        atomic_json(self.run_dir / 'checkpoint.json', {
            'fingerprint': self.request['fingerprint'], 'speed': self.speed,
            'completed': self.completed,
            'samples': self.samples,
            'returns': self.returns,
            'legacy_without_bias': self.legacy_without_bias})

    def finish_stop(self):
        if not self.armed:
            return
        self.armed = False
        deadline = time.monotonic()+1.2
        while time.monotonic() < deadline:
            self.publish(0)
            self.estop_pub.publish(Bool(data=True))
            self.spin_for(0.05)

    def run(self):
        result = None
        error = None
        awaiting_confirmation = False
        try:
            self.preflight()
            self.stage = 'baseline'
            values = self.stationary(2.0, phase='baseline')
            if statistics.pstdev(values) > 0.01:
                raise RuntimeError('Vehicle or IMU is moving during the stationary baseline')
            bias = statistics.median(values)
            if not math.isfinite(bias) or abs(bias) > .05:
                raise RuntimeError('Gyro stationary bias is too large; check mounting and sensor data')
            self.gyro_bias = bias
            atomic_json(self.run_dir / 'baseline.json', {'gyro_bias_rad_s': bias})
            self.pose_time = time.monotonic()
            self.track_pose = True
            pending = [(index, stage) for index, stage in enumerate(STAGES) if stage[0] not in self.completed]
            if pending and 'stage_id' in self.request and self.request['stage_id'] != pending[0][1][0]:
                raise RuntimeError('Confirmed maneuver no longer matches saved progress; confirm again')
            selected = pending if self.request.get('mode', 'single') == 'all' else pending[:1]
            for index, stage in selected:
                self.stage_index = index
                self.run_stage(*stage)
                ready = enough_steady_motion(self.samples, stage[0])
                if ready:
                    self.completed.append(stage[0])
                # Save a finished pass even if it needs more samples on a later,
                # separately confirmed attempt. An interrupted pass is not saved.
                self.save_checkpoint()
                if not ready:
                    raise RuntimeError(f'{stage[1]}: more steady data is needed after the bounded pass; '
                                       'usable samples saved. Confirm this maneuver again to collect more')
            if len(self.completed) == len(STAGES):
                result = analyze(self.samples, self.config, bias)
            else:
                awaiting_confirmation = True
        except Exception as exc:
            error = str(exc)
        finally:
            self.finish_stop()
            self.log.close()
        if (self.run_dir / 'cancel').exists():
            error = read_json(self.run_dir / 'cancel-reason.json', {}).get('reason', 'Calibration cancelled') + '; vehicle stopped'
            result = None
        if result is not None:
            result['returns'] = self.returns
            result['return_pose_source'] = 'RealSense gyro and VESC odometry dead reckoning; not external position verification'
            result['fingerprint'] = self.request['fingerprint']
            result['test_speed_mps'] = self.speed
            result['recorded_at'] = time.time()
            atomic_json(self.run_dir / 'result.json', result)
            self.status('complete', 'Steering passed all six checks' if result['good'] else 'Calibration recommendation ready')
        elif awaiting_confirmation and not error:
            next_label = next(label for key, label, *_ in STAGES if key not in self.completed)
            self.status('awaiting_confirmation', f'Maneuver finished and vehicle stopped. Reset E-stop, '
                        f'confirm the clear area, then start {next_label}.')
        else:
            self.status('failed', error or 'Calibration did not finish')


def main():
    rclpy.init()
    worker = Worker(sys.argv[1])
    try:
        worker.run()
    finally:
        worker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
