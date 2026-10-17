"""Steering calibration analysis and durable, on-vehicle test sessions.

This module has no ROS imports. The separate worker owns ROS subscriptions and
motion; the control center arbitrates its commands with a short watchdog.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import statistics
import subprocess
import threading
import time

STAGES = (
    ('forward', 'Forward only', 1, 0.0),
    ('reverse', 'Backward only', -1, 0.0),
    ('left_full', 'Maximum left turn', 1, 1.0),
    ('left_half', 'Half maximum left turn', 1, 0.5),
    ('right_full', 'Maximum right turn', 1, -1.0),
    ('right_half', 'Half maximum right turn', 1, -0.5),
)
STRAIGHT_TOLERANCE_DEG = 0.5
TURN_RMS_TOLERANCE_DEG = 1.5
MODEL_RMS_LIMIT_DEG = 1.5
MIN_MEASURED_SPEED = 0.15


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return default


def fingerprint(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()


def load_checkpoint(run_dir, signature, speed):
    """Only reuse completed motion with identical hardware and requested speed.

    Motion completion alone is insufficient. Retain usable samples and only skip
    maneuvers with enough steady data and a saved stationary gyro bias.
    """
    run_dir = Path(run_dir)
    request = read_json(run_dir / 'request.json', {})
    if request.get('fingerprint') != signature or request.get('speed') != speed:
        return {'completed': [], 'samples': [], 'returns': []}
    checkpoint = read_json(run_dir / 'checkpoint.json')
    if checkpoint is not None:
        if checkpoint.get('fingerprint') == signature and checkpoint.get('speed') == speed:
            return measurement_checkpoint(checkpoint, run_dir.name)
        return {'completed': [], 'samples': [], 'returns': []}
    returns = read_json(run_dir / 'returns.json', [])
    completed = [key for key, *_ in STAGES if any(r.get('stage') == key for r in returns)]
    samples = []
    try:
        for line in (run_dir / 'samples.jsonl').read_text().splitlines():
            try:
                sample = json.loads(line)
                if sample.get('stage') in completed:
                    samples.append({**sample, 'gyro_bias': sample.get('gyro_bias')})
            except ValueError:
                continue  # An interrupted final log line cannot complete a stage.
    except OSError:
        completed = []
        returns = []
    return measurement_checkpoint({'fingerprint': signature, 'speed': speed, 'completed': completed,
            'samples': samples, 'returns': returns, 'legacy_without_bias': bool(completed)}, run_dir.name)


def steady_samples(samples, key):
    direction = next(direction for stage, _, direction, _ in STAGES if stage == key)
    return [s for s in samples if s['stage'] == key and s.get('settled', False)
            and direction * s['speed'] >= MIN_MEASURED_SPEED
            and all(math.isfinite(s[n]) for n in ('speed', 'yaw_rate', 'servo', 't'))]


def enough_steady_motion(samples, key):
    values = steady_samples(samples, key)
    # Count measured time within each pass, never a pause or the gap between sessions.
    previous = {}
    duration = 0.0
    for sample in values:
        segment = sample.get('segment', 'legacy')
        if segment in previous:
            delta = sample['t'] - previous[segment]
            if 0 < delta <= .5:
                duration += delta
        previous[segment] = sample['t']
    return len(values) >= 20 and duration >= 1.0


def measurement_checkpoint(checkpoint, session):
    recorded = list(dict.fromkeys(checkpoint.get('recorded_stages', []) + checkpoint.get('completed', [])
                                  + [r['stage'] for r in checkpoint.get('returns', [])]))
    rows = checkpoint.get('samples', [])
    retained = [{**row, 'segment': row.get('segment', f'{session}:{row["stage"]}')}
                for row in rows if isinstance(row.get('gyro_bias'), (int, float))
                and math.isfinite(row['gyro_bias']) and abs(row['gyro_bias']) <= .05]
    completed = [key for key, *_ in STAGES if key in recorded and enough_steady_motion(retained, key)]
    return {**checkpoint, 'samples': retained, 'completed': completed, 'recorded_stages': recorded,
            'retry_stages': [key for key in recorded if key not in completed],
            'legacy_without_bias': bool(checkpoint.get('legacy_without_bias') or len(retained) != len(rows))}



def _line_fit(points):
    """Fit servo = offset + gain * measured steering; stages get equal weight."""
    x_mean = statistics.mean(x for x, _ in points)
    y_mean = statistics.mean(y for _, y in points)
    denominator = sum((x - x_mean) ** 2 for x, _ in points)
    if denominator < math.radians(5) ** 2:
        raise ValueError('Turning measurements do not span enough steering angle')
    gain = sum((x - x_mean) * (y - y_mean) for x, y in points) / denominator
    return y_mean - gain * x_mean, gain


def analyze(samples, config, gyro_bias):
    """Estimate a bounded steering calibration, never a speed or wheelbase scale.

Samples are time-aligned in the worker. Reject low-speed and acceleration samples
rather than dividing gyro noise by near-zero speed. Curvature fitting weights each
sample by speed squared; a single aggregate point per stage avoids long-stage bias.
"""
    if not math.isfinite(gyro_bias) or abs(gyro_bias) > 0.05:
        raise ValueError('Gyro stationary bias is too large; check mounting and sensor data')
    wheelbase = config['wheelbase']
    driver = config['driver']
    old_gain = driver['steering_angle_to_servo_gain']
    old_center = driver['steering_angle_to_servo_offset']
    stage_results = []
    points = []
    for key, label, direction, scale in STAGES:
        values = steady_samples(samples, key)
        if not enough_steady_motion(samples, key):
            raise ValueError(f'{label}: insufficient steady motion; try a longer clear path or check the drivetrain')
        if any(s.get('gyro_bias', gyro_bias) is None for s in values):
            raise ValueError(f'{label}: older run did not save its IMU baseline; start a new sequence for a verified fit')
        if any(not math.isfinite(s.get('gyro_bias', gyro_bias)) or abs(s.get('gyro_bias', gyro_bias)) > .05 for s in values):
            raise ValueError(f'{label}: invalid saved IMU baseline')
        curvature = sum(s['speed'] * (s['yaw_rate'] - s.get('gyro_bias', gyro_bias)) for s in values) / sum(s['speed'] ** 2 for s in values)
        measured = math.atan(wheelbase * curvature)
        servo = statistics.median(s['servo'] for s in values)
        commanded = config['max_steering_angle'] * scale
        scatter = statistics.median(abs(math.atan(wheelbase * (s['yaw_rate'] - s.get('gyro_bias', gyro_bias)) / s['speed']) - measured) for s in values)
        if scatter > math.radians(3):
            raise ValueError(f'{label}: turning response is inconsistent; inspect tire slip, surface and sensor mounting')
        stage_results.append({'id': key, 'label': label, 'samples': len(values),
                              'measured_speed_mps': statistics.median(abs(s['speed']) for s in values),
                              'commanded_degrees': math.degrees(commanded),
                              'measured_degrees': math.degrees(measured),
                              'error_degrees': math.degrees(measured - commanded),
                              'scatter_degrees': math.degrees(scatter), 'servo': servo})
        points.append((measured, servo))
    by_id = {r['id']: r for r in stage_results}
    left = by_id['left_full']['measured_degrees']
    right = by_id['right_full']['measured_degrees']
    if left - right < 10 or not (right < by_id['right_half']['measured_degrees'] < by_id['left_half']['measured_degrees'] < left):
        raise ValueError('Steering response is reversed or not monotonic; check linkage and sensor axes before applying a fit')
    if abs(by_id['forward']['measured_degrees'] - by_id['reverse']['measured_degrees']) > 1.0:
        raise ValueError('Forward and reverse centers disagree by over 1 degree; check play, alignment and surface slope')
    center, gain = _line_fit(points)
    if gain * old_gain <= 0 or not 0.5 <= abs(gain / old_gain) <= 2.0:
        raise ValueError('Estimated steering gain is outside the supported calibration range')
    model_rms = math.degrees(math.sqrt(statistics.mean((x - (y - center) / gain) ** 2 for x, y in points)))
    if model_rms > MODEL_RMS_LIMIT_DEG:
        raise ValueError('A single steering gain cannot explain the measurements; inspect mechanical asymmetry')
    lower, upper = driver['servo_min'], driver['servo_max']
    if not lower + 0.02 < center < upper - 0.02:
        raise ValueError('Estimated center is too close to a servo travel limit')
    # Do not extend the tested steering envelope or the established servo travel.
    steering_limit = min(config['max_steering_angle'],
                         (center - lower - 0.005) / abs(gain),
                         (upper - center - 0.005) / abs(gain))
    if steering_limit < 0.08:
        raise ValueError('Calibration leaves too little usable steering travel')
    straight_error = max(abs(by_id[k]['measured_degrees']) for k in ('forward', 'reverse'))
    turn_rms = math.sqrt(statistics.mean(r['error_degrees'] ** 2 for r in stage_results[2:]))
    good = straight_error <= STRAIGHT_TOLERANCE_DEG and turn_rms <= TURN_RMS_TOLERANCE_DEG
    return {'good': good, 'stages': stage_results,
            'straight_error_degrees': straight_error, 'turn_rms_error_degrees': turn_rms,
            'model_rms_degrees': model_rms, 'gyro_bias_rad_s': gyro_bias,
            'thresholds': {'straight_degrees': STRAIGHT_TOLERANCE_DEG, 'turn_rms_degrees': TURN_RMS_TOLERANCE_DEG},
            'current': {'steering_angle_to_servo_offset': old_center,
                        'steering_angle_to_servo_gain': old_gain,
                        'max_steering_angle': config['max_steering_angle']},
            'recommendation': {'steering_angle_to_servo_offset': round(center, 6),
                               'steering_angle_to_servo_gain': round(gain, 6),
                               'max_steering_angle': math.floor(steering_limit * 1000) / 1000},
            'scope': 'Steering center and gain at the tested speed. Wheelbase and motor-speed scale are unchanged; they require independent distance measurements.'}


COMPACT_SPEED = 1.0
STRAIGHT_DISTANCE_LIMIT = 6.0
STRAIGHT_SECONDS_LIMIT = 12.0
TURN_DISTANCE_LIMIT = 3.0
TURN_SECONDS_LIMIT = 12.0


class CalibrationService:
    def __init__(self, manager):
        self.manager = manager
        self.lock = threading.RLock()
        self.process = None
        self.run_dir = None
        self.owner_token = None
        self.log_handle = None

    @property
    def root(self):
        return self.manager.workspace / '.carkit' / 'calibration'

    def method(self):
        selection = self.manager.current_config or read_json(self.manager.workspace / '.carkit/config.json', {})
        chassis = selection.get('chassis', '')
        supported = chassis in ('vesc_rplidar', 'f1tenth')
        return {'chassis': chassis, 'method': 'vesc_realsense_imu' if supported else None,
                'method_label': 'F1TENTH / VESC · RealSense IMU' if supported else 'OSRacer calibration' if chassis == 'osracer' else 'Chassis calibration',
                'supported': supported}

    def profile(self):
        import yaml
        method = self.method()
        chassis = method['chassis']
        if not method['supported']:
            raise ValueError('OSRacer requires no calibration in CARKit' if chassis == 'osracer' else 'Select a supported chassis to configure calibration')
        path = self.manager.workspace / 'carkit/core/carkit_bringup/config/hardware' / f'{chassis}.yaml'
        config = yaml.safe_load(path.read_text(encoding='utf-8'))
        return path, config

    def busy(self):
        return self.process is not None and self.process.poll() is None

    def snapshot(self):
        with self.lock:
            try:
                _, config = self.profile()
                signature = fingerprint(config)
            except (ValueError, OSError) as error:
                return {**self.method(), 'status': 'not_required' if self.method()['chassis'] == 'osracer' else 'requires_calibration', 'supported': False, 'reason': str(error), 'running': False, 'can_apply': False, 'result': None, 'stages': []}
            pointer = read_json(self.root / 'latest.json', {})
            run_dir = self.root / pointer.get('run_id', '_none')
            record = read_json(run_dir / 'status.json', {})
            result = read_json(run_dir / 'result.json')
            running = self.busy()
            recorded_speed = read_json(run_dir / 'request.json', {}).get('speed', (result or {}).get('test_speed_mps'))
            same = pointer.get('fingerprint') == signature and recorded_speed == COMPACT_SPEED
            good = bool(same and record.get('phase') == 'complete' and result and result.get('good'))
            reason = 'Current steering settings passed all six checks' if good else 'Current steering settings need a complete verification run'
            if record.get('phase') == 'failed':
                reason = record.get('message', 'Calibration did not complete')
                if reason.startswith(('Return position or heading is outside tolerance', 'Accumulated return drift')):
                    reason = 'The previous run stopped on a return-pose tolerance check; that check is now diagnostic only'
            if record.get('phase') in ('preflight', 'baseline', 'running', 'returning', 'stopping') and not running:
                reason = 'The calibration worker stopped unexpectedly; reset E-stop and run again'
            checkpoint = load_checkpoint(run_dir, signature, COMPACT_SPEED) if same else {}
            completed = checkpoint.get('completed', [])
            if same and record.get('phase') == 'complete':
                completed = [k for k, *_ in STAGES]
            next_stage = next(({'id': k, 'label': label} for k, label, *_ in STAGES if k not in completed), None)
            if record.get('phase') == 'awaiting_confirmation' and same:
                reason = record.get('message', 'Stopped; confirm the next maneuver to continue')
            return {**self.method(), 'completed_stages': completed, 'next_stage': next_stage,
                    'retry_stages': checkpoint.get('retry_stages', []),
                    'can_resume': bool(checkpoint.get('samples') and record.get('phase') != 'complete'),
                    'legacy_measurements': bool(checkpoint.get('legacy_without_bias')),
                    'status': 'running' if running else 'good' if good else 'requires_calibration',
                    'supported': True, 'running': running, 'reason': reason,
                    'run_id': pointer.get('run_id'), 'same_configuration': same,
                    'progress': record, 'result': result,
                    'can_apply': bool(not running and same and record.get('phase') == 'complete' and result and not result.get('good')),
                    'hardware': {'chassis': config['chassis'], 'max_steering_degrees': math.degrees(config['max_steering_angle']),
                                 'center': config['driver']['steering_angle_to_servo_offset'],
                                 'gain': config['driver']['steering_angle_to_servo_gain']},
                    'stages': [{'id': k, 'label': label} for k, label, _, _ in STAGES]}

    def start(self, request):
        with self.lock:
            if self.busy():
                raise RuntimeError('A calibration is already running')
            if request.get('clear_area_confirmed') is not True:
                raise ValueError('Confirm the clear, attended test area before starting')
            if not self.manager.process or self.manager.process.poll() is not None:
                raise RuntimeError('Launch the vehicle stack before calibration')
            if self.manager.job and self.manager.job.poll() is None:
                raise RuntimeError('Wait for the build job to finish')
            launch = self.manager.current_config or {}
            components = launch.get('components', {})
            if not all(components.get(k, True) for k in ('chassis', 'sensors')) or not launch.get('camera', True) or not launch.get('lidar', True):
                raise ValueError('Calibration requires the chassis, camera IMU and LiDAR to be enabled')
            path, config = self.profile()
            speed = float(request.get('speed', COMPACT_SPEED))
            if not math.isfinite(speed) or speed != COMPACT_SPEED or speed > min(config['max_speed'], config['max_reverse_speed']):
                raise ValueError('Compact calibration requires 1.0 m/s within both chassis limits')
            mode = request.get('mode', 'single')
            if mode not in ('single', 'all'):
                raise ValueError('Choose single or all calibration checks')
            previous = read_json(self.root / 'latest.json', {})
            previous_dir = self.root / previous.get('run_id', '_none')
            checkpoint = load_checkpoint(previous_dir, fingerprint(config), speed) if request.get('restart_sequence') is not True else {}
            if request.get('restart_sequence') is not True and previous.get('fingerprint') == fingerprint(config) and read_json(previous_dir / 'request.json', {}).get('speed') == speed and read_json(previous_dir / 'status.json', {}).get('phase') == 'complete':
                raise ValueError('This sequence is complete; explicitly start a new sequence to repeat it')
            next_stage = next((k for k, *_ in STAGES if k not in checkpoint.get('completed', [])), None)
            if 'stage_id' in request and request['stage_id'] != next_stage:
                raise ValueError('The next maneuver changed; refresh and confirm it again')
            run_id = time.strftime('%Y%m%dT%H%M%S', time.gmtime()) + '-' + secrets.token_hex(3)
            self.run_dir = self.root / run_id
            self.run_dir.mkdir(parents=True)
            self.owner_token = secrets.token_urlsafe(32)
            atomic_json(self.run_dir / 'request.json', {'config': config, 'speed': speed, 'fingerprint': fingerprint(config), 'stage_id': next_stage, 'mode': mode})
            atomic_json(self.run_dir / 'checkpoint.json', {
                'fingerprint': fingerprint(config), 'speed': speed, 'completed': [],
                'samples': [], 'returns': [], **checkpoint, 'resumed_from': previous.get('run_id')})
            (self.run_dir / 'hardware-before.yaml').write_text(path.read_text(encoding='utf-8'), encoding='utf-8')
            atomic_json(self.run_dir / 'lease.json', {'deadline': time.monotonic() + 8})
            atomic_json(self.run_dir / 'status.json', {'phase': 'preflight', 'message': 'Checking sensors and control authority', 'stage_index': -1})
            atomic_json(self.root / 'latest.json', {'run_id': run_id, 'fingerprint': fingerprint(config)})
            if self.log_handle:
                self.log_handle.close()
            self.log_handle = (self.run_dir / 'worker.log').open('w', encoding='utf-8')
            worker = Path(__file__).with_name('calibration_worker.py')
            self.process = subprocess.Popen(['bash', '-c',
                'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && source install/setup.bash && exec python3 "$1" "$2"',
                'carkit-calibration', str(worker), str(self.run_dir)], cwd=self.manager.workspace,
                stdout=self.log_handle, stderr=subprocess.STDOUT, start_new_session=True)
            return {'run_id': run_id, 'owner_token': self.owner_token, 'stage_id': next_stage, 'mode': mode}

    def heartbeat(self, request):
        with self.lock:
            if not self.busy() or not self.run_dir or self.run_dir.name != request.get('run_id'):
                raise RuntimeError('Calibration is no longer running')
            if not secrets.compare_digest(self.owner_token or '', str(request.get('owner_token', ''))):
                raise ValueError('Only the browser that started calibration can keep it running')
            atomic_json(self.run_dir / 'lease.json', {'deadline': time.monotonic() + 1.5})

    def cancel(self, reason="Calibration stopped by the server"):
        with self.lock:
            if self.busy() and self.run_dir:
                atomic_json(self.run_dir / 'cancel-reason.json', {'reason': str(reason)[:200]})
                (self.run_dir / 'cancel').touch()
                # The worker handles SIGINT through the same finally-stop path.
                self.process.send_signal(signal.SIGINT)

    def wait_stopped(self):
        self.cancel()
        process = self.process
        if process and process.poll() is None:
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()  # Arbiter's independent 300 ms watchdog latches E-stop.
                process.wait(timeout=2)

    def apply(self, request):
        with self.lock:
            snapshot = self.snapshot()
            if request.get('restart_confirmed') is not True:
                raise ValueError('Applying calibration requires a stack restart')
            if not snapshot['can_apply'] or request.get('run_id') != snapshot['run_id']:
                raise ValueError('No applicable recommendation for the current hardware configuration')
            path, config = self.profile()
            recommendation = snapshot['result']['recommendation']
            source = path.read_text(encoding='utf-8')
            for key, value in recommendation.items():
                source, count = re.subn(r'(?m)^(\s*' + re.escape(key) + r':)\s*[^\n]+',
                                        lambda m: f'{m.group(1)} {float(value)}', source)
                if count != 1:
                    raise ValueError(f'Cannot safely update {key} in the hardware profile')
            # Check all actuator endpoints before writing or restarting anything.
            import yaml
            updated = yaml.safe_load(source)
            driver = updated['driver']
            for angle in (-updated['max_steering_angle'], updated['max_steering_angle']):
                servo = driver['steering_angle_to_servo_offset'] + angle * driver['steering_angle_to_servo_gain']
                if not driver['servo_min'] <= servo <= driver['servo_max']:
                    raise ValueError('Recommendation exceeds the established servo travel')
            launch = dict(self.manager.current_config or {})
            if not launch:
                raise RuntimeError('Launch the current configuration before applying a recommendation')
            self.manager.stop()
            temporary = path.with_suffix('.yaml.tmp')
            temporary.write_text(source, encoding='utf-8')
            temporary.replace(path)
            atomic_json(self.root / snapshot['run_id'] / 'applied.json', {'fingerprint': fingerprint(updated), 'recommendation': recommendation})
            self.manager.launch(launch)
            return {'message': 'Calibration applied. Run all six checks again to verify the new settings.'}
