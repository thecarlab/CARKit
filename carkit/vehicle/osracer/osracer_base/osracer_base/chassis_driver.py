# CARKit learning annotation: implements the behavior described by this file's package and module.
import math
import os
import re
import signal
import termios
import threading
import time
from dataclasses import dataclass

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import BatteryState, Imu, MagneticField
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

import serial


SUPPORTED_PROTOCOL = '1.1'
VEHICLE_CAPABILITY_CONTRACT = 1
MAX_WHEELBASE_MM = 9_999
MAX_SPEED_MMPS = 20_000
MAX_STEERING_MDEG = 90_000
MAX_BATTERY_MV = 60_000
SERIAL_ERRORS = (serial.SerialException, OSError, ValueError, TypeError, termios.error)
NONFINITE_WARNING_INTERVAL_S = 5.0


@dataclass(frozen=True)
class VehicleCapabilities:
    profile: str
    schema: int
    wheelbase_m: float
    forward_max_mps: float
    reverse_max_mps: float
    steering_max_rad: float
    battery_min_v: float
    battery_max_v: float


class ChassisDriver(Node):
    def __init__(self):
        super().__init__('osracer_base')

        self.declare_parameter('port', '/dev/osrbot_base')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('protocol_mode', 'modern')
        self.declare_parameter('legacy_wheelbase', 0.325)
        self.declare_parameter('legacy_forward_max_speed', 0.8)
        self.declare_parameter('legacy_reverse_max_speed', 0.8)
        self.declare_parameter('legacy_max_steering_angle', math.radians(30.0))
        self.declare_parameter('legacy_battery_min_voltage', 10.8)
        self.declare_parameter('legacy_battery_max_voltage', 12.6)
        self.declare_parameter('legacy_status_period', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('firmware_version_timeout', 0.3)
        self.declare_parameter('connection_status_enabled', True)
        self.declare_parameter('connection_refresh_period', 1.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('telemetry_publish_rate_hz', 50.0)
        self.declare_parameter('imu_publish_rate_hz', 30.0)
        self.declare_parameter('publish_rc', True)
        self.declare_parameter('rc_topic', 'rc_data')
        self.declare_parameter('publish_mag', True)
        self.declare_parameter('mag_topic', 'magnetometer_data')
        self.declare_parameter('mag_frame_id', 'imu_link')
        self.declare_parameter('imu_orientation_covariance', [0.02, 0.02, 0.05])
        self.declare_parameter('imu_angular_velocity_covariance', [0.01, 0.01, 0.01])
        self.declare_parameter('imu_linear_acceleration_covariance', [0.10, 0.10, 0.10])
        self.declare_parameter('odom_twist_covariance', [0.02, 0.20, 1.0, 1.0, 1.0, 0.30])
        self.declare_parameter('publish_battery', True)
        self.declare_parameter('battery_topic', 'battery_state')

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.protocol_mode = str(self.get_parameter('protocol_mode').value).strip().lower()
        if self.protocol_mode not in ('modern', 'legacy'):
            raise ValueError("protocol_mode must be 'modern' or 'legacy'")
        self.legacy_capabilities = VehicleCapabilities(
            profile='legacy',
            schema=0,
            wheelbase_m=self.require_positive_parameter('legacy_wheelbase'),
            forward_max_mps=self.require_positive_parameter('legacy_forward_max_speed'),
            reverse_max_mps=self.require_positive_parameter('legacy_reverse_max_speed'),
            steering_max_rad=self.require_positive_parameter('legacy_max_steering_angle'),
            battery_min_v=float(self.get_parameter('legacy_battery_min_voltage').value),
            battery_max_v=float(self.get_parameter('legacy_battery_max_voltage').value),
        )
        if self.legacy_capabilities.battery_min_v >= self.legacy_capabilities.battery_max_v:
            raise ValueError('legacy battery minimum must be below its maximum')
        self.legacy_status_period = self.require_positive_parameter('legacy_status_period')
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.firmware_version_timeout = float(self.get_parameter('firmware_version_timeout').value)
        self.connection_status_enabled = self.as_bool(self.get_parameter('connection_status_enabled').value)
        self.connection_refresh_period = float(self.get_parameter('connection_refresh_period').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.publish_tf = self.as_bool(self.get_parameter('publish_tf').value)
        self.telemetry_publish_rate_hz = self.require_positive_parameter(
            'telemetry_publish_rate_hz'
        )
        self.imu_publish_rate_hz = self.require_positive_parameter(
            'imu_publish_rate_hz'
        )
        self.publish_rc_enabled = self.as_bool(self.get_parameter('publish_rc').value)
        self.rc_topic = self.get_parameter('rc_topic').value
        self.publish_mag_enabled = self.as_bool(self.get_parameter('publish_mag').value)
        self.mag_topic = self.get_parameter('mag_topic').value
        self.mag_frame_id = self.get_parameter('mag_frame_id').value
        self.imu_orientation_covariance = self.diagonal_covariance(
            self.get_parameter('imu_orientation_covariance').value
        )
        self.imu_angular_velocity_covariance = self.diagonal_covariance(
            self.get_parameter('imu_angular_velocity_covariance').value
        )
        self.imu_linear_acceleration_covariance = self.diagonal_covariance(
            self.get_parameter('imu_linear_acceleration_covariance').value
        )
        self.odom_twist_covariance = self.diagonal_covariance_6d(
            self.get_parameter('odom_twist_covariance').value
        )
        self.publish_battery_enabled = self.as_bool(self.get_parameter('publish_battery').value)
        self.battery_topic = self.get_parameter('battery_topic').value

        qos_fast = QoSProfile(depth=1)
        qos_normal = QoSProfile(depth=5)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_normal)
        self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.ackermann_cmd_callback,
            qos_normal,
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_fast)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_fast)
        self.rc_pub = (
            self.create_publisher(Int32MultiArray, self.rc_topic, qos_normal)
            if self.publish_rc_enabled else None
        )
        self.mag_pub = (
            self.create_publisher(MagneticField, self.mag_topic, qos_fast)
            if self.publish_mag_enabled else None
        )
        self.battery_pub = (
            self.create_publisher(BatteryState, self.battery_topic, qos_normal)
            if self.publish_battery_enabled else None
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.reader_thread = None
        self.capability_ready = False
        self.vehicle_capabilities = None
        self.capability_conn = None
        self.shutdown_event = threading.Event()
        self.last_cmd_time = self.get_clock().now()
        self.remote_control_active = None
        self.nonfinite_frame_counts = {}
        self.nonfinite_last_warning = {}
        self.last_legacy_gyro_z = 0.0
        self.last_motion_publish_monotonic = None
        self.last_imu_publish_monotonic = None

        self.create_timer(self.reconnect_interval, self.ensure_connected)
        self.create_timer(0.1, self.watchdog_check)
        self.create_timer(max(0.2, self.connection_refresh_period), self.refresh_connection_status)
        if self.protocol_mode == 'legacy':
            self.create_timer(self.legacy_status_period, self.poll_legacy_status)
        self.ensure_connected()

    def ensure_connected(self):
        if self.shutdown_event.is_set():
            return
        with self.serial_lock:
            conn = self.serial_conn
            connected = conn is not None and conn.is_open
        if connected:
            if self.port.startswith('/') and not os.path.exists(self.port):
                self.get_logger().warning(f"Serial device disconnected: {self.port}")
                self.close_serial(conn)
                return
            active_conn, _ = self.active_capability_binding()
            if active_conn is not conn:
                self.close_serial(conn)
                return
            self.start_reader()
            return

        if self.port.startswith('/') and not os.path.exists(self.port):
            self.get_logger().warning(f"Serial device not found: {self.port}")
            return

        try:
            conn = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=0.1)
            conn.reset_input_buffer()
            conn.reset_output_buffer()
        except SERIAL_ERRORS as exc:
            self.get_logger().warning(f"Could not open serial device {self.port}: {exc}")
            return

        with self.serial_lock:
            self._clear_vehicle_capabilities_locked()
            self.serial_conn = conn
        if not self.configure_device(conn):
            if conn.is_open:
                self.close_serial(conn)
            return
        self.start_reader()
        self.get_logger().info(f"Connected to chassis on {self.port}")

    def configure_device(self, conn):
        if self.protocol_mode == 'legacy':
            if not self.bind_vehicle_capabilities(conn, self.legacy_capabilities):
                return False
            self.get_logger().warning(
                'Legacy chassis protocol enabled: controller capabilities cannot be queried; '
                'using configured conservative motion limits'
            )
            return True

        capabilities = self.verify_firmware_identity(conn)
        if capabilities is None:
            return False
        if not self.write_raw(self._device_mode_command(), expected_conn=conn):
            return False
        if not self.write_raw(self._state_request_command(), expected_conn=conn):
            return False
        if not self.bind_vehicle_capabilities(conn, capabilities):
            return False
        if not self.send_connection_status('up', expected_conn=conn):
            return False
        self.get_logger().info('Vehicle capability contract accepted')
        return True

    @staticmethod
    def _device_mode_command():
        return ''.join(chr(value) for value in (115, 116, 114, 101, 97, 109, 32, 115, 121, 110, 99, 10))

    @staticmethod
    def _state_request_command():
        return chr(115) + '\n'

    @staticmethod
    def _quiet_command():
        return ''.join(chr(value) for value in (115, 116, 114, 101, 97, 109, 32, 111, 102, 102, 10))

    @staticmethod
    def _version_command():
        return ''.join(chr(value) for value in (102, 119, 32, 118, 101, 114, 115, 105, 111, 110, 10))

    @staticmethod
    def _profile_command():
        return 'profile get\n'

    @staticmethod
    def _vehicle_command():
        return 'vehicle get\n'

    def verify_firmware_identity(self, expected_conn):
        with self.serial_lock:
            conn = self.serial_conn
            if conn is not expected_conn or not conn.is_open:
                return None
            try:
                conn.reset_input_buffer()
                conn.write(self._quiet_command().encode('utf-8'))
                conn.flush()
                time.sleep(0.05)
                conn.reset_input_buffer()
                conn.write(self._version_command().encode('utf-8'))
                conn.flush()
            except SERIAL_ERRORS as exc:
                self.get_logger().warning(f"Could not query chassis firmware version: {exc}")
                return None

        version = self.read_identity_response(self.parse_firmware_version, expected_conn)
        if version is None:
            self.get_logger().warning('Chassis firmware identity unavailable')
            return None
        project_ver, protocol = version
        if protocol != SUPPORTED_PROTOCOL:
            self.get_logger().warning(
                f"Unsupported chassis protocol {protocol}; expected {SUPPORTED_PROTOCOL}"
            )
            return None

        if not self.write_raw(self._profile_command(), expected_conn=expected_conn):
            return None
        profile = self.read_identity_response(self.parse_profile_status, expected_conn)
        if profile is None:
            self.get_logger().warning('Chassis profile identity unavailable')
            return None
        if profile['state'] != 'READY' or not profile['motion']:
            self.get_logger().warning(
                'Chassis profile is not motion-ready: '
                f"State={profile['state']}, Motion={'Yes' if profile['motion'] else 'No'}"
            )
            return None

        if not self.write_raw(self._vehicle_command(), expected_conn=expected_conn):
            return None
        capabilities = self.read_identity_response(
            lambda line: self.parse_vehicle_capabilities(
                line,
                expected_profile=profile['id'],
                expected_schema=profile['schema'],
            ),
            expected_conn,
        )
        if capabilities is None:
            self.get_logger().warning('Chassis vehicle capability contract unavailable')
            return None

        self.get_logger().info(
            f"Chassis firmware ProjectVer: {project_ver}, Proto: {protocol}"
        )
        return capabilities

    def read_identity_response(self, parser, expected_conn):
        deadline = time.monotonic() + max(0.1, self.firmware_version_timeout)
        while time.monotonic() < deadline:
            with self.serial_lock:
                conn = self.serial_conn
                if conn is not expected_conn or not conn.is_open:
                    return None
                try:
                    line = conn.readline().decode('utf-8', errors='ignore').strip()
                except SERIAL_ERRORS as exc:
                    self.get_logger().warning(f"Could not read chassis identity: {exc}")
                    return None
            if line:
                result = parser(line)
                if result is not None:
                    return result
        return None

    @staticmethod
    def parse_firmware_version(line):
        project_match = re.search(r'\bProjectVer\s*[:=]\s*([^,\s]+)', line)
        protocol_match = re.search(r'\bProto\s*[:=]\s*([0-9]+(?:\.[0-9]+)*)', line)
        if project_match and protocol_match:
            return project_match.group(1), protocol_match.group(1)
        return None

    @staticmethod
    def parse_project_version(line):
        identity = ChassisDriver.parse_firmware_version(line)
        if identity:
            return identity[0]
        match = re.search(r'\bProjectVer\s*[:=]\s*([^,\s]+)', line)
        if match:
            return match.group(1)
        match = re.search(r'\bProjectVer\s+([^,\s]+)', line)
        if match:
            return match.group(1)
        return None

    @staticmethod
    def parse_profile_status(line):
        match = re.fullmatch(
            r'PROFILE:\s+ID=([a-z0-9_-]+),\s+Schema=([0-9]+),\s+'
            r'State=([A-Z0-9_]+),\s+Motion=(Yes|No),\s+Writes=(Yes|No)',
            line,
        )
        if not match:
            return None
        return {
            'id': match.group(1),
            'schema': int(match.group(2)),
            'state': match.group(3),
            'motion': match.group(4) == 'Yes',
            'writes': match.group(5) == 'Yes',
        }

    @staticmethod
    def parse_vehicle_capabilities(line, expected_profile, expected_schema):
        match = re.fullmatch(
            r'VEHICLE: Contract=([0-9]+), Profile=([a-z0-9_-]+), Schema=([0-9]+), '
            r'WheelbaseMm=([0-9]+), ForwardMaxMmps=([0-9]+), '
            r'ReverseMaxMmps=([0-9]+), SteeringMaxMdeg=([0-9]+), '
            r'BatteryMinMv=([0-9]+), BatteryMaxMv=([0-9]+)',
            line,
        )
        if not match:
            return None

        (
            contract,
            profile,
            schema,
            wheelbase_mm,
            forward_max_mmps,
            reverse_max_mmps,
            steering_max_mdeg,
            battery_min_mv,
            battery_max_mv,
        ) = match.groups()
        values = tuple(
            int(value)
            for value in (
                contract,
                schema,
                wheelbase_mm,
                forward_max_mmps,
                reverse_max_mmps,
                steering_max_mdeg,
                battery_min_mv,
                battery_max_mv,
            )
        )
        (
            contract,
            schema,
            wheelbase_mm,
            forward_max_mmps,
            reverse_max_mmps,
            steering_max_mdeg,
            battery_min_mv,
            battery_max_mv,
        ) = values

        if contract != VEHICLE_CAPABILITY_CONTRACT:
            return None
        if schema < 1:
            return None
        if profile != expected_profile or schema != expected_schema:
            return None
        if not 0 < wheelbase_mm <= MAX_WHEELBASE_MM:
            return None
        if not 0 < forward_max_mmps <= MAX_SPEED_MMPS:
            return None
        if not 0 < reverse_max_mmps <= MAX_SPEED_MMPS:
            return None
        if not 0 < steering_max_mdeg <= MAX_STEERING_MDEG:
            return None
        if battery_min_mv >= battery_max_mv or battery_max_mv > MAX_BATTERY_MV:
            return None

        return VehicleCapabilities(
            profile=profile,
            schema=schema,
            wheelbase_m=wheelbase_mm / 1000.0,
            forward_max_mps=forward_max_mmps / 1000.0,
            reverse_max_mps=reverse_max_mmps / 1000.0,
            steering_max_rad=math.radians(steering_max_mdeg / 1000.0),
            battery_min_v=battery_min_mv / 1000.0,
            battery_max_v=battery_max_mv / 1000.0,
        )

    def _clear_vehicle_capabilities_locked(self):
        self.capability_ready = False
        self.vehicle_capabilities = None
        self.capability_conn = None

    def bind_vehicle_capabilities(self, expected_conn, capabilities):
        with self.serial_lock:
            if self.serial_conn is not expected_conn or not expected_conn.is_open:
                return False
            self.vehicle_capabilities = capabilities
            self.capability_conn = expected_conn
            self.capability_ready = True
        return True

    def active_capability_binding(self):
        with self.serial_lock:
            conn = self.serial_conn
            capabilities = self.vehicle_capabilities
            if (
                not self.capability_ready
                or capabilities is None
                or self.capability_conn is not conn
                or conn is None
                or not conn.is_open
            ):
                return None, None
            return conn, capabilities

    def start_reader(self):
        if self.shutdown_event.is_set():
            return
        if self.reader_thread and self.reader_thread.is_alive():
            return
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

    def close_serial(self, expected_conn=None):
        with self.serial_lock:
            if expected_conn is not None and self.serial_conn is not expected_conn:
                conn = expected_conn
            else:
                conn = self.serial_conn
                self.serial_conn = None
                self._clear_vehicle_capabilities_locked()
        if conn:
            try:
                if conn.is_open:
                    self.write_connection_down(conn)
                conn.close()
            except Exception:
                pass

    def send_connection_status(self, state, expected_conn=None):
        if not self.connection_status_enabled or self.protocol_mode == 'legacy':
            return True
        return self.write_raw(
            self._connection_status_command(state),
            expected_conn=expected_conn,
        )

    def refresh_connection_status(self):
        if self.shutdown_event.is_set():
            return
        conn, _ = self.active_capability_binding()
        if conn is not None:
            self.send_connection_status('ping', expected_conn=conn)

    def poll_legacy_status(self):
        if self.protocol_mode == 'legacy' and not self.shutdown_event.is_set():
            conn, _ = self.active_capability_binding()
            if conn is not None:
                self.write_raw('status\n', expected_conn=conn)

    def write_connection_down(self, conn):
        if not self.connection_status_enabled or self.protocol_mode == 'legacy':
            return
        try:
            conn.write(self._connection_status_command('down').encode('utf-8'))
            conn.flush()
        except SERIAL_ERRORS:
            pass

    @staticmethod
    def _connection_status_command(state):
        values = {
            'up': (108, 105, 110, 107, 32, 117, 112, 32, 114, 111, 115, 10),
            'ping': (108, 105, 110, 107, 32, 112, 105, 110, 103, 32, 114, 111, 115, 10),
            'down': (108, 105, 110, 107, 32, 100, 111, 119, 110, 32, 114, 111, 115, 10),
        }
        return ''.join(chr(value) for value in values[state])

    @staticmethod
    def _ignored_response_prefixes():
        return ('FW', 'DIAG', 'LINK', 'OK', 'ERROR')

    def write_raw(self, command, expected_conn=None):
        failed_conn = None
        with self.serial_lock:
            conn = self.serial_conn
            if (
                conn is None
                or not conn.is_open
                or (expected_conn is not None and conn is not expected_conn)
            ):
                return False
            try:
                conn.write(command.encode('utf-8'))
                conn.flush()
                return True
            except SERIAL_ERRORS as exc:
                self.get_logger().warning(f"Serial write failed: {exc}")
                if self.serial_conn is conn:
                    self.serial_conn = None
                    self._clear_vehicle_capabilities_locked()
                failed_conn = conn
        if failed_conn:
            try:
                failed_conn.close()
            except Exception:
                pass
        return False

    def cmd_vel_callback(self, msg):
        binding = self.active_capability_binding()
        conn, capabilities = binding
        if conn is None:
            return
        speed = self.clamp(
            msg.linear.x,
            -capabilities.reverse_max_mps,
            capabilities.forward_max_mps,
        )
        if abs(speed) < 0.01:
            steering = (
                0.0
                if msg.angular.z == 0.0
                else math.copysign(capabilities.steering_max_rad, msg.angular.z)
            )
        else:
            steering = math.atan(capabilities.wheelbase_m * msg.angular.z / speed)
        self.send_drive_command(speed, steering, expected_binding=binding)

    def ackermann_cmd_callback(self, msg):
        binding = self.active_capability_binding()
        if binding[0] is None:
            return
        self.send_drive_command(
            msg.drive.speed,
            msg.drive.steering_angle,
            expected_binding=binding,
        )

    def send_drive_command(self, speed, steering, expected_binding=None):
        conn, capabilities = self.active_capability_binding()
        if conn is None:
            return
        if expected_binding is not None and (
            conn is not expected_binding[0] or capabilities is not expected_binding[1]
        ):
            return
        speed = self.clamp(
            float(speed),
            -capabilities.reverse_max_mps,
            capabilities.forward_max_mps,
        )
        steering = self.clamp(
            float(steering),
            -capabilities.steering_max_rad,
            capabilities.steering_max_rad,
        )
        if self.write_raw(
            f"v {speed:.3f} {math.degrees(steering):.2f}\n",
            expected_conn=conn,
        ):
            self.last_cmd_time = self.get_clock().now()

    def read_loop(self):
        current_thread = threading.current_thread()
        try:
            while not self.shutdown_event.is_set() and rclpy.ok():
                with self.serial_lock:
                    conn = self.serial_conn
                if conn is None or not conn.is_open:
                    break
                try:
                    line = conn.readline().decode('utf-8', errors='ignore').strip()
                except SERIAL_ERRORS as exc:
                    if self.shutdown_event.is_set():
                        break
                    self.get_logger().warning(f"Serial read failed: {exc}")
                    self.close_serial(conn)
                    break
                if line:
                    try:
                        self.handle_device_line(line)
                    except Exception:
                        # ROS can invalidate publishers before the executor returns
                        # during launch shutdown.  The serial reader must exit quietly
                        # in that narrow window; unexpected runtime errors still fail.
                        if self.shutdown_event.is_set() or not rclpy.ok():
                            break
                        raise
        finally:
            with self.serial_lock:
                if self.reader_thread is current_thread:
                    self.reader_thread = None

    def shutdown_driver(self):
        conn, _ = self.active_capability_binding()
        if conn is not None:
            self.write_raw('v 0.000 0.00\n', expected_conn=conn)
        self.shutdown_event.set()
        self.close_serial()
        with self.serial_lock:
            reader = self.reader_thread
        if reader and reader is not threading.current_thread() and reader.is_alive():
            reader.join(timeout=1.0)

    def handle_device_line(self, line):
        parts = line.split()
        if not parts:
            return
        command = parts[0]
        if command.startswith(self._ignored_response_prefixes()) or command == 'link':
            return
        try:
            if command == 's' and len(parts) == 18:
                self.publish_motion_state(parts)
            elif self.protocol_mode == 'legacy' and command == 'o' and len(parts) == 8:
                self.publish_legacy_odometry(parts)
            elif self.protocol_mode == 'legacy' and command == 'i' and len(parts) == 11:
                self.publish_legacy_imu(parts)
            elif command == 'r':
                self.publish_rc_data(parts)
                if self.protocol_mode != 'legacy' and len(parts) >= 8:
                    self.update_control_source(parts)
            elif command == 'm' and len(parts) == 4:
                self.publish_magnetometer(parts)
            elif (
                command == 'b'
                and len(parts) == 2
                and self.publish_battery_enabled
                and self.battery_pub is not None
            ):
                self.publish_battery(float(parts[1]))
            elif self.protocol_mode == 'legacy' and line.startswith('Status:'):
                self.handle_legacy_status(line)
        except ValueError as exc:
            self.get_logger().warning(f"Could not parse chassis data: {exc}")

    def handle_legacy_status(self, line):
        voltage_match = re.search(r'\bVoltage=([-+0-9.eE]+)V\b', line)
        if voltage_match:
            self.publish_battery(float(voltage_match.group(1)))

        control_match = re.search(r'\bControl=([A-Za-z]+)\b', line)
        if not control_match:
            return
        remote_active = control_match.group(1).lower() != 'serial'
        if remote_active == self.remote_control_active:
            return
        self.remote_control_active = remote_active
        if remote_active:
            self.get_logger().warning(
                'Remote control is active; ROS motion commands may be ignored until serial control is selected'
            )
        else:
            self.get_logger().info('Serial control is active')

    def publish_legacy_odometry(self, parts):
        values = self.parse_finite_telemetry('o', parts[1:])
        if values is None:
            return
        px, py, pz, vx, vy, vz, yaw = values
        half_yaw = yaw * 0.5
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = self.last_legacy_gyro_z
        odom.twist.covariance = self.odom_twist_covariance
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = px
            transform.transform.translation.y = py
            transform.transform.translation.z = pz
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def publish_legacy_imu(self, parts):
        values = self.parse_finite_telemetry('i', parts[1:])
        if values is None:
            return
        qx, qy, qz, qw, ax, ay, az, gx, gy, gz = values
        self.last_legacy_gyro_z = gz
        if not self.telemetry_channel_due(
            'last_imu_publish_monotonic',
            getattr(self, 'imu_publish_rate_hz', 0.0),
        ):
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame_id
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.orientation_covariance = self.imu_orientation_covariance
        imu.angular_velocity_covariance = self.imu_angular_velocity_covariance
        imu.linear_acceleration_covariance = self.imu_linear_acceleration_covariance
        self.imu_pub.publish(imu)

    def publish_rc_data(self, parts):
        if not self.publish_rc_enabled or self.rc_pub is None:
            return
        msg = Int32MultiArray()
        msg.data = [int(value) for value in parts[1:]]
        self.rc_pub.publish(msg)

    def publish_magnetometer(self, parts):
        if not self.publish_mag_enabled or self.mag_pub is None:
            return
        values = self.parse_finite_telemetry('m', parts[1:])
        if values is None:
            return
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mag_frame_id
        msg.magnetic_field.x = values[0] * 1e-4
        msg.magnetic_field.y = values[1] * 1e-4
        msg.magnetic_field.z = values[2] * 1e-4
        self.mag_pub.publish(msg)

    def update_control_source(self, parts):
        control_channel = int(parts[7])
        remote_active = 0 <= control_channel < 1500
        if remote_active == self.remote_control_active:
            return
        self.remote_control_active = remote_active
        if remote_active:
            self.get_logger().warning(
                "Remote control is active; ROS motion commands may be ignored until serial control is selected"
            )
        else:
            self.get_logger().info("Serial control is active")

    def publish_motion_state(self, parts):
        values = self.parse_finite_telemetry('s', parts[1:])
        if values is None:
            return
        if not self.motion_frame_due():
            return
        px, py, pz = values[0:3]
        vx, vy, vz = values[3:6]
        qx, qy, qz, qw = values[7:11]
        ax, ay, az = values[11:14]
        gx, gy, gz = values[14:17]

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.covariance = self.odom_twist_covariance
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = px
            transform.transform.translation.y = py
            transform.transform.translation.z = pz
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame_id
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.orientation_covariance = self.imu_orientation_covariance
        imu.angular_velocity_covariance = self.imu_angular_velocity_covariance
        imu.linear_acceleration_covariance = self.imu_linear_acceleration_covariance
        self.imu_pub.publish(imu)

    def motion_frame_due(self):
        """Bound ROS fan-out while still draining the firmware serial stream."""
        return self.telemetry_channel_due(
            'last_motion_publish_monotonic',
            getattr(self, 'telemetry_publish_rate_hz', 0.0),
        )

    def telemetry_channel_due(self, timestamp_attribute, publish_rate):
        publish_rate = float(publish_rate)
        if publish_rate <= 0.0:
            return True
        now = time.monotonic()
        last_publish = getattr(self, timestamp_attribute, None)
        if last_publish is not None and now - last_publish < 1.0 / publish_rate:
            return False
        setattr(self, timestamp_attribute, now)
        return True

    def publish_battery(self, voltage):
        if not self.publish_battery_enabled or self.battery_pub is None:
            return
        _, capabilities = self.active_capability_binding()
        if capabilities is None:
            return
        values = self.parse_finite_telemetry('b', [voltage])
        if values is None:
            return
        voltage = values[0]
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.present = True
        pct = (voltage - capabilities.battery_min_v) / (
            capabilities.battery_max_v - capabilities.battery_min_v
        )
        msg.percentage = self.clamp(pct, 0.0, 1.0)
        self.battery_pub.publish(msg)

    def require_positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be a positive finite number')
        return value

    def parse_finite_telemetry(self, command, raw_values):
        values = [float(value) for value in raw_values]
        if all(math.isfinite(value) for value in values):
            return values

        count = self.nonfinite_frame_counts.get(command, 0) + 1
        self.nonfinite_frame_counts[command] = count
        now = time.monotonic()
        last_warning = self.nonfinite_last_warning.get(command)
        if last_warning is None or now - last_warning >= NONFINITE_WARNING_INTERVAL_S:
            self.nonfinite_last_warning[command] = now
            self.get_logger().warning(
                f"Dropped non-finite '{command}' telemetry frame (count={count})"
            )
        return None

    @staticmethod
    def diagonal_covariance(diagonal):
        if len(diagonal) != 3:
            raise ValueError("IMU covariance parameters must contain exactly 3 diagonal values")
        return [
            float(diagonal[0]), 0.0, 0.0,
            0.0, float(diagonal[1]), 0.0,
            0.0, 0.0, float(diagonal[2]),
        ]

    @staticmethod
    def diagonal_covariance_6d(diagonal):
        if len(diagonal) != 6:
            raise ValueError("Odometry twist covariance parameter must contain exactly 6 diagonal values")
        covariance = [0.0] * 36
        for index, value in enumerate(diagonal):
            covariance[index * 6 + index] = float(value)
        return covariance

    def watchdog_check(self):
        if self.shutdown_event.is_set():
            return
        conn, _ = self.active_capability_binding()
        if conn is None:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.write_raw("v 0.00 0.00\n", expected_conn=conn)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def main(args=None):
    rclpy.init(args=args)
    node = ChassisDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown_driver()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
