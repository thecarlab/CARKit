import importlib.util
import hashlib
import json
import math
import re
import signal
import sys
import threading
import types
import unittest
from collections import deque
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]
DRIVER_PATH = REPO_ROOT / 'osracer_base' / 'chassis_driver.py'
FIXTURE_PATH = REPO_ROOT / 'test' / 'fixtures' / 'proto_1_1' / 'session.json'
PROFILE_CONFIG_DIR = REPO_ROOT / 'config' / 'vehicles'
WORKFLOW_PATH = REPO_ROOT / '.github' / 'workflows' / 'ros2-ci.yml'
DESCRIPTION_LAUNCH_PATH = REPO_ROOT / 'launch' / 'description.launch.py'
DESCRIPTION_LAUNCH_SHA256 = 'd67bdcd272aa275cd449d3feb234443aa1f61aae7360018e5af8242bdf219f27'


class _Vector:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quaternion(_Vector):
    def __init__(self):
        super().__init__()
        self.w = 0.0


class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ''


class _Twist:
    def __init__(self):
        self.linear = _Vector()
        self.angular = _Vector()


class _AckermannDrive:
    def __init__(self):
        self.speed = 0.0
        self.steering_angle = 0.0


class _AckermannDriveStamped:
    def __init__(self):
        self.header = _Header()
        self.drive = _AckermannDrive()


class _Odometry:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ''
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=_Vector(), orientation=_Quaternion())
        )
        self.twist = types.SimpleNamespace(twist=_Twist(), covariance=[0.0] * 36)


class _Imu:
    def __init__(self):
        self.header = _Header()
        self.orientation = _Quaternion()
        self.angular_velocity = _Vector()
        self.linear_acceleration = _Vector()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


class _MagneticField:
    def __init__(self):
        self.header = _Header()
        self.magnetic_field = _Vector()


class _BatteryState:
    def __init__(self):
        self.header = _Header()
        self.voltage = 0.0
        self.percentage = 0.0
        self.present = False


class _Int32MultiArray:
    def __init__(self):
        self.data = []


class _TransformStamped:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ''
        self.transform = types.SimpleNamespace(translation=_Vector(), rotation=_Quaternion())


class _SerialException(Exception):
    pass


class _ExternalShutdownException(Exception):
    pass


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(str(message))

    def warning(self, message):
        self.warnings.append(str(message))


class _Delta:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _TimePoint:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return _Delta(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return self.nanoseconds


class _Clock:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    def now(self):
        return _TimePoint(self.nanoseconds)


class _FakeSerial:
    def __init__(
        self,
        version_line=None,
        profile_line=None,
        vehicle_line=None,
        fail_on_write=None,
    ):
        self.is_open = True
        self.version_line = version_line
        self.profile_line = profile_line
        self.vehicle_line = vehicle_line
        self.fail_on_write = fail_on_write
        self.lines = deque()
        self.writes = []
        self.close_count = 0

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def write(self, payload):
        text = payload.decode('utf-8')
        if text == self.fail_on_write:
            raise _SerialException('synthetic write failure')
        self.writes.append(text)
        if text == 'fw version\n' and self.version_line:
            self.lines.append((self.version_line + '\n').encode('utf-8'))
        if text == 'profile get\n' and self.profile_line:
            self.lines.append((self.profile_line + '\n').encode('utf-8'))
        if text == 'vehicle get\n' and self.vehicle_line:
            self.lines.append((self.vehicle_line + '\n').encode('utf-8'))
        return len(payload)

    def flush(self):
        pass

    def readline(self):
        return self.lines.popleft() if self.lines else b''

    def close(self):
        self.close_count += 1
        self.is_open = False


def _module(name, **attributes):
    result = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(result, key, value)
    return result


def _load_driver_module():
    class _Node:
        pass

    class _QoSProfile:
        def __init__(self, depth):
            self.depth = depth

    class _Message:
        pass

    class _TransformBroadcaster:
        def __init__(self, node):
            self.node = node

    rclpy = _module('rclpy', ok=lambda: True)
    stubs = {
        'rclpy': rclpy,
        'rclpy.executors': _module(
            'rclpy.executors',
            ExternalShutdownException=_ExternalShutdownException,
        ),
        'rclpy.node': _module('rclpy.node', Node=_Node),
        'rclpy.qos': _module('rclpy.qos', QoSProfile=_QoSProfile),
        'ackermann_msgs': _module('ackermann_msgs'),
        'ackermann_msgs.msg': _module(
            'ackermann_msgs.msg',
            AckermannDriveStamped=_AckermannDriveStamped,
        ),
        'geometry_msgs': _module('geometry_msgs'),
        'geometry_msgs.msg': _module(
            'geometry_msgs.msg',
            TransformStamped=_TransformStamped,
            Twist=_Twist,
        ),
        'nav_msgs': _module('nav_msgs'),
        'nav_msgs.msg': _module('nav_msgs.msg', Odometry=_Odometry),
        'sensor_msgs': _module('sensor_msgs'),
        'sensor_msgs.msg': _module(
            'sensor_msgs.msg',
            BatteryState=_BatteryState,
            Imu=_Imu,
            MagneticField=_MagneticField,
        ),
        'std_msgs': _module('std_msgs'),
        'std_msgs.msg': _module('std_msgs.msg', Int32MultiArray=_Int32MultiArray),
        'tf2_ros': _module('tf2_ros', TransformBroadcaster=_TransformBroadcaster),
        'serial': _module(
            'serial',
            Serial=lambda *args, **kwargs: None,
            SerialException=_SerialException,
        ),
    }
    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location('chassis_driver_under_test', DRIVER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


DRIVER = _load_driver_module()


def _fixture():
    return json.loads(FIXTURE_PATH.read_text(encoding='utf-8'))


def _vehicle_response(**overrides):
    values = {
        'Contract': 1,
        'Profile': 'fixture_alpha',
        'Schema': 7,
        'WheelbaseMm': 1375,
        'ForwardMaxMmps': 4321,
        'ReverseMaxMmps': 1234,
        'SteeringMaxMdeg': 27123,
        'BatteryMinMv': 11111,
        'BatteryMaxMv': 14999,
    }
    values.update(overrides)
    return (
        f"VEHICLE: Contract={values['Contract']}, Profile={values['Profile']}, "
        f"Schema={values['Schema']}, WheelbaseMm={values['WheelbaseMm']}, "
        f"ForwardMaxMmps={values['ForwardMaxMmps']}, "
        f"ReverseMaxMmps={values['ReverseMaxMmps']}, "
        f"SteeringMaxMdeg={values['SteeringMaxMdeg']}, "
        f"BatteryMinMv={values['BatteryMinMv']}, BatteryMaxMv={values['BatteryMaxMv']}"
    )


def _driver_shell():
    driver = object.__new__(DRIVER.ChassisDriver)
    driver.serial_lock = threading.Lock()
    driver.serial_conn = None
    driver.reader_thread = None
    driver.shutdown_event = threading.Event()
    driver.port = 'loop://sanitized'
    driver.baudrate = 460800
    driver.firmware_version_timeout = 0.01
    driver.capability_ready = False
    driver.vehicle_capabilities = None
    driver.capability_conn = None
    driver.connection_status_enabled = True
    driver.protocol_mode = 'modern'
    driver.remote_control_active = None
    driver.publish_rc_enabled = True
    driver.publish_mag_enabled = True
    driver.publish_battery_enabled = True
    driver.nonfinite_frame_counts = {}
    driver.nonfinite_last_warning = {}
    driver.last_legacy_gyro_z = 0.0
    driver.rc_pub = mock.Mock()
    driver.mag_pub = mock.Mock()
    driver.battery_pub = mock.Mock()
    driver.logger = _Logger()
    driver.get_logger = lambda: driver.logger
    driver.start_reader = mock.Mock()
    return driver


def _synthetic_capabilities():
    return types.SimpleNamespace(
        profile='fixture_alpha',
        schema=7,
        wheelbase_m=1.375,
        forward_max_mps=4.321,
        reverse_max_mps=1.234,
        steering_max_rad=math.radians(27.123),
        battery_min_v=11.111,
        battery_max_v=14.999,
    )


def _bind_synthetic_capabilities(driver, serial_conn=None):
    conn = serial_conn or _FakeSerial()
    driver.serial_conn = conn
    driver.capability_conn = conn
    driver.vehicle_capabilities = _synthetic_capabilities()
    driver.capability_ready = True
    return conn


class FixtureContractTests(unittest.TestCase):
    def test_fixture_is_sanitized_and_proto_1_1(self):
        data = _fixture()
        text = FIXTURE_PATH.read_text(encoding='utf-8')

        self.assertEqual(data['protocol'], '1.1')
        self.assertFalse(data['sanitization']['private_source_copied'])
        self.assertTrue(data['sanitization']['identifiers_removed'])
        self.assertTrue(data['sanitization']['telemetry_values_synthetic'])
        self.assertFalse(data['sanitization']['vehicle_profile_values_included'])
        self.assertNotRegex(text, re.compile(r'\b[0-9A-F]{12}\b'))
        for forbidden in ('NEORACER', 'OSRCOREV', 'T005', 'customer', 'serial_number'):
            self.assertNotIn(forbidden, text)
        for vehicle_name in ('neo', 'red', 'blue'):
            self.assertNotRegex(text.lower(), rf'\b{vehicle_name}\b')

    def test_fixture_records_startup_and_watchdog_contract(self):
        data = _fixture()
        self.assertEqual(
            data['startup']['expected_host_commands'],
            [
                'stream off\n',
                'fw version\n',
                'profile get\n',
                'vehicle get\n',
                'stream sync\n',
                's\n',
                'link up ros\n',
            ],
        )
        self.assertEqual(data['control']['watchdog_seconds'], 0.5)
        self.assertEqual(data['control']['watchdog_command'], 'v 0.00 0.00\n')
        self.assertEqual(data['control']['cmd_vel_near_zero_mps'], 0.01)
        self.assertEqual(data['startup']['firmware_version_timeout_seconds'], 0.3)

    def test_fixture_records_anonymous_vehicle_capability_contract(self):
        contract = json.loads(
            (REPO_ROOT / 'test/fixtures/proto_1_1/firmware_contract.json').read_text(
                encoding='utf-8'
            )
        )
        self.assertEqual(contract['protocol'], DRIVER.SUPPORTED_PROTOCOL)
        self.assertEqual(contract['vehicle_capability']['request'], 'vehicle get\n')
        self.assertEqual(contract['vehicle_capability']['contract'], 1)
        self.assertEqual(
            contract['vehicle_capability']['response_fields'],
            [
                'Contract', 'Profile', 'Schema', 'WheelbaseMm', 'ForwardMaxMmps',
                'ReverseMaxMmps', 'SteeringMaxMdeg', 'BatteryMinMv', 'BatteryMaxMv',
            ],
        )
        self.assertEqual(list(PROFILE_CONFIG_DIR.glob('*.yaml')), [])

    def test_fixture_records_sensor_contract(self):
        data = _fixture()
        self.assertEqual(data['publishers']['rc_topic'], 'rc_data')
        self.assertEqual(data['publishers']['mag_topic'], 'magnetometer_data')
        self.assertEqual(data['publishers']['mag_frame_id'], 'imu_link')
        self.assertEqual(data['publishers']['battery_topic'], 'battery_state')
        self.assertEqual(data['publishers']['gauss_to_tesla'], 1e-4)
        self.assertEqual(data['covariance_diagonals']['imu_orientation'], [0.02, 0.02, 0.05])
        self.assertEqual(data['covariance_diagonals']['odom_twist'], [0.02, 0.20, 1.0, 1.0, 1.0, 0.30])


class PublicApiTests(unittest.TestCase):
    def test_ackermann_cmd_uses_carkit_stamped_message(self):
        source = DRIVER_PATH.read_text(encoding='utf-8')
        self.assertIs(DRIVER.AckermannDriveStamped, _AckermannDriveStamped)
        self.assertIn('AckermannDriveStamped', source)

    def test_base_uses_one_canonical_parameter_api(self):
        source = DRIVER_PATH.read_text(encoding='utf-8')
        launch = (REPO_ROOT / 'launch' / 'chassis_driver.launch.py').read_text(encoding='utf-8')
        odom_view_launch = (REPO_ROOT / 'launch' / 'odom_view.launch.py').read_text(encoding='utf-8')
        for name in (
            'publish_rc', 'rc_topic', 'publish_mag', 'mag_topic', 'mag_frame_id',
            'imu_orientation_covariance', 'imu_angular_velocity_covariance',
            'imu_linear_acceleration_covariance', 'odom_twist_covariance',
            'publish_battery', 'battery_topic',
        ):
            self.assertIn(f"declare_parameter('{name}'", source)
            self.assertIn(f"DeclareLaunchArgument('{name}'", launch)
            self.assertIn(f"DeclareLaunchArgument('{name}'", odom_view_launch)
        for removed_name in (
            'vehicle_profile', 'profile_schema', 'wheelbase', 'max_speed', 'speed_mode',
            'max_steering_angle', 'battery_voltage_min', 'battery_voltage_max',
        ):
            self.assertNotIn(f"declare_parameter('{removed_name}'", source)
            self.assertNotIn(f"DeclareLaunchArgument('{removed_name}'", launch)
            self.assertNotIn(f"DeclareLaunchArgument('{removed_name}'", odom_view_launch)
        self.assertNotIn("'profile_file'", launch)
        self.assertNotIn('config/vehicles', (REPO_ROOT / 'setup.py').read_text(encoding='utf-8'))
        for legacy_name in (
            'port_name', 'baud_rate', 'odom_frame', 'base_frame', 'imu_frame',
            'max_steering_angle_deg', 'cmd_watchdog_timeout_s', 'reconnect_interval_s',
            'firmware_version_timeout_s', 'link_status_enabled', 'link_ping_period_s',
        ):
            self.assertNotIn(f"declare_parameter('{legacy_name}'", source)
            self.assertNotIn(f"DeclareLaunchArgument('{legacy_name}'", launch)
        self.assertIn('ParameterValue(', launch)
        self.assertEqual(launch.count('value_type=list[float]'), 4)
        self.assertIn("DeclareLaunchArgument('protocol_mode', default_value='modern')", launch)
        self.assertIn("declare_parameter('firmware_version_timeout', 0.3)", source)
        self.assertIn("DeclareLaunchArgument('firmware_version_timeout', default_value='0.3')", launch)
        package_xml = (REPO_ROOT / 'package.xml').read_text(encoding='utf-8')
        self.assertIn('<depend>std_msgs</depend>', package_xml)
        self.assertIn('<build_type>ament_python</build_type>', package_xml)
        self.assertNotIn('<buildtool_depend>ament_python</buildtool_depend>', package_xml)
        self.assertIn('<maintainer email="winter@osrbot.com">osrbot</maintainer>', package_xml)
        setup_source = (REPO_ROOT / 'setup.py').read_text(encoding='utf-8')
        self.assertIn("maintainer_email='winter@osrbot.com'", setup_source)

    def test_readmes_document_runtime_capability_adaptation(self):
        readmes = {
            'README.md': 'docs/vehicle_capability_contract.md',
            'README_zh.md': 'docs/vehicle_capability_contract_zh.md',
        }
        for filename, contract_doc in readmes.items():
            text = (REPO_ROOT / filename).read_text(encoding='utf-8')
            self.assertIn('vehicle get', text)
            self.assertIn(contract_doc, text)
            self.assertNotIn('config/vehicles/', text)
            self.assertNotIn('vehicle_profile:=', text)
            self.assertNotIn('battery_voltage_min', text)
            self.assertNotIn('battery_voltage_max', text)
            for legacy, canonical in (
                ('port_name', 'port'),
                ('baud_rate', 'baudrate'),
                ('odom_frame', 'odom_frame_id'),
                ('base_frame', 'base_frame_id'),
                ('imu_frame', 'imu_frame_id'),
                ('cmd_watchdog_timeout_s', 'cmd_timeout'),
                ('reconnect_interval_s', 'reconnect_interval'),
                ('firmware_version_timeout_s', 'firmware_version_timeout'),
                ('link_status_enabled', 'connection_status_enabled'),
                ('link_ping_period_s', 'connection_refresh_period'),
                ('mag_frame', 'mag_frame_id'),
            ):
                self.assertRegex(text, rf'`{legacy}`\s*\|\s*`{canonical}`')

    def test_tf_launch_and_frame_contract_are_unchanged(self):
        digest = hashlib.sha256(DESCRIPTION_LAUNCH_PATH.read_bytes()).hexdigest()
        self.assertEqual(digest, DESCRIPTION_LAUNCH_SHA256)

        launch = (REPO_ROOT / 'launch/chassis_driver.launch.py').read_text(encoding='utf-8')
        odom_view = (REPO_ROOT / 'launch/odom_view.launch.py').read_text(encoding='utf-8')
        for name, default in (
            ('odom_frame_id', 'odom'),
            ('base_frame_id', 'base_footprint'),
            ('imu_frame_id', 'imu_link'),
            ('mag_frame_id', 'imu_link'),
        ):
            declaration = f"DeclareLaunchArgument('{name}', default_value='{default}')"
            self.assertIn(declaration, launch)
            self.assertIn(declaration, odom_view)
            self.assertIn(f"'{name}': LaunchConfiguration('{name}')", launch)
            self.assertIn(f"'{name}': LaunchConfiguration('{name}')", odom_view)


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.driver = _driver_shell()
        self.driver.publish_motion_state = mock.Mock()
        self.driver.publish_battery = mock.Mock()
        self.driver.publish_rc_data = mock.Mock()
        self.driver.publish_magnetometer = mock.Mock()
        self.driver.update_control_source = mock.Mock()
        self.data = _fixture()

    def test_dispatches_sanitized_proto_frames(self):
        telemetry = self.data['telemetry']
        self.driver.handle_device_line(telemetry['sync'])
        self.driver.handle_device_line(telemetry['battery'])
        self.driver.handle_device_line(telemetry['remote'])
        self.driver.handle_device_line(telemetry['magnetometer'])

        self.driver.publish_motion_state.assert_called_once()
        self.driver.publish_battery.assert_called_once_with(12.10)
        self.driver.publish_rc_data.assert_called_once()
        self.driver.publish_magnetometer.assert_called_once()
        self.driver.update_control_source.assert_called_once()

    def test_ignores_unknown_and_non_baseline_frames(self):
        for line in self.data['telemetry']['ignored_responses']:
            self.driver.handle_device_line(line)
        for index in (0, 1, 3):
            self.driver.handle_device_line(self.data['malformed'][index])

        self.driver.publish_motion_state.assert_not_called()
        self.driver.publish_battery.assert_not_called()
        self.driver.publish_rc_data.assert_not_called()
        self.driver.publish_magnetometer.assert_not_called()
        self.driver.update_control_source.assert_not_called()

    def test_bad_battery_value_is_logged_without_raising(self):
        self.driver.handle_device_line(self.data['malformed'][2])
        self.assertEqual(len(self.driver.logger.warnings), 1)

    def test_project_version_parser_accepts_proto_1_1_fixture(self):
        line = self.data['startup']['firmware_version_response']
        self.assertEqual(DRIVER.ChassisDriver.parse_project_version(line), 'PUBLIC_TEST_FIXTURE')
        self.assertEqual(
            DRIVER.ChassisDriver.parse_firmware_version(line),
            ('PUBLIC_TEST_FIXTURE', '1.1'),
        )

    def test_profile_status_parser_accepts_exact_contract(self):
        self.assertEqual(
            DRIVER.ChassisDriver.parse_profile_status(
                self.data['startup']['firmware_profile_response']
            ),
            {
                'id': 'fixture_alpha',
                'schema': 7,
                'state': 'READY',
                'motion': True,
                'writes': True,
            },
        )
        self.assertIsNone(DRIVER.ChassisDriver.parse_profile_status('PROFILE: ID=fixture_alpha'))

    def test_vehicle_capability_parser_accepts_exact_contract_and_converts_units(self):
        """Verify that vehicle capability parser accepts exact contract and converts
        units."""
        capabilities = DRIVER.ChassisDriver.parse_vehicle_capabilities(
            self.data['startup']['vehicle_capability_response'],
            expected_profile='fixture_alpha',
            expected_schema=7,
        )

        self.assertEqual(capabilities.profile, 'fixture_alpha')
        self.assertEqual(capabilities.schema, 7)
        self.assertAlmostEqual(capabilities.wheelbase_m, 1.375)
        self.assertAlmostEqual(capabilities.forward_max_mps, 4.321)
        self.assertAlmostEqual(capabilities.reverse_max_mps, 1.234)
        self.assertAlmostEqual(capabilities.steering_max_rad, math.radians(27.123))
        self.assertAlmostEqual(capabilities.battery_min_v, 11.111)
        self.assertAlmostEqual(capabilities.battery_max_v, 14.999)

    def test_vehicle_capability_parser_accepts_s1_kconfig_boundaries(self):
        self.assertEqual(DRIVER.MAX_WHEELBASE_MM, 9_999)
        self.assertEqual(DRIVER.MAX_SPEED_MMPS, 20_000)
        self.assertEqual(DRIVER.MAX_STEERING_MDEG, 90_000)
        self.assertEqual(DRIVER.MAX_BATTERY_MV, 60_000)
        boundary_lines = (
            _vehicle_response(
                WheelbaseMm=1,
                ForwardMaxMmps=1,
                ReverseMaxMmps=1,
                SteeringMaxMdeg=1,
                BatteryMinMv=0,
                BatteryMaxMv=1,
            ),
            _vehicle_response(
                WheelbaseMm=9_999,
                ForwardMaxMmps=20_000,
                ReverseMaxMmps=20_000,
                SteeringMaxMdeg=90_000,
                BatteryMinMv=59_999,
                BatteryMaxMv=60_000,
            ),
        )

        for line in boundary_lines:
            with self.subTest(line=line):
                self.assertIsNotNone(
                    DRIVER.ChassisDriver.parse_vehicle_capabilities(
                        line,
                        expected_profile='fixture_alpha',
                        expected_schema=7,
                    )
                )

    def test_vehicle_capability_parser_rejects_malformed_binding_and_ranges(self):
        """Verify that vehicle capability parser rejects malformed binding and
        ranges."""
        valid = _vehicle_response()
        malformed = [
            valid.rsplit(', BatteryMaxMv=', 1)[0],
            valid + ', Extra=1',
            valid.replace(
                ', ReverseMaxMmps=1234',
                ', ForwardMaxMmps=4321, ReverseMaxMmps=1234',
            ),
            valid.replace(
                'WheelbaseMm=1375, ForwardMaxMmps=4321',
                'ForwardMaxMmps=4321, WheelbaseMm=1375',
            ),
            _vehicle_response(WheelbaseMm='+1375'),
            _vehicle_response(WheelbaseMm='-1375'),
            _vehicle_response(WheelbaseMm='1375.0'),
            _vehicle_response(WheelbaseMm=''),
            _vehicle_response(Contract=2),
            _vehicle_response(Profile='fixture_beta'),
            _vehicle_response(Schema=8),
            _vehicle_response(WheelbaseMm=0),
            _vehicle_response(ForwardMaxMmps=0),
            _vehicle_response(ReverseMaxMmps=0),
            _vehicle_response(SteeringMaxMdeg=0),
            _vehicle_response(BatteryMinMv=14999, BatteryMaxMv=14999),
            _vehicle_response(BatteryMinMv=15000, BatteryMaxMv=14999),
            _vehicle_response(WheelbaseMm=DRIVER.MAX_WHEELBASE_MM + 1),
            _vehicle_response(ForwardMaxMmps=DRIVER.MAX_SPEED_MMPS + 1),
            _vehicle_response(ReverseMaxMmps=DRIVER.MAX_SPEED_MMPS + 1),
            _vehicle_response(SteeringMaxMdeg=DRIVER.MAX_STEERING_MDEG + 1),
            _vehicle_response(BatteryMaxMv=DRIVER.MAX_BATTERY_MV + 1),
            'UNKNOWN: synthetic response',
        ]

        for line in malformed:
            with self.subTest(line=line):
                self.assertIsNone(
                    DRIVER.ChassisDriver.parse_vehicle_capabilities(
                        line,
                        expected_profile='fixture_alpha',
                        expected_schema=7,
                    )
                )

        self.assertIsNone(
            DRIVER.ChassisDriver.parse_vehicle_capabilities(
                _vehicle_response(Schema=0),
                expected_profile='fixture_alpha',
                expected_schema=0,
            )
        )

    def test_non_telemetry_response_prefixes_are_ignored(self):
        for line in ('FW_VERSION: synthetic', 'LINK: synthetic', 'ERROR_DETAIL: synthetic', 'link pong ros'):
            self.driver.handle_device_line(line)
        self.assertEqual(self.driver.logger.warnings, [])
        self.driver.publish_motion_state.assert_not_called()
        self.driver.publish_rc_data.assert_not_called()


class SensorPublicationTests(unittest.TestCase):
    def setUp(self):
        self.driver = _driver_shell()
        self.driver.clock = _Clock(123)
        self.driver.get_clock = lambda: self.driver.clock
        self.data = _fixture()

    def configure_motion_publishers(self):
        covariance = self.data['covariance_diagonals']
        self.driver.odom_frame_id = 'odom'
        self.driver.base_frame_id = 'base_footprint'
        self.driver.imu_frame_id = 'imu_link'
        self.driver.tf_broadcaster = mock.Mock()
        self.driver.odom_pub = mock.Mock()
        self.driver.imu_pub = mock.Mock()
        self.driver.odom_twist_covariance = DRIVER.ChassisDriver.diagonal_covariance_6d(
            covariance['odom_twist']
        )
        self.driver.imu_orientation_covariance = DRIVER.ChassisDriver.diagonal_covariance(
            covariance['imu_orientation']
        )
        self.driver.imu_angular_velocity_covariance = DRIVER.ChassisDriver.diagonal_covariance(
            covariance['imu_angular_velocity']
        )
        self.driver.imu_linear_acceleration_covariance = DRIVER.ChassisDriver.diagonal_covariance(
            covariance['imu_linear_acceleration']
        )

    def test_rc_frame_publishes_all_channels(self):
        self.driver.publish_rc_enabled = True
        self.driver.rc_pub = mock.Mock()
        parts = self.data['telemetry']['remote'].split()

        self.driver.publish_rc_data(parts)

        message = self.driver.rc_pub.publish.call_args.args[0]
        self.assertEqual(message.data, [int(value) for value in parts[1:]])

    def test_disabled_rc_publication_keeps_publisher_quiet(self):
        self.driver.publish_rc_enabled = False
        self.driver.rc_pub = mock.Mock()

        self.driver.publish_rc_data(self.data['telemetry']['remote'].split())

        self.driver.rc_pub.publish.assert_not_called()

    def test_magnetometer_converts_gauss_to_tesla(self):
        self.driver.publish_mag_enabled = True
        self.driver.mag_frame_id = 'imu_link'
        self.driver.mag_pub = mock.Mock()

        self.driver.publish_magnetometer(self.data['telemetry']['magnetometer'].split())

        message = self.driver.mag_pub.publish.call_args.args[0]
        self.assertEqual(message.header.stamp, 123)
        self.assertEqual(message.header.frame_id, 'imu_link')
        self.assertAlmostEqual(message.magnetic_field.x, 0.25e-4)
        self.assertAlmostEqual(message.magnetic_field.y, -0.5e-4)
        self.assertAlmostEqual(message.magnetic_field.z, 1.0e-4)

    def test_sync_frame_maps_every_field_with_one_timestamp(self):
        self.configure_motion_publishers()
        self.driver.publish_motion_state(
            's 1 2 3 4 5 6 7 0.1 0.2 0.3 0.4 8 9 10 11 12 13'.split()
        )

        odom = self.driver.odom_pub.publish.call_args.args[0]
        imu = self.driver.imu_pub.publish.call_args.args[0]
        transform = self.driver.tf_broadcaster.sendTransform.call_args.args[0]

        self.assertEqual(odom.header.stamp, 123)
        self.assertEqual(imu.header.stamp, odom.header.stamp)
        self.assertEqual(transform.header.stamp, odom.header.stamp)
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_footprint')
        self.assertEqual(transform.header.frame_id, 'odom')
        self.assertEqual(transform.child_frame_id, 'base_footprint')
        self.assertEqual(
            (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
            (1.0, 2.0, 3.0),
        )
        self.assertEqual(
            (odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z),
            (4.0, 5.0, 6.0),
        )
        self.assertEqual(
            (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ),
            (0.1, 0.2, 0.3, 0.4),
        )
        self.assertEqual(
            (imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z),
            (8.0, 9.0, 10.0),
        )
        self.assertEqual(
            (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z),
            (11.0, 12.0, 13.0),
        )
        self.assertEqual(
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            (1.0, 2.0, 3.0),
        )
        self.assertEqual(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
            (0.1, 0.2, 0.3, 0.4),
        )
        self.assertEqual(odom.twist.covariance, self.driver.odom_twist_covariance)
        self.assertEqual(imu.orientation_covariance, self.driver.imu_orientation_covariance)
        self.assertEqual(imu.angular_velocity_covariance, self.driver.imu_angular_velocity_covariance)
        self.assertEqual(imu.linear_acceleration_covariance, self.driver.imu_linear_acceleration_covariance)

    def test_sync_frame_publication_is_rate_limited(self):
        self.configure_motion_publishers()
        self.driver.telemetry_publish_rate_hz = 50.0
        self.driver.last_motion_publish_monotonic = None
        parts = 's 1 2 3 4 5 6 7 0.1 0.2 0.3 0.4 8 9 10 11 12 13'.split()

        with mock.patch.object(
            DRIVER.time,
            'monotonic',
            side_effect=[1.0, 1.005, 1.021],
        ):
            self.driver.publish_motion_state(parts)
            self.driver.publish_motion_state(parts)
            self.driver.publish_motion_state(parts)

        self.assertEqual(self.driver.odom_pub.publish.call_count, 2)
        self.assertEqual(self.driver.imu_pub.publish.call_count, 2)
        self.assertEqual(self.driver.tf_broadcaster.sendTransform.call_count, 2)

    def test_legacy_odom_and_imu_frames_publish_standard_messages(self):
        self.configure_motion_publishers()
        self.driver.publish_legacy_imu(
            'i 0.1 0.2 0.3 0.9 1.0 2.0 3.0 0.01 0.02 0.03'.split()
        )
        self.driver.publish_legacy_odometry('o 1.0 2.0 0.0 0.4 0.0 0.0 0.5'.split())

        imu = self.driver.imu_pub.publish.call_args.args[0]
        odom = self.driver.odom_pub.publish.call_args.args[0]
        self.assertEqual(imu.header.frame_id, 'imu_link')
        self.assertEqual(imu.angular_velocity.z, 0.03)
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_footprint')
        self.assertEqual(odom.twist.twist.linear.x, 0.4)
        self.assertEqual(odom.twist.twist.angular.z, 0.03)
        self.assertAlmostEqual(odom.pose.pose.orientation.z, math.sin(0.25))
        self.assertAlmostEqual(odom.pose.pose.orientation.w, math.cos(0.25))

    def test_legacy_imu_publication_is_rate_limited(self):
        self.configure_motion_publishers()
        self.driver.imu_publish_rate_hz = 50.0
        self.driver.last_imu_publish_monotonic = None
        parts = 'i 0.1 0.2 0.3 0.9 1.0 2.0 3.0 0.01 0.02 0.03'.split()

        with mock.patch.object(
            DRIVER.time,
            'monotonic',
            side_effect=[1.0, 1.005, 1.021],
        ):
            self.driver.publish_legacy_imu(parts)
            self.driver.publish_legacy_imu(parts)
            self.driver.publish_legacy_imu(parts)

        self.assertEqual(self.driver.imu_pub.publish.call_count, 2)
        self.assertEqual(self.driver.last_legacy_gyro_z, 0.03)

    def test_nonfinite_sync_values_drop_the_entire_frame(self):
        self.configure_motion_publishers()
        valid = 's 1 2 3 4 5 6 7 0.1 0.2 0.3 0.4 8 9 10 11 12 13'.split()
        for token in ('nan', 'inf', '-inf'):
            for index in range(1, len(valid)):
                with self.subTest(token=token, index=index):
                    parts = list(valid)
                    parts[index] = token
                    self.driver.publish_motion_state(parts)
                    self.driver.odom_pub.publish.assert_not_called()
                    self.driver.imu_pub.publish.assert_not_called()
                    self.driver.tf_broadcaster.sendTransform.assert_not_called()

    def test_nonfinite_mag_and_battery_values_are_not_published(self):
        self.driver.publish_mag_enabled = True
        self.driver.mag_pub = mock.Mock()
        self.driver.publish_battery_enabled = True
        self.driver.battery_pub = mock.Mock()
        _bind_synthetic_capabilities(self.driver)

        self.driver.publish_magnetometer('m nan 0.0 0.0'.split())
        self.driver.publish_battery(math.inf)

        self.driver.mag_pub.publish.assert_not_called()
        self.driver.battery_pub.publish.assert_not_called()

    def test_nonfinite_warning_is_counted_and_rate_limited(self):
        self.configure_motion_publishers()
        invalid = 's nan 2 3 4 5 6 7 0.1 0.2 0.3 0.4 8 9 10 11 12 13'.split()
        with mock.patch.object(DRIVER.time, 'monotonic', side_effect=[0.0, 1.0, 6.0]):
            for _ in range(3):
                self.driver.publish_motion_state(invalid)

        self.assertEqual(
            self.driver.logger.warnings,
            [
                "Dropped non-finite 's' telemetry frame (count=1)",
                "Dropped non-finite 's' telemetry frame (count=3)",
            ],
        )

    def test_covariance_helpers_reject_wrong_lengths(self):
        with self.assertRaisesRegex(ValueError, 'exactly 3'):
            DRIVER.ChassisDriver.diagonal_covariance([1.0, 2.0])
        with self.assertRaisesRegex(ValueError, 'exactly 6'):
            DRIVER.ChassisDriver.diagonal_covariance_6d([1.0] * 5)

    def test_battery_publish_switch_is_honored_by_parser(self):
        self.driver.publish_motion_state = mock.Mock()
        self.driver.publish_rc_data = mock.Mock()
        self.driver.publish_magnetometer = mock.Mock()
        self.driver.update_control_source = mock.Mock()
        self.driver.publish_battery = mock.Mock()
        self.driver.publish_battery_enabled = False
        self.driver.battery_pub = None

        self.driver.handle_device_line(self.data['telemetry']['battery'])

        self.driver.publish_battery.assert_not_called()

    def test_battery_message_uses_controller_capability_range(self):
        self.driver.publish_battery_enabled = True
        self.driver.battery_pub = mock.Mock()
        _bind_synthetic_capabilities(self.driver)

        self.driver.publish_battery(13.055)

        message = self.driver.battery_pub.publish.call_args.args[0]
        self.assertEqual(message.header.stamp, 123)
        self.assertEqual(message.voltage, 13.055)
        self.assertAlmostEqual(message.percentage, 0.5)
        self.assertTrue(message.present)

    def test_battery_is_not_published_before_capability_handshake(self):
        self.driver.publish_battery_enabled = True
        self.driver.battery_pub = mock.Mock()

        self.driver.publish_battery(12.0)

        self.driver.battery_pub.publish.assert_not_called()


class ConnectionLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.driver = _driver_shell()
        self.data = _fixture()

    def test_startup_sequence_matches_fixture(self):
        serial_conn = _FakeSerial(
            self.data['startup']['firmware_version_response'],
            self.data['startup']['firmware_profile_response'],
            self.data['startup']['vehicle_capability_response'],
        )
        with mock.patch.object(DRIVER.serial, 'Serial', return_value=serial_conn), mock.patch.object(
            DRIVER.time, 'sleep', return_value=None
        ):
            self.driver.ensure_connected()

        self.assertEqual(serial_conn.writes, self.data['startup']['expected_host_commands'])
        self.driver.start_reader.assert_called_once_with()
        self.assertTrue(self.driver.capability_ready)
        self.assertIs(self.driver.capability_conn, serial_conn)
        self.assertAlmostEqual(self.driver.vehicle_capabilities.forward_max_mps, 4.321)
        self.assertAlmostEqual(self.driver.vehicle_capabilities.reverse_max_mps, 1.234)
        self.assertIn('Vehicle capability contract accepted', self.driver.logger.infos)
        log_text = '\n'.join(self.driver.logger.infos + self.driver.logger.warnings)
        for private_value in ('1375', '4321', '1234', '27123', '11111', '14999'):
            self.assertNotIn(private_value, log_text)
        self.assertTrue(any(message.startswith('Connected to chassis') for message in self.driver.logger.infos))

    def test_identity_reader_skips_unrelated_lines_before_exact_response(self):
        serial_conn = _FakeSerial()
        serial_conn.lines.extend((
            b'OK: previous command\n',
            b'DIAG: synthetic notice\n',
            (self.data['startup']['firmware_version_response'] + '\n').encode('utf-8'),
        ))
        self.driver.serial_conn = serial_conn

        result = self.driver.read_identity_response(
            self.driver.parse_firmware_version,
            serial_conn,
        )

        self.assertEqual(result, ('PUBLIC_TEST_FIXTURE', '1.1'))

    def test_failed_initialization_closes_connection_and_does_not_start_reader(self):
        """Verify that failed initialization closes connection and does not start
        reader."""
        for failed_command in ('profile get\n', 'vehicle get\n', 'stream sync\n', 's\n'):
            with self.subTest(failed_command=failed_command):
                driver = _driver_shell()
                serial_conn = _FakeSerial(
                    self.data['startup']['firmware_version_response'],
                    self.data['startup']['firmware_profile_response'],
                    self.data['startup']['vehicle_capability_response'],
                    fail_on_write=failed_command,
                )
                with mock.patch.object(DRIVER.serial, 'Serial', return_value=serial_conn), mock.patch.object(
                    DRIVER.time, 'sleep', return_value=None
                ):
                    driver.ensure_connected()

                self.assertIsNone(driver.serial_conn)
                self.assertFalse(serial_conn.is_open)
                self.assertEqual(serial_conn.close_count, 1)
                driver.start_reader.assert_not_called()
                self.assertFalse(any(message.startswith('Connected to chassis') for message in driver.logger.infos))

    def test_firmware_version_query_failure_blocks_connection(self):
        serial_conn = _FakeSerial(fail_on_write='stream off\n')
        with mock.patch.object(DRIVER.serial, 'Serial', return_value=serial_conn), mock.patch.object(
            DRIVER.time, 'sleep', return_value=None
        ):
            self.driver.ensure_connected()

        self.assertEqual(serial_conn.writes, ['link down ros\n'])
        self.assertFalse(serial_conn.is_open)
        self.assertIsNone(self.driver.serial_conn)
        self.driver.start_reader.assert_not_called()

    def test_protocol_profile_and_vehicle_contract_fail_before_stream(self):
        cases = (
            (
                'FW_VERSION: ProjectVer=PUBLIC_TEST_FIXTURE, Proto=2.0',
                self.data['startup']['firmware_profile_response'],
                self.data['startup']['vehicle_capability_response'],
                ['stream off\n', 'fw version\n', 'link down ros\n'],
            ),
            (
                self.data['startup']['firmware_version_response'],
                'PROFILE: ID=fixture_alpha, Schema=7, State=BACKUP_REQUIRED, Motion=No, Writes=No',
                self.data['startup']['vehicle_capability_response'],
                ['stream off\n', 'fw version\n', 'profile get\n', 'link down ros\n'],
            ),
            (
                self.data['startup']['firmware_version_response'],
                self.data['startup']['firmware_profile_response'],
                _vehicle_response(Profile='fixture_beta'),
                [
                    'stream off\n', 'fw version\n', 'profile get\n', 'vehicle get\n',
                    'link down ros\n',
                ],
            ),
            (
                self.data['startup']['firmware_version_response'],
                self.data['startup']['firmware_profile_response'],
                _vehicle_response(Schema=8),
                [
                    'stream off\n', 'fw version\n', 'profile get\n', 'vehicle get\n',
                    'link down ros\n',
                ],
            ),
        )
        for version_line, profile_line, vehicle_line, expected_writes in cases:
            with self.subTest(vehicle_line=vehicle_line):
                driver = _driver_shell()
                serial_conn = _FakeSerial(version_line, profile_line, vehicle_line)
                with mock.patch.object(
                    DRIVER.serial, 'Serial', return_value=serial_conn
                ), mock.patch.object(DRIVER.time, 'sleep', return_value=None):
                    driver.ensure_connected()

                self.assertEqual(serial_conn.writes, expected_writes)
                self.assertFalse(serial_conn.is_open)
                self.assertIsNone(driver.serial_conn)
                driver.start_reader.assert_not_called()

    def test_missing_or_unknown_vehicle_response_fails_closed(self):
        for vehicle_line in (None, 'UNKNOWN: synthetic response', 'VEHICLE: Contract=1'):
            with self.subTest(vehicle_line=vehicle_line):
                driver = _driver_shell()
                serial_conn = _FakeSerial(
                    self.data['startup']['firmware_version_response'],
                    self.data['startup']['firmware_profile_response'],
                    vehicle_line,
                )
                with mock.patch.object(
                    DRIVER.serial, 'Serial', return_value=serial_conn
                ), mock.patch.object(DRIVER.time, 'sleep', return_value=None):
                    driver.ensure_connected()

                self.assertFalse(driver.capability_ready)
                self.assertIsNone(driver.vehicle_capabilities)
                self.assertIsNone(driver.serial_conn)
                self.assertFalse(serial_conn.is_open)
                driver.start_reader.assert_not_called()

    def test_missing_serial_path_keeps_driver_available_for_retry(self):
        self.driver.port = '/dev/definitely-not-present-osracer-base'
        with mock.patch.object(DRIVER.os.path, 'exists', return_value=False), mock.patch.object(
            DRIVER.serial, 'Serial'
        ) as serial_factory:
            self.driver.ensure_connected()

        self.assertFalse(self.driver.shutdown_event.is_set())
        self.assertIsNone(self.driver.serial_conn)
        self.assertFalse(self.driver.capability_ready)
        serial_factory.assert_not_called()
        self.driver.start_reader.assert_not_called()

    def test_new_device_open_clears_stale_capabilities_before_handshake(self):
        stale_conn = _bind_synthetic_capabilities(self.driver)
        stale_conn.is_open = False
        self.driver.serial_conn = None
        replacement_conn = _FakeSerial(fail_on_write='stream off\n')

        with mock.patch.object(
            DRIVER.serial, 'Serial', return_value=replacement_conn
        ), mock.patch.object(DRIVER.time, 'sleep', return_value=None):
            self.driver.ensure_connected()

        self.assertFalse(self.driver.capability_ready)
        self.assertIsNone(self.driver.vehicle_capabilities)
        self.assertIsNone(self.driver.capability_conn)
        self.assertIsNone(self.driver.serial_conn)
        self.assertFalse(replacement_conn.is_open)

    def test_write_failure_closes_exact_failed_connection(self):
        serial_conn = _FakeSerial(fail_on_write='synthetic\n')
        self.driver.serial_conn = serial_conn

        self.assertFalse(self.driver.write_raw('synthetic\n'))
        self.assertIsNone(self.driver.serial_conn)
        self.assertFalse(serial_conn.is_open)
        self.assertEqual(serial_conn.close_count, 1)

    def test_disappeared_device_is_closed_before_reconnect_attempt(self):
        serial_conn = _FakeSerial()
        self.driver.port = '/dev/osrbot_base'
        self.driver.serial_conn = serial_conn
        with mock.patch.object(DRIVER.os.path, 'exists', return_value=False), mock.patch.object(
            DRIVER.serial, 'Serial'
        ) as serial_factory:
            self.driver.ensure_connected()

        self.assertIsNone(self.driver.serial_conn)
        self.assertFalse(serial_conn.is_open)
        self.driver.start_reader.assert_not_called()
        serial_factory.assert_not_called()

    def test_shutdown_sends_link_down_before_close(self):
        serial_conn = _FakeSerial()
        _bind_synthetic_capabilities(self.driver, serial_conn)
        self.driver.close_serial()

        self.assertEqual(serial_conn.writes, [self.data['startup']['shutdown_host_command']])
        self.assertFalse(serial_conn.is_open)
        self.assertFalse(self.driver.capability_ready)
        self.assertIsNone(self.driver.vehicle_capabilities)

    def test_expected_shutdown_read_error_is_silent_and_does_not_close_again(self):
        """Verify that expected shutdown read error is silent and does not close
        again."""
        serial_conn = _FakeSerial()
        self.driver.serial_conn = serial_conn
        self.driver.close_serial = mock.Mock()

        def fail_during_shutdown():
            self.driver.shutdown_event.set()
            raise TypeError('synthetic expected shutdown read failure')

        serial_conn.readline = fail_during_shutdown
        self.driver.read_loop()

        self.assertEqual(self.driver.logger.warnings, [])
        self.driver.close_serial.assert_not_called()

    def test_shutdown_waits_for_reader_and_preserves_link_down_cleanup(self):
        serial_conn = _FakeSerial()
        reader = mock.Mock()
        reader.is_alive.return_value = True
        _bind_synthetic_capabilities(self.driver, serial_conn)
        self.driver.reader_thread = reader

        self.driver.shutdown_driver()

        self.assertTrue(self.driver.shutdown_event.is_set())
        self.assertEqual(
            serial_conn.writes,
            ['v 0.000 0.00\n', self.data['startup']['shutdown_host_command']],
        )
        self.assertFalse(serial_conn.is_open)
        reader.join.assert_called_once_with(timeout=1.0)

    def test_shutdown_prevents_timer_reconnect(self):
        self.driver.shutdown_event.set()

        with mock.patch.object(DRIVER.serial, 'Serial') as serial_factory:
            self.driver.ensure_connected()

        serial_factory.assert_not_called()
        self.driver.start_reader.assert_not_called()

    def test_close_cleanup_swallows_nonstandard_close_error(self):
        serial_conn = _FakeSerial()
        serial_conn.close = mock.Mock(side_effect=RuntimeError('synthetic close failure'))
        self.driver.serial_conn = serial_conn

        self.driver.close_serial()

        self.assertIsNone(self.driver.serial_conn)
        serial_conn.close.assert_called_once_with()

    def test_connected_device_gets_periodic_link_ping(self):
        serial_conn = _FakeSerial()
        _bind_synthetic_capabilities(self.driver, serial_conn)

        self.driver.refresh_connection_status()

        self.assertEqual(serial_conn.writes, [self.data['startup']['periodic_host_command']])

    def test_stale_reader_closes_only_its_captured_connection(self):
        stale_conn = _FakeSerial()
        replacement_conn = _FakeSerial()
        _bind_synthetic_capabilities(self.driver, stale_conn)

        def fail_after_reconnect():
            _bind_synthetic_capabilities(self.driver, replacement_conn)
            raise TypeError('synthetic stale reader failure')

        stale_conn.readline = fail_after_reconnect
        self.driver.read_loop()

        self.assertIs(self.driver.serial_conn, replacement_conn)
        self.assertFalse(stale_conn.is_open)
        self.assertTrue(replacement_conn.is_open)
        self.assertTrue(self.driver.capability_ready)
        self.assertIs(self.driver.capability_conn, replacement_conn)

    def test_current_connection_failure_clears_bound_capabilities(self):
        serial_conn = _FakeSerial(fail_on_write='synthetic\n')
        _bind_synthetic_capabilities(self.driver, serial_conn)

        self.assertFalse(self.driver.write_raw('synthetic\n'))

        self.assertIsNone(self.driver.serial_conn)
        self.assertFalse(self.driver.capability_ready)
        self.assertIsNone(self.driver.vehicle_capabilities)
        self.assertIsNone(self.driver.capability_conn)

    def test_reader_finally_preserves_replacement_under_serial_lock(self):
        class CountingLock:
            def __init__(self):
                self.lock = threading.Lock()
                self.enter_count = 0

            def __enter__(self):
                self.lock.acquire()
                self.enter_count += 1
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                self.lock.release()

        serial_conn = _FakeSerial()
        replacement_reader = object()
        counting_lock = CountingLock()
        self.driver.serial_lock = counting_lock
        self.driver.serial_conn = serial_conn
        self.driver.reader_thread = threading.current_thread()

        def fail_after_reader_replacement():
            self.driver.reader_thread = replacement_reader
            raise TypeError('synthetic reader replacement')

        serial_conn.readline = fail_after_reader_replacement
        self.driver.read_loop()

        self.assertIs(self.driver.reader_thread, replacement_reader)
        self.assertEqual(counting_lock.enter_count, 3)

    def test_type_error_on_write_fails_closed(self):
        serial_conn = _FakeSerial()
        serial_conn.write = mock.Mock(side_effect=TypeError('synthetic write type error'))
        self.driver.serial_conn = serial_conn

        self.assertFalse(self.driver.write_raw('synthetic\n'))
        self.assertIsNone(self.driver.serial_conn)
        self.assertFalse(serial_conn.is_open)

    def test_write_failure_cleanup_swallows_nonstandard_close_error(self):
        serial_conn = _FakeSerial(fail_on_write='synthetic\n')
        serial_conn.close = mock.Mock(side_effect=RuntimeError('synthetic close failure'))
        self.driver.serial_conn = serial_conn

        self.assertFalse(self.driver.write_raw('synthetic\n'))

        self.assertIsNone(self.driver.serial_conn)
        serial_conn.close.assert_called_once_with()


class MainLifecycleTests(unittest.TestCase):
    def test_repeated_sigint_is_ignored_during_cleanup(self):
        node = mock.Mock()
        with mock.patch.object(DRIVER, 'ChassisDriver', return_value=node), mock.patch.object(
            DRIVER.rclpy, 'init', create=True
        ), mock.patch.object(
            DRIVER.rclpy, 'spin', side_effect=KeyboardInterrupt(), create=True
        ), mock.patch.object(
            DRIVER.rclpy, 'ok', return_value=False
        ), mock.patch.object(
            DRIVER.rclpy, 'shutdown', create=True
        ), mock.patch.object(signal, 'signal') as set_signal:
            node.shutdown_driver.side_effect = lambda: self.assertEqual(
                set_signal.call_args,
                mock.call(signal.SIGINT, signal.SIG_IGN),
            )

            DRIVER.main()

        node.shutdown_driver.assert_called_once_with()

    def test_external_shutdown_is_clean_and_does_not_shutdown_context_twice(self):
        """Verify that external shutdown is clean and does not shutdown context
        twice."""
        node = mock.Mock()
        with mock.patch.object(DRIVER, 'ChassisDriver', return_value=node), mock.patch.object(
            DRIVER.rclpy, 'init', create=True
        ), mock.patch.object(
            DRIVER.rclpy, 'spin', side_effect=_ExternalShutdownException(), create=True
        ), mock.patch.object(
            DRIVER.rclpy, 'ok', return_value=False
        ), mock.patch.object(
            DRIVER.rclpy, 'shutdown', create=True
        ) as shutdown:
            DRIVER.main()

        node.shutdown_driver.assert_called_once_with()
        node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()

    def test_already_shutdown_context_is_not_shutdown_again(self):
        node = mock.Mock()
        with mock.patch.object(DRIVER, 'ChassisDriver', return_value=node), mock.patch.object(
            DRIVER.rclpy, 'init', create=True
        ), mock.patch.object(
            DRIVER.rclpy, 'spin', create=True
        ), mock.patch.object(
            DRIVER.rclpy, 'ok', return_value=False
        ), mock.patch.object(
            DRIVER.rclpy, 'shutdown', create=True
        ) as shutdown:
            DRIVER.main()

        node.shutdown_driver.assert_called_once_with()
        shutdown.assert_not_called()


class ControlMappingTests(unittest.TestCase):
    def setUp(self):
        self.driver = _driver_shell()
        self.serial_conn = _bind_synthetic_capabilities(self.driver)
        self.driver.clock = _Clock(123)
        self.driver.get_clock = lambda: self.driver.clock
        self.driver.last_cmd_time = _TimePoint(0)
        self.driver.write_raw = mock.Mock(return_value=True)

    def test_ackermann_drive_matches_accepted_serial_mapping(self):
        message = DRIVER.AckermannDriveStamped()
        message.drive.speed = 0.3
        message.drive.steering_angle = 0.1

        self.driver.ackermann_cmd_callback(message)

        self.driver.write_raw.assert_called_once_with(
            'v 0.300 5.73\n', expected_conn=self.serial_conn
        )

    def test_ackermann_uses_direction_specific_limits_and_steering_limit(self):
        message = DRIVER.AckermannDriveStamped()
        message.drive.speed = 9.0
        message.drive.steering_angle = math.radians(40.0)

        self.driver.ackermann_cmd_callback(message)

        self.driver.write_raw.assert_called_once_with(
            'v 4.321 27.12\n', expected_conn=self.serial_conn
        )
        self.assertEqual(self.driver.last_cmd_time.nanoseconds, 123)

        self.driver.write_raw.reset_mock()
        message.drive.speed = -9.0
        self.driver.ackermann_cmd_callback(message)
        self.driver.write_raw.assert_called_once_with(
            'v -1.234 27.12\n', expected_conn=self.serial_conn
        )

    def test_twist_maps_to_ackermann_steering(self):
        message = DRIVER.Twist()
        message.linear.x = 0.4
        message.angular.z = 0.1
        expected_degrees = math.degrees(math.atan(1.375 * 0.1 / 0.4))

        self.driver.cmd_vel_callback(message)

        self.driver.write_raw.assert_called_once_with(
            f'v 0.400 {expected_degrees:.2f}\n', expected_conn=self.serial_conn
        )

    def test_twist_clamps_speed_before_geometry_and_firmware_steering(self):
        message = DRIVER.Twist()
        message.linear.x = -8.0
        message.angular.z = 8.0

        self.driver.cmd_vel_callback(message)

        self.driver.write_raw.assert_called_once_with(
            'v -1.234 -27.12\n', expected_conn=self.serial_conn
        )

    def test_cmd_vel_near_zero_boundary(self):
        message = DRIVER.Twist()
        message.linear.x = 0.009
        message.angular.z = 0.0005

        self.driver.cmd_vel_callback(message)

        self.driver.write_raw.assert_called_once_with(
            'v 0.009 27.12\n', expected_conn=self.serial_conn
        )

        self.driver.write_raw.reset_mock()
        message.linear.x = 0.01
        expected_degrees = math.degrees(math.atan(1.375 * message.angular.z / message.linear.x))
        self.driver.cmd_vel_callback(message)
        self.driver.write_raw.assert_called_once_with(
            f'v 0.010 {expected_degrees:.2f}\n', expected_conn=self.serial_conn
        )

    def test_failed_command_does_not_refresh_watchdog(self):
        self.driver.write_raw.return_value = False
        previous = self.driver.last_cmd_time
        message = DRIVER.AckermannDriveStamped()
        message.drive.speed = 0.2

        self.driver.ackermann_cmd_callback(message)

        self.assertIs(self.driver.last_cmd_time, previous)

    def test_commands_before_handshake_are_not_sent(self):
        self.driver.capability_ready = False
        self.driver.vehicle_capabilities = None
        self.driver.capability_conn = None
        twist = DRIVER.Twist()
        twist.linear.x = 0.2
        ackermann = DRIVER.AckermannDriveStamped()
        ackermann.drive.speed = 0.2

        self.driver.cmd_vel_callback(twist)
        self.driver.ackermann_cmd_callback(ackermann)

        self.driver.write_raw.assert_not_called()

    def test_stale_capability_binding_cannot_send_after_device_swap(self):
        stale_binding = (
            self.driver.serial_conn,
            self.driver.vehicle_capabilities,
        )
        _bind_synthetic_capabilities(self.driver, _FakeSerial())

        self.driver.send_drive_command(0.2, 0.1, expected_binding=stale_binding)

        self.driver.write_raw.assert_not_called()


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        self.driver = _driver_shell()
        self.serial_conn = _bind_synthetic_capabilities(self.driver)
        self.driver.clock = _Clock()
        self.driver.get_clock = lambda: self.driver.clock
        self.driver.last_cmd_time = _TimePoint(0)
        self.driver.cmd_timeout = 0.5
        self.driver.write_raw = mock.Mock(return_value=True)

    def test_watchdog_stops_only_after_500_ms(self):
        self.driver.clock.nanoseconds = 499_000_000
        self.driver.watchdog_check()
        self.driver.write_raw.assert_not_called()

        self.driver.clock.nanoseconds = 501_000_000
        self.driver.watchdog_check()
        self.driver.write_raw.assert_called_once_with(
            'v 0.00 0.00\n', expected_conn=self.serial_conn
        )

    def test_watchdog_boundary_matches_validated_behavior(self):
        self.driver.clock.nanoseconds = 500_000_000
        self.driver.watchdog_check()
        self.driver.write_raw.assert_not_called()

    def test_watchdog_does_not_write_before_handshake(self):
        self.driver.clock.nanoseconds = 501_000_000
        self.driver.capability_ready = False
        self.driver.vehicle_capabilities = None
        self.driver.capability_conn = None

        self.driver.watchdog_check()

        self.driver.write_raw.assert_not_called()


class CiContractTests(unittest.TestCase):
    def test_workflow_is_read_only_and_runs_ros_build_tests(self):
        workflow = WORKFLOW_PATH.read_text(encoding='utf-8')
        self.assertIn('permissions:', workflow)
        self.assertIn('contents: read', workflow)
        self.assertEqual(workflow.count('branches: [main]'), 2)
        self.assertNotIn('branches: [main, ros2]', workflow)
        self.assertIn('humble', workflow)
        self.assertIn('jazzy', workflow)
        self.assertIn('ubuntu-22.04', workflow)
        self.assertIn('ubuntu-24.04', workflow)
        self.assertRegex(workflow, r'actions/checkout@[0-9a-f]{40}')
        self.assertRegex(workflow, r'ros-tooling/setup-ros@[0-9a-f]{40}')
        self.assertIn('persist-credentials: false', workflow)
        self.assertIn('rosdep install', workflow)
        self.assertIn('PYTHONPYCACHEPREFIX', workflow)
        self.assertIn('python3 -m compileall -q setup.py osracer_base launch test', workflow)
        self.assertNotIn('python3 -m py_compile', workflow)
        self.assertIn("ET.parse('package.xml')", workflow)
        self.assertIn("rglob('*.json')", workflow)
        self.assertNotIn("Path('config/vehicles')", workflow)
        self.assertNotIn('import yaml', workflow)
        self.assertIn(
            'uses: astral-sh/ruff-action@278981a28ce3188b1e39527901f38254bf3aac89',
            workflow,
        )
        self.assertRegex(workflow, r"version:\s*['\"]0\.15\.13['\"]")
        self.assertRegex(workflow, r'args:\s*check')
        self.assertRegex(workflow, r'src:\s*\.')
        self.assertIn('colcon build', workflow)
        self.assertIn('colcon test', workflow)
        self.assertIn('colcon test-result --verbose', workflow)
        for forbidden in ('git push', 'pull_request_target', 'create-pull-request', 'release: write'):
            self.assertNotIn(forbidden, workflow)


if __name__ == '__main__':
    unittest.main()
