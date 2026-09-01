# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    args = [
        DeclareLaunchArgument('port', default_value='/dev/osrbot_base'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('protocol_mode', default_value='modern'),
        DeclareLaunchArgument('legacy_wheelbase', default_value='0.325'),
        DeclareLaunchArgument('legacy_forward_max_speed', default_value='0.8'),
        DeclareLaunchArgument('legacy_reverse_max_speed', default_value='0.8'),
        DeclareLaunchArgument('legacy_max_steering_angle', default_value='0.5235987756'),
        DeclareLaunchArgument('legacy_battery_min_voltage', default_value='10.8'),
        DeclareLaunchArgument('legacy_battery_max_voltage', default_value='12.6'),
        DeclareLaunchArgument('legacy_status_period', default_value='1.0'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('reconnect_interval', default_value='2.0'),
        DeclareLaunchArgument('firmware_version_timeout', default_value='0.3'),
        DeclareLaunchArgument('connection_status_enabled', default_value='true'),
        DeclareLaunchArgument('connection_refresh_period', default_value='1.0'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('telemetry_publish_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('imu_publish_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('publish_rc', default_value='true'),
        DeclareLaunchArgument('rc_topic', default_value='rc_data'),
        DeclareLaunchArgument('publish_mag', default_value='true'),
        DeclareLaunchArgument('mag_topic', default_value='magnetometer_data'),
        DeclareLaunchArgument('mag_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('imu_orientation_covariance', default_value='[0.02, 0.02, 0.05]'),
        DeclareLaunchArgument('imu_angular_velocity_covariance', default_value='[0.01, 0.01, 0.01]'),
        DeclareLaunchArgument('imu_linear_acceleration_covariance', default_value='[0.10, 0.10, 0.10]'),
        DeclareLaunchArgument('odom_twist_covariance', default_value='[0.02, 0.20, 1.0, 1.0, 1.0, 0.30]'),
        DeclareLaunchArgument('publish_battery', default_value='true'),
        DeclareLaunchArgument('battery_topic', default_value='battery_state'),
    ]

    driver = Node(
        package='osracer_base',
        executable='chassis_driver',
        name='osracer_base',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'protocol_mode': LaunchConfiguration('protocol_mode'),
            'legacy_wheelbase': ParameterValue(
                LaunchConfiguration('legacy_wheelbase'), value_type=float
            ),
            'legacy_forward_max_speed': ParameterValue(
                LaunchConfiguration('legacy_forward_max_speed'), value_type=float
            ),
            'legacy_reverse_max_speed': ParameterValue(
                LaunchConfiguration('legacy_reverse_max_speed'), value_type=float
            ),
            'legacy_max_steering_angle': ParameterValue(
                LaunchConfiguration('legacy_max_steering_angle'), value_type=float
            ),
            'legacy_battery_min_voltage': ParameterValue(
                LaunchConfiguration('legacy_battery_min_voltage'), value_type=float
            ),
            'legacy_battery_max_voltage': ParameterValue(
                LaunchConfiguration('legacy_battery_max_voltage'), value_type=float
            ),
            'legacy_status_period': ParameterValue(
                LaunchConfiguration('legacy_status_period'), value_type=float
            ),
            'cmd_timeout': ParameterValue(LaunchConfiguration('cmd_timeout'), value_type=float),
            'reconnect_interval': ParameterValue(
                LaunchConfiguration('reconnect_interval'), value_type=float
            ),
            'firmware_version_timeout': ParameterValue(
                LaunchConfiguration('firmware_version_timeout'), value_type=float
            ),
            'connection_status_enabled': ParameterValue(
                LaunchConfiguration('connection_status_enabled'), value_type=bool
            ),
            'connection_refresh_period': ParameterValue(
                LaunchConfiguration('connection_refresh_period'), value_type=float
            ),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            'telemetry_publish_rate_hz': ParameterValue(
                LaunchConfiguration('telemetry_publish_rate_hz'), value_type=float
            ),
            'imu_publish_rate_hz': ParameterValue(
                LaunchConfiguration('imu_publish_rate_hz'), value_type=float
            ),
            'publish_rc': ParameterValue(LaunchConfiguration('publish_rc'), value_type=bool),
            'rc_topic': LaunchConfiguration('rc_topic'),
            'publish_mag': ParameterValue(LaunchConfiguration('publish_mag'), value_type=bool),
            'mag_topic': LaunchConfiguration('mag_topic'),
            'mag_frame_id': LaunchConfiguration('mag_frame_id'),
            'imu_orientation_covariance': ParameterValue(
                LaunchConfiguration('imu_orientation_covariance'), value_type=list[float]
            ),
            'imu_angular_velocity_covariance': ParameterValue(
                LaunchConfiguration('imu_angular_velocity_covariance'), value_type=list[float]
            ),
            'imu_linear_acceleration_covariance': ParameterValue(
                LaunchConfiguration('imu_linear_acceleration_covariance'), value_type=list[float]
            ),
            'odom_twist_covariance': ParameterValue(
                LaunchConfiguration('odom_twist_covariance'), value_type=list[float]
            ),
            'publish_battery': ParameterValue(
                LaunchConfiguration('publish_battery'), value_type=bool
            ),
            'battery_topic': LaunchConfiguration('battery_topic'),
        }],
    )

    return LaunchDescription(args + [driver])
