# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    package_share = FindPackageShare('osracer_base')

    args = [
        DeclareLaunchArgument('port', default_value='/dev/osrbot_base'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
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
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([package_share, 'rviz', 'odom_view.rviz']),
        ),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'chassis_driver.launch.py'])
        ),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'telemetry_publish_rate_hz': LaunchConfiguration(
                'telemetry_publish_rate_hz'
            ),
            'imu_publish_rate_hz': LaunchConfiguration('imu_publish_rate_hz'),
            'publish_rc': LaunchConfiguration('publish_rc'),
            'rc_topic': LaunchConfiguration('rc_topic'),
            'publish_mag': LaunchConfiguration('publish_mag'),
            'mag_topic': LaunchConfiguration('mag_topic'),
            'mag_frame_id': LaunchConfiguration('mag_frame_id'),
            'imu_orientation_covariance': LaunchConfiguration('imu_orientation_covariance'),
            'imu_angular_velocity_covariance': LaunchConfiguration('imu_angular_velocity_covariance'),
            'imu_linear_acceleration_covariance': LaunchConfiguration('imu_linear_acceleration_covariance'),
            'odom_twist_covariance': LaunchConfiguration('odom_twist_covariance'),
            'publish_battery': LaunchConfiguration('publish_battery'),
            'battery_topic': LaunchConfiguration('battery_topic'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(args + [driver, rviz])
