"""Standalone VESC/RPLidar bringup using the shared input and safety pipeline."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keys = ('start_chassis', 'start_sensors', 'start_camera', 'start_lidar')
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value='true') for key in keys],
        DeclareLaunchArgument('hardware_config', default_value=''),
        DeclareLaunchArgument('manual_input', default_value='keyboard'),
        DeclareLaunchArgument('web_bridge', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('carkit_bringup'), 'launch', 'carkit.launch.py'])),
            launch_arguments={
                **{key: LaunchConfiguration(key) for key in keys},
                'hardware_config': LaunchConfiguration('hardware_config'),
                'manual_input': LaunchConfiguration('manual_input'),
                'web_bridge': LaunchConfiguration('web_bridge'),
                'chassis': 'vesc_rplidar', 'profile': 'reference',
                'start_planning': 'false', 'start_perception': 'false',
                'start_behavior': 'false', 'start_control': 'true',
            }.items()),
    ])
