"""VESC, serial RPLidar and RealSense hardware adapter (ROS 2 Humble)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('carkit_bringup')) /
                 'config' / 'vesc_rplidar.yaml')
    chassis = IfCondition(LaunchConfiguration('start_chassis'))

    def sensor(name):
        return IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_sensors'), "' == 'true' and '",
            LaunchConfiguration(name), "' == 'true'",
        ]))

    return LaunchDescription([
        DeclareLaunchArgument('start_chassis', default_value='true'),
        DeclareLaunchArgument('start_sensors', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        Node(package='vesc_driver', executable='vesc_driver_node',
             name='vesc_driver', parameters=[config], condition=chassis, output='screen'),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
             name='ackermann_to_vesc', parameters=[config], condition=chassis,
             remappings=[('ackermann_cmd', '/ackermann_cmd')], output='screen'),
        Node(package='vesc_ackermann', executable='vesc_to_odom_node',
             name='vesc_to_odom', parameters=[config], condition=chassis, output='screen'),
        Node(package='sllidar_ros2', executable='sllidar_node', name='carkit_rplidar',
             parameters=[config], remappings=[('scan', '/scan/raw')],
             condition=sensor('start_lidar'), output='screen'),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             namespace='camera', name='camera', parameters=[config],
             condition=sensor('start_camera'), output='screen'),
        # Nominal mounting transforms: measure these for each assembled car.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='vesc_base_to_laser', condition=sensor('start_lidar'),
             arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='vesc_base_to_camera', condition=sensor('start_camera'),
             arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
    ])
