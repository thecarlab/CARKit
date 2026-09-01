from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf_node(name, parent_frame, child_frame, x, y, z, yaw, pitch, roll):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            x,
            y,
            z,
            yaw,
            pitch,
            roll,
            parent_frame,
            child_frame,
        ],
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('base_link_frame_id', default_value='base_link'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('laser_frame_id', default_value='laser_frame'),
        DeclareLaunchArgument('base_link_x', default_value='0.0'),
        DeclareLaunchArgument('base_link_y', default_value='0.0'),
        DeclareLaunchArgument('base_link_z', default_value='0.0'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.06'),
        DeclareLaunchArgument('laser_x', default_value='0.12'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.12'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
    ]

    base_to_link = static_tf_node(
        'base_footprint_to_base_link',
        LaunchConfiguration('base_frame_id'),
        LaunchConfiguration('base_link_frame_id'),
        LaunchConfiguration('base_link_x'),
        LaunchConfiguration('base_link_y'),
        LaunchConfiguration('base_link_z'),
        '0.0',
        '0.0',
        '0.0',
    )

    base_to_imu = static_tf_node(
        'base_link_to_imu',
        LaunchConfiguration('base_link_frame_id'),
        LaunchConfiguration('imu_frame_id'),
        LaunchConfiguration('imu_x'),
        LaunchConfiguration('imu_y'),
        LaunchConfiguration('imu_z'),
        '0.0',
        '0.0',
        '0.0',
    )

    base_to_laser = static_tf_node(
        'base_link_to_laser',
        LaunchConfiguration('base_link_frame_id'),
        LaunchConfiguration('laser_frame_id'),
        LaunchConfiguration('laser_x'),
        LaunchConfiguration('laser_y'),
        LaunchConfiguration('laser_z'),
        LaunchConfiguration('laser_yaw'),
        '0.0',
        '0.0',
    )

    return LaunchDescription(args + [base_to_link, base_to_imu, base_to_laser])
