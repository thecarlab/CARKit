"""Explicit hardware nodes; vendor launch files cannot introduce extra command owners."""
from launch_ros.actions import Node

from .hardware import vesc_parameters


def vesc_nodes(config):
    params = vesc_parameters(config)
    # Disable alternative motor-control modes. Ackermann uses ERPM and servo only.
    params.update({f'{mode}_{bound}': 0.0 for mode in
                   ('duty_cycle', 'current', 'brake', 'position') for bound in ('min', 'max')})
    nodes = [
        Node(package=package, executable=executable, name=name,
             parameters=[params], output='screen',
             remappings=[('ackermann_cmd', 'ackermann_cmd/feedback')]
             if package == 'vesc_ackermann' else [])
        for package, executable, name in (
            ('vesc_driver', 'vesc_driver_node', 'vesc_driver'),
            ('vesc_ackermann', 'ackermann_to_vesc_node', 'ackermann_to_vesc'),
        )
    ]
    nodes.append(Node(
        package='carkit_vehicle_runtime', executable='vesc_speed_control',
        name='vesc_speed_control', output='screen', parameters=[{
            'speed_to_erpm_gain': float(config['driver']['speed_to_erpm_gain']),
            'speed_to_erpm_offset': float(config['driver']['speed_to_erpm_offset']),
            'max_speed': float(config['max_speed']),
            'max_reverse_speed': float(config['max_reverse_speed']),
        }]))
    nodes.append(Node(
        package='carkit_vehicle_runtime', executable='realsense_wheel_odometry',
        name='realsense_wheel_odometry', output='screen', parameters=[{
            'speed_to_erpm_gain': float(config['driver']['speed_to_erpm_gain']),
            'speed_to_erpm_offset': float(config['driver']['speed_to_erpm_offset']),
            'max_measured_speed': 1.5 * max(config['max_speed'], config['max_reverse_speed']),
        }]))
    return nodes


def mounting_nodes(config, camera=True, lidar=True):
    actions = []
    for frame, values in config['mounts'].items():
        if (frame == 'laser' and not lidar) or (frame == 'camera_link' and not camera):
            continue
        args = []
        for key, value in zip(('x', 'y', 'z', 'yaw', 'pitch', 'roll'), values):
            args += [f'--{key}', str(value)]
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'carkit_base_to_{frame}',
            arguments=args + ['--frame-id', 'base_link', '--child-frame-id', frame]))
    return actions


def vesc_sensor_nodes(config, camera=True, lidar=True):
    actions = []
    if lidar:
        params = dict(config['lidar'])
        kind = params.pop('type')
        if kind == 'rplidar':
            params.update(channel_type='serial', frame_id='laser', inverted=False, angle_compensate=True)
            actions.append(Node(package='sllidar_ros2', executable='sllidar_node',
                                name='carkit_rplidar', parameters=[params],
                                remappings=[('scan', '/scan/raw')], output='screen'))
        else:
            params['laser_frame_id'] = 'laser'
            actions.append(Node(package='urg_node', executable='urg_node_driver',
                                name='carkit_lidar', parameters=[params],
                                remappings=[('scan', '/scan/raw')], output='screen'))
    # The camera checkbox controls video. VESC localization still needs the
    # RealSense motion streams when video is disabled.
    if config['camera']['type'] == 'realsense':
        params = {key: value for key, value in config['camera'].items() if key != 'type'}
        params.update(enable_color=camera, enable_depth=False, enable_infra1=False,
                      enable_infra2=False, enable_gyro=True, enable_accel=True)
        params.setdefault('rgb_camera.color_profile', '640x480x15')
        actions.append(Node(package='realsense2_camera', executable='realsense2_camera_node',
                            namespace='camera', name='camera', parameters=[params], output='screen',
                            remappings=[(f'/camera/camera/{stream}/sample',
                                         f'/camera/camera/{stream}/sample_raw')
                                        for stream in ('gyro', 'accel')]))
        actions.append(Node(package='carkit_vehicle_runtime', executable='imu_rate_limit',
                            name='imu_rate_limit', output='screen'))
    return actions
