"""One validated hardware contract for the selected assembled vehicle."""

import math
from pathlib import Path

import yaml


def load_hardware(chassis, path=None):
    if path is None or not str(path):
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('carkit_bringup')) / 'config' / 'hardware' / f'{chassis}.yaml'
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if config['chassis'] != chassis:
        raise ValueError('Hardware profile chassis does not match selected chassis')
    for key in ('wheelbase', 'max_speed', 'max_reverse_speed', 'max_steering_angle'):
        value = config[key]
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if config['max_steering_angle'] >= math.pi / 2:
        raise ValueError('max_steering_angle must be less than pi/2')
    footprint = config['footprint']
    if not all(math.isfinite(footprint[k]) for k in ('length', 'width', 'center_x', 'center_y')):
        raise ValueError('Footprint values must be finite')
    if footprint['length'] <= 0 or footprint['width'] <= 0:
        raise ValueError('Footprint dimensions must be positive')
    for frame, mount in config['mounts'].items():
        if len(mount) != 6 or not all(math.isfinite(v) for v in mount):
            raise ValueError(f'Invalid mounting transform for {frame}')
    if chassis != 'osracer':
        driver = config['driver']
        for key in ('speed_to_erpm_gain', 'speed_to_erpm_offset',
                    'steering_angle_to_servo_gain', 'steering_angle_to_servo_offset',
                    'servo_min', 'servo_max'):
            if not math.isfinite(driver[key]):
                raise ValueError(f'{key} must be finite')
        if driver['speed_to_erpm_gain'] == 0 or driver['steering_angle_to_servo_gain'] == 0:
            raise ValueError('VESC calibration gains must be nonzero')
        # A nonzero offset produces motor rotation for a zero/stop command.
        if driver['speed_to_erpm_offset'] != 0:
            raise ValueError('speed_to_erpm_offset must be zero for a safe stop')
        low, high = driver['servo_min'], driver['servo_max']
        if not 0 <= low < high <= 1:
            raise ValueError('Invalid servo limits')
        for angle in (-config['max_steering_angle'], config['max_steering_angle']):
            servo = driver['steering_angle_to_servo_offset'] + angle * driver['steering_angle_to_servo_gain']
            if not low <= servo <= high:
                raise ValueError('Steering limit exceeds calibrated servo range')
    return config


def footprint_string(config):
    f = config['footprint']
    x0, x1 = f['center_x'] - f['length']/2, f['center_x'] + f['length']/2
    y0, y1 = f['center_y'] - f['width']/2, f['center_y'] + f['width']/2
    return str([[x1, y1], [x1, y0], [x0, y0], [x0, y1]])


def navigation_arguments(config):
    return {
        'wheelbase': str(config['wheelbase']),
        'max_speed': str(config['max_speed']),
        # Existing reference Nav2 policy remains forward-only.
        'max_reverse_speed': '0.0',
        'max_steering_angle': str(config['max_steering_angle']),
        'desired_linear_vel': str(min(0.75, config['max_speed'])),
        # Keep 10% radius reserve for tracking corrections and grid discretization.
        'minimum_turning_radius': str(1.10 * config['wheelbase'] / math.tan(config['max_steering_angle'])),
        'footprint': footprint_string(config),
    }


def scan_parameters(config):
    f = config['footprint']
    return {'vehicle_length_m': float(f['length']), 'vehicle_width_m': float(f['width']),
            'footprint_center_x_m': float(f['center_x']), 'footprint_center_y_m': float(f['center_y'])}


def vesc_parameters(config):
    params = dict(config['driver'])
    gain = params['speed_to_erpm_gain']
    limits = (-config['max_reverse_speed'] * gain, config['max_speed'] * gain)
    params.update(speed_min=min(limits), speed_max=max(limits),
                  wheelbase=float(config['wheelbase']), odom_frame='odom', base_frame='base_link',
                  publish_tf=False, use_servo_cmd_to_calc_angular_velocity=False)
    return params
