import copy
import importlib.util
import math
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from carkit_bringup.hardware import load_hardware, navigation_arguments, scan_parameters, vesc_parameters
import carkit_bringup.hardware_launch as hardware_launch

ROOT = Path(__file__).parents[1]


def hardware(chassis):
    return load_hardware(chassis, ROOT / 'config' / 'hardware' / f'{chassis}.yaml')


@pytest.mark.parametrize('chassis', ['osracer', 'f1tenth', 'vesc_rplidar'])
def test_shared_geometry_and_actuator_limits(chassis):
    config = hardware(chassis)
    nav = navigation_arguments(config)
    scan = scan_parameters(config)
    assert float(nav['wheelbase']) == config['wheelbase']
    assert float(nav['max_speed']) == config['max_speed']
    assert float(nav['minimum_turning_radius']) == pytest.approx(1.10 * config['wheelbase'] / math.tan(config['max_steering_angle']))
    footprint = yaml.safe_load(nav['footprint'])
    assert max(p[0] for p in footprint) - min(p[0] for p in footprint) == pytest.approx(scan['vehicle_length_m'])
    assert (max(p[0] for p in footprint) + min(p[0] for p in footprint))/2 == pytest.approx(scan['footprint_center_x_m'])
    if chassis != 'osracer':
        params = vesc_parameters(config)
        assert params['wheelbase'] == config['wheelbase']
        assert params['publish_tf'] is False
        assert params['speed_max'] / params['speed_to_erpm_gain'] == config['max_speed']
        for sign in (-1, 1):
            servo = params['steering_angle_to_servo_offset'] + sign*config['max_steering_angle']*params['steering_angle_to_servo_gain']
            assert params['servo_min'] <= servo <= params['servo_max']


@pytest.mark.parametrize('chassis', ['osracer', 'f1tenth', 'vesc_rplidar'])
@pytest.mark.parametrize('profile', ['reference', 'ada_high_school'])
@pytest.mark.parametrize('manual_input', ['keyboard', 'joystick'])
def test_integrated_launch_has_one_command_owner_and_shared_geometry(monkeypatch, chassis, profile, manual_input):
    spec = importlib.util.spec_from_file_location('tested_launch', ROOT/'launch'/'carkit.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    monkeypatch.setattr(launch, '_profile', lambda context: yaml.safe_load((ROOT/'config'/'profiles'/f'{profile}.yaml').read_text()))
    monkeypatch.setattr(launch, 'load_hardware', lambda chassis, path: hardware(chassis))
    monkeypatch.setattr(launch, '_course_parameters', lambda *args: [])
    monkeypatch.setattr(launch, '_include', lambda package, file, arguments=None: dict(include=package, arguments=arguments or {}))
    monkeypatch.setattr(launch, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(hardware_launch, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    for action in launch.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(chassis=chassis, profile=profile, manual_input=manual_input)
    actions = [a for a in launch._launch_stack(context) if isinstance(a, dict)]
    inputs = [a for a in actions if a.get('include') == 'carkit_human_control']
    assert len(inputs) == (1 if manual_input == 'joystick' else 0)
    if inputs:
        assert inputs[0]['arguments']['direct_relay'] == 'false'
    assert not any(a.get('include') == 'f1tenth_stack' for a in actions)
    if chassis == 'osracer':
        assert not any(a.get('executable') == 'vesc_speed_control' for a in actions)
        platform = next(a for a in actions if a.get('include') == 'osracer_bringup' and 'start_joystick' in a['arguments'])
        assert platform['arguments']['start_joystick'] == 'false'
        assert not any(a.get('executable') == 'realsense_wheel_odometry' for a in actions)
    else:
        assert len([a for a in actions if a.get('executable') == 'vesc_speed_control']) == 1
        converter = next(a for a in actions if a.get('executable') == 'ackermann_to_vesc_node')
        assert converter['remappings'] == [('ackermann_cmd', 'ackermann_cmd/feedback')]
        assert len([a for a in actions if a.get('executable') == 'realsense_wheel_odometry']) == 1
        assert not any(a.get('executable') == 'vesc_to_odom_node' for a in actions)
    if chassis != 'osracer':
        for name in ('vesc_speed_control', 'realsense_wheel_odometry', 'imu_rate_limit'):
            assert next(a for a in actions if a.get('executable') == name)['package'] == 'carkit_vehicle_runtime'
    config = hardware(chassis)
    arbiter = [a for a in actions if a.get('include') == 'carkit_control_center']
    assert len(arbiter) == 1
    assert float(arbiter[0]['arguments']['max_speed']) == config['max_speed']
    assert arbiter[0]['arguments']['require_measured_odometry'] == str(chassis != 'osracer').lower()
    bridge = next(a for a in actions if a.get('package') == 'carkit_web_bridge')
    assert bridge['parameters'][0]['manual_input'] == manual_input
    assert bridge['parameters'][0]['max_steering_angle'] == config['max_steering_angle']
    for key, value in config['footprint'].items():
        assert bridge['parameters'][0][f'footprint_{key}'] == value
    assert len([a for a in actions if a.get('executable') == 'odom_tf_broadcaster']) == 1
    if profile == 'reference':
        navigation = next(a for a in actions if a.get('include') == 'carkit_navigation')
        assert navigation['arguments']['start_odom_tf'] == 'false'
        assert float(navigation['arguments']['wheelbase']) == config['wheelbase']
    context.launch_configurations['start_control'] = 'false'
    manual_actions = launch._launch_stack(context)
    assert len([a for a in manual_actions if isinstance(a, dict) and a.get('include') == 'carkit_control_center']) == 1


def test_invalid_profile_cannot_enable_out_of_range_actuation(tmp_path):
    config = hardware('vesc_rplidar')
    for change in ({'max_steering_angle': 0.5}, {'wheelbase': float('nan')}, {'chassis': 'osracer'}):
        invalid = copy.deepcopy(config)
        invalid.update(change)
        path = tmp_path/'hardware.yaml'
        path.write_text(yaml.safe_dump(invalid))
        with pytest.raises(ValueError):
            load_hardware('vesc_rplidar', path)


@pytest.mark.parametrize('chassis', ['vesc_rplidar', 'f1tenth'])
def test_disabling_video_keeps_realsense_motion_sensors(monkeypatch, chassis):
    monkeypatch.setattr(hardware_launch, 'Node', lambda **kwargs: kwargs)
    nodes = hardware_launch.vesc_sensor_nodes(hardware(chassis), camera=False, lidar=False)
    camera = next(a for a in nodes if a['package'] == 'realsense2_camera')
    assert len([n for n in nodes if n.get('executable') == 'imu_rate_limit']) == 1
    assert camera['remappings'] == [(f'/camera/camera/{s}/sample', f'/camera/camera/{s}/sample_raw') for s in ('gyro', 'accel')]
    params = camera['parameters'][0]
    assert not params['enable_color']
    assert params['enable_gyro'] and params['enable_accel']
