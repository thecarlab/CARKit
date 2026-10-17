import copy
import json
import math
from pathlib import Path
from unittest.mock import Mock, patch

import pytest
import yaml

from carkit_webui.calibration import STAGES, CalibrationService, analyze, atomic_json, fingerprint

PROFILE = Path(__file__).parents[3] / 'core/carkit_bringup/config/hardware/vesc_rplidar.yaml'


def config():
    return yaml.safe_load(PROFILE.read_text())


def samples(profile, center=None, gain=None):
    center = profile['driver']['steering_angle_to_servo_offset'] if center is None else center
    gain = profile['driver']['steering_angle_to_servo_gain'] if gain is None else gain
    output = []
    for index, (key, _, direction, scale) in enumerate(STAGES):
        servo = profile['driver']['steering_angle_to_servo_offset'] + scale*profile['max_steering_angle']*profile['driver']['steering_angle_to_servo_gain']
        actual_angle = (servo-center)/gain
        for j in range(45):
            speed = direction*(0.33+0.005*math.sin(j))
            output.append({'stage':key, 't':index*8+j*.05, 'settled':True,
                           'speed':speed, 'yaw_rate':speed*math.tan(actual_angle)/profile['wheelbase'] + .003,
                           'servo':servo})
    return output


def test_matching_calibration_is_green_only_after_all_six_stages():
    c = config()
    r = analyze(samples(c), c, .003)
    assert r['good']
    assert r['straight_error_degrees'] < 1e-6
    assert r['turn_rms_error_degrees'] < 1e-6
    with pytest.raises(ValueError, match='insufficient'):
        analyze(samples(c)[:-45], c, .003)


def test_fit_recovers_known_center_and_gain_without_changing_motor_scale():
    c = config()
    r = analyze(samples(c, center=.475, gain=-1.35), c, .003)
    assert not r['good']
    assert r['recommendation']['steering_angle_to_servo_offset'] == pytest.approx(.475)
    assert r['recommendation']['steering_angle_to_servo_gain'] == pytest.approx(-1.35)
    assert set(r['recommendation']) == {'steering_angle_to_servo_offset', 'steering_angle_to_servo_gain', 'max_steering_angle'}
    for angle in [-r['recommendation']['max_steering_angle'], r['recommendation']['max_steering_angle']]:
        servo = .475+angle*-1.35
        assert c['driver']['servo_min'] <= servo <= c['driver']['servo_max']


def test_stationary_or_wrong_direction_samples_cannot_produce_good_result():
    c = config()
    s = samples(c)
    for row in s:
        row['speed'] = .01
    with pytest.raises(ValueError, match='insufficient'):
        analyze(s, c, .003)


def test_reversed_sensor_axis_and_inconsistent_reverse_center_are_rejected():
    c = config()
    s = samples(c)
    for row in s:
        row['yaw_rate'] = -row['yaw_rate']
    with pytest.raises(ValueError, match='reversed|monotonic'):
        analyze(s, c, -.003)
    s = samples(c)
    for row in s:
        if row['stage'] == 'reverse':
            row['yaw_rate'] += .08
    with pytest.raises(ValueError, match='Forward and reverse'):
        analyze(s, c, .003)


@pytest.fixture
def service(tmp_path):
    path = tmp_path / 'carkit/core/carkit_bringup/config/hardware/vesc_rplidar.yaml'
    path.parent.mkdir(parents=True)
    path.write_text(PROFILE.read_text())
    manager = Mock()
    manager.workspace = tmp_path
    manager.current_config = {'chassis':'vesc_rplidar', 'components':{'chassis':True,'sensors':True}}
    manager.process.poll.return_value = None
    manager.job = None
    return CalibrationService(manager)


def test_motion_requires_confirmation_and_running_stack(service):
    with pytest.raises(ValueError, match='Confirm'):
        service.start({})
    service.manager.process = None
    with pytest.raises(RuntimeError, match='Launch'):
        service.start({'clear_area_confirmed':True})


def test_heartbeat_cannot_be_extended_by_an_observer(service):
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        spawn.return_value.poll.return_value = None
        response = service.start({'clear_area_confirmed':True})
        with pytest.raises(ValueError, match='Only the browser'):
            service.heartbeat({'run_id':response['run_id'],'owner_token':'wrong'})
        service.heartbeat(response)
        assert service.snapshot()['running']
        with pytest.raises(RuntimeError, match='already running'):
            service.start({'clear_area_confirmed':True})
        service.cancel('Stop calibration button pressed')
        assert (service.run_dir/'cancel').exists()
        assert json.loads((service.run_dir/'cancel-reason.json').read_text())['reason'] == 'Stop calibration button pressed'
        service.log_handle.close()


def finished(service, good=True):
    path, c = service.profile()
    run_id = 'finished'
    root = service.root/run_id
    root.mkdir(parents=True)
    atomic_json(service.root/'latest.json', {'run_id':run_id, 'fingerprint':fingerprint(c)})
    atomic_json(root/'status.json', {'phase':'complete'})
    r = analyze(samples(c, center=None if good else .475), c, .003)
    r['test_speed_mps'] = 1.0
    atomic_json(root/'request.json', {'speed':1.0})
    atomic_json(root/'result.json', r)
    return path


def test_profile_changes_invalidate_green_status(service):
    path = finished(service)
    assert service.snapshot()['status'] == 'good'
    c = config();c['wheelbase'] += .01
    path.write_text(yaml.safe_dump(c))
    assert service.snapshot()['status'] == 'requires_calibration'
    assert not service.snapshot()['can_apply']


def test_apply_is_reviewed_bounded_and_invalidates_previous_validation(service):
    path = finished(service, good=False)
    with pytest.raises(ValueError, match='restart'):
        service.apply({'run_id':'finished'})
    service.apply({'run_id':'finished','restart_confirmed':True})
    saved = yaml.safe_load(path.read_text())
    assert saved['driver']['steering_angle_to_servo_offset'] == pytest.approx(.475)
    service.manager.stop.assert_called_once()
    service.manager.launch.assert_called_once()
    assert service.snapshot()['status'] == 'requires_calibration'
    assert not service.snapshot()['can_apply']


def test_changed_hardware_cannot_receive_old_recommendation(service):
    path = finished(service, good=False)
    c = yaml.safe_load(path.read_text());c['driver']['steering_angle_to_servo_offset'] += .001
    path.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError, match='No applicable'):
        service.apply({'run_id':'finished','restart_confirmed':True})


def test_chassis_switch_selects_method_and_never_reuses_vesc_result_for_osracer(service):
    finished(service)
    assert service.snapshot()['status'] == 'good'
    assert service.snapshot()['method'] == 'vesc_realsense_imu'
    service.manager.current_config['chassis'] = 'osracer'
    snapshot = service.snapshot()
    assert snapshot['chassis'] == 'osracer'
    assert snapshot['method'] is None
    assert not snapshot['supported']
    assert snapshot['status'] != 'good'
    assert snapshot['result'] is None
    assert not snapshot['can_apply']
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        with pytest.raises(ValueError, match='OSRacer'):
            service.start({'clear_area_confirmed': True})
        spawn.assert_not_called()
    service.manager.current_config['chassis'] = 'vesc_rplidar'
    assert service.snapshot()['status'] == 'good'


def test_running_chassis_takes_priority_over_saved_selection(service):
    (service.manager.workspace / '.carkit').mkdir()
    atomic_json(service.manager.workspace / '.carkit/config.json', {'chassis': 'osracer'})
    assert service.method()['method'] == 'vesc_realsense_imu'
    service.manager.current_config = None
    assert service.method()['chassis'] == 'osracer'
    assert not service.method()['supported']


@pytest.mark.parametrize('chassis', ['vesc_rplidar', 'f1tenth'])
def test_both_vesc_profiles_select_realsense_method(service, chassis):
    service.manager.current_config['chassis'] = chassis
    assert service.method()['method'] == 'vesc_realsense_imu'
    assert service.method()['supported']


def test_compact_routine_rejects_other_speeds(service):
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        for speed in (.25, .5, .6, 1.1):
            with pytest.raises(ValueError, match='Compact calibration'):
                service.start({'clear_area_confirmed': True, 'speed': speed})
        spawn.assert_not_called()


def test_each_session_uses_its_own_saved_gyro_bias():
    c = config()
    rows = samples(c)
    for row in rows:
        bias = .02 if row['stage'] in ('forward', 'left_full') else -.015
        row['yaw_rate'] += bias-.003
        row['gyro_bias'] = bias
    assert analyze(rows, c, .005)['good']
    rows[0]['gyro_bias'] = None
    with pytest.raises(ValueError, match='older run'):
        analyze(rows, c, .005)


def checkpoint_run(service):
    _, c = service.profile()
    root = service.root/'interrupted'
    root.mkdir(parents=True)
    signature = fingerprint(c)
    atomic_json(service.root/'latest.json', {'run_id':root.name, 'fingerprint':signature})
    atomic_json(root/'request.json', {'config':c,'fingerprint':signature,'speed':1.0})
    atomic_json(root/'status.json', {'phase':'failed','message':'Obstacle'})
    checkpoint = {'fingerprint':signature,'speed':1.0,'completed':['forward'],
                  'samples':[{**r, 'gyro_bias':.003} for r in samples(c) if r['stage']=='forward'],
                  'returns':[{'stage':'forward'}]}
    atomic_json(root/'checkpoint.json', checkpoint)
    return root, checkpoint


def test_completed_maneuvers_survive_service_restart_and_retry(service):
    _, checkpoint = checkpoint_run(service)
    recreated = CalibrationService(service.manager)
    assert recreated.snapshot()['completed_stages'] == ['forward']
    assert recreated.snapshot()['can_resume']
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        spawn.return_value.poll.return_value = None
        recreated.start({'clear_area_confirmed':True})
        copied = json.loads((recreated.run_dir/'checkpoint.json').read_text())
        assert copied['completed'] == ['forward']
        assert [{k:v for k,v in row.items() if k != 'segment'} for row in copied['samples']] == checkpoint['samples']
        assert json.loads((recreated.run_dir/'request.json').read_text())['stage_id'] == 'reverse'
        recreated.log_handle.close()


def test_explicit_new_sequence_discards_checkpoint(service):
    checkpoint_run(service)
    with patch('carkit_webui.calibration.subprocess.Popen'):
        service.start({'clear_area_confirmed':True,'restart_sequence':True})
        assert json.loads((service.run_dir/'checkpoint.json').read_text())['completed'] == []
        service.log_handle.close()


def test_changed_profile_or_speed_does_not_reuse_saved_motion(service):
    from carkit_webui.calibration import load_checkpoint
    root, checkpoint = checkpoint_run(service)
    assert not load_checkpoint(root, checkpoint['fingerprint'], .25)['completed']
    assert not load_checkpoint(root, 'changed', 1.0)['completed']
    path, c = service.profile()
    c['wheelbase'] += .01
    path.write_text(yaml.safe_dump(c))
    assert not service.snapshot()['completed_stages']


def test_legacy_finished_returns_are_preserved_without_inventing_bias(service):
    from carkit_webui.calibration import load_checkpoint
    root, checkpoint = checkpoint_run(service)
    (root/'checkpoint.json').unlink()
    atomic_json(root/'returns.json', checkpoint['returns'])
    rows = samples(config())
    (root/'samples.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n{partial')
    restored = load_checkpoint(root, checkpoint['fingerprint'], 1.0)
    assert restored['completed'] == []
    assert restored['recorded_stages'] == ['forward']
    assert restored['retry_stages'] == ['forward']
    assert restored['samples'] == []
    assert service.snapshot()['legacy_measurements']


def test_finished_sequence_cannot_accidentally_repeat(service):
    finished(service)
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        with pytest.raises(ValueError, match='sequence is complete'):
            service.start({'clear_area_confirmed':True})
        spawn.assert_not_called()


def test_previous_test_speed_cannot_verify_or_resume_new_speed(service):
    finished(service)
    atomic_json(service.root/'finished'/'request.json', {'speed':.5})
    snapshot = service.snapshot()
    assert snapshot['status'] == 'requires_calibration'
    assert not snapshot['completed_stages']
    assert not snapshot['can_apply']
    with patch('carkit_webui.calibration.subprocess.Popen'):
        service.start({'clear_area_confirmed': True, 'speed':1.0})
        assert json.loads((service.run_dir/'request.json').read_text())['speed'] == 1.0
        assert not json.loads((service.run_dir/'checkpoint.json').read_text())['completed']
        service.log_handle.close()


def test_finished_motion_with_too_few_samples_is_pending_and_preserved(service):
    root, checkpoint = checkpoint_run(service)
    checkpoint['samples'] = checkpoint['samples'][:5]
    atomic_json(root/'checkpoint.json', checkpoint)
    snapshot = service.snapshot()
    assert snapshot['completed_stages'] == []
    assert snapshot['retry_stages'] == ['forward']
    assert snapshot['next_stage']['id'] == 'forward'
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        with pytest.raises(ValueError, match='next maneuver changed'):
            service.start({'clear_area_confirmed':True,'stage_id':'reverse'})
        spawn.assert_not_called()
        service.start({'clear_area_confirmed':True,'stage_id':'forward'})
        copied = json.loads((service.run_dir/'checkpoint.json').read_text())
        assert len(copied['samples']) == 5
        assert copied['completed'] == []
        service.log_handle.close()


def test_gaps_between_passes_do_not_count_as_steady_motion():
    from carkit_webui.calibration import enough_steady_motion
    rows = samples(config())[:20]
    for index, row in enumerate(rows):
        row['segment'] = f'pass-{index}'
        row['t'] = index*10.
    assert not enough_steady_motion(rows, 'forward')
    for index, row in enumerate(rows):
        row['segment'] = 'first' if index < 10 else 'second'
        row['t'] = (index % 10)*.1
    assert enough_steady_motion(rows, 'forward')


def test_waiting_for_confirmation_does_not_start_another_worker(service):
    root, _ = checkpoint_run(service)
    atomic_json(root/'status.json', {'phase':'awaiting_confirmation','message':'Confirm the next maneuver'})
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        for _ in range(3):
            snapshot = service.snapshot()
        spawn.assert_not_called()
        assert snapshot['next_stage']['id'] == 'reverse'
        with pytest.raises(ValueError, match='Confirm'):
            service.start({'stage_id':'reverse'})
        spawn.assert_not_called()


@pytest.mark.parametrize('mode', ['single', 'all'])
def test_run_mode_is_explicit_and_saved(service, mode):
    with patch('carkit_webui.calibration.subprocess.Popen'):
        result = service.start({'clear_area_confirmed':True,'mode':mode})
        assert result['mode'] == mode
        assert json.loads((service.run_dir/'request.json').read_text())['mode'] == mode
        service.log_handle.close()


def test_unknown_run_mode_cannot_start_motion(service):
    with patch('carkit_webui.calibration.subprocess.Popen') as spawn:
        with pytest.raises(ValueError, match='single or all'):
            service.start({'clear_area_confirmed':True,'mode':'automatic'})
        spawn.assert_not_called()
