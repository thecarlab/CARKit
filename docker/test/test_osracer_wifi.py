import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('wifi', Path(__file__).parents[1] / 'configure_osracer_wifi.py')
wifi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wifi)


def test_bind_expected_networks_and_disable_other_autoconnect():
    ssid, changes = wifi.plan([
        {'uuid': 'osr', 'ssid': 'OSR-F912'},
        {'uuid': 'edu', 'ssid': 'eduroam'},
        {'uuid': 'other', 'ssid': 'guest'},
    ], 'onboard', 'usb')
    changes = dict(changes)
    assert ssid == 'OSR-F912'
    assert changes['osr']['connection.interface-name'] == 'onboard'
    assert changes['edu']['connection.interface-name'] == 'usb'
    assert changes['edu']['ipv4.route-metric'] == '50'
    assert changes['edu']['ipv4.never-default'] == 'no'
    assert 'ipv4.route-metric' not in changes['osr']
    assert changes['other'] == {'connection.autoconnect': 'no'}


def test_missing_credentials_and_ambiguous_osr_fail_before_changes():
    with pytest.raises(RuntimeError):
        wifi.plan([{'uuid': 'o', 'ssid': 'OSR-F912'}], 'onboard', 'usb')
    with pytest.raises(RuntimeError):
        wifi.plan([{'uuid': 'a', 'ssid': 'OSR-A'}, {'uuid': 'b', 'ssid': 'OSR-B'},
                   {'uuid': 'e', 'ssid': 'eduroam'}], 'onboard', 'usb')
