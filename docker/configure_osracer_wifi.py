#!/usr/bin/env python3
"""Bind saved OSRacer Wi-Fi profiles without interrupting active connections."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time


def nm(*args):
    return subprocess.check_output(['nmcli', '--escape', 'no', *args], text=True).strip()


def adapters():
    onboard, usb = [], []
    for path in Path('/sys/class/net').glob('*/wireless'):
        target = str(path.resolve())
        (usb if re.search(r'/usb\d+/', target) else onboard).append(path.parent.name)
    if len(onboard) != 1 or len(usb) != 1:
        raise RuntimeError('Expected exactly one onboard and one USB Wi-Fi adapter; no profiles changed')
    return onboard[0], usb[0]


def plan(profiles, onboard, usb, osr_ssid=None):
    choices = {p['ssid'] for p in profiles if p['ssid'].startswith('OSR-')}
    if osr_ssid is None:
        if len(choices) != 1:
            raise RuntimeError('Provide --osr-ssid when no unique saved OSR network exists')
        osr_ssid = choices.pop()
    if not any(p['ssid'] == osr_ssid for p in profiles) or not any(p['ssid'] == 'eduroam' for p in profiles):
        raise RuntimeError('Save the OSR and eduroam credentials first; no profiles changed')
    result = []
    for p in profiles:
        settings = {'connection.autoconnect': 'no'}
        if p['ssid'] in (osr_ssid, 'eduroam'):
            settings = {
                'connection.interface-name': onboard if p['ssid'] == osr_ssid else usb,
                'connection.autoconnect': 'yes',
                'connection.autoconnect-priority': '100',
            }
        if p['ssid'] == 'eduroam':
            settings.update({'ipv4.route-metric': '50', 'ipv4.never-default': 'no'})
        result.append((p['uuid'], settings))
    return osr_ssid, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--osr-ssid')
    parser.add_argument('--apply', action='store_true', help='Save policy; default is a read-only preview')
    args = parser.parse_args()
    onboard, usb = adapters()
    profiles = []
    for uuid in nm('-g', 'UUID', 'connection', 'show').splitlines():
        if nm('-g', 'connection.type', 'connection', 'show', uuid) != '802-11-wireless':
            continue
        profiles.append({'uuid': uuid, 'ssid': nm('-g', '802-11-wireless.ssid', 'connection', 'show', uuid)})
    ssid, changes = plan(profiles, onboard, usb, args.osr_ssid)
    print(json.dumps({'onboard': onboard, 'onboard_ssid': ssid, 'usb': usb, 'usb_ssid': 'eduroam', 'changes': changes}, indent=2))
    if not args.apply:
        return
    if os.geteuid() != 0:
        raise RuntimeError('--apply requires root')
    # Refuse to leave an already active connection on the wrong network.
    for interface, expected in ((onboard, ssid), (usb, 'eduroam')):
        active = nm('-g', 'GENERAL.CON-UUID', 'device', 'show', interface)
        if active and active != '--':
            actual = nm('-g', '802-11-wireless.ssid', 'connection', 'show', active)
            if actual != expected:
                raise RuntimeError(f'{interface} is connected to {actual}; connect it to {expected} locally first')
    backup = []
    for uuid, settings in changes:
        backup.append((uuid, {key: nm('-g', key, 'connection', 'show', uuid) for key in settings}))
    directory = Path('/var/lib/carkit/network-backups')
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    path = directory / f'wifi-{time.time_ns()}.json'
    path.write_text(json.dumps(backup, indent=2) + '\n')
    path.chmod(0o600)
    try:
        for uuid, settings in changes:
            nm('connection', 'modify', uuid, *[item for pair in settings.items() for item in pair])
    except Exception:
        for uuid, settings in backup:
            nm('connection', 'modify', uuid, *[item for pair in settings.items() for item in pair])
        raise
    # A service drop-in survives reporter reinstallation and contains no secrets.
    dropin = Path('/etc/systemd/system/carkit-webmonitor.service.d')
    dropin.mkdir(parents=True, exist_ok=True)
    (dropin / '30-osracer-interface.conf').write_text(
        '[Service]\nEnvironment="CARKIT_REPORT_INTERFACE=' + usb + '"\n')
    # EnvironmentFile takes precedence over Environment: update an existing entry too.
    env = Path('/etc/carkit-webmonitor.env')
    if env.exists():
        lines = [line for line in env.read_text().splitlines() if not line.startswith('CARKIT_REPORT_INTERFACE=')]
        env.write_text('\n'.join(lines + ['CARKIT_REPORT_INTERFACE=' + usb]) + '\n')
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    print('Policy saved. No connection was disconnected or activated. Backup:', path)


if __name__ == '__main__':
    main()
