# CARKit learning annotation: implements the behavior described by this file's package and module.
import os
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


RULE_NAME = '99-osrbot-osracer.rules'


def main():
    src = find_rule_file()
    if not src.exists():
        raise SystemExit(f'ERROR: udev rule not found: {src}')

    dst = Path('/etc/udev/rules.d') / RULE_NAME
    run_sudo(['install', '-m', '0644', str(src), str(dst)])
    run_sudo(['udevadm', 'control', '--reload-rules'])
    run_sudo(['udevadm', 'trigger'])
    run_sudo(['usermod', '-a', '-G', 'dialout', os.environ.get('USER', '')])

    print(f'Installed {dst}')
    print('Reconnect the vehicle USB cable.')
    print('Log out and log back in if this user was not already in the dialout group.')


def find_rule_file():
    try:
        return Path(get_package_share_directory('osracer_base')) / 'udev' / RULE_NAME
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[2] / 'udev' / RULE_NAME


def run_sudo(args):
    if not shutil.which('sudo'):
        raise SystemExit('ERROR: sudo is required to install udev rules.')
    subprocess.run(['sudo', *args], check=True)


if __name__ == '__main__':
    main()
