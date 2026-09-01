# CARKit learning annotation: implements the behavior described by this file's package and module.
import glob
import os
import subprocess
from pathlib import Path


def main():
    device = Path('/dev/osrbot_base')
    print(f'Checking OSRacer device: {device}')
    if device.exists():
        if not device.is_char_device():
            print(f'ERROR: {device} exists but is not a serial character device.')
            print('Run ./docker/setup_osracer_device.sh on the host, then restart Docker.')
            raise SystemExit(1)
        print(f'Device path: {device.resolve()}')
        if not os.access(device, os.R_OK | os.W_OK):
            print(f'ERROR: current user cannot read and write {device}.')
            print('Confirm dialout group membership and restart the container.')
            raise SystemExit(1)
        print_udev_info(device)
        return

    print(f'MISSING {device}')
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if candidates:
        print('Available serial devices:')
        for candidate in candidates:
            print(f'  {candidate}')
    else:
        print('No /dev/ttyACM* or /dev/ttyUSB* devices found.')
    print('Install the udev rule, then reconnect the vehicle USB cable.')
    raise SystemExit(1)


def print_udev_info(device):
    try:
        result = subprocess.run(
            ['udevadm', 'info', '--query=property', '--name', str(device)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except FileNotFoundError:
        return

    props = parse_udev_properties(result.stdout)
    vendor_id = props.get('ID_VENDOR_ID', '')
    model_id = props.get('ID_MODEL_ID', '')
    serial = props.get('ID_SERIAL_SHORT', '')
    manufacturer = normalize_udev_text(props.get('ID_VENDOR', ''))
    product = normalize_udev_text(props.get('ID_MODEL', ''))

    if vendor_id:
        print(f'USB vendor ID: {vendor_id}')
    if model_id:
        print(f'USB product ID: {model_id}')
    if manufacturer:
        print(f'Manufacturer: {manufacturer}')
    if product:
        print(f'Product: {product}')
    if serial:
        print(f'Serial: {serial}')


def parse_udev_properties(text):
    props = {}
    for line in text.splitlines():
        if '=' not in line:
            continue
        key, value = line.split('=', 1)
        props[key] = value
    return props


def normalize_udev_text(value):
    text = value.replace('_', ' ').strip()
    while '  ' in text:
        text = text.replace('  ', ' ')
    return text


if __name__ == '__main__':
    main()
