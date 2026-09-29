"""Read host/USB metadata for loss diagnosis. Never open a tty or send CAN.

Run on the Jetson with Python's standard library. No sudo, module loading,
kernel setting changes or device probing are performed by this tool.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError as error:
        return {'unavailable': type(error).__name__}


def command(args):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=4, check=False)
        return {'returncode': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'unavailable': type(error).__name__}


def capture():
    begin = time.monotonic_ns()
    result = {'schema': 'can-host-snapshot-v1', 'wall_time_ns': time.time_ns(),
              'begin_monotonic_ns': begin, 'uname': platform.uname()._asdict(),
              'boot_id': read('/proc/sys/kernel/random/boot_id'),
              'serial_devices_opened': False, 'motor_writes': 0,
              'kernel_or_power_settings_changed': False}
    try:
        config = gzip.decompress(Path('/proc/config.gz').read_bytes()).decode()
        result['kernel_config'] = [s for s in config.splitlines() if any(
            token in s for token in ('CONFIG_USB_MON', 'CONFIG_KPROBES', 'CONFIG_FTRACE',
                                     'CONFIG_TRACING', 'CONFIG_BPF_EVENTS'))]
    except (OSError, EOFError, ValueError) as error:
        result['kernel_config'] = {'unavailable': str(error)}
    result['usb_devices'] = {}
    for p in sorted(Path('/sys/bus/usb/devices').glob('*')):
        if not (p/'idVendor').exists():
            continue
        # Only serial adapters and their hubs. No USB payload or unrelated device content.
        if read(p/'idVendor') != '1a86' and read(p/'bDeviceClass') != '09':
            continue
        result['usb_devices'][p.name] = {name: read(p/name) for name in (
            'idVendor', 'idProduct', 'busnum', 'devnum', 'speed', 'bDeviceClass',
            'power/control', 'power/runtime_status', 'power/autosuspend_delay_ms')}
    result['tty_mapping'] = {}
    for p in sorted(Path('/sys/class/tty').glob('ttyUSB*')):
        result['tty_mapping'][p.name] = {
            'device': str((p/'device').resolve()), 'driver': str((p/'device/driver').resolve())}
    result['ch341_module'] = command(['modinfo', '-n', 'ch341'])
    if result['ch341_module'].get('returncode') == 0:
        try:
            result['ch341_module']['sha256'] = hashlib.sha256(
                Path(result['ch341_module']['stdout'].strip()).read_bytes()).hexdigest()
        except OSError as error:
            result['ch341_module']['hash_error'] = str(error)
    result['kernel_log'] = command(['dmesg', '--color=never'])
    result['usbmon_module'] = command(['modinfo', 'usbmon'])
    result['end_monotonic_ns'] = time.monotonic_ns()
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    # Claim the output before gathering data, never overwrite earlier evidence.
    with args.output.open('x') as stream:
        json.dump(capture(), stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
