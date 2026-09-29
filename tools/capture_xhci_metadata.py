"""Bounded, passive xHCI trace for CH341 adapters; never opens a serial device.

Requires root and existing tracepoints, not usbmon or a kernel/module change.
Creates private trace instances and removes them in finally. Only metadata is
recorded. Trace overhead must be compared with an untraced run before timing
claims. The text trace has no payload and does not print the status field.
The CH341 giveback tracepoint's nonzero status filter matched every successful
transfer on this Jetson, so this tool does not present it as an error count.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import time
import uuid

ROOT = Path('/sys/kernel/tracing')
USB = Path('/sys/bus/usb/devices')
EVENTS = ('xhci_urb_enqueue', 'xhci_urb_giveback', 'xhci_urb_dequeue')


def trace_stats_lossless(cpu_stats):
    if not cpu_stats:
        return False
    for stats in cpu_stats.values():
        counters = re.findall(r'^(?:overrun|dropped events):\s*(\d+)\s*$', stats, re.M)
        if len(counters) != 2 or any(int(value) for value in counters):
            return False
    return True


def device_pipes(names, root=USB):
    result = {}
    for name in names:
        if not re.fullmatch(r'\d+-\d+(?:\.\d+)*', name) or name in result:
            raise ValueError('Unique USB topology paths required')
        path = root/name
        if (path/'idVendor').read_text().strip() != '1a86' or (path/'idProduct').read_text().strip() != '7523':
            raise ValueError('Explicit CH341 1a86:7523 adapter required')
        dev = int((path/'devnum').read_text())
        if not 1 <= dev <= 127:
            raise ValueError('Invalid USB address')
        pipes = []
        for interface in root.glob(name + ':*'):
            for ep in interface.glob('ep_*'):
                if int((ep/'bmAttributes').read_text(), 16) & 3 != 2:
                    continue
                address = int((ep/'bEndpointAddress').read_text(), 16)
                # CH341 passes the full endpoint address to the pipe macro.
                # The IN direction bit therefore survives both at bit 22 and
                # at USB_DIR_IN (bit 7), as observed in the actual xHCI trace.
                pipes.append((3 << 30) | (dev << 8) | (address << 15) | (address & 128))
        if len(pipes) != 2:
            raise ValueError('Expected one bulk input and one bulk output endpoint')
        result[name] = {'busnum': int((path/'busnum').read_text()), 'devnum': dev, 'pipes': sorted(pipes)}
    if len({p for v in result.values() for p in v['pipes']}) != sum(len(v['pipes']) for v in result.values()):
        raise ValueError('Ambiguous USB pipe mapping')
    return result


def capture(output, names, seconds, *, discover_bulk=False):
    if os.geteuid() != 0 or not 1 <= seconds <= 30:
        raise ValueError('Root and a finite 1..30 second capture required')
    mapping = device_pipes(names)
    # Discovery is deliberately short and still records metadata only.  It is
    # needed when the controller's receive URBs do not match the pipe values
    # reconstructed from the endpoint descriptors.  Do not infer that a
    # nonzero tracepoint status is a USB error without checking kernel timing.
    filters = ('type == 2' if discover_bulk else '(' + ' || '.join(
        'pipe == ' + str(p) for m in mapping.values() for p in m['pipes']) + ')')
    output.mkdir(mode=0o700, parents=False, exist_ok=False)
    report = {'schema': 'xhci-metadata-v1', 'devices': mapping, 'seconds': seconds,
              'clock': 'mono', 'motor_writes': 0, 'serial_devices_opened': False,
              'payload_captured': False, 'can_delivery_proven': False,
              'bulk_pipe_discovery': discover_bulk,
              'nonzero_trace_status_is_usb_error_proven': False,
              'pipe_filter_has_no_bus_field': True, 'instances_removed': False,
              'trace_lossless': False,
              'status': 'PREPARING', 'errors': [], 'instance_filters': {}}
    instances = []
    previous = {}
    def interrupted(signum, _):
        raise InterruptedError(f'Capture interrupted by signal {signum}')
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, interrupted)
    try:
        for kind in ('all',):
            path = ROOT/'instances'/('dog-usb-' + uuid.uuid4().hex[:12] + '-' + kind)
            path.mkdir()
            instances.append((kind, path))
            (path/'tracing_on').write_text('0')
            (path/'buffer_size_kb').write_text('4096')
            (path/'trace_clock').write_text('mono')
            if '[mono]' not in (path/'trace_clock').read_text():
                raise RuntimeError('Monotonic trace clock was not selected')
            for event in EVENTS:
                event_path = path/'events/xhci-hcd'/event
                fmt = (event_path/'format').read_text()
                (output/(kind + '-' + event + '-format.txt')).write_text(fmt)
                if 'field:unsigned int pipe;' not in fmt or 'field:int status;' not in fmt:
                    raise RuntimeError('Unexpected kernel event fields')
                (event_path/'filter').write_text(filters)
                report['instance_filters'][kind + '/' + event] = (event_path/'filter').read_text().strip()
                (event_path/'enable').write_text('1')
            (path/'tracing_on').write_text('1')
        report['begin_monotonic_ns'] = time.monotonic_ns()
        print('USB_TRACE_READY', flush=True)
        time.sleep(seconds)
        report['status'] = 'CAPTURED_REVIEW_REQUIRED'
    except BaseException as error:
        report['status'] = 'INCOMPLETE'
        report['errors'].append(repr(error))
    finally:
        report['end_monotonic_ns'] = time.monotonic_ns()
        for _, path in instances:
            try:
                (path/'tracing_on').write_text('0')
            except OSError as error:
                report['errors'].append('disable: ' + repr(error))
        for kind, path in instances:
            try:
                (output/(kind + '-trace.txt')).write_text((path/'trace').read_text())
                report[kind + '_cpu_stats'] = {p.parent.name: p.read_text()
                                               for p in path.glob('per_cpu/cpu*/stats')}
            except OSError as error:
                report['errors'].append('read: ' + repr(error))
            try:
                (path/'events/enable').write_text('0')
                path.rmdir()  # tracefs removes its virtual children; never shell rm -r.
            except OSError as error:
                report['errors'].append('cleanup: ' + repr(error))
        report['instances_removed'] = bool(instances) and all(not p.exists() for _, p in instances)
        report['trace_lossless'] = trace_stats_lossless(report.get('all_cpu_stats', {}))
        if not report['trace_lossless']:
            report['errors'].append('Trace CPU buffer overrun, dropped events, or unreadable counters')
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if report['errors']:
            report['status'] = 'INCOMPLETE'
        (output/'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'instances_removed': report['instances_removed'],
                      'trace_lossless': report['trace_lossless'],
                      'output': str(output), 'motor_writes': 0}), flush=True)
    return 0 if report['status'] == 'CAPTURED_REVIEW_REQUIRED' and report['instances_removed'] else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--usb-device', action='append', required=True)
    p.add_argument('--seconds', type=int, default=15)
    p.add_argument('--discover-bulk', action='store_true',
                   help='Short metadata-only capture of all bulk pipes to identify actual receive pipe IDs')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    raise SystemExit(capture(args.output, args.usb_device, args.seconds,
                             discover_bulk=args.discover_bulk))


if __name__ == '__main__':
    main()
