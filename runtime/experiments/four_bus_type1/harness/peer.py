"""Synthetic RS05 x3 peers for four socketpair "USB2CAN ports" (separate process).

Motor-free timing harness only. This process owns the peer ends of four
anonymous socketpairs (passed by file descriptor), parses the host's AT frames
and answers like three RS05 motors behind one USB2CAN adapter, with reply
timing drawn from ``latency_model`` (recorded hardware delays):

* first request of a burst: reply after ``*_first`` (~1.75 ms after the write)
* later requests of the same burst: all replies sent together, once, after
  ``*_rest`` measured from the last write of the burst (~2.8 ms)
* a lone Type17 (rotating voltage read): ``single`` (~2.78 ms)

Optional deterministic injections (``--control-fd``, see ``injection.py``): a
control line ``hold PORT EXTRA_NS CYCLE`` adds EXTRA_NS to the later replies of
the next Type1 burst on PORT, ``voltage PORT EXTRA_NS CYCLE`` to the next Type17
single. The latency-model draws are unchanged; each application is logged.

Type2 replies are mode 2 for Type1 and Type3, mode 0 for STOP/Type18/other.
Type17 voltage replies 39-40 V. It runs in its own process (own GIL), pinned
to CPU5 on Linux with 1 us timer slack, and spins for the last 200 us before
each due reply. It opens no device and sends nothing anywhere but the four
socketpair ends it was given.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import random
import select
import socket
import struct
import sys
import time

from singularitydog_hw import can_readonly as codec
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.motor_version_probe import VERSION_PREFIX
try:
    from experiments.four_bus_type1.harness import latency_model
except ImportError:  # Run by path from a copied harness directory.
    import latency_model

SPIN_NS = 200_000 if sys.platform == 'linux' else 3_000_000  # macOS select oversleeps by ~1 ms
BURST_SPLIT_NS = 1_500_000
FREQ = []
BUSY = False  # --peer-busy-spin: poll continuously (keeps the CPU4/5 cpufreq policy at its maximum under schedutil)


def frame(can_id, data):
    return b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def u16(raw):
    return round((raw+12.57)*65535/25.14)


def type2(mid, raw, *, mode, fault=0, temperature_dc=250):
    return frame((2 << 24) | (mode << 22) | (fault << 16) | (mid << 8) | 0xfd,
                 struct.pack('>4H', u16(raw), 32767, 32767, temperature_dc))


def type17(mid, name, value):
    index, fmt, _ = codec.PARAMETERS[name]
    data = struct.pack('<H', index)+bytes(2)+struct.pack('<'+fmt, value)
    return frame((17 << 24) | (mid << 8) | 0xfd, data+bytes(8-len(data)))


NAMES = {codec.PARAMETERS[name][0]: name for name in codec.PARAMETERS}


class Motors:
    def __init__(self, config, seed):
        self.raw = {int(k): v for k, v in config['raw_by_id'].items()}
        self.uid = {int(k): v for k, v in config['uid_by_id'].items()}
        self.firmware = {int(k): v for k, v in config['firmware_by_id'].items()}
        self.random = random.Random(seed)
        self.enabled = set()
        self.kinds = {}

    def reply(self, value):
        mid, kind = value.destination, value.kind
        self.kinds[kind] = self.kinds.get(kind, 0)+1
        if kind == 0:
            return frame((mid << 8) | 0xfe, bytes.fromhex(self.uid[mid]))
        if kind == 17:
            name = NAMES[int.from_bytes(value.data[:2], 'little')]
            values = {'run_mode': 0, 'voltage': 39.+self.random.random(), 'can_timeout': 4000,
                      'position': self.raw[mid], 'velocity': 0.}
            return type17(mid, name, values[name])
        if kind == 4 and value.data[:2] == b'\x00\xc4':
            return frame((2 << 24) | (mid << 8) | 0xfd, VERSION_PREFIX+bytes.fromhex(self.firmware[mid])+b'\0')
        if kind == 3:
            self.enabled.add(mid)
            return type2(mid, self.raw[mid], mode=2)
        if kind == 1:
            return type2(mid, self.raw[mid], mode=2 if mid in self.enabled else 0)
        if kind == 4:
            self.enabled.discard(mid)
        return type2(mid, self.raw[mid], mode=0)


def setup_process(cpu, slack_ns):
    report = {'platform': sys.platform}
    if sys.platform == 'linux':
        if cpu is not None:
            os.sched_setaffinity(0, {cpu})
            report['cpu_mask'] = sorted(os.sched_getaffinity(0))
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(29, ctypes.c_ulong(slack_ns), 0, 0, 0) != 0:  # PR_SET_TIMERSLACK
            report['timer_slack_error'] = ctypes.get_errno()
        report['timer_slack_ns'] = libc.prctl(30, 0, 0, 0, 0)  # PR_GET_TIMERSLACK
    return report


def read_control(control_fd, buffer, pending, by_name):
    """Queue control lines per (fd, stage); returns False when the host closed the pipe."""
    data = os.read(control_fd, 4096)
    if not data:
        return False
    buffer += data
    while b'\n' in buffer:
        line, _, rest = bytes(buffer).partition(b'\n')
        buffer[:] = rest
        stage, port, extra, cycle = line.decode().split()
        if stage not in ('hold', 'voltage') or port not in by_name:
            raise ValueError('Peer control line invalid')
        pending.setdefault((by_name[port], stage), []).append((int(extra), int(cycle), port))
    return True


def serve(sockets, motors, sampler, log, *, stop_fd, control_fd=None, port_names=None, applied=None):
    """Single-threaded event loop; returns when the host closes every socket or stop_fd closes."""
    parsers = {fd: ATParser() for fd in sockets}
    state = {fd: {'last_rx': 0, 'pending_rest': [], 'rest_due': None, 'family': None, 'extra': 0} for fd in sockets}
    singles = []  # (due_ns, fd, bytes, meta)
    open_fds = set(sockets)
    readers = [sockets[fd] for fd in sockets]+[stop_fd]
    pending, buffer = {}, bytearray()
    by_name = {name: fd for fd, name in (port_names or {}).items()}
    if control_fd is not None:
        readers.insert(0, control_fd)  # Serviced first: an armed injection precedes its burst.
    freq_paths = sorted(Path('/sys/devices/system/cpu/cpufreq').glob('policy[0-9]*/scaling_cur_freq')) \
        if sys.platform == 'linux' else []
    next_freq = 0
    while open_fds:
        now = time.monotonic_ns()
        if freq_paths and now >= next_freq and not any(s['rest_due'] for s in state.values()) and not singles:
            # Read-only cpufreq sample every 100 ms, only while no reply is pending.
            FREQ.append((now, [int(p.read_text()) for p in freq_paths]))
            next_freq = now+100_000_000
        due = [item[0] for item in singles]+[s['rest_due'] for s in state.values() if s['rest_due'] is not None]
        nearest = min(due) if due else None
        timeout = (0. if BUSY else .05) if nearest is None else max(0., (nearest-now-SPIN_NS)/1e9)
        ready, _, _ = select.select(readers, [], [], timeout)
        if control_fd is not None and control_fd in ready:
            ready.remove(control_fd)
            if not read_control(control_fd, buffer, pending, by_name):
                readers.remove(control_fd); control_fd = None
        for sock in ready:
            if sock is stop_fd:
                if not os.read(stop_fd, 64):
                    return
                continue
            fd = sock.fileno()
            try:
                raw = sock.recv(4096)
            except BlockingIOError:
                continue
            arrived = time.monotonic_ns()
            if not raw:
                open_fds.discard(fd); readers.remove(sock)
                continue
            for value in parsers[fd].feed(raw):
                port = state[fd]
                family = 'type1' if value.kind == 1 else 'stop'
                response = motors.reply(value)
                meta = (fd, value.destination, value.kind, arrived)
                if (port['rest_due'] is None and not singles_for(singles, fd) and
                        arrived-port['last_rx'] > BURST_SPLIT_NS):
                    # First request of a new burst (or the lone rotating voltage read).
                    key = 'single' if value.kind == 17 else family+'_first'
                    due = arrived+sampler(key)
                    stage = 'voltage' if value.kind == 17 else 'hold' if value.kind == 1 else None
                    if pending.get((fd, stage)):
                        extra, cycle, name = pending[fd, stage].pop(0)
                        applied.append({'stage': stage, 'port': name, 'cycle': cycle, 'extra_ns': extra,
                                        'arrived_ns': arrived})
                        if stage == 'voltage':
                            due += extra
                        else:
                            port['extra'] = extra  # Later replies of this hold burst.
                    singles.append((due, fd, response, meta))
                else:
                    port['pending_rest'].append((response, meta))
                    port['rest_due'] = arrived+sampler(family+'_rest')+port['extra']
                port['last_rx'] = arrived
        # Spin to each due time, then send.
        while True:
            now = time.monotonic_ns()
            due = [item for item in singles if item[0] <= now]
            rest = [fd for fd, s in state.items() if s['rest_due'] is not None and s['rest_due'] <= now]
            if not due and not rest:
                nearest = min([item[0] for item in singles]+[s['rest_due'] for s in state.values()
                                                             if s['rest_due'] is not None], default=None)
                if nearest is None or nearest-now > SPIN_NS:
                    break
                if select.select(readers, [], [], 0)[0]:
                    break  # Service host writes first; due replies are re-examined right after.
                continue
            for item in due:
                singles.remove(item)
                send(sockets[item[1]], item[2], [(item[3], item[0])], log)
            for fd in rest:
                port = state[fd]
                payload = b''.join(response for response, _ in port['pending_rest'])
                send(sockets[fd], payload, [(meta, port['rest_due']) for _, meta in port['pending_rest']], log)
                port['pending_rest'], port['rest_due'], port['extra'] = [], None, 0


def singles_for(singles, fd):
    return any(item[1] == fd for item in singles)


def send(sock, payload, metas, log):
    try:
        sock.sendall(payload)
    except OSError:
        return
    sent = time.monotonic_ns()
    for (fd, mid, kind, arrived), due in metas:
        log.append((fd, mid, kind, arrived, due, sent))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fds', required=True, help='comma-separated inherited peer socket FDs (port order)')
    parser.add_argument('--stop-fd', type=int, required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--log', required=True)
    parser.add_argument('--control-fd', type=int, help='optional injection control pipe (read end)')
    args = parser.parse_args(argv)
    config = json.loads(Path(args.config).read_text())
    global SPIN_NS, BUSY
    BUSY = bool(config.get('busy_spin'))
    SPIN_NS = int(config.get('spin_ns') or SPIN_NS)
    process = setup_process(config.get('cpu'), config.get('timer_slack_ns', 1000))
    process['spin_ns'] = SPIN_NS
    process['busy_spin'] = BUSY
    model = latency_model.load(config.get('latency_model_path') or latency_model.DEFAULT_PATH)
    sampler = latency_model.Sampler(model, config.get('latency_mode', 'empirical'), config.get('seed', 1),
                                    fixed=config.get('fixed_latency_ns'))
    motors = Motors(config, config.get('seed', 1))
    sockets = {}
    ports = {}
    for index, fd in enumerate(int(x) for x in args.fds.split(',')):
        sock = socket.socket(fileno=fd)
        sock.setblocking(False)
        sockets[sock.fileno()] = sock
        ports[sock.fileno()] = config['ports'][index]
    log, applied = [], []
    started = time.monotonic_ns()
    try:
        serve(sockets, motors, sampler, log, stop_fd=args.stop_fd, control_fd=args.control_fd,
              port_names=ports, applied=applied)
    finally:
        lateness = [(sent-due) for (_, _, _, _, due, sent) in log]
        summary = {'process': process, 'latency_mode': sampler.mode, 'medians_ns': sampler.medians,
                   'replies': len(log), 'kinds': {str(k): v for k, v in motors.kinds.items()},
                   'started_ns': started, 'finished_ns': time.monotonic_ns(),
                   'send_lateness_ns': latency_model.describe(lateness, scale=1) if lateness else {'n': 0},
                   'send_lateness_over_100us': sum(1 for x in lateness if x > 100_000),
                   'cpufreq_samples': {'fields': ['monotonic_ns', 'policy_cur_khz...'], 'rows': FREQ},
                   'rows_fields': ['port', 'mid', 'kind', 'arrived_ns', 'due_ns', 'sent_ns'],
                   **({'injections_applied': applied} if args.control_fd is not None else {}),
                   'rows': [(ports[fd], mid, kind, arrived, due, sent) for fd, mid, kind, arrived, due, sent in log]}
        Path(args.log).write_text(json.dumps(summary, separators=(',', ':')))


if __name__ == '__main__':
    main()
