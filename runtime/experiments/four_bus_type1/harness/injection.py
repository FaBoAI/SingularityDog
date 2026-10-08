"""Deterministic fault injections for the motor-free harness (harness process only).

* ``--inject-hold-tail PORT:CYCLE:+MS``: the synthetic peer on PORT answers the
  later replies (requests 2 and 3) of cycle CYCLE's hold burst MS later than its
  latency model draws (the r2 cycle 70 shape: one missed CH341 read turn).
* ``--inject-voltage-tail PORT:CYCLE:+MS``: the same for cycle CYCLE's rotating
  Type17 voltage reply on PORT.
* ``--inject-host-stall CYCLE:MS[:gil|sleep]``: the runner's main thread stalls
  for MS at release+0.16 ms of cycle CYCLE (the r1 cycle 26 location: after
  Boundary 1 and the hold gates, before any hold write). Default path: in the
  first hold ``submit`` of that cycle (the runner's owner pools are a harness
  subclass only while a stall is selected); pre-armed (F3): right after the
  actual release wait returns, before the IMU submit. ``gil`` (default) keeps the
  GIL held while the thread is off-CPU (``PyDLL`` usleep: a preempted GIL
  holder); ``sleep`` releases the GIL (a descheduled main thread). The stall
  starts at max(hook, release+0.16 ms); its actual length is recorded (macOS
  timers oversleep by ~1-2 ms).

Tails are sent to the peer over a control pipe before the cycle's release wait
(the previous cycle's output burst has fully replied by then), and the peer
applies each to the next matching burst on that port. The draw sequence of the
latency model is unchanged, so a seed replays with and without an injection.
With no injection selected nothing is wrapped and no control pipe exists.
Nothing here opens a device or changes any runner check or limit.
"""
import ctypes
import math
import os
import time

PORTS = ('port0', 'port1', 'port2', 'port3')
STALL_AT_NS = 160_000
MAX_TAIL_MS = 10.
MAX_STALL_MS = 15.
STALL_MODES = ('gil', 'sleep')


def need(condition, message):
    if not condition:
        raise ValueError(message)


def _port(text):
    text = text.strip()
    name = text if text.startswith('port') else 'port'+text
    need(name in PORTS, 'Injection port must be port0..port3')
    return name


def _cycle(text):
    need(text.strip().isdigit(), 'Injection cycle must be a non-negative integer')
    return int(text)


def _ms(text, limit):
    try:
        value = float(text)
    except ValueError:
        raise ValueError('Injection milliseconds must be a number')
    need(math.isfinite(value) and 0 < value <= limit, 'Injection milliseconds must be in (0, %g]' % limit)
    return value


def parse_tail(text, stage):
    """'PORT:CYCLE:+MS' -> {'stage', 'port', 'cycle', 'extra_ns'}; '+' is required."""
    parts = text.split(':')
    need(len(parts) == 3 and parts[2].startswith('+'), stage+' tail must be PORT:CYCLE:+MS')
    value = _ms(parts[2][1:], MAX_TAIL_MS)
    return {'stage': stage, 'port': _port(parts[0]), 'cycle': _cycle(parts[1]), 'extra_ms': value,
            'extra_ns': round(value*1e6)}


def parse_stall(text):
    """'CYCLE:MS[:gil|sleep]' -> {'cycle', 'ms', 'mode'}."""
    parts = text.split(':')
    need(len(parts) in (2, 3), 'Host stall must be CYCLE:MS[:gil|sleep]')
    mode = parts[2] if len(parts) == 3 else 'gil'
    need(mode in STALL_MODES, 'Host stall mode must be gil or sleep')
    return {'cycle': _cycle(parts[0]), 'ms': _ms(parts[1], MAX_STALL_MS), 'mode': mode}


def parse(hold_tails=(), voltage_tails=(), stalls=()):
    tails = [parse_tail(t, 'hold') for t in hold_tails or ()]+[parse_tail(t, 'voltage') for t in voltage_tails or ()]
    keys = [(t['stage'], t['port'], t['cycle']) for t in tails]
    need(len(keys) == len(set(keys)), 'Duplicate tail injection for one stage/port/cycle')
    stalls = [parse_stall(s) for s in stalls or ()]
    need(len({s['cycle'] for s in stalls}) == len(stalls), 'Duplicate host stall for one cycle')
    return tails, stalls


_GIL_HELD = None


def stall_gil(ns):
    """Off-CPU with the GIL held (PyDLL keeps it across the foreign call)."""
    global _GIL_HELD
    if _GIL_HELD is None:
        libc = ctypes.PyDLL(None)
        libc.usleep.argtypes = [ctypes.c_uint]
        _GIL_HELD = libc.usleep
    remaining = ns
    while remaining > 0:  # usleep may refuse >= 1 s on some libcs; stalls are <= 15 ms.
        step = min(remaining, 900_000_000)
        _GIL_HELD(max(1, step//1000))
        remaining -= step


def stall_sleep(ns, clock=time.monotonic_ns):
    """Off-CPU with the GIL released (time.sleep), until at least ``ns`` elapsed."""
    end = clock()+ns
    while True:
        remaining = end-clock()
        if remaining <= 0:
            return
        time.sleep(remaining/1e9)


class Injections:
    """Wraps the release wait and the current guard; ``empty`` wraps nothing."""
    def __init__(self, tails=(), stalls=(), *, prearmed=False, control_fd=None, clock=time.monotonic_ns,
                 stall_functions=None):
        self.tails, self.stalls = list(tails), {s['cycle']: s for s in stalls}
        need(not self.tails or control_fd is not None, 'Tail injection needs the peer control pipe')
        self.prearmed, self.control_fd, self.clock = prearmed, control_fd, clock
        self.stall_functions = stall_functions or {'gil': stall_gil, 'sleep': lambda ns: stall_sleep(ns, clock)}
        self.calls = 0
        self.sent, self.performed = [], []
        self.pending = None

    @property
    def empty(self):
        return not self.tails and not self.stalls

    def wrap_release_wait(self, wait):
        if self.empty:
            return wait

        def wrapped(scheduled):
            per_cycle = 2 if self.prearmed else 1
            cycle, phase = divmod(self.calls, per_cycle)
            self.calls += 1
            if phase == 0:
                for tail in self.tails:
                    if tail['cycle'] == cycle:
                        line = '%s %s %d %d\n' % (tail['stage'], tail['port'], tail['extra_ns'], cycle)
                        need(os.write(self.control_fd, line.encode()) == len(line), 'Peer control write short')
                        self.sent.append(dict(tail, sent_ns=self.clock()))
            actual = wait(scheduled)
            if phase == per_cycle-1 and cycle in self.stalls:
                if self.prearmed:
                    self._stall(cycle, scheduled, 'after_actual_release_wait_prearmed')
                else:
                    self.pending = (cycle, scheduled)
            return actual
        return wrapped

    def executor_class(self, base):
        """Default path only: stall on main in the first hold submit of a stalled cycle."""
        if not self.stalls or self.prearmed:
            return base
        injections = self

        class StallingExecutor(base):
            def submit(self, *args, **kwargs):
                if injections.pending is not None:
                    cycle, release = injections.pending
                    injections.pending = None
                    injections._stall(cycle, release, 'before_first_hold_submit')
                return super().submit(*args, **kwargs)
        return StallingExecutor

    def _stall(self, cycle, release, hook):
        spec = self.stalls[cycle]
        while self.clock() < release+STALL_AT_NS:
            pass
        started = self.clock()
        self.stall_functions[spec['mode']](round(spec['ms']*1e6))
        ended = self.clock()
        self.performed.append({'cycle': cycle, 'mode': spec['mode'], 'requested_ms': spec['ms'], 'hook': hook,
                               'release_ns': release, 'start_ns': started, 'end_ns': ended,
                               'start_after_release_ms': (started-release)/1e6, 'actual_ms': (ended-started)/1e6})

    def report(self, peer_applied=None):
        applied = list(peer_applied or ())
        return {'deterministic': True, 'stall_at_after_release_ms': STALL_AT_NS/1e6,
                'tails_requested': self.tails, 'tails_sent': self.sent, 'tails_applied_by_peer': applied,
                'tails_not_applied': [t for t in self.sent if not any(
                    a['stage'] == t['stage'] and a['port'] == t['port'] and a['cycle'] == t['cycle'] for a in applied)],
                'stalls_requested': list(self.stalls.values()), 'stalls_performed': self.performed}

    def row_fields(self, cycle, applied=()):
        """Per-cycle labels for the harness rows (only injected cycles get them)."""
        fields = {}
        for item in applied:
            if item['cycle'] == cycle:
                fields[f"inject_{item['stage']}_tail_{item['port']}_ms"] = item['extra_ns']/1e6
        for item in self.performed:
            if item['cycle'] == cycle:
                fields.update(inject_host_stall_ms=item['actual_ms'], inject_host_stall_mode=item['mode'],
                              inject_host_stall_start_after_release_ms=item['start_after_release_ms'])
        return fields
