"""Motor-free four-bus Type1 timing harness (zero_gain_timing, real runner/transport/library).

MEASUREMENT HARNESS ONLY. It never opens /dev/ttyUSB*, /dev/serial/*, I2C or
any motor/IMU device, never writes /sys, needs no root and sends nothing
anywhere except four anonymous socketpairs to its own synthetic peer process.
Nothing it reports is a timing qualification or an output approval.

What is REAL here (unchanged repository code, imported):
  * ``type1_runner.run`` (all phases A-J, the cycle loop, every gate)
  * ``type1_transport.Type1Transport`` created by ``Type1Transport.create`` over
    the genuine subset-active native library loaded by ``type1_transport.load_library``
    (``sda_subset_exchange``: 900 us request gap, window 3, native deadlines)
  * native release wait ``four_bus_diagnostic.foreground.make_release_wait`` (spin 500 us)
  * Linux main scope ``foreground.MainScope`` (CPU4, switch interval 100 us,
    timer slack 1000 ns, GC deferred) and ``pipeline.LinuxWorkerScope``
    (owners CPU0..3, IMU 0..3, slack 1000 ns), the host OutputWatchdog,
    ``foreground.read_imu``, envelope ``PolicyMotionEnvelope``, encoder, budget.

What is SYNTHETIC (labelled in the report):
  * four RS05x3 peers in a separate process (``peer.py``) on CPU5, replying
    with delays drawn from recorded hardware (``latency_model.json``)
  * IMU device (``read_sample`` blocks ~0.9 ms in GIL-released sleeps)
  * current guard (same syscall shape as the foreground guard on scratch files)
  * observer: ``synthetic`` (Python work + chunked GIL-released CPU burn, sized
    from the recorded consume_profile) or ``torch-policy`` (the real checked
    policy module loaded read-only from a model profile; Jetson only)

Envelope gap handling (``--envelope-gap-mode``):
  * ``strict`` (default): the runner's real behaviour; the wrapper only records
    command/sample intervals and the first >21 ms interval aborts the run.
  * ``measure``: CLEARLY-LABELLED MEASUREMENT MODE. A command/sample interval
    above max_sample_gap_s is recorded as a would-be violation and only that
    one comparison is skipped for that call, so the run continues; every other
    envelope check, every runner gate and the 21 ms value itself are unchanged.
    This lives only in this harness process (wrapping the envelope class the
    runner instantiates); repository files are not modified.

Usage (from ``runtime/``; macOS validation builds a fresh library):
  PYTHONPATH=. python3 -B -m experiments.four_bus_type1.harness.run_harness \
      --output /fresh/dir --build-library --duration 2 --envelope-gap-mode measure
Jetson (harness directory copied next to an unmodified kit; no root, no devices):
  PYTHONPATH=$KIT/runtime $VENV/bin/python3 -B harness/run_harness.py --output /fresh/dir \
      --library $BUILD/libdog_four_bus_type1_transport.so --duration 20 --envelope-gap-mode measure \
      --observer torch-policy --model-profile $PROFILE --cpu-keepalive 0,1,2,3,4 --seed N
  (--cpu-keepalive / --uclamp-min only EMULATE the root power scope; under the canonical
   jetson_latency_power_scope.py scope run without them.)
Exact Jetson commands (tests, one pinned build, run matrix, injection replays, analysis): README.md here.
Opt-in runner options (default: none, the reviewed R8 pacing) are forwarded unchanged:
  --command-phase-offset-us K, --decode-once, --prearmed-hold-lead-us L, --gc-freeze, --timing-evidence
  (F3 uses the genuine native sda_subset_exchange_at of the loaded library, never an emulation: it needs
   a build whose receipt records exchange_at_abi 1; --build-library builds the current source).
Deterministic injections (``injection.py``; none by default, then nothing is wrapped):
  --inject-hold-tail PORT:CYCLE:+MS, --inject-voltage-tail PORT:CYCLE:+MS, --inject-host-stall CYCLE:MS[:gil|sleep]
Outputs: harness-report.json (summary + per-cycle rows), cycles.csv, peer-log.json;
``analyze.py RUN_DIR...`` summarises per configuration and re-attributes every >21 ms interval offline.
Rows carry the natural gate (``natural_gate_after_release_ms``, the final-gate end) and the
command time actually given to the envelope (``command_after_release_ms``) separately.
"""
import argparse
import copy
import csv
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

for _name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')  # Before any Torch import (single-thread math).

from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.policy_motion_envelope import PolicyMotionEnvelope
from experiments.four_bus_diagnostic import foreground as FG
from experiments.four_bus_diagnostic import pipeline
from experiments.four_bus_diagnostic.transport_adapter import Batch, Group, PORTS
from experiments.four_bus_type1 import build, type1_runner as R, type1_transport as T
from experiments.four_bus_type1 import type1_foreground as F, type1_profile as P
from experiments.four_bus_type1.timing_evidence import TimingEvidence
from experiments.four_bus_type1 import test_type1_runner as fixture
try:
    from experiments.four_bus_type1.harness import injection, latency_model
except ImportError:  # Copied harness directory run by path next to an unmodified kit runtime.
    import injection
    import latency_model

SCHEMA = 'singularitydog.four-bus-type1-motor-free-timing-harness.v1'
HERE = Path(__file__).resolve().parent
LINUX = sys.platform == 'linux'
RUNTIME = Path(R.__file__).resolve().parents[2]
SOURCE_FILES = ('experiments/four_bus_type1/type1_runner.py', 'experiments/four_bus_type1/type1_transport.py',
                'experiments/four_bus_type1/type1_profile.py', 'experiments/four_bus_type1/type1_foreground.py',
                'experiments/four_bus_type1/timing_evidence.py',
                'experiments/four_bus_type1/test_type1_runner.py', 'experiments/four_bus_type1/build.py',
                'experiments/four_bus_type1/subset_active.cpp', 'experiments/four_bus_diagnostic/foreground.py',
                'experiments/four_bus_diagnostic/pipeline.py', 'experiments/four_bus_diagnostic/transport_adapter.py',
                'singularitydog_hw/policy_motion_envelope.py', 'singularitydog_hw/native_active_transport.py',
                'singularitydog_hw/policy_output_runtime.py', 'singularitydog_hw/policy_post_reply_timing.py')
HARNESS_FILES = ('run_harness.py', 'peer.py', 'latency_model.py', 'latency_model.json', 'injection.py', 'analyze.py')
DEG = math.radians(1)
GAP_LIMIT_MS = 21.
MARGIN_MS = 20.8  # Synthesis acceptance: zero intervals above this.
# Recorded consume_profile (t1z2r1/r2 medians, ms): python before model .60, model_call 1.65, after .14.
CONSUME_PYTHON_BEFORE_MS = .60
CONSUME_MODEL_MS = 1.65
CONSUME_PYTHON_AFTER_MS = .14
IMU_SELECT_MS = .10
IMU_TRANSACTION_SPLIT = (.17, .66, .17)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def need(condition, message):
    if not condition:
        raise RuntimeError(message)


# ----------------------------------------------------------------------------- envelope wrapper
class EnvelopeRecorder:
    """Per-step command/sample interval record; label of the gap handling mode."""
    def __init__(self, mode):
        if mode not in ('strict', 'measure'):
            raise ValueError('Envelope gap mode must be strict or measure')
        self.mode, self.rows, self.created = mode, [], []

    def envelope_class(self):
        recorder = self

        class RecordingEnvelope(PolicyMotionEnvelope):
            """Unchanged PolicyMotionEnvelope; only the gap comparison may be skipped in measure mode."""
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                recorder.created.append({'now_s': kwargs.get('now_s'), 'max_sample_gap_s': self.max_sample_gap_s,
                                         'initial_sample_monotonic_s': self._last_sample_at})

            def step(self, target_model_rad, sample, *, now_s):
                dt = now_s-self._last_now
                sample_dt = sample.monotonic_s-self._last_sample_at
                limit = self.max_sample_gap_s
                violation = dt > limit or sample_dt > limit
                row = {'step': len(recorder.rows), 'now_ns': round(now_s*1e9),
                       'sample_monotonic_ns': round(sample.monotonic_s*1e9),
                       'command_interval_ms': dt*1e3, 'sample_interval_ms': sample_dt*1e3,
                       'command_gap_violation': dt > limit, 'sample_gap_violation': sample_dt > limit,
                       'gap_check_skipped_for_measurement': False, 'result': None}
                recorder.rows.append(row)
                if violation and recorder.mode == 'measure':
                    row['gap_check_skipped_for_measurement'] = True
                    self.max_sample_gap_s = math.inf
                    try:
                        command = super().step(target_model_rad, sample, now_s=now_s)
                    except BaseException as error:
                        row['result'] = type(error).__name__+': '+str(error)
                        raise
                    finally:
                        self.max_sample_gap_s = limit
                    row['result'] = 'ok_measurement_only'
                    return command
                try:
                    command = super().step(target_model_rad, sample, now_s=now_s)
                except BaseException as error:
                    row['result'] = type(error).__name__+': '+str(error)
                    raise
                row['result'] = 'ok'
                return command
        return RecordingEnvelope


# ----------------------------------------------------------------------------- scopes (macOS fallbacks)
class PortableMainScope:
    """macOS validation only: switch interval, GC deferral; no CPU/slack control exists there."""
    def __init__(self):
        self.report = {}

    def __enter__(self):
        self.original = {'switch_s': sys.getswitchinterval(), 'gc_enabled': gc.isenabled()}
        sys.setswitchinterval(.0001)
        gc.disable()
        during = {'main_cpu_mask': None, 'nice': os.getpriority(os.PRIO_PROCESS, 0), 'timer_slack_ns': None,
                  'switch_interval_s': sys.getswitchinterval(), 'single_thread_math_verified': False,
                  'power_scope_verified': False, 'gc_deferred_during_cycles': True, 'portable_non_linux_scope': True}
        self.report['during'] = during
        return during

    def __exit__(self, *exc):
        sys.setswitchinterval(math.nextafter(self.original['switch_s'], math.inf))
        if self.original['gc_enabled']:
            gc.enable()
        self.report['restored'] = True
        return False


class PortableWorkerScope:
    def __init__(self, port, mask):
        self.mask = list(mask)

    def __enter__(self):
        return {'native_tid': threading.get_native_id(), 'cpu_mask': self.mask, 'timer_slack_ns': None,
                'file_only_mock_readback': True, 'portable_non_linux_scope': True}

    def __exit__(self, *exc):
        return False


def portable_release_wait(cancel_fd, observer):
    """macOS validation only (pselect oversleeps ~2 ms there): sleep to -3 ms, then spin; Linux uses the native wait."""
    import select as _select
    armed = []
    def wait(scheduled):
        while True:
            if _select.select([cancel_fd], [], [], 0)[0]:
                raise RuntimeError('Cancelled before active release')
            remaining = scheduled-time.monotonic_ns()
            if remaining <= 0:
                break
            if remaining > 3_000_000:
                time.sleep((remaining-3_000_000)/1e9)
        if not armed:
            observer.arm_run(scheduled); armed.append(True)
        return time.monotonic_ns()
    return wait


def portable_command_wait(cancel_fd):
    """macOS validation only: the portable release wait without arming the observer (F1)."""
    import select as _select
    def wait(target):
        while True:
            if _select.select([cancel_fd], [], [], 0)[0]:
                raise RuntimeError('Cancelled during command phase wait')
            remaining = target-time.monotonic_ns()
            if remaining <= 0:
                return time.monotonic_ns()
            if remaining > 3_000_000:
                time.sleep((remaining-3_000_000)/1e9)
    return wait


class HarnessMainScope(FG.MainScope):
    """The foreground MainScope; nice/power readbacks are reported truthfully, not asserted."""


# ----------------------------------------------------------------------------- synthetic devices
class SyntheticIMU:
    """ICM20948 stand-in: ``read_sample`` blocks like the I2C ioctls (GIL released), returns sensor frame."""
    def __init__(self, durations_ms, seed):
        import random
        self.random = random.Random(seed)
        self.durations = durations_ms
        self.sequence = 0
        self.calls = []

    def read_sample(self):
        time.sleep(IMU_SELECT_MS/1e3)
        started = time.monotonic_ns()
        total = self.random.choice(self.durations)
        for part in IMU_TRANSACTION_SPLIT:
            time.sleep(total*part/1e3)
        finished = time.monotonic_ns()
        self.sequence += 1
        self.calls.append((started, finished, threading.get_native_id()))
        return {'sequence': self.sequence, 'frame': 'sensor', 'monotonic_ns': (started+finished)//2,
                'read_started_monotonic_ns': started, 'read_finished_monotonic_ns': finished,
                'timestamp_source': 'host_read_interval_midpoint', 'synthetic_harness_imu': True,
                'accel_m_s2': [0., 0., 9.81], 'gyro_rad_s': [0., 0., 0.]}


class EmulatedCurrentGuard:
    """Same syscall shape as foreground CurrentGuard (boot pread, ancestor lstat, alias/target stat) on scratch files."""
    def __init__(self, root, boot_fd, boot_bytes, stop):
        self.stop, self.boot_fd, self.boot_bytes = stop, boot_fd, boot_bytes
        base = root/'dev'
        (base/'serial'/'by-path').mkdir(parents=True)
        self.ports = {}
        for index, port in enumerate(PORTS):
            target = base/f'ttyUSB{index}'
            target.write_bytes(b'')
            alias = base/'serial'/'by-path'/f'synthetic-port{index}'
            alias.symlink_to(f'../../ttyUSB{index}')
            self.ports[port] = (str(alias), str(target), os.readlink(alias))
        self.ancestors = {str(p): os.lstat(p) for p in (base/'serial'/'by-path', base/'serial', base, root)}
        self.ancestors = {k: (v.st_dev, v.st_ino, v.st_mode) for k, v in self.ancestors.items()}
        self.identity = {port: (os.lstat(a).st_ino, os.stat(a).st_ino) for port, (a, _, _) in self.ports.items()}
        self.boot_identity = os.fstat(boot_fd).st_ino
        self.calls = []

    def __call__(self):
        started = time.monotonic_ns()
        try:
            need(not self.stop.is_set(), 'Foreground cancelled')
            need(os.fstat(self.boot_fd).st_ino == self.boot_identity and
                 os.pread(self.boot_fd, 128, 0).strip() == self.boot_bytes, 'Boot identity changed')
            for path, value in self.ancestors.items():
                info = os.lstat(path)
                need((info.st_dev, info.st_ino, info.st_mode) == value, 'Ancestor changed')
            for port, (alias, target, link) in self.ports.items():
                need(os.lstat(alias).st_ino == self.identity[port][0] and os.readlink(alias) == link and
                     os.stat(alias).st_ino == self.identity[port][1] == os.lstat(target).st_ino, 'Alias changed')
        finally:
            self.calls.append((started, time.monotonic_ns(), threading.get_native_id()))


# ----------------------------------------------------------------------------- observers
def _python_work(snapshot):
    """Real GIL-holding Python work of the observer's pre-model sections (copy, validate, JSON+hash)."""
    owned = copy.deepcopy(snapshot)
    values = [row['value'] for row in owned['motors']]
    if len(values) != 24 or not all(math.isfinite(v) for v in values):
        raise RuntimeError('Synthetic observer snapshot contract')
    text = json.dumps(owned, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def _pad_python(until_ns):
    """GIL-holding (switch-interval preemptible) Python busy loop."""
    x = 0
    while time.monotonic_ns() < until_ns:
        x += 1
    return x


class SyntheticObserver:
    """Time-realistic stand-in for StatefulPolicyObserver.consume; no Torch."""
    kind = 'synthetic'

    def __init__(self, python_before_ms, model_ms, python_after_ms, model_gil):
        self.before_ns, self.model_ns, self.after_ns = (int(x*1e6) for x in (python_before_ms, model_ms, python_after_ms))
        self.model_gil = model_gil
        self.targets = [fixture.MODEL[mid] for mid in shadow.CAN_ORDER]
        self.calls, self.profiles, self.armed = 0, [], None
        self.chunk = b'\0'*65536
        self.chunks_per_ms = None

    def calibrate(self):
        """Hash chunks per ms on the calling (main, pinned) thread; hashlib releases the GIL >2 KiB."""
        for _ in range(3):
            begin = time.perf_counter_ns(); count = 0
            while time.perf_counter_ns()-begin < 20_000_000:
                hashlib.sha256(self.chunk); count += 1
            rate = count/((time.perf_counter_ns()-begin)/1e6)
        self.chunks_per_ms = rate
        return {'hash_chunk_bytes': len(self.chunk), 'chunks_per_ms': rate}

    def arm_run(self, scheduled):
        self.armed = scheduled

    def model_call(self):
        end = time.monotonic_ns()+self.model_ns
        if self.model_gil == 'held':
            _pad_python(end)
            return
        while time.monotonic_ns() < end:  # One GIL release/re-acquire per ~40 us chunk, like per-op dispatch.
            hashlib.sha256(self.chunk)

    def consume(self, snapshot):
        t0 = time.monotonic_ns()
        digest = _python_work(snapshot)
        _pad_python(t0+self.before_ns)
        t1 = time.monotonic_ns()
        self.model_call()
        t2 = time.monotonic_ns()
        result = {'status': 'TICK_OBSERVED_NO_OUTPUT', 'tick_index': self.calls, 'output_allowed': False,
                  'q_target_rad_diagnostic_only': list(self.targets), 'provenance_sha256': digest}
        _pad_python(t2+self.after_ns)
        t3 = time.monotonic_ns()
        self.calls += 1
        self.profiles.append((t0, t1, t2, t3))
        return result

    def warmup(self, count):
        for _ in range(count):
            self.model_call()


class TorchPolicyObserver(SyntheticObserver):
    """The real checked policy module (read-only file load); Python sections as SyntheticObserver."""
    kind = 'torch_policy'

    def __init__(self, profile_path, python_before_ms, python_after_ms):
        super().__init__(python_before_ms, 0., python_after_ms, 'torch')
        import torch
        from array import array
        torch.set_num_threads(1)
        if torch.get_num_interop_threads() != 1:
            torch.set_num_interop_threads(1)
        from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked, policy_live_profile
        self.torch, self.checked = torch, checked
        profile = policy_live_profile.load_profile(profile_path, require_approved=False)
        original, proof = fk.diagnostic_load(profile)
        self.policy, self.wrapper, self.proof = checked.load(profile, original, proof, active=False)
        self.buffers = tuple(array('f', [0.]*n) for n in (3, 3, 3, 12, 12, 12))
        with torch.inference_mode():
            self.tensors = tuple(torch.frombuffer(b, dtype=torch.float32).reshape(1, len(b)) for b in self.buffers)
        rows = ([0., 0., 0.], [0., 0., -1.], [0., 0., 0.], [0., .4, -.8]*4, [0.]*12, [0.]*12)
        for buf, row in zip(self.buffers, rows):
            for index, value in enumerate(row):
                buf[index] = value
        self.description = {'profile_path': str(profile_path), 'profile_sha256': sha(profile_path),
                            'torch_version': torch.__version__, 'threads': torch.get_num_threads(),
                            'interop_threads': torch.get_num_interop_threads()}

    def calibrate(self):
        return {'model': 'torch_policy_checked_call'}

    def model_call(self):
        with self.torch.inference_mode():
            target = self.checked.checked_call(self.wrapper, self.tensors)
            self.policy.last_actor_output.tolist(); self.policy.last_observation.tolist()
        return target


def model_setup_for(observer):
    def setup(run_observer, *, pre_calls, post_calls):
        gc.collect()
        observer.warmup(pre_calls)
        observer.warmup(post_calls)
        return {'pre_calls': pre_calls, 'post_calls': post_calls, 'reset_verified': True,
                'harness_observer_kind': observer.kind, 'real_model_verified': False}
    return setup


# ----------------------------------------------------------------------------- environment readback
def read_text(path):
    try:
        return Path(path).read_text().strip()
    except OSError as error:
        return 'unreadable: '+type(error).__name__


def environment_readback():
    value = {'platform': sys.platform, 'python': sys.version.split()[0], 'pid': os.getpid(),
             'nice': os.getpriority(os.PRIO_PROCESS, 0), 'uid': os.getuid(),
             'switch_interval_s_at_start': sys.getswitchinterval(),
             'math_environment': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')}}
    if LINUX:
        import resource
        value['rlimit_nice'] = resource.getrlimit(resource.RLIMIT_NICE)
        value['affinity_at_start'] = sorted(os.sched_getaffinity(0))
        cpu = Path('/sys/devices/system/cpu')
        value['cpufreq'] = {p.name: {n: read_text(p/n) for n in ('scaling_min_freq', 'scaling_max_freq',
                                                                  'scaling_cur_freq', 'scaling_governor')}
                            for p in sorted((cpu/'cpufreq').glob('policy[0-9]*'))}
        value['cpuidle_disable'] = {f'cpu{c}': {read_text(s/'name'): read_text(s/'disable')
                                                for s in sorted((cpu/f'cpu{c}'/'cpuidle').glob('state[0-9]*'))}
                                    for c in range(os.cpu_count() or 6)}
        value['emc'] = {n: read_text(Path('/sys/class/devfreq/bwmgr')/n) for n in ('min_freq', 'max_freq', 'cur_freq')}
        value['loadavg'] = read_text('/proc/loadavg')
        value['power_scope_established'] = (
            all(row['scaling_min_freq'] == row['scaling_max_freq'] for row in value['cpufreq'].values()) and
            all(flags.get('c7') == '1' for flags in value['cpuidle_disable'].values()))
    return value


# ----------------------------------------------------------------------------- statistics
def describe(values):
    values = [v for v in values if v is not None]
    if not values:
        return {'n': 0}
    q = latency_model.quantile
    return {'n': len(values), 'min': min(values), 'median': q(values, .5), 'mean': sum(values)/len(values),
            'p90': q(values, .9), 'p99': q(values, .99), 'p999': q(values, .999), 'max': max(values),
            'stdev': (sum((v-sum(values)/len(values))**2 for v in values)/len(values))**.5}


def per_cycle_rows(measured, recorder, guard, imu, observer):
    cycles = measured.get('cycles') or []
    steps = recorder.rows
    guard_calls = sorted(guard.calls)
    observer_profiles = list(observer.profiles)
    rows = []
    # A cycle's window starts at its first own work: the F3 pre-arm wake (Boundary 1 runs before
    # the release and before begin_ns), else begin_ns. begin_ns stays None when an F3 cycle aborts
    # between the wake and the release; the release then bounds the window.
    begins = [next((c[k] for k in ('prearm_wake_ns', 'begin_ns', 'release_ns') if c.get(k) is not None), None)
              for c in cycles]
    for index, cycle in enumerate(cycles):
        begin = begins[index]
        nxt = next((b for b in begins[index+1:] if b is not None), math.inf)
        row = {key: cycle.get(key) for key in (
            'index', 'slot', 'label', 'completed', 'release_ns', 'begin_ns', 'first_ns', 'hold_first_write_ns',
            'hold_last_reply_ns', 'acquired_ns', 'gather_ns', 'infer_end_ns', 'voltage_last_reply_ns',
            'voltage_join_ns', 'final_gate_ns', 'encode_end_ns', 'output_submit_ns', 'output_first_write_ns',
            'final_write_ns', 'output_last_reply_ns', 'reply_return_ns', 'cycle_end_ns', 'hard_end_ns',
            'join_deadline_ns', 'request_count', 'weight')}
        for key in ('natural_gate_ns', 'command_ns', 'prearm_wake_ns', 'submit_done_ns', 'evidence_cost_ns'):
            if key in cycle:  # Present only when the corresponding option is selected.
                row[key] = cycle[key]
        release, gate = cycle.get('release_ns'), cycle.get('final_gate_ns')
        rel = lambda value, base=release: None if value is None or base is None else (value-base)/1e6
        # Natural gate = final-gate end (F1 records it again as natural_gate_ns); command = the time given to step.
        row['natural_gate_after_release_ms'] = rel(cycle.get('natural_gate_ns', gate))
        row['command_after_release_ms'] = rel(cycle.get('command_ns', gate))
        row['hold_first_write_after_release_ms'] = rel(cycle.get('hold_first_write_ns'))
        row['output_last_reply_after_begin_ms'] = rel(cycle.get('output_last_reply_ns'), cycle.get('begin_ns'))
        for port, value in (cycle.get('owner_entry_ns') or {}).items():
            row[f'hold_{port}_owner_entry_ns'] = value
        for name, counters in (cycle.get('thread_counter_delta') or {}).items():
            for field, value in (counters or {}).items():
                row[f'thread_{name}_{field}'] = value
        for stage in ('hold', 'voltage', 'output'):
            batches = cycle.get(stage) or {}
            for port in PORTS:
                batch = batches.get(port)
                if type(batch) is Batch:
                    row[f'{stage}_{port}_owner_begin_ns'] = int(batch.stats.begin_ns)
                    row[f'{stage}_{port}_first_write_ns'] = min(int(r.start_ns) for r in batch.records)
                    row[f'{stage}_{port}_last_reply_ns'] = max(int(r.received_ns) for r in batch.records)
                    row[f'{stage}_{port}_owner_completed_ns'] = int(batch.completed_ns)
                    row[f'{stage}_{port}_owner_end_ns'] = int(batch.stats.end_ns)
                    row[f'{stage}_{port}_records'] = ' '.join(
                        '%d/%d/%d/%d' % (r.start_ns, r.finish_ns, r.read_start_ns, r.received_ns) for r in batch.records)
        imu_value = cycle.get('imu')
        if isinstance(imu_value, dict):
            row['imu_read_started_ns'] = imu_value.get('read_started_monotonic_ns')
            row['imu_read_finished_ns'] = imu_value.get('read_finished_monotonic_ns')
        checks = [c for c in guard_calls if begin is not None and begin <= c[0] < nxt]
        for k, (start, end, _) in enumerate(checks[:3], 1):
            row[f'current_check{k}_start_ns'], row[f'current_check{k}_end_ns'] = start, end
        profiles = [p for p in observer_profiles if begin is not None and begin <= p[0] < nxt]
        if profiles:
            row['observer_start_ns'], row['observer_model_start_ns'], row['observer_model_end_ns'], row['observer_end_ns'] = profiles[0]
        if index < len(steps):
            step = steps[index]
            if cycle.get('command_ns', gate) is not None:  # Float s round trip; ~0 when aligned.
                row['envelope_now_minus_command_ns'] = step['now_ns']-cycle.get('command_ns', gate)
            row.update(envelope_now_ns=step['now_ns'], envelope_sample_ns=step['sample_monotonic_ns'],
                       command_interval_ms=step['command_interval_ms'], sample_interval_ms=step['sample_interval_ms'],
                       command_gap_violation=step['command_gap_violation'],
                       sample_gap_violation=step['sample_gap_violation'],
                       gap_check_skipped_for_measurement=step['gap_check_skipped_for_measurement'],
                       envelope_result=step['result'])
        rows.append(row)
    return rows


def attribute_replies(rows, peer_log):
    """Join host records to peer rows: write->peer arrival, modelled device delay, peer send, host read lag."""
    import bisect
    medians = peer_log.get('medians_ns') or {}
    by_key = {}
    for port, mid, kind, arrived, due, sent in peer_log['rows']:
        by_key.setdefault((port, mid, kind), []).append((arrived, due, sent))
    for value in by_key.values():
        value.sort()
    arrivals = {key: [v[0] for v in value] for key, value in by_key.items()}
    kinds = {'hold': 1, 'voltage': 17, 'output': 1}
    for row in rows:
        for stage, kind in kinds.items():
            for port in PORTS:
                text = row.get(f'{stage}_{port}_records')
                if not text:
                    continue
                ids = fixture.PORT_IDS[port]
                records = [tuple(int(x) for x in item.split('/')) for item in text.split()]
                mids = ids if len(records) == 3 else (None,)
                worst = {'write_to_peer_arrival_ms': 0., 'modelled_delay_excess_ms': 0., 'peer_send_late_ms': 0.,
                         'host_read_lag_ms': 0., 'write_spacing_max_ms': 0.}
                matched = []
                for k, (start, finish, read_start, received) in enumerate(records):
                    candidates = [key for key in by_key if key[0] == port and key[2] == kind and
                                  (mids[k] is None or key[1] == mids[k])]
                    best = None
                    for key in candidates:
                        i = bisect.bisect_left(arrivals[key], start)
                        if i < len(arrivals[key]) and arrivals[key][i]-start < 15_000_000:
                            item = by_key[key][i]
                            if best is None or item[0] < best[0]:
                                best = item
                    matched.append(best)
                    if k:
                        worst['write_spacing_max_ms'] = max(worst['write_spacing_max_ms'], (start-records[k-1][0])/1e6)
                last_arrival = max((m[0] for m in matched[1:] if m is not None), default=None)
                for k, ((start, finish, read_start, received), best) in enumerate(zip(records, matched)):
                    if best is None:
                        continue
                    arrived, due, sent = best
                    if kind == 17:
                        reference, median = arrived, medians.get('single')
                    elif k == 0:
                        reference, median = arrived, medians.get('type1_first')
                    else:  # Rest replies are scheduled from the burst's last write.
                        reference, median = last_arrival, medians.get('type1_rest')
                    worst['write_to_peer_arrival_ms'] = max(worst['write_to_peer_arrival_ms'], (arrived-start)/1e6)
                    if median is not None and reference is not None:
                        worst['modelled_delay_excess_ms'] = max(worst['modelled_delay_excess_ms'], (due-reference-median)/1e6)
                    worst['peer_send_late_ms'] = max(worst['peer_send_late_ms'], (sent-due)/1e6)
                    if received:
                        worst['host_read_lag_ms'] = max(worst['host_read_lag_ms'], (received-sent)/1e6)
                for name, value in worst.items():
                    row[f'{stage}_{port}_{name}'] = value


def classify(row, stage='hold', threshold_ms=.5):
    """Dominant cause of a late stage on its worst port."""
    causes = []
    for port in PORTS:
        for name, label in (('modelled_delay_excess_ms', 'modelled_device_reply_tail'),
                            ('host_read_lag_ms', 'host_read_lag_owner_not_running'),
                            ('write_to_peer_arrival_ms', 'synthetic_peer_read_late_harness_artifact'),
                            ('peer_send_late_ms', 'synthetic_peer_send_late_harness_artifact')):
            value = row.get(f'{stage}_{port}_{name}')
            if value is not None and value > threshold_ms:
                causes.append((value, label, port))
        spacing = row.get(f'{stage}_{port}_write_spacing_max_ms')
        if spacing is not None and spacing > .92+threshold_ms:
            causes.append((spacing-.92, 'host_write_spacing_late', port))
    causes.sort(reverse=True)
    return [{'cause': c, 'port': p, 'ms': v} for v, c, p in causes]


def _cause_counts(breakdown):
    counts = {}
    for item in breakdown:
        causes = item['hold_causes'] or item['voltage_causes']
        key = causes[0]['cause'] if causes else 'host_processing_or_scheduling_no_reply_anomaly'
        if item.get('injected'):
            key = 'injected_'+'+'.join(sorted({k.split('_')[1] for k in item['injected'] if k.endswith('_ms')}))
        counts[key] = counts.get(key, 0)+1
    return counts


def summarize(rows, measured):
    def ms(a, b):
        return None if a is None or b is None else (a-b)/1e6
    complete = [r for r in rows if r.get('command_interval_ms') is not None]
    steady = [r for r in complete if r['index'] > 0]
    stages = {
        'release_lateness': [ms(r['begin_ns'], r['release_ns']) for r in rows],
        'begin_to_hold_first_write': [ms(r['hold_first_write_ns'], r['begin_ns']) for r in rows],
        'begin_to_hold_last_reply': [ms(r['hold_last_reply_ns'], r['begin_ns']) for r in rows],
        'hold_last_reply_to_acquired': [ms(r['acquired_ns'], r['hold_last_reply_ns']) for r in rows],
        'begin_to_acquired': [ms(r['acquired_ns'], r['begin_ns']) for r in rows],
        'acquired_to_gather': [ms(r['gather_ns'], r['acquired_ns']) for r in rows],
        'gather_to_infer_end': [ms(r['infer_end_ns'], r['gather_ns']) for r in rows],
        'infer_end_to_voltage_join': [ms(r['voltage_join_ns'], r['infer_end_ns']) for r in rows],
        'voltage_join_to_final_gate': [ms(r['final_gate_ns'], r['voltage_join_ns']) for r in rows],
        'begin_to_final_gate': [ms(r['final_gate_ns'], r['begin_ns']) for r in rows],
        'release_to_natural_gate': [r.get('natural_gate_after_release_ms') for r in rows],
        'release_to_command': [r.get('command_after_release_ms') for r in rows],
        'release_to_hold_first_write': [ms(r['hold_first_write_ns'], r['release_ns']) for r in rows],
        'final_gate_to_encode_end': [ms(r['encode_end_ns'], r['final_gate_ns']) for r in rows],
        'output_submit_to_first_write': [ms(r['output_first_write_ns'], r['output_submit_ns']) for r in rows],
        'output_submit_to_reply_return': [ms(r['reply_return_ns'], r['output_submit_ns']) for r in rows],
        'iteration': [ms(r['cycle_end_ns'], r['begin_ns']) for r in rows],
        'imu_read': [ms(r.get('imu_read_finished_ns'), r.get('imu_read_started_ns')) for r in rows],
        'begin_to_imu_read_start': [ms(r.get('imu_read_started_ns'), r['begin_ns']) for r in rows],
        'observer_total': [ms(r.get('observer_end_ns'), r.get('observer_start_ns')) for r in rows],
        'observer_model': [ms(r.get('observer_model_end_ns'), r.get('observer_model_start_ns')) for r in rows],
        'current_check1': [ms(r.get('current_check1_end_ns'), r.get('current_check1_start_ns')) for r in rows],
        'current_check2': [ms(r.get('current_check2_end_ns'), r.get('current_check2_start_ns')) for r in rows],
        'current_check3': [ms(r.get('current_check3_end_ns'), r.get('current_check3_start_ns')) for r in rows],
    }
    for port in PORTS:
        stages[f'begin_to_hold_owner_begin_{port}'] = [ms(r.get(f'hold_{port}_owner_begin_ns'), r['begin_ns']) for r in rows]
        stages[f'begin_to_voltage_last_reply_{port}'] = [ms(r.get(f'voltage_{port}_last_reply_ns'), r['begin_ns']) for r in rows]
    intervals = {}
    for name in ('command_interval_ms', 'sample_interval_ms'):
        values = [r[name] for r in steady]
        intervals[name] = {'steady_excluding_cycle0': describe(values),
                           'over_21ms': sum(1 for v in values if v > GAP_LIMIT_MS),
                           'over_20_8ms': sum(1 for v in values if v > MARGIN_MS),
                           'over_20_5ms': sum(1 for v in values if v > 20.5),
                           'cycle0_value_ms': complete[0][name] if complete else None,
                           'cycles_over_21ms': [r['index'] for r in steady if r[name] > GAP_LIMIT_MS]}
    begin_intervals = [ms(b['begin_ns'], a['begin_ns']) for a, b in zip(rows, rows[1:])]
    first_write = [r['hold_first_write_ns'] for r in rows]
    write_gaps = [ms(b, a) for a, b in zip(first_write, first_write[1:]) if a and b]
    violations = [r['index'] for r in steady if r.get('command_gap_violation') or r.get('sample_gap_violation')]
    edges = (('release_lateness', 'release_ns', 'begin_ns'), ('begin_to_hold_first_write', 'begin_ns', 'hold_first_write_ns'),
             ('hold_first_write_to_last_reply', 'hold_first_write_ns', 'hold_last_reply_ns'),
             ('hold_last_reply_to_acquired', 'hold_last_reply_ns', 'acquired_ns'),
             ('acquired_to_gather', 'acquired_ns', 'gather_ns'), ('gather_to_infer_end', 'gather_ns', 'infer_end_ns'),
             ('infer_end_to_voltage_join', 'infer_end_ns', 'voltage_join_ns'),
             ('voltage_join_to_final_gate', 'voltage_join_ns', 'final_gate_ns'))
    by_index = {r['index']: r for r in rows}
    breakdown = []
    for index in violations:
        now, before = by_index[index], by_index.get(index-1)
        if before is None:
            continue
        deltas = {}
        for name, a, b in edges:
            x, y = ms(now[b], now[a]), ms(before[b], before[a])
            deltas[name] = None if x is None or y is None else x-y
        breakdown.append({'index': index, 'label': now['label'], 'previous_label': before['label'],
                          'command_interval_ms': now['command_interval_ms'], 'sample_interval_ms': now['sample_interval_ms'],
                          'begin_interval_ms': ms(now['begin_ns'], before['begin_ns']),
                          'delta_vs_previous_cycle_ms': deltas,
                          'largest_delta_stage': max((k for k in deltas if deltas[k] is not None), key=lambda k: deltas[k], default=None),
                          'hold_last_reply_by_port_ms': {p: ms(now.get(f'hold_{p}_last_reply_ns'), now['begin_ns']) for p in PORTS},
                          'hold_owner_begin_by_port_ms': {p: ms(now.get(f'hold_{p}_owner_begin_ns'), now['begin_ns']) for p in PORTS},
                          'voltage_last_reply_by_port_ms': {p: ms(now.get(f'voltage_{p}_last_reply_ns'), now['begin_ns']) for p in PORTS},
                          'injected': {k: v for k, v in now.items() if k.startswith('inject_')},
                          'hold_causes': classify(now, 'hold'), 'voltage_causes': classify(now, 'voltage'),
                          'previous_output_causes': classify(before, 'output')})
    final_gate_offsets = [ms(r['final_gate_ns'], r['begin_ns']) for r in complete]
    consecutive = [b-a for a, b in zip(final_gate_offsets, final_gate_offsets[1:]) if a is not None and b is not None]
    anomaly = {}
    for stage in ('hold', 'voltage', 'output'):
        for name in ('modelled_delay_excess_ms', 'host_read_lag_ms', 'write_to_peer_arrival_ms', 'peer_send_late_ms',
                     'write_spacing_max_ms'):
            values = [r.get(f'{stage}_{p}_{name}') for r in rows for p in PORTS]
            values = [v for v in values if v is not None]
            anomaly[f'{stage}.{name}'] = dict(describe(values), over_0_5ms=sum(1 for v in values if v > .5 + (.92 if 'spacing' in name else 0)))
    return {'cycles_recorded': len(rows), 'envelope_steps': len(complete),
            'reply_attribution_ms_all_cycles_ports': anomaly,
            'completed_cycles': measured.get('completed_cycles'),
            'gap_violations_total': len(violations), 'gap_violation_cycles': violations,
            'gap_violation_rate': len(violations)/len(steady) if steady else None,
            'violation_breakdown': breakdown,
            'violation_primary_cause_counts': _cause_counts(breakdown),
            'largest_delta_stage_counts': {name: sum(1 for b in breakdown if b['largest_delta_stage'] == name)
                                           for name, _, _ in edges},
            'consecutive_delta_begin_to_final_gate_ms': describe(consecutive),
            'intervals': intervals, 'begin_interval_ms': describe(begin_intervals),
            'consecutive_hold_first_write_gap_ms': describe(write_gaps),
            'stage_ms': {k: describe(v) for k, v in stages.items()}}


# ----------------------------------------------------------------------------- main
def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', required=True, help='fresh directory (must not exist)')
    p.add_argument('--duration', type=int, choices=R.DURATIONS, default=20,
                   help='runner duration (2/10/20 s; 20 s is ~1000 cycles)')
    p.add_argument('--envelope-gap-mode', choices=('strict', 'measure'), default='strict')
    p.add_argument('--latency', choices=('empirical', 'median'), default='empirical')
    p.add_argument('--latency-model', default=str(latency_model.DEFAULT_PATH))
    p.add_argument('--observer', choices=('synthetic', 'torch-policy'), default='synthetic')
    p.add_argument('--model-profile', help='torch-policy: original model profile JSON (read-only)')
    p.add_argument('--model-ms', type=float, default=CONSUME_MODEL_MS)
    p.add_argument('--model-gil', choices=('released', 'held'), default='released')
    p.add_argument('--python-before-ms', type=float, default=CONSUME_PYTHON_BEFORE_MS)
    p.add_argument('--python-after-ms', type=float, default=CONSUME_PYTHON_AFTER_MS)
    p.add_argument('--library', help='prebuilt libdog_four_bus_type1_transport.so (with its build-record.json)')
    p.add_argument('--library-sha256', help='optional expected library SHA256')
    p.add_argument('--build-library', action='store_true', help='build a fresh library into the output dir')
    p.add_argument('--peer-cpu', type=int, default=5)
    p.add_argument('--peer-busy-spin', action='store_true',
                   help='peer polls continuously on its CPU (holds the CPU4/5 schedutil policy at max frequency '
                        'when the performance scope cannot be established without root)')
    p.add_argument('--no-pin', action='store_true', help='skip the harness process taskset 0-4 and pinning')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--uclamp-min', type=int, default=None,
                   help='Linux: unprivileged per-task UTIL_CLAMP_MIN for every harness thread (schedutil frequency '
                        'boost while runnable); a partial EMULATION of the root performance scope, C7 unchanged')
    # Opt-in runner options, forwarded unchanged (the default run is the reviewed R8 pacing).
    p.add_argument('--command-phase-offset-us', type=int, help='F1 command time release+K (type1_profile range)')
    p.add_argument('--decode-once', action='store_true', help='F2b decode-once takeouts and final gate')
    p.add_argument('--prearmed-hold-lead-us', type=int, help='F3 pre-armed hold lead (needs exchange_at in the build)')
    p.add_argument('--gc-freeze', action='store_true', help='F4 gc.freeze() before the first release')
    p.add_argument('--timing-evidence', action='store_true', help='F0 per-cycle/per-run timing evidence')
    # Deterministic injections (none by default).
    p.add_argument('--inject-hold-tail', action='append', metavar='PORT:CYCLE:+MS',
                   help='later hold replies of CYCLE on PORT +MS (peer side; repeatable)')
    p.add_argument('--inject-voltage-tail', action='append', metavar='PORT:CYCLE:+MS',
                   help='Type17 voltage reply of CYCLE on PORT +MS (peer side; repeatable)')
    p.add_argument('--inject-host-stall', action='append', metavar='CYCLE:MS[:gil|sleep]',
                   help='main thread stall of MS at release+0.16 ms of CYCLE (gil: GIL held; sleep: released)')
    p.add_argument('--cpu-keepalive', default='',
                   help='comma CPUs for SCHED_IDLE nice19 spinner processes (Linux): an unprivileged EMULATION of the '
                        'canonical performance scope (cpufreq min=max, C7 disabled) that this harness cannot set '
                        'without root; normal-policy threads preempt them on wakeup')
    return p


def set_uclamp_min(value):
    """Unprivileged sched_setattr(UTIL_CLAMP_MIN) on the calling thread; later threads inherit it (aarch64/x86_64)."""
    import ctypes
    import platform

    class Attr(ctypes.Structure):
        _fields_ = [('size', ctypes.c_uint32), ('sched_policy', ctypes.c_uint32), ('sched_flags', ctypes.c_uint64),
                    ('sched_nice', ctypes.c_int32), ('sched_priority', ctypes.c_uint32),
                    ('sched_runtime', ctypes.c_uint64), ('sched_deadline', ctypes.c_uint64),
                    ('sched_period', ctypes.c_uint64), ('sched_util_min', ctypes.c_uint32),
                    ('sched_util_max', ctypes.c_uint32)]
    numbers = {'aarch64': (274, 275), 'x86_64': (314, 315)}[platform.machine()]
    libc = ctypes.CDLL(None, use_errno=True)
    attr = Attr(); attr.size = ctypes.sizeof(Attr)
    attr.sched_flags = 0x08 | 0x10 | 0x20  # KEEP_POLICY | KEEP_PARAMS | UTIL_CLAMP_MIN
    attr.sched_util_min = value
    if libc.syscall(numbers[0], 0, ctypes.byref(attr), 0) != 0:
        raise OSError(ctypes.get_errno(), 'sched_setattr uclamp_min failed')
    back = Attr()
    if libc.syscall(numbers[1], 0, ctypes.byref(back), ctypes.sizeof(Attr), 0) != 0:
        raise OSError(ctypes.get_errno(), 'sched_getattr failed')
    return {'uclamp_min': back.sched_util_min, 'uclamp_max': back.sched_util_max, 'policy': back.sched_policy}


KEEPALIVE_CODE = '''
import os, sys
cpu = int(sys.argv[1])
os.sched_setaffinity(0, {cpu})
os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
os.nice(19)
sys.stdout.write("%d %s %d\\n" % (cpu, sorted(os.sched_getaffinity(0)), os.sched_getscheduler(0))); sys.stdout.flush()
yield_ = os.sched_yield
while True:
    yield_()  # Re-pick every ~us: an EEVDF-ineligible woken task is not left behind a SCHED_IDLE spinner for a 4 ms tick.
'''


def start_keepalive(cpus):
    processes = []
    for cpu in cpus:
        process = subprocess.Popen([sys.executable, '-B', '-c', KEEPALIVE_CODE, str(cpu)], stdout=subprocess.PIPE,
                                   text=True)
        processes.append(process)
    readback = [process.stdout.readline().strip() for process in processes]
    return processes, readback


def exchange_at_receipt(library_path):
    """F3 uses the genuine native exchange_at only: the receipt must record exchange_at_abi 1."""
    try:
        record = json.loads((Path(library_path).parent/'build-record.json').read_text())
    except (OSError, ValueError):
        record = {}
    scope = record.get('four_bus_subset_active') if type(record) is dict else None
    need(type(scope) is dict and scope.get('exchange_at_abi') == 1,
         'Pre-armed hold (F3) needs a Type1 build whose receipt records exchange_at_abi 1; use --build-library')
    return {'kind': 'native_sda_subset_exchange_at', 'receipt_exchange_at_abi': 1, 'emulated': False}


def load_library(args, output):
    if args.build_library:
        path = build.build(output/'library')
    else:
        need(args.library, '--library or --build-library required')
        path = Path(args.library).resolve(strict=True)
    directory = path.parent
    digest = sha(path)
    if args.library_sha256:
        need(digest == args.library_sha256, 'Library SHA256 differs from --library-sha256')
    pins = {'library_sha256': digest, 'ordinary_source_sha256': sha(directory/'ordinary_transport.cpp'),
            'subset_stop_source_sha256': sha(directory/'subset_stop.cpp'),
            'extension_source_sha256': sha(directory/'transport.cpp'),
            'build_record_sha256': sha(directory/'build-record.json')}
    library = T.load_library(path, expected_sha256=digest, ordinary_source_sha256=pins['ordinary_source_sha256'],
        subset_stop_source_sha256=pins['subset_stop_source_sha256'],
        extension_source_sha256=pins['extension_source_sha256'], build_record_sha256=pins['build_record_sha256'])
    return library, dict(pins, path=str(path))


def main(argv=None):
    args = parser().parse_args(argv)
    options = P.pacing_options(P.selected_pacing(P.cli_options(args)))  # Only the selected options.
    tails, stalls = injection.parse(args.inject_hold_tail, args.inject_voltage_tail, args.inject_host_stall)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {'schema': SCHEMA, 'status': 'STARTING', 'harness_only_no_devices': True,
              'timing_qualification_granted_here': False, 'output_approval_granted_here': False,
              'argv': sys.argv if argv is None else list(argv), 'envelope_gap_mode': args.envelope_gap_mode,
              'envelope_gap_mode_meaning': (
                  'strict: real runner behaviour, the first >21 ms command/sample interval aborts' if args.envelope_gap_mode == 'strict'
                  else 'MEASUREMENT MODE: >21 ms intervals are recorded as would-be violations and only that gap '
                       'comparison is skipped for that call; all other checks unchanged; not a valid run'),
              'environment': environment_readback(),
              'runtime_root': str(RUNTIME),
              'source_sha256': {name: sha(RUNTIME/name) for name in SOURCE_FILES},
              'harness_sha256': {name: sha(HERE/name) for name in HARNESS_FILES if (HERE/name).exists()}}
    if LINUX and not args.no_pin:
        os.sched_setaffinity(0, {0, 1, 2, 3, 4})  # Own process only: the canonical taskset -c 0-4.
        report['environment']['affinity_after_harness_taskset'] = sorted(os.sched_getaffinity(0))
    keepalive = []
    if args.uclamp_min is not None:
        need(LINUX and 0 <= args.uclamp_min <= 1024, 'uclamp_min needs Linux and 0..1024')
        report['uclamp_readback'] = set_uclamp_min(args.uclamp_min)  # Before any thread is created.
        report['power_scope_emulation'] = ('per_task_uclamp_min_%d_frequency_only_C7_enabled_nice0_'
                                           'EMULATION_not_the_canonical_root_power_scope' % args.uclamp_min)
    if args.cpu_keepalive:
        need(LINUX, 'CPU keepalive emulation is Linux-only')
        keepalive, report['cpu_keepalive_readback'] = start_keepalive(int(x) for x in args.cpu_keepalive.split(','))
        report['power_scope_emulation'] = ('SCHED_IDLE_nice19_sched_yield_spinners_on_cpus_'+args.cpu_keepalive+
                                           '_EMULATION_not_the_canonical_root_power_scope')
        time.sleep(1.)  # Let schedutil raise the frequency before setup.
    library, report['library'] = load_library(args, output)
    report['library']['sda_wait_until_available'] = getattr(library, 'sda_wait_until', None) is not None
    if 'prearmed_hold_lead_us' in options:
        report['exchange_at'] = exchange_at_receipt(report['library']['path'])

    # Synthetic peers in their own process (own GIL), CPU5, connected by four socketpairs.
    pairs = {port: socket.socketpair() for port in PORTS}
    for host, _ in pairs.values():
        host.setblocking(False)
    stop_r, stop_w = os.pipe()
    peer_config = {'ports': list(PORTS), 'cpu': args.peer_cpu if LINUX and not args.no_pin else None,
                   'timer_slack_ns': 1000, 'latency_mode': args.latency, 'busy_spin': args.peer_busy_spin, 'latency_model_path': args.latency_model,
                   'seed': args.seed, 'raw_by_id': {str(m): fixture.RAW[m] for m in fixture.IDS},
                   'uid_by_id': {str(m): fixture.UID[m] for m in fixture.IDS},
                   'firmware_by_id': {str(m): fixture.FIRMWARE[m] for m in fixture.IDS}}
    (output/'peer-config.json').write_text(json.dumps(peer_config))
    peer_fds = [pairs[port][1].fileno() for port in PORTS]
    control_r = control_w = None
    extra_args, extra_fds = [], ()
    if tails:  # Peer control pipe only when a tail is injected.
        control_r, control_w = os.pipe()
        extra_args, extra_fds = ['--control-fd', str(control_r)], (control_r,)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(
        x for x in (str(RUNTIME), str(HERE), os.environ.get('PYTHONPATH')) if x))
    peer = subprocess.Popen([sys.executable, '-B', str(HERE/'peer.py'),
                             '--fds', ','.join(map(str, peer_fds)), '--stop-fd', str(stop_r),
                             '--config', str(output/'peer-config.json'), '--log', str(output/'peer-log.json'),
                             *extra_args], pass_fds=(*peer_fds, stop_r, *extra_fds), env=env)
    for _, remote in pairs.values():
        remote.close()
    os.close(stop_r)
    if control_r is not None:
        os.close(control_r)

    # Cancel pipe, boot identity (Linux: read-only /proc boot_id), stop event, signals.
    cancel_r, cancel_w = os.pipe()
    os.set_blocking(cancel_r, False); os.set_blocking(cancel_w, False)
    if LINUX:
        boot_fd = os.open('/proc/sys/kernel/random/boot_id', os.O_RDONLY)
    else:
        handle = tempfile.TemporaryFile(); handle.write(b'00000000-0000-0000-0000-000000000000\n'); handle.flush()
        boot_fd = os.dup(handle.fileno()); handle.close()
    boot_bytes = os.pread(boot_fd, 128, 0).strip()
    boot_id = boot_bytes.decode()
    stop = threading.Event()
    cancels = []

    def cancel_from(source):
        def request():
            cancels.append({'source': source, 'monotonic_ns': time.monotonic_ns()})
            stop.set()
            try:
                os.write(cancel_w, b'x')
            except BlockingIOError:
                pass
        return request
    previous = {sig: signal.signal(sig, lambda s, f: cancel_from('signal')()) for sig in (signal.SIGINT, signal.SIGTERM)}

    def light():
        need(not stop.is_set(), 'Foreground cancelled')

    guard = EmulatedCurrentGuard(output/'scratch-guard', boot_fd, boot_bytes, stop)
    imu_durations = [.908, .919, .89, .93, .95, .91, .9, .92]  # Recorded read_started->finished (ms) spread.
    imu_device = SyntheticIMU(imu_durations, args.seed)
    if args.observer == 'torch-policy':
        need(args.model_profile, '--model-profile required for torch-policy')
        observer = TorchPolicyObserver(args.model_profile, args.python_before_ms, args.python_after_ms)
        report['observer'] = observer.description
    else:
        observer = SyntheticObserver(args.python_before_ms, args.model_ms, args.python_after_ms, args.model_gil)
        report['observer'] = {'kind': 'synthetic', 'python_before_ms': args.python_before_ms,
                              'model_ms': args.model_ms, 'model_gil': args.model_gil,
                              'python_after_ms': args.python_after_ms}
    transports, scopes = {}, {}
    injected = injection.Injections(tails, stalls, prearmed='prearmed_hold_lead_us' in options, control_fd=control_w)

    def factory(group):
        raw = {mid: fixture.RAW[mid] for mid in group.ids}
        value = T.Type1Transport.create(library, pairs[group.port][0].fileno(), group=group, cancel_fd=cancel_r,
            boot_fd=boot_fd, boot_id=boot_id,
            axis_raw_bounds={mid: (raw[mid]-2.99*DEG, raw[mid]+2.99*DEG) for mid in group.ids},
            kp_cap_by_id=dict.fromkeys(group.ids, 0.), kd_cap_by_id=dict.fromkeys(group.ids, 0.),
            cancel_all=cancel_from(group.port+'_transport_failure'), decode_once=options.get('decode_once') is True,
            prearmed_hold='prearmed_hold_lead_us' in options)
        transports[group.port] = value
        return value

    def worker(port, mask):
        value = pipeline.LinuxWorkerScope(port, mask) if LINUX and not args.no_pin else PortableWorkerScope(port, mask)
        scopes[port] = value
        return value

    main_scope = HarnessMainScope(True, False) if LINUX and not args.no_pin else PortableMainScope()
    recorder = EnvelopeRecorder(args.envelope_gap_mode)
    admitted = fixture.admitted('zero_gain_timing', args.duration)
    admitted['first_cycle_post_reply'] = False  # As type1_foreground.runner_admitted selects.
    admitted['pacing'] = P.selected_pacing(options)
    report['options'] = {'pacing_options': options, 'timing_evidence': args.timing_evidence}
    command_wait = None
    if 'command_phase_offset_us' in options:
        command_wait = F.make_command_wait(library, cancel_r) if LINUX else portable_command_wait(cancel_r)
        report['command_wait'] = ('native_sda_wait_until_spin500us' if LINUX else
                                  'PORTABLE_MACOS_VALIDATION_ONLY_python_sleep_then_spin')
    if LINUX:
        release_wait = FG.make_release_wait(library, cancel_r, observer)
        report['release_wait'] = 'native_sda_wait_until_spin500us'
    else:
        release_wait = portable_release_wait(cancel_r, observer)
        report['release_wait'] = 'PORTABLE_MACOS_VALIDATION_ONLY_python_sleep_then_spin'
    if not injected.empty:
        report['injections_selected'] = {'tails': tails, 'stalls': stalls,
                                         'meaning': 'DETERMINISTIC FAULT INJECTION; not a clean timing run'}
    original_envelope, original_executor = R.PolicyMotionEnvelope, R.ThreadPoolExecutor
    measured = None
    try:
        if hasattr(observer, 'calibrate'):
            report['observer_calibration'] = observer.calibrate()
        gc.collect()
        R.PolicyMotionEnvelope = recorder.envelope_class()  # Harness-process wrapper only.
        R.ThreadPoolExecutor = injected.executor_class(original_executor)  # Unchanged unless a stall is selected.
        report['run_started_ns'] = time.monotonic_ns()
        measured = R.run(admitted, factory=factory, imu_read=lambda: FG.read_imu(imu_device, light),
            observer=observer, check_current=guard, check_cancelled=light,
            cancel_io=cancel_from('runner_emergency'),
            model_setup=model_setup_for(observer), worker_scope=worker, main_scope=lambda: main_scope,
            release_wait=injected.wrap_release_wait(release_wait), announce=lambda: None,
            backend_usage={'kind': 'injected_file_only_mock',
                           'harness_truth': 'genuine Type1Transport + genuine subset-active library over socketpair '
                                            'synthetic peers; kind label chosen because nice-10/power-scope readbacks '
                                            'required by the genuine label are not established by this harness',
                           'library_sha256': report['library']['library_sha256']},
            execute=True, command_wait=command_wait,
            timing_evidence=TimingEvidence() if args.timing_evidence else None)
        report['run_finished_ns'] = time.monotonic_ns()
    finally:
        R.PolicyMotionEnvelope, R.ThreadPoolExecutor = original_envelope, original_executor
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        for host, _ in pairs.values():
            host.close()
        os.close(stop_w)
        if control_w is not None:
            os.close(control_w)
        try:
            peer.wait(timeout=10)
        except subprocess.TimeoutExpired:
            peer.kill(); peer.wait()
        os.close(cancel_r); os.close(cancel_w); os.close(boot_fd)
        for process in keepalive:
            process.kill(); process.wait()

    report['status'] = 'HARNESS_FINISHED'
    report['runner'] = {key: measured.get(key) for key in (
        'status', 'primary_error', 'errors', 'completed_cycles', 'normal_ramp_completed', 'stop_reason',
        'stop_confirmed', 'physical_cutoff_required', 'max_iteration_ms', 'post_reply_late_cycles',
        'post_reply_deadline_allowance_uses', 'worker_settings', 'operating_settings', 'watchdog_settings',
        'gc_enabled_during_cycles', 'stop_at_s', 'max_stop_s', 'voltage_guard', 'restoration', 'epoch_ns')}
    report['runner'].update({key: measured[key] for key in (  # Opt-in fields, only when selected.
        'command_phase', 'decode_once_selected', 'takeout_check', 'prearmed_hold', 'gc_freeze', 'gc_freeze_selected',
        'timing_evidence') if key in measured})
    report['runner'].update(pacing=measured.get('pacing'), final_gate=measured.get('final_gate'),
                            full_current_check_points=measured.get('full_current_check_points'))
    report['runner'] = json.loads(json.dumps(report['runner'], default=str))
    report['main_scope_report'] = json.loads(json.dumps(getattr(main_scope, 'report', None), default=str))
    report['cancel_requests'] = cancels
    report['transport_failures'] = {p: list(t.failures) for p, t in transports.items()}
    report['envelope_created'] = recorder.created
    report['envelope_steps'] = recorder.rows
    rows = per_cycle_rows(measured, recorder, guard, imu_device, observer)
    try:
        peer_log = json.loads((output/'peer-log.json').read_text())
        report['peer'] = {k: v for k, v in peer_log.items() if k != 'rows'}
    except (OSError, ValueError) as error:
        peer_log = None
        report['peer'] = {'error': repr(error)}
    if peer_log is not None:
        attribute_replies(rows, peer_log)
    if not injected.empty:
        applied = (peer_log or {}).get('injections_applied') or []
        report['injections'] = injected.report(applied)
        for row in rows:
            row.update(injected.row_fields(row['index'], applied))
    report['summary'] = summarize(rows, measured)
    report['cycles'] = rows
    (output/'harness-report.json').write_text(json.dumps(report, indent=1, default=str)+'\n')
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with (output/'cycles.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    brief = {'status': measured.get('status'), 'primary_error': measured.get('primary_error'),
             'completed_cycles': measured.get('completed_cycles'), 'envelope_gap_mode': args.envelope_gap_mode,
             'gap_violations_total': report['summary']['gap_violations_total'],
             'options': report['options'], 'injections_performed': bool(report.get('injections')),
             'command_interval_ms': report['summary']['intervals']['command_interval_ms'],
             'sample_interval_ms': report['summary']['intervals']['sample_interval_ms'],
             'peer_send_lateness_ns': report['peer'].get('send_lateness_ns'),
             'output': str(output)}
    print(json.dumps(brief, indent=1, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
