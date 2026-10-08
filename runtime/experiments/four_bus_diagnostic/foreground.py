"""Pinned four-bus foreground STOP-proxy diagnostic; default is file-only PLAN.

Run with ``python -B -m experiments.four_bus_diagnostic.foreground`` from the
selected kit's runtime PYTHONPATH. The caller establishes privileged nice,
CPU/C7/EMC and one-thread math environment. This module verifies those values;
it does not change power settings or grant Type1/output/timing eligibility.
"""
import argparse
import copy
from contextlib import ExitStack
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import sys
import threading
import time

if not __package__:
    sys.path.insert(0, str(Path(__file__).absolute().parents[2]))
    __package__ = 'experiments.four_bus_diagnostic'

from . import model_bridge, pipeline, topology, transport_adapter

SCHEMA = 'singularitydog.four-bus-foreground-stop-proxy.v1'
FALSE_FLAGS = {'output_allowed': False, 'approved_for_runtime': False,
               'active_controller_qualification': False, 'timing_admission_eligible': False,
               'live_type1_qualified': False, 'motor_enable_sent': False,
               'learned_targets_sent': False, 'physical_future_observations': None}
BOOT_PATH = Path('/proc/sys/kernel/random/boot_id')


def need(value, message):
    if not value:
        raise ValueError(message)


def strict_json(raw):
    return json.loads(raw, object_pairs_hook=topology.strict_pairs,
                      parse_constant=topology.bad_constant)


class Pins:
    """Authenticate exact regular bytes; no historical provenance tree walk."""
    def __init__(self):
        self.values = {}

    def read(self, path, digest):
        path = str(Path(path))
        need(path not in self.values or self.values[path] == digest, 'Conflicting input pin: '+path)
        raw = topology.read_pinned(path, digest)
        self.values[path] = digest
        return raw

    def json(self, path, digest):
        raw = self.read(path, digest)
        need(len(raw) <= 32*1024*1024, 'Bounded pinned JSON required')
        value = strict_json(raw)
        need(type(value) is dict, 'Pinned JSON object required')
        return value

    def envelope(self, reference):
        need(type(reference) is dict and set(reference) == {'path', 'sha256'}, 'Exact input reference required')
        return {'reference': dict(reference),
                'raw_json': self.read(reference['path'], reference['sha256']).decode('utf-8')}

    def verify(self):
        for path, digest in tuple(self.values.items()):
            self.read(path, digest)


def verify_manifest(root, manifest, count, pins):
    root = Path(root)
    need(root.is_absolute() and root.is_dir() and not any(p.is_symlink() for p in (root, *root.parents)),
         'Absolute nonsymlink source kit required')
    files = manifest.get('files')
    need(type(count) is int and count > 0 and type(files) is dict and
         manifest.get('file_count') == count == len(files) and
         manifest.get('output_allowed') is False and manifest.get('approved_for_runtime') is False,
         'Exact complete unapproved source inventory/count required')
    for name, row in files.items():
        relative = Path(name)
        need(type(name) is str and not relative.is_absolute() and '..' not in relative.parts and
             type(row) is dict and type(row.get('bytes')) is int and type(row.get('mode')) is int,
             'Invalid source inventory member')
        path = root/relative
        raw = pins.read(path, row.get('sha256'))
        need(len(raw) == row['bytes'] and stat.S_IMODE(path.stat().st_mode) == row['mode'],
             'Source extent/mode changed: '+str(path))
    actual = set()
    for path in root.rglob('*'):
        need(not path.is_symlink(), 'Source kit symlink rejected: '+str(path))
        if path.is_file():
            actual.add(str(path.relative_to(root)))
    need(actual == set(files), 'Unlisted or missing source kit files')


def verify_origins(root, manifest):
    runtime = Path(root)/'runtime'
    for name, module in tuple(sys.modules.items()):
        if not (name.startswith('singularitydog_hw.') or name.startswith('experiments.four_bus_diagnostic')
                or name.startswith('experiments.private_checked_policy_dispatch.')):
            continue
        filename = getattr(module, '__file__', None)
        if filename is None:
            continue
        path = Path(filename).absolute()
        expected = runtime/(name.replace('.', '/')+'.py')
        if name == 'experiments.four_bus_diagnostic':
            expected = runtime/'experiments/four_bus_diagnostic/__init__.py'
        need(path == expected and str(path.relative_to(root)) in manifest['files'],
             'Imported module origin differs from selected kit: '+name)
    own = Path(__file__).absolute()
    need(own == runtime/'experiments/four_bus_diagnostic/foreground.py', 'Foreground source origin differs')


def fresh_output(value, source_root):
    path = Path(value)
    need(path.is_absolute() and not path.exists() and not any(p.is_symlink() for p in (path, *path.parents)),
         'Fresh absolute nonsymlink private output required')
    need(Path(source_root) not in path.parents and not any((p/'.git').exists() for p in path.parents),
         'Output must be outside source kit and Git')
    return path


def prepare_native_guard(args, manifest, pins):
    """File-only exact current-guard binding; never import/load its CDLL."""
    library = Path(args.current_guard_library)
    need(library.is_absolute(), 'Absolute current-guard library required')
    pins.read(library, args.current_guard_library_sha256)
    record = pins.json(library.parent/'build-record.json', args.current_guard_build_sha256)
    source = pins.read(library.parent/'native_current_guard.cpp', args.current_guard_source_sha256)
    need(record.get('schema') == 'singularitydog.four-bus-current-guard-build.v1' and
         type(record.get('abi')) is int and record['abi'] == 1 and
         record.get('binary_sha256') == args.current_guard_library_sha256 and
         record.get('source_sha256') == args.current_guard_source_sha256 and
         type(record.get('source_bytes')) is int and record['source_bytes'] == len(source) and
         record.get('CAN_IO_available') is False and record.get('output_allowed') is False and
         record.get('timing_admission_eligible') is False and
         manifest['files'].get('runtime/experiments/four_bus_diagnostic/native_current_guard.cpp',
                               {}).get('sha256') == args.current_guard_source_sha256,
         'Exact selected current-guard source/build/binary contract required')
    return {'schema': 'singularitydog.four-bus-native-current-guard-file-plan.v1', 'abi': 1,
        'library': {'path': str(library), 'sha256': args.current_guard_library_sha256},
        'source': {'path': str(library.parent/'native_current_guard.cpp'),
                   'sha256': args.current_guard_source_sha256},
        'build': {'path': str(library.parent/'build-record.json'), 'sha256': args.current_guard_build_sha256},
        'loads_library_in_plan': False, 'CAN_IO_available': False, 'output_allowed': False}


def make_config(args, model_plan, capture):
    """Only an explicit CLI boolean selects the distinct full-four acquisition."""
    need(type(args.combined_acquisition) is bool, 'Explicit combined-acquisition boolean required')
    boundary = getattr(args, 'boundary_current_checks', False)
    need(type(boundary) is bool, 'Explicit boundary current-check boolean required')
    identity = getattr(args, 'final_gate_input_identity', False)
    need(type(identity) is bool, 'Explicit final-gate input identity boolean required')
    once = getattr(args, 'decode_once', False)
    need(type(once) is bool, 'Explicit decode-once boolean required')
    groups = tuple(transport_adapter.Group(port, tuple(capture['ids_by_port'][port])) for port in topology.PORTS)
    return pipeline.Config(groups, model_plan['measured_input_profile'],
        {int(mid): value for mid, value in model_plan['runtime_offsets_by_id'].items()}, capture, args.cycles,
        combined_acquisition=args.combined_acquisition, boundary_current_checks=boundary,
        final_gate_input_identity=identity, decode_once=once)


def select_snapshot_builder(plan, *, combined_acquisition, decode_once=False):
    need(type(combined_acquisition) is bool, 'Explicit combined-acquisition boolean required')
    need(type(decode_once) is bool, 'Explicit decode-once boolean required')
    if combined_acquisition:
        return model_bridge.combined_batch_snapshot_builder(plan)
    if decode_once:
        return model_bridge.batch_snapshot_builder(plan, decode_once=True)
    return model_bridge.batch_snapshot_builder(plan)


def prepare(args):
    """Pure file/source/model-artifact verification, without Torch or devices."""
    pins = Pins()
    manifest = pins.json(args.source_manifest, args.source_manifest_sha256)
    verify_manifest(args.source_root, manifest, args.source_count, pins)
    verify_origins(args.source_root, manifest)
    source = pins.json(args.source_binding, args.source_binding_sha256)
    profile_raw = pins.read(args.original_model_profile, args.original_model_profile_sha256)
    original = strict_json(profile_raw)
    need(source.get('four_bus_source_manifest') ==
         {'path': args.source_manifest, 'sha256': args.source_manifest_sha256}, 'New whole-source binding differs')
    frozen = source['frozen_model_source_manifest']
    old_manifest = pins.json(frozen['path'], frozen['sha256'])
    need(old_manifest.get('file_count') == 820 and len(old_manifest.get('files', {})) == 820,
         'Exact original warmed model-source inventory required')
    for name, digest in source['frozen_model_source_sha256'].items():
        need(manifest['files'].get('runtime/'+name, {}).get('sha256') == digest and
             old_manifest['files'].get('runtime/'+name, {}).get('sha256') == digest,
             'Model source bytes must remain the original warmed source: '+name)
    refs = original['artifacts']
    calibration, mount, bias = (pins.envelope(refs[key]) for key in ('calibration', 'mount', 'bias'))
    accel = pins.envelope(refs['accel_input_hypothesis']) if original.get('accel_input_hypothesis') else None
    capture_ref = {'path': args.topology, 'sha256': args.topology_sha256}
    capture = pins.json(args.topology, args.topology_sha256)
    need(capture.get('motor_power_epoch') == args.power_epoch, 'Explicit current power epoch differs from capture')
    event_raw = pins.read(args.events, args.events_sha256).decode('utf-8')
    model_plan = model_bridge.prepare_model_plan(topology=pins.envelope(capture_ref),
        events={'reference': {'path': args.events, 'sha256': args.events_sha256}, 'raw_jsonl': event_raw},
        calibration=calibration, model_profile={'reference': {'path': args.original_model_profile,
            'sha256': args.original_model_profile_sha256}, 'raw_json': profile_raw.decode('utf-8')},
        source_binding=pins.envelope({'path': args.source_binding, 'sha256': args.source_binding_sha256}),
        mount=mount, bias=bias, accel_hypothesis=accel)
    from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked, policy_live_profile
    profile = policy_live_profile.load_profile(args.original_model_profile, require_approved=False)
    need(profile.get('approved_for_supported_policy_output') is False and profile.get('review') is None and
         profile.get('output_allowed', False) is False, 'Original model dependency profile must remain unapproved')
    fk_plan, checked_plan = fk.plan(profile), checked.plan(profile)
    need(profile.get('_checked_model_plan') == checked_plan, 'Original loader checked proof differs')
    for path, digest in checked_plan['input_sha256'].items():
        pins.read(path, digest)
    library = Path(args.subset_library)
    need(library.is_absolute(), 'Absolute subset library required')
    pins.read(library, args.subset_library_sha256)
    build = pins.json(library.parent/'build-record.json', args.subset_build_sha256)
    pins.read(library.parent/'transport.cpp', args.subset_source_sha256)
    pins.read(library.parent/'ordinary_transport.cpp', args.ordinary_source_sha256)
    scope = build.get('four_bus_subset_stop', {})
    need(build.get('abi') == 1 and scope.get('schema') == 'singularitydog.four-bus-subset-build.v1' and
         scope.get('original_active_abi') == 1 and
         build.get('binary_sha256') == args.subset_library_sha256 and
         build.get('source_sha256') == args.subset_source_sha256 and
         scope.get('ordinary_source_sha256') == args.ordinary_source_sha256 and
         scope.get('extension_source_sha256') == args.subset_source_sha256 and
         scope.get('allowed_masks') == [7, 56] and scope.get('abi') == 1 and
         scope.get('output_allowed') is False and scope.get('timing_admission_eligible') is False,
         'Exact new subset build/source/binary binding required')
    need(manifest['files']['runtime/experiments/native_active_transport/transport.cpp']['sha256'] ==
         args.ordinary_source_sha256 and
         manifest['files']['runtime/experiments/four_bus_diagnostic/subset_stop.cpp']['sha256'] ==
         args.subset_source_sha256, 'Subset build must include the selected current source bytes')
    guard_plan = prepare_native_guard(args, manifest, pins)
    config = make_config(args, model_plan, capture)
    planned = pipeline.plan(config)
    output = fresh_output(args.output, args.source_root)
    holder_receipt = None
    need((args.holder_receipt is None) == (args.holder_receipt_sha256 is None),
         'Root holder receipt needs both path and SHA')
    if args.holder_receipt is not None:
        holder_receipt = pins.json(args.holder_receipt, args.holder_receipt_sha256)
        validate_holder_receipt(holder_receipt, capture)
    pins.verify()
    result = {'schema': SCHEMA, 'status': 'PLAN', **FALSE_FLAGS, 'opens_devices': False,
        'source_count': args.source_count, 'source_manifest_sha256': args.source_manifest_sha256,
        'model_plan': model_plan, 'pipeline_plan': planned, 'fk_plan': fk_plan,
        'checked_plan': checked_plan, 'current_guard_plan': guard_plan, 'input_sha256': dict(pins.values),
        'original_model_profile_used_for_dependencies_only': True,
        'canonical_outer_power_scope_required': True, 'new_four_bus_timing_proof_required': True}
    return {'plan': result, 'pins': pins, 'manifest': manifest, 'profile': profile,
            'config': config, 'model_plan': model_plan, 'output': output,
            'holder_receipt': holder_receipt}


def power_readback(cpu_root=Path('/sys/devices/system/cpu'), emc_root=Path('/sys/class/devfreq/bwmgr')):
    """Read only the caller's established performance settings; never write."""
    frequencies = {}
    for policy in sorted((cpu_root/'cpufreq').glob('policy[0-9]*')):
        frequencies[policy.name] = {name: int((policy/name).read_text()) for name in
                                   ('scaling_min_freq', 'scaling_max_freq')}
    need(frequencies and all(row['scaling_min_freq'] == row['scaling_max_freq'] == 1728000
                            for row in frequencies.values()), 'Canonical CPU minimum/maximum not established')
    c7 = {}
    for cpu in range(6):
        states = [p for p in (cpu_root/f'cpu{cpu}/cpuidle').glob('state[0-9]*')
                  if (p/'name').read_text().strip().lower() == 'c7']
        need(states and all(int((p/'disable').read_text()) == 1 for p in states), 'Canonical C7 readback failed')
        c7[str(cpu)] = {p.name: int((p/'disable').read_text()) for p in states}
    emc = {name: int((emc_root/name).read_text()) for name in ('min_freq', 'max_freq')}
    need(emc['min_freq'] == emc['max_freq'] == 3199000000, 'Canonical EMC readback failed')
    return {'cpu_frequencies': frequencies, 'c7': c7, 'emc': emc, 'power_settings_written_here': False}


def startup_readback():
    need(sys.platform == 'linux' and os.getuid() == os.geteuid() > 0 and
         os.getgid() == os.getegid() > 0 and os.getpriority(os.PRIO_PROCESS, 0) == -10 and
         os.sched_getaffinity(0) == {0, 1, 2, 3, 4}, 'Actual nonroot nice-10/taskset0-4 required before setup')
    env = {name: os.environ.get(name) for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')}
    need(all(value == '1' for value in env.values()), 'One-thread math environment required before Torch import')
    return {'uid': os.getuid(), 'gid': os.getgid(), 'nice': -10, 'cpus': [0, 1, 2, 3, 4],
            'math_environment': env, 'power': power_readback()}


class MainScope:
    """Only main-thread affinity/slack/switch/GC; exact independent rollback."""
    def __init__(self, math_verified, power_verified):
        self.math_verified, self.power_verified = math_verified, power_verified
        self.report, self.slack = {}, None
        self.original = None

    def __enter__(self):
        from singularitydog_hw.thread_timer_slack import TimerSlack
        self.original = {'cpus': sorted(os.sched_getaffinity(0)), 'switch_s': sys.getswitchinterval(),
                         'gc_enabled': gc.isenabled()}
        try:
            os.sched_setaffinity(0, {4})
            sys.setswitchinterval(.0001)
            self.slack = TimerSlack(1000)
            self.slack.__enter__()
            gc.disable()
            need(os.sched_getaffinity(0) == {4} and abs(sys.getswitchinterval()-.0001) <= 1e-15,
                 'Main CPU/switch readback failed')
            during = {'main_cpu_mask': [4], 'nice': os.getpriority(os.PRIO_PROCESS, 0),
                'timer_slack_ns': self.slack.report['parent']['during_ns'],
                'switch_interval_s': sys.getswitchinterval(), 'single_thread_math_verified': self.math_verified,
                'power_scope_verified': self.power_verified, 'gc_deferred_during_cycles': not gc.isenabled()}
            self.report.update(before=dict(self.original), during=dict(during))
            return during
        except BaseException as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise

    def __exit__(self, kind, primary, trace):
        errors = []
        if self.slack is not None:
            try:
                self.slack.__exit__(kind, primary, trace)
            except BaseException as error:
                errors.append(error)
        if self.original is not None:
            for restore in (lambda: os.sched_setaffinity(0, set(self.original['cpus'])),
                            lambda: sys.setswitchinterval(math.nextafter(self.original['switch_s'], math.inf)),
                            lambda: gc.enable() if self.original['gc_enabled'] else gc.disable()):
                try:
                    restore()
                except BaseException as error:
                    errors.append(error)
            try:
                actual = {'cpus': sorted(os.sched_getaffinity(0)), 'switch_s': sys.getswitchinterval(),
                          'gc_enabled': gc.isenabled()}
                need(actual == self.original, 'Exact main CPU/switch/GC restoration failed')
                self.report['after'] = actual
            except BaseException as error:
                errors.append(error)
        self.report.update(restored=not errors, errors=[str(error) for error in errors],
                           timer_slack=None if self.slack is None else self.slack.report)
        if errors:
            if primary is not None:
                primary.add_note('Main restoration: '+'; '.join(str(error) for error in errors))
            else:
                raise errors[0]
        return False


def selected_warmup(observer, policy, wrapper, torch, h, *, pre_calls, post_calls):
    """Original selected10+10 calls; post stage uses exact observer buffers."""
    from singularitydog_hw.policy_observer_replay import warmup_policy
    need(pre_calls == post_calls == 10, 'Original two ten-call stages required')
    before = os.sched_getaffinity(0)
    need(before == {4} and observer._input_tensors is not None,
         'Selected warmup requires pinned main and original persistent tensors')
    record = {'pre_calls': pre_calls, 'post_calls': post_calls, 'pre_cpu_mask': [0, 1, 2, 3, 4],
              'post_cpu_mask': [4], 'four_bus_pre_warmup_temporarily_restores_original_taskset': True}
    try:
        os.sched_setaffinity(0, {0, 1, 2, 3, 4})
        need(os.sched_getaffinity(0) == {0, 1, 2, 3, 4}, 'Pre-warm placement readback failed')
        record['gc_collected'] = gc.collect()
        warmup_policy(policy, torch, h, 10, checked_dispatch_wrapper=wrapper)
        os.sched_setaffinity(0, {4})
        need(os.sched_getaffinity(0) == {4}, 'Post-warm placement readback failed')
        warmup_policy(policy, torch, h, 10, input_tensors=observer._input_tensors,
                      checked_dispatch_wrapper=wrapper)
        observer.prepare_run(warmup_completed=True)
        record.update(reset_verified=True, selected_model_warmup_verified=True)
    except BaseException as error:
        try:
            os.sched_setaffinity(0, before)
        except BaseException as cleanup:
            error.add_note('Warmup CPU rollback: '+str(cleanup))
        raise
    return record


def owner_bounds(plan, group):
    lower, upper = {}, {}
    for mid in range(group.first_id, group.first_id+6):
        row = plan['axes'][str(mid)]
        lo, hi = row['global_bounds_rad']
        sign, offset = row['sign'], row['fixed_offset_rad']
        values = ((lo-offset)/sign, (hi-offset)/sign)
        lower[mid], upper[mid] = min(values), max(values)
    return lower, upper


def observed_holders(bindings, proc=Path('/proc')):
    wanted = {row['st_rdev'] for row in bindings.values()}
    found, inaccessible = [], []
    for pid in proc.iterdir():
        if not pid.name.isdecimal() or int(pid.name) == os.getpid():
            continue
        try:
            descriptors = list((pid/'fd').iterdir())
        except PermissionError:
            inaccessible.append(int(pid.name)); continue
        except FileNotFoundError:
            continue
        for descriptor in descriptors:
            try:
                info = descriptor.stat()
            except (FileNotFoundError, PermissionError):
                continue
            if stat.S_ISCHR(info.st_mode) and info.st_rdev in wanted:
                found.append({'pid': int(pid.name), 'fd': descriptor.name, 'st_rdev': info.st_rdev})
    need(not found, 'Current CAN descriptor already held by another process: '+str(found))
    return {'observed_holders': found, 'inaccessible_pids': inaccessible,
            'root_owned_descriptors_all_visible': not inaccessible,
            'cooperating_and_per_port_locks_and_exclusive_serial_still_required': True}


def validate_holder_receipt(receipt, capture):
    """Optional root-wide observation; not a model/physical/runtime grant."""
    need(type(receipt) is dict and receipt.get('schema') == 'singularitydog.four-bus-root-holder-check.v1' and
         receipt.get('run_as_uid') == 0 and receipt.get('boot_id') == capture['boot_before'] and
         receipt.get('ports') == capture['ports'] and receipt.get('holders') == [] and
         receipt.get('complete_fd_inventory') is True and
         type(receipt.get('checked_monotonic_ns')) is int and
         receipt['checked_monotonic_ns'] >= capture['finished_monotonic_ns'] and
         receipt.get('output_allowed') is False, 'Pinned root current four-port holder observation differs')
    return receipt


def make_release_wait(library, cancel_fd, observer):
    from singularitydog_hw.native_active_transport import wait_until
    armed = False
    def wait(scheduled):
        nonlocal armed
        actual = wait_until(library, cancel_fd, scheduled, spin_us=500)
        if not armed:
            observer.arm_run(scheduled)
            armed = True
        return actual
    return wait


def read_imu(device, check, *, clock=time.monotonic_ns, sleep=time.sleep):
    deadline = clock()+20_000_000
    while clock() < deadline:
        check()
        sample = device.read_sample()
        if sample is not None:
            return sample
        sleep(.0005)
    raise TimeoutError('No new IMU within original20ms')


class CancelBinding:
    """Clear the live writer before ExitStack closes its descriptor."""
    def __init__(self):
        self.stop = threading.Event()
        self.writer = None
        self.errors = []

    def bind(self, stack, writer):
        need(self.writer is None, 'Cancellation writer already bound')
        self.writer = writer
        stack.callback(self.clear)

    def clear(self):
        self.writer = None

    def cancel(self):
        self.stop.set()
        if self.writer is not None:
            try:
                os.write(self.writer, b'x')
            except BlockingIOError:
                pass
            except OSError as error:
                self.errors.append(str(error))


def _device_identity(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_rdev)


def _alias_identity(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_size,
            info.st_mtime_ns, info.st_ctime_ns)


class CurrentGuard:
    """Setup/full resolve, then exact FD/alias/target identity and boot pread.

    Native sessions retain their original independent per-write FD/boot guard.
    Per-call original clocks are buffered, without disk IO or JSON in cycles.
    """
    def __init__(self, bindings, descriptors, boot_fd, boot_id, stop, *, clock=time.monotonic_ns):
        topology.check_bindings(bindings)
        self.bindings, self.boot_fd, self.boot_id, self.stop = copy.deepcopy(bindings), boot_fd, boot_id, stop
        self.clock, self.calls, self.identities, self.ancestors = clock, [], {}, {}
        self.boot_bytes = boot_id.encode('ascii')
        need(os.pread(boot_fd, 128, 0).strip() == self.boot_bytes, 'Setup boot FD differs')
        self.boot_fd_identity = _device_identity(os.fstat(boot_fd))
        for port in topology.PORTS:
            row = bindings[port]
            for parent in (*Path(row['path']).parents, *Path(row['resolved']).parents):
                key = str(parent)
                if key not in self.ancestors:
                    info = os.lstat(key)
                    need(stat.S_ISDIR(info.st_mode), 'Nonsymlink device ancestor directory required: '+key)
                    self.ancestors[key] = (info.st_dev, info.st_ino, info.st_mode)
            alias, current, resolved, opened = (os.lstat(row['path']), os.stat(row['path']),
                os.lstat(row['resolved']), os.fstat(descriptors[port]))
            identity = _device_identity(opened)
            need(stat.S_ISLNK(alias.st_mode) and stat.S_ISCHR(opened.st_mode) and
                 opened.st_rdev == row['st_rdev'] and
                 _device_identity(current) == _device_identity(resolved) == identity,
                 'Setup alias/canonical/opened device identity differs')
            link = os.readlink(row['path'])
            need(os.path.normpath(os.path.join(os.path.dirname(row['path']), link)) == row['resolved'],
                 'Direct by-path symlink to canonical character device required')
            self.identities[port] = (_alias_identity(alias), identity, link)
        self.setup = {'full_path_resolution_verified': True,
            'boot_fd_identity': self.boot_fd_identity,
            'ancestor_directory_identity': {p: list(v) for p, v in self.ancestors.items()},
            'port_identity': {p: {'alias': list(v[0]), 'target_and_opened_fd': list(v[1]), 'link_text': v[2]}
                              for p, v in self.identities.items()},
            'native_per_write_fd_boot_guard_retained': True,
            'same_opened_inode_and_alias_ancestor_identity_is_additional_guard': True}

    def __call__(self):
        self._check(check_cancel=True)

    def _check(self, *, check_cancel):
        record = {'started_ns': self.clock(), 'thread_native_id': threading.get_native_id(), 'ok': False}
        record['cancellation_check_applied'] = check_cancel
        try:
            if check_cancel:
                need(not self.stop.is_set(), 'Foreground cancelled')
            need(_device_identity(os.fstat(self.boot_fd)) == self.boot_fd_identity and
                 os.pread(self.boot_fd, 128, 0).strip() == self.boot_bytes, 'Current boot FD/bytes changed')
            record['boot_checked_ns'] = self.clock()
            for path, original in self.ancestors.items():
                info = os.lstat(path)
                need((info.st_dev, info.st_ino, info.st_mode) == original,
                     'Current device ancestor identity changed: '+path)
            record['ancestors_checked_ns'] = self.clock()
            for port in topology.PORTS:
                row, (alias, target, link) = self.bindings[port], self.identities[port]
                need(_alias_identity(os.lstat(row['path'])) == alias and
                     os.readlink(row['path']) == link and
                     _device_identity(os.stat(row['path'])) == target and
                     _device_identity(os.lstat(row['resolved'])) == target,
                     'Current by-path/canonical device identity changed: '+port)
            record['ports_checked_ns'] = self.clock()
            record['ok'] = True
        except BaseException as error:
            record['error_type'], record['error'] = type(error).__name__, str(error)
            raise
        finally:
            record['finished_ns'] = self.clock()
            self.calls.append(record)

    def finish(self):
        topology.check_bindings(self.bindings)
        need(_device_identity(os.fstat(self.boot_fd)) == self.boot_fd_identity and
             os.pread(self.boot_fd, 128, 0).strip() == self.boot_bytes, 'End boot FD identity changed')
        # Cleanup verifies identity even after cancellation, once all owners
        # have joined. This does not admit another measured phase or output.
        self._check(check_cancel=False)
        return {'setup': self.setup, 'end_full_path_resolution_verified': True,
                'timing_scope': 'instrumented original current-check call wall intervals; concurrent calls overlap',
                'calls': list(self.calls)}


def timed_snapshot_builder(builder, records, *, clock=time.monotonic_ns):
    """Measure the unchanged pure Batch decode/projection callback, not inference."""
    def build(*args, **kwargs):
        record = {'started_ns': clock(), 'thread_native_id': threading.get_native_id(), 'ok': False}
        try:
            result = builder(*args, **kwargs)
            record['ok'] = True
            return result
        except BaseException as error:
            record['error_type'], record['error'] = type(error).__name__, str(error)
            raise
        finally:
            record['finished_ns'] = clock()
            records.append(record)
    return build


def stage_timings(measurement):
    """Use original pipeline clocks; retain incomplete-cycle nulls."""
    rows = []
    for row in measurement.get('records', []):
        release, gather, infer = row.get('release_ns'), row.get('gather_end_ns'), row.get('infer_end_ns')
        rows.append({'cycle': row.get('cycle'), 'release_ns': release, 'gather_end_ns': gather,
            'infer_end_ns': infer, 'gather_wall_ns': None if release is None or gather is None else gather-release,
            'inference_wall_ns': None if gather is None or infer is None else infer-gather,
            'cycle_end_ns': row.get('cycle_end_ns'), 'completed': row.get('completed')})
    return rows


def finish_current_guard(guard, report, note):
    """Post-owner end verification and independent close before FD teardown."""
    try:
        report['current_check_profile'] = guard.finish()
    except BaseException as error:
        report['current_check_profile'] = {'setup': guard.setup,
            'end_full_path_resolution_verified': False, 'calls': list(guard.calls)}
        note(error)
    finally:
        try:
            guard.close()
            report['current_check_profile']['native_guard_closed'] = True
        except BaseException as error:
            report['current_check_profile']['native_guard_closed'] = False
            note(error)


def execute(args, prepared):
    """Explicit current STOP-only execution. Only the human/root calls this."""
    output = prepared['output']
    output.mkdir(mode=0o700)
    report = {'schema': SCHEMA, 'status': 'STARTING', **FALSE_FLAGS, 'errors': [],
              'input_sha256': dict(prepared['pins'].values), 'settings_restoration': {}}
    primary = None
    device = None
    scope = None
    worker_scopes = {}
    observer = None
    current_guard = None
    snapshot_times = []
    handlers = {}
    cancellation = CancelBinding()
    stop, cancel = cancellation.stop, cancellation.cancel
    def note(error):
        nonlocal primary
        report['status'] = 'ABORTED'
        report['errors'].append(type(error).__name__+': '+str(error))
        if primary is None:
            primary = error
        else:
            primary.add_note('Foreground cleanup: '+str(error))
    try:
        prepared['pins'].verify()
        report['startup'] = startup_readback()
        capture = prepared['config'].topology
        need(BOOT_PATH.read_text().strip() == capture['boot_before'], 'Current boot differs from new capture')
        bindings = topology.bind_ports({p: capture['ports'][p]['path'] for p in topology.PORTS})
        need(bindings == capture['ports'], 'Current four device identities differ from capture')
        report['holders'] = observed_holders(bindings)
        if prepared['holder_receipt'] is not None:
            validate_holder_receipt(prepared['holder_receipt'], capture)
            need(prepared['holder_receipt']['checked_monotonic_ns'] <= time.monotonic_ns(),
                 'Root holder observation is from a future clock')
            report['root_holder_observation'] = prepared['holder_receipt']
        import torch
        torch.set_num_threads(1)
        if torch.get_num_interop_threads() != 1:
            torch.set_num_interop_threads(1)
        need(torch.get_num_threads() == torch.get_num_interop_threads() == 1, 'Actual Torch one-thread readback failed')
        from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked
        from singularitydog_hw.policy_observer import StatefulPolicyObserver
        from singularitydog_hw import (can_timing_probe, dual_can_pipeline_benchmark, imu,
                                      thread_timer_slack, policy_observer_replay)
        profile = prepared['profile']
        original_policy, original_proof = fk.diagnostic_load(profile)
        policy, wrapper, model_proof = checked.load(profile, original_policy, original_proof, active=False)
        observer = model_bridge.create_guarded_observer(prepared['model_plan'], StatefulPolicyObserver,
            policy=policy, max_ticks=args.cycles, torch_module=torch, checked_dispatch_wrapper=wrapper)
        report['model_source'] = model_proof
        verify_origins(args.source_root, prepared['manifest'])
        library = transport_adapter.load_library(args.subset_library,
            expected_sha256=args.subset_library_sha256, ordinary_source_sha256=args.ordinary_source_sha256,
            extension_source_sha256=args.subset_source_sha256, build_record_sha256=args.subset_build_sha256)
        from . import native_current_guard
        guard_library = native_current_guard.load_library(args.current_guard_library,
            expected_sha256=args.current_guard_library_sha256, source_sha256=args.current_guard_source_sha256,
            build_record_sha256=args.current_guard_build_sha256)
        verify_origins(args.source_root, prepared['manifest'])
        report['current_guard_backend'] = prepared['plan']['current_guard_plan']
        with ExitStack() as stack:
            stack.enter_context(can_timing_probe.ownership_locks())
            for port in topology.PORTS:
                stack.enter_context(dual_can_pipeline_benchmark.port_lock(bindings[port]['resolved']))
            topology.check_bindings(bindings)
            report['holders_after_locks'] = observed_holders(bindings)
            cr, cw = os.pipe()
            os.set_blocking(cr, False); os.set_blocking(cw, False)
            stack.callback(os.close, cr); stack.callback(os.close, cw)
            cancellation.bind(stack, cw)
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                handlers[sig] = signal.getsignal(sig)
                signal.signal(sig, lambda signum, frame: cancel())
            def check():
                need(not stop.is_set(), 'Foreground cancelled')
                need(BOOT_PATH.read_text().strip() == capture['boot_before'], 'Current boot changed')
                topology.check_bindings(bindings)
            import serial
            descriptors, boot_files = {}, {}
            for port in topology.PORTS:
                check()
                serial_port = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=.1, exclusive=True)
                stack.callback(serial_port.close)
                serial_port.dtr = False; serial_port.rts = False
                serial_port.port = bindings[port]['path']; serial_port.open()
                info = os.fstat(serial_port.fileno())
                need(stat.S_ISCHR(info.st_mode) and info.st_rdev == bindings[port]['st_rdev'],
                     'Opened CAN descriptor differs from current topology')
                descriptors[port] = serial_port.fileno()
                boot_files[port] = stack.enter_context(BOOT_PATH.open('rb'))
            current_guard = native_current_guard.NativeCurrentGuard(guard_library, bindings, descriptors,
                boot_files[topology.PORTS[0]].fileno(), capture['boot_before'], stop)
            check = current_guard
            # LIFO: this runs before the four boot/serial descriptors close,
            # including exceptions before entering or after leaving pipeline.
            stack.callback(finish_current_guard, current_guard, report, note)
            device = imu.ICM20948()
            try:
                report['imu_configuration'] = device.start()
                check()
                scope = MainScope(True, True)
                def factory(group):
                    lo, hi = owner_bounds(prepared['model_plan'], group)
                    return transport_adapter.ThreeAxisTransport.create(library, descriptors[group.port], group=group,
                        cancel_fd=cr, boot_fd=boot_files[group.port].fileno(), boot_id=capture['boot_before'],
                        raw_lower_by_id=lo, raw_upper_by_id=hi,
                        combined_acquisition=prepared['config'].combined_acquisition,
                        decode_once=prepared['config'].decode_once)
                def worker(port, mask):
                    value = pipeline.LinuxWorkerScope(port, mask)
                    worker_scopes[port] = value
                    return value
                def warm(run, **counts):
                    value = selected_warmup(run, policy, wrapper, torch, profile['h_hypothesis'], **counts)
                    sources = prepared['plan']['checked_plan']['source_sha256']
                    value.update(real_model_verified=True,
                        model_artifact_sha256=prepared['plan']['checked_plan']['references']['checked_model']['sha256'],
                        model_source_sha256=hashlib.sha256(json.dumps(sources, sort_keys=True,
                            separators=(',', ':')).encode()).hexdigest(),
                        model_source_sha256_by_path=dict(sources))
                    return value
                def cancelled():
                    need(not stop.is_set(), 'Foreground cancelled')
                # Boundary selection: the IMU poll loop shares the light check;
                # the full guard runs at the pipeline's three cycle boundaries.
                imu_check = cancelled if prepared['config'].boundary_current_checks else check
                measured = pipeline.run(prepared['config'], factory=factory,
                    imu_read=lambda: read_imu(device, imu_check), observer=observer,
                    snapshot_builder=timed_snapshot_builder(
                        select_snapshot_builder(prepared['model_plan'],
                            combined_acquisition=prepared['config'].combined_acquisition,
                            decode_once=prepared['config'].decode_once), snapshot_times),
                    check_current=check, cancel_io=cancel, model_setup=warm, worker_scope=worker,
                    main_scope=lambda: scope, backend_usage={'kind': 'genuine_original_active_subset_cpp',
                        'library_sha256': args.subset_library_sha256, 'build_record_sha256': args.subset_build_sha256,
                        'source_manifest_sha256': args.source_manifest_sha256},
                    release_wait=make_release_wait(library, cr, observer), check_cancelled=cancelled,
                    execute=True)
                report['measurement'] = pipeline.evidence_report(measured)
                report['observer'] = observer.finish()
                report['status'] = measured['status']
                if measured['status'] != 'COMPLETE_STOP_PROXY_DIAGNOSTIC':
                    note(RuntimeError(str(measured.get('primary_error'))))
            finally:
                try:
                    device.close()
                    report['settings_restoration']['imu'] = device.restore_status
                    need(device.restore_status in ('restored', 'not_needed'), 'IMU register restoration failed')
                except BaseException as error:
                    note(error)
            cancellation.clear()
    except BaseException as error:
        note(error)
    finally:
        cancellation.clear()
        report['cancellation_write_errors'] = list(cancellation.errors)
        if current_guard is not None and 'current_check_profile' not in report:
            report['current_check_profile'] = {'setup': current_guard.setup,
                'end_full_path_resolution_verified': False, 'calls': list(current_guard.calls)}
        report['snapshot_decode_projection_profile'] = snapshot_times
        report['original_pipeline_stage_timings'] = stage_timings(report.get('measurement', {}))
        for error in cancellation.errors:
            note(OSError('Cancellation writer: '+error))
        for sig, handler in handlers.items():
            try:
                signal.signal(sig, handler)
            except BaseException as error:
                note(error)
        report['settings_restoration']['signal_handlers'] = all(signal.getsignal(sig) == handler
                                                               for sig, handler in handlers.items())
        report['settings_restoration']['main'] = None if scope is None else scope.report
        report['settings_restoration']['workers'] = {port: {'timer_slack': value.slack.report if value.slack else None,
            'original_cpu_mask': sorted(value.original) if value.original is not None else None,
            'restored_by_pipeline': report.get('measurement', {}).get('restoration', {}).get(port)}
            for port, value in worker_scopes.items()}
        try:
            prepared['pins'].verify()
            verify_manifest(args.source_root, prepared['manifest'], args.source_count, prepared['pins'])
            verify_origins(args.source_root, prepared['manifest'])
            from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked
            need(fk.plan(prepared['profile']) == prepared['plan']['fk_plan'] and
                 checked.plan(prepared['profile']) == prepared['plan']['checked_plan'], 'Model file proof changed after run')
            report['source_and_input_pins_unchanged'] = True
        except BaseException as error:
            report['source_and_input_pins_unchanged'] = False
            note(error)
        report['failure_retained'] = primary is not None
        report['physical_post_trial_observation'] = None
        report['outer_cpu_c7_emc_restoration_verified_here'] = False
        report['outer_wrapper_must_append_actual_restoration'] = True
        report['plan_source_manifest_sha256'] = args.source_manifest_sha256
        try:
            target = output/'report.json'
            raw = (json.dumps(report, indent=2, allow_nan=False)+'\n').encode()
            with os.fdopen(os.open(target, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), 'wb') as handle:
                handle.write(raw); handle.flush(); os.fsync(handle.fileno())
        except BaseException as error:
            print('Foreground report publication failed: '+repr(error), file=sys.stderr)
            raise
    return report


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    for name in ('source-manifest', 'source-binding', 'original-model-profile', 'topology', 'events',
                 'subset-library', 'current-guard-library'):
        result.add_argument('--'+name, required=True)
        result.add_argument('--'+name+'-sha256', required=True)
    result.add_argument('--source-root', required=True)
    result.add_argument('--source-count', type=int, required=True)
    result.add_argument('--subset-build-sha256', required=True)
    result.add_argument('--subset-source-sha256', required=True)
    result.add_argument('--ordinary-source-sha256', required=True)
    result.add_argument('--current-guard-build-sha256', required=True)
    result.add_argument('--current-guard-source-sha256', required=True)
    result.add_argument('--power-epoch', required=True)
    result.add_argument('--holder-receipt')
    result.add_argument('--holder-receipt-sha256')
    result.add_argument('--output', required=True)
    result.add_argument('--cycles', type=int, choices=(5, 501), default=5)
    result.add_argument('--execute', action='store_true', help='Explicit STOP-proxy only; no enable or Type1')
    result.add_argument('--combined-acquisition', action='store_true',
        help='Explicit single native exchange of three STOP-feedback and one voltage; default separate phases')
    result.add_argument('--boundary-current-checks', action='store_true',
        help='Explicit full current guard only at cycle start, before inference and before output; '
             '(either acquisition branch)')
    result.add_argument('--final-gate-input-identity', action='store_true',
        help='Explicit post-inference check by raw-byte re-verification and pre-inference IMU/snapshot '
             'equality instead of rebuilding the snapshot (either acquisition branch)')
    result.add_argument('--decode-once', action='store_true',
        help='Explicit: decode each exchange once into read-only frozen rows; later takeouts compare raw '
             'byte images, snapshot frames parse owned bytes and plain data is copied via pickle')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    prepared = prepare(args)
    if not args.execute:
        print(json.dumps(prepared['plan'], indent=2, allow_nan=False))
        return 0
    report = execute(args, prepared)
    print(json.dumps({'status': report['status'], 'report': str(prepared['output']/'report.json'), **FALSE_FLAGS}))
    return 0 if report['status'] == 'COMPLETE_STOP_PROXY_DIAGNOSTIC' and not report['failure_retained'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
